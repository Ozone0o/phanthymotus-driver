#!/usr/bin/env python3
"""PNDbotics Adam Pro MCP driver entrypoint.

Loads config.yaml, connects to the robot's upper-level motion-control gRPC
service, aggregates the device plugins into a single MCP JSON-RPC HTTP server,
and registers itself with Agent Core.

Usage:
    python3 main.py

Environment:
    CONFIG_PATH  — config.yaml path (default: alongside this file)
"""

from __future__ import annotations

import json
import os
import re
import signal
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml


# ── Config ────────────────────────────────────────────────────────────────────

def _load_config() -> dict:
    path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _resolve_namespace(config: dict) -> str:
    value = str(config.get("ros_namespace", "")).strip()
    if value:
        return re.sub(r"[^a-zA-Z0-9_]", "_", value)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


# ── Bundle ────────────────────────────────────────────────────────────────────

class AdamDeviceBundle:
    def __init__(self, config: dict, namespace: str, client, dds=None):
        from device import DdsStatePlugin, HandPlugin, LocoPlugin, StatePlugin

        self._plugins: list = []
        plugins = config.get("plugins", {})

        state_config = dict(config.get("state", {}))
        if plugins.get("state", {}).get("enabled", True):
            self._plugins.append(StatePlugin(state_config, namespace, client))

        if dds is not None and plugins.get("dds_state", {}).get("enabled", True):
            self._plugins.append(DdsStatePlugin(plugins.get("dds_state", {}), namespace, dds))

        if dds is not None and plugins.get("hand", {}).get("enabled", True):
            self._plugins.append(HandPlugin(plugins.get("hand", {}), namespace, dds))

        if plugins.get("loco", {}).get("enabled", True):
            self._plugins.append(LocoPlugin(plugins.get("loco", {}), namespace, client))

    def start_all(self) -> None:
        for plugin in self._plugins:
            try:
                plugin.start()
                print(f"[bundle] {type(plugin).__name__} started", flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"[bundle] {type(plugin).__name__} start failed: {exc}", flush=True)

    def stop_all(self) -> None:
        for plugin in reversed(self._plugins):
            try:
                plugin.stop()
            except Exception as exc:  # noqa: BLE001
                print(f"[bundle] {type(plugin).__name__} stop failed: {exc}", flush=True)

    def get_all_tools(self) -> list[dict]:
        tools: list[dict] = []
        for plugin in self._plugins:
            if hasattr(plugin, "get_tools"):
                tools.extend(plugin.get_tools())
            else:
                tools.append(plugin.get_tool())
        return tools

    def dispatch(self, tool_name: str, arguments: dict) -> dict | None:
        for plugin in self._plugins:
            tool_defs = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            for definition in tool_defs:
                if definition["name"] != tool_name:
                    continue
                args = dict(arguments)
                if definition["type"] == "resource":
                    return plugin.dispatch(tool_name, args)
                action = args.pop("action", tool_name)
                args["_tool_name"] = tool_name
                return plugin.dispatch(action, args)
        return None


_bundle: AdamDeviceBundle | None = None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' not in msg or "200" not in msg:
                print(f"[mcp] {self.address_string()} {msg}", flush=True)

        def _send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self._send_json(200, {"state": "running", "driver": "pndbotics-adam"})
            else:
                self._send_json(404, {"error": "not found"})

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()

        def do_POST(self):
            if self.path != "/mcp":
                self._send_json(404, {"error": "not found"})
                return
            length = int(self.headers.get("Content-Length", 0))
            try:
                rpc = json.loads(self.rfile.read(length))
            except Exception:
                self._send_json(400, {"jsonrpc": "2.0", "id": None,
                                      "error": {"code": -32700, "message": "Parse error"}})
                return
            request_id = rpc.get("id")
            if request_id is None:
                self.send_response(202)
                self.end_headers()
                return

            def ok(result):
                self._send_json(200, {"jsonrpc": "2.0", "id": request_id, "result": result})

            def error(code, message):
                self._send_json(200, {"jsonrpc": "2.0", "id": request_id,
                                      "error": {"code": code, "message": message}})

            method = rpc.get("method", "")
            params = rpc.get("params") or {}
            try:
                if method == "initialize":
                    ok({"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                        "serverInfo": {"name": "pndbotics-adam-bundle", "version": "1.0.0"}})
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    result = _bundle.dispatch(name, params.get("arguments") or {})
                    if result is None:
                        error(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(result, ensure_ascii=False)}]})
                else:
                    error(-32601, f"Method not found: {method}")
            except (TypeError, ValueError) as exc:
                error(-32602, str(exc))
            except Exception as exc:
                import traceback

                traceback.print_exc()
                error(-32603, str(exc))

    return Handler


# ── Registration ──────────────────────────────────────────────────────────────

def _start_registration(port: int, config: dict) -> None:
    import ssl
    import time
    import urllib.request

    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "id": "adam-driver",
        "name": config.get("name", "PNDbotics Adam Pro"),
        "url": f"http://localhost:{port}/mcp",
        "transport": "http",
        "category": "driver",
    }).encode()
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE

    def register_loop():
        while True:
            try:
                request = urllib.request.Request(
                    f"{agent_core_url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with urllib.request.urlopen(request, timeout=3, context=context):
                    pass
                time.sleep(30)
            except Exception as exc:
                print(f"[register] failed: {exc}; retrying in 5s", flush=True)
                time.sleep(5)

    threading.Thread(target=register_loop, daemon=True, name="register").start()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    global _bundle

    config = _load_config()
    namespace = _resolve_namespace(config)
    port = int(config.get("mcp_port", 15709))
    grpc_server = os.environ.get("GRPC_SERVER") or str(config.get("grpc_server", "localhost:6666"))
    grpc_timeout = float(config.get("grpc_timeout_sec", 2.0))
    print(f"[bundle] namespace={namespace} port={port} grpc_server={grpc_server}", flush=True)

    from grpc_client import RobotControlClient

    client = RobotControlClient(grpc_server, grpc_timeout)

    dds = None
    dds_config = config.get("dds", {})
    if dds_config.get("enabled", True):
        from dds_client import DdsClient

        dds = DdsClient(dds_config)
        dds.start()
        status = dds.status()
        if status.get("state") == "error":
            print(f"[bundle] DDS unavailable: {status.get('error')}", flush=True)

    # ── ROS2 republish bridge (live dashboard streams) ──
    # Optional: degrades to pull-only tools/call if rclpy is unavailable.
    executor = None
    state_node = None
    ros_ready = False
    if dds is not None:
        try:
            import rclpy
            import rclpy.executors
            from ros_bridge import AdamStateNode

            rclpy.init()
            executor = rclpy.executors.MultiThreadedExecutor()
            state_node = AdamStateNode(namespace, dds, client)
            executor.add_node(state_node)
            state_node.start_polling()
            ros_ready = True
        except Exception as exc:  # noqa: BLE001 — degrade to pull-only without ROS
            print(f"[bundle] ROS2 republish unavailable (live streams disabled): {exc}", flush=True)

    _bundle = AdamDeviceBundle(config, namespace, client, dds)
    _bundle.start_all()

    if ros_ready:
        def _spin():
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)

        threading.Thread(target=_spin, daemon=True, name="adam_spin").start()

    _start_registration(port, config)

    server = ThreadingHTTPServer(("", port), make_handler())
    print(f"[bundle] MCP server http://localhost:{port}/mcp", flush=True)
    print(f"[bundle] gRPC {'connected' if client.connected() else 'unreachable (state will report errors)'}",
          flush=True)
    if dds is not None:
        print(f"[bundle] DDS {dds.status()['state']} (domain={dds_config.get('domain_id', 1)})", flush=True)
    if ros_ready:
        print("[bundle] ROS2 republish ready (live dashboard streams enabled)", flush=True)

    stopping = threading.Event()

    def shutdown(signum, _frame):
        if stopping.is_set():
            return
        stopping.set()
        print(f"[bundle] signal {signum}; stopping", flush=True)
        _bundle.stop_all()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        if state_node is not None:
            state_node.stop_polling()
        client.close()
        if dds is not None:
            dds.stop()
        if ros_ready:
            executor.shutdown()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
