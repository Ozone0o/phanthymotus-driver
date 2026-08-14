#!/usr/bin/env python3
"""MCP HTTP server for the PNDbotics Adam driver."""

from __future__ import annotations

import json
import os
import re
import signal
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml


def _load_config() -> dict:
    path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(path, encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def _resolve_namespace(config: dict) -> str:
    configured = str(config.get("ros_namespace", "")).strip()
    value = configured or socket.gethostname()
    return re.sub(r"[^a-zA-Z0-9_]", "_", value)


_bundle = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            message = fmt % args
            if '"POST /mcp' in message and "200" in message:
                return
            print(f"[mcp] {self.address_string()} {message}")

        def _send(self, status: int, body: str):
            encoded = body.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()
            self.wfile.write(encoded)

        def do_GET(self):
            self.send_response(404)
            self.end_headers()

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", 0))
                request = json.loads(self.rfile.read(length))
            except Exception:
                self._send(400, json.dumps({
                    "jsonrpc": "2.0", "id": None,
                    "error": {"code": -32700, "message": "Parse error"},
                }))
                return

            request_id = request.get("id")
            if request_id is None:
                self.send_response(202)
                self.end_headers()
                return

            method = request.get("method", "")
            params = request.get("params") or {}

            def ok(result):
                self._send(200, json.dumps({
                    "jsonrpc": "2.0", "id": request_id, "result": result,
                }))

            def error(code, message):
                self._send(200, json.dumps({
                    "jsonrpc": "2.0", "id": request_id,
                    "error": {"code": code, "message": message},
                }))

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {
                            "name": "adam-device-bundle",
                            "version": "1.0.0",
                        },
                    })
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    args = params.get("arguments") or {}
                    result = _bundle.dispatch(name, args)
                    if result is None:
                        error(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{
                            "type": "text", "text": json.dumps(result),
                        }]})
                else:
                    error(-32601, f"Method not found: {method}")
            except Exception as exc:
                import traceback
                traceback.print_exc()
                error(-32603, str(exc))

    return Handler


def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with Agent Core and keep the registration alive."""
    import ssl
    import time
    import urllib.request

    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url": f"http://localhost:{mcp_port}/mcp",
        "category": category,
    }).encode()
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE

    def register_loop():
        while True:
            try:
                request = urllib.request.Request(
                    f"{agent_core_url}/api/mcp",
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(request, timeout=3, context=context):
                    pass
                time.sleep(30)
            except Exception as exc:
                print(f"[register] failed: {exc}, retrying in 5s")
                time.sleep(5)

    threading.Thread(target=register_loop, daemon=True, name="register").start()


def _make_dds_channels(config: dict):
    """Create all DDS readers/writers before rclpy is initialized."""
    channels = {
        "lowstate_sub": None,
        "arm_lowstate_sub": None,
        "handstate_sub": None,
        "lowcmd_pub": None,
        "hand_pub": None,
    }
    try:
        from pndbotics_sdk_py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import (
            HandCmd_,
            HandState_,
            LowCmd_,
            LowState_,
        )

        domain_id = int(config.get("dds_domain_id", 0))
        interface = str(config.get("dds_network_interface", "")).strip()
        if interface:
            ChannelFactoryInitialize(domain_id, interface)
        else:
            ChannelFactoryInitialize(domain_id)
        print(
            f"[adam] DDS initialized (domain={domain_id}, "
            f"iface={interface or 'auto'})"
        )

        channels["lowstate_sub"] = ChannelSubscriber("rt/lowstate", LowState_)
        channels["arm_lowstate_sub"] = ChannelSubscriber("rt/lowstate", LowState_)
        channels["handstate_sub"] = ChannelSubscriber("rt/handstate", HandState_)
        channels["lowcmd_pub"] = ChannelPublisher("rt/lowcmd", LowCmd_)
        channels["hand_pub"] = ChannelPublisher("rt/handcmd", HandCmd_)
        for channel in channels.values():
            channel.Init()
        print("[adam] DDS channels created")
    except ImportError:
        print("[adam] WARNING: pndbotics_sdk_py not found; DDS disabled")
    except Exception as exc:
        print(f"[adam] WARNING: DDS initialization failed: {exc}")
    return channels


def main():
    global _bundle

    config = _load_config()
    namespace = _resolve_namespace(config)
    network_interface = sys.argv[1] if len(sys.argv) > 1 else None
    if network_interface:
        config["dds_network_interface"] = network_interface
    mcp_port = int(config.get("mcp_port", 15702))
    print(
        f"[adam] namespace={namespace} "
        f"variant={config.get('variant', 'pro')} mcp_port={mcp_port}"
    )

    channels = _make_dds_channels(config)

    from grpc_client import AdamGrpcClient
    grpc_host = os.environ.get("GRPC_HOST", config.get("grpc_host", "localhost"))
    grpc_port = int(os.environ.get("GRPC_PORT", config.get("grpc_port", 6666)))
    grpc_client = AdamGrpcClient(grpc_host, grpc_port)
    grpc_client.connect()
    print(f"[adam] gRPC client → {grpc_host}:{grpc_port}")

    import rclpy
    import rclpy.executors
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    print("[adam] ROS2 initialized")

    from device import AdamDeviceBundle
    _bundle = AdamDeviceBundle(
        config,
        namespace,
        executor,
        grpc_client,
        **channels,
    )
    _bundle.start_all()
    print(f"[adam] Bundle loaded ({len(_bundle.get_all_tools())} tools)")

    def spin_ros():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    threading.Thread(target=spin_ros, daemon=True, name="ros2_spin").start()
    _start_registration(mcp_port, config.get("name", "PNPbotics Adam"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[adam] MCP server → http://localhost:{mcp_port}")

    def shutdown(signum, frame):
        print(f"[adam] signal {signum}, shutting down")
        _bundle.stop_all()
        grpc_client.close()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        grpc_client.close()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
