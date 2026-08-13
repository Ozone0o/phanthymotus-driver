#!/usr/bin/env python3
"""
dji/mavic3e/main.py — DJI Mavic 3E 无人机设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
通过 bridge_client 与 psdk_bridge (C 进程) 通信，或在 mock 模式下模拟响应。

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
    AGENT_CORE_URL — Agent Core 地址（默认 https://localhost:15678）
"""

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

import rclpy
import rclpy.executors


# ── Config ────────────────────────────────────────────────────────────────────

def _load_config() -> dict:
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


def _resolve_namespace(cfg: dict) -> str:
    ns = cfg.get("ros_namespace", "").strip()
    if ns:
        return re.sub(r"[^a-zA-Z0-9_]", "_", ns)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


# ── Bundle ────────────────────────────────────────────────────────────────────

class Mavic3EDeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor, bridge):
        self._plugins: list = []
        self._bridge = bridge
        self._namespace = namespace
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("telemetry", {}).get("enabled", False):
            from device import TelemetryPlugin
            self._plugins.append(TelemetryPlugin(
                plugins_cfg["telemetry"], namespace, executor, bridge))
            print("[bundle] TelemetryPlugin loaded")

        if plugins_cfg.get("camera_stream", {}).get("enabled", False):
            from device import CameraStreamPlugin
            self._plugins.append(CameraStreamPlugin(
                plugins_cfg["camera_stream"], namespace, executor, bridge))
            print("[bundle] CameraStreamPlugin loaded")

        if plugins_cfg.get("perception", {}).get("enabled", False):
            from device import PerceptionPlugin
            self._plugins.append(PerceptionPlugin(
                plugins_cfg["perception"], namespace, executor, bridge))
            print("[bundle] PerceptionPlugin loaded")

        if plugins_cfg.get("hms", {}).get("enabled", False):
            from device import HmsPlugin
            self._plugins.append(HmsPlugin(
                plugins_cfg["hms"], namespace, executor, bridge))
            print("[bundle] HmsPlugin loaded")

        if plugins_cfg.get("flight", {}).get("enabled", False):
            from device import FlightPlugin
            self._plugins.append(FlightPlugin(
                plugins_cfg["flight"], namespace, executor, bridge))
            print("[bundle] FlightPlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(
                plugins_cfg["camera"], namespace, executor, bridge))
            print("[bundle] CameraPlugin loaded")

        if plugins_cfg.get("gimbal", {}).get("enabled", False):
            from device import GimbalPlugin
            self._plugins.append(GimbalPlugin(
                plugins_cfg["gimbal"], namespace, executor, bridge))
            print("[bundle] GimbalPlugin loaded")

        if plugins_cfg.get("waypoint", {}).get("enabled", False):
            from device import WaypointPlugin
            self._plugins.append(WaypointPlugin(
                plugins_cfg["waypoint"], namespace, executor, bridge))
            print("[bundle] WaypointPlugin loaded")

        if plugins_cfg.get("time_sync", {}).get("enabled", False):
            from device import TimeSyncPlugin
            self._plugins.append(TimeSyncPlugin(
                plugins_cfg["time_sync"], namespace, executor, bridge))
            print("[bundle] TimeSyncPlugin loaded")

    def start_all(self) -> None:
        for i, p in enumerate(self._plugins):
            try:
                p.start()
            except Exception as e:
                print(f"[bundle] Plugin {i} ({type(p).__name__}) start() FAILED: {e}", flush=True)
                import traceback
                traceback.print_exc()
        print(f"[bundle] All {len(self._plugins)} plugins started", flush=True)

    def stop_all(self) -> None:
        for p in self._plugins:
            try:
                p.stop()
            except Exception:
                pass
        self._bridge.stop()
        print("[bundle] All plugins stopped")

    def get_all_tools(self) -> list:
        tools = [self._model_tool()]
        for p in self._plugins:
            if hasattr(p, "get_tools"):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def _model_tool(self) -> dict:
        return {
            "name": "model",
            "type": "resource",
            "description": "DJI Mavic 3E aircraft metadata (cameras, gimbal range, specs)",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        if tool_name == "model":
            info_path = Path(__file__).parent / "resource" / "mavic3e_info.json"
            info = json.loads(info_path.read_text())
            # Merge dynamic aircraft info from bridge
            try:
                resp = self._bridge.get_aircraft_info()
                if resp and resp.get("ok"):
                    info["aircraft_info"] = resp["data"]
            except Exception:
                pass
            return info
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args["_tool_name"] = tool_name
                    return p.dispatch(action, args)
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: Mavic3EDeviceBundle | None = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and "200" in msg:
                return
            print(f"[mcp] {self.address_string()} {msg}")

        def _send(self, status: int, body: str):
            encoded = body.encode()
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
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            try:
                rpc = json.loads(raw)
            except Exception:
                self._send(400, json.dumps({
                    "jsonrpc": "2.0", "id": None,
                    "error": {"code": -32700, "message": "Parse error"},
                }))
                return

            rid = rpc.get("id")
            method = rpc.get("method", "")
            params = rpc.get("params") or {}

            if rid is None:
                self.send_response(202)
                self.end_headers()
                return

            def ok(result):
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid, "result": result}))

            def err(code, msg):
                self._send(200, json.dumps({
                    "jsonrpc": "2.0", "id": rid,
                    "error": {"code": code, "message": msg},
                }))

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "dji-mavic3e-bundle", "version": "1.0.0"},
                    })
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    args = params.get("arguments") or {}
                    result = _bundle.dispatch(name, args)
                    if result is None:
                        err(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(result)}]})
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:
                err(-32603, str(e))

    return Handler


# ── Device auto-detect (container startup) ────────────────────────────────

# DJI exposes a CDC ACM control interface in addition to the UART exposed by an
# external E-Port USB-UART adapter.  The adapter's VID/PID must not be used as
# the identity of the E-Port: FTDI, CH340, CP210x, PL2303, and other adapters
# are all valid as long as Linux exposes them as a USB serial device and the
# PSDK handshake succeeds.
_DJI_USB_VID = "2ca3"


def _describe_uart_device(device_path: str) -> dict:
    """Return the USB identity and kernel driver for a serial device."""
    vid, pid = _get_usb_ids(device_path)
    return {
        "path": device_path,
        "vid": vid,
        "pid": pid,
        "driver": _get_usb_driver(device_path),
    }


def _enumerate_uart_devices() -> list[dict]:
    """Enumerate USB-backed serial devices in E-Port probe order.

    ttyUSB devices are preferred because the Mavic 3E commonly exposes its
    own DJI CDC ACM control port at the same time.  Other USB serial devices,
    including non-DJI ACM adapters, are tried next.  DJI ACM devices are kept
    as the last fallback for direct-E-Port configurations.
    """
    import glob

    devices = sorted(set(
        glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    ))
    candidates = []
    for device_path in devices:
        candidate = _describe_uart_device(device_path)
        if not candidate["vid"] or not candidate["pid"]:
            # Built-in UARTs and stale device nodes do not have USB identity;
            # the C HAL cannot report a valid E-Port USB device for them.
            continue
        candidates.append(candidate)

    def _probe_order(candidate: dict) -> tuple[int, str]:
        path = candidate["path"]
        if os.path.basename(path).startswith("ttyUSB"):
            return 0, path
        if candidate["vid"] != _DJI_USB_VID:
            return 1, path
        return 2, path

    return sorted(candidates, key=_probe_order)


def _detect_uart_devices(timeout: int = 30) -> list[dict]:
    """Wait for at least one USB serial candidate and return all candidates."""
    import time as _t

    start = _t.time()
    while True:
        candidates = _enumerate_uart_devices()
        if candidates:
            summary = ", ".join(
                f"{c['path']} (driver={c['driver'] or 'unknown'} "
                f"VID={c['vid']} PID={c['pid']})"
                for c in candidates
            )
            print(f"[bundle] USB serial candidates: {summary}")
            return candidates

        elapsed = _t.time() - start
        if elapsed > timeout:
            print(f"[bundle] WARNING: no USB serial device found after {timeout}s")
            return []
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            print(f"[bundle] waiting for USB serial device... ({int(elapsed)}s)")
        _t.sleep(1)


def _get_usb_ids(device_path: str) -> tuple[str, str]:
    """Read USB VID/PID from sysfs for a tty device."""
    dev_name = os.path.basename(os.path.realpath(device_path))
    sysfs_device = os.path.realpath(f"/sys/class/tty/{dev_name}/device")
    if not os.path.exists(sysfs_device):
        return "", ""

    # Walk up the resolved sysfs tree.  The USB device descriptor is normally
    # one level above the tty interface, but walking makes this work through
    # composite adapters and USB hubs as well.
    base = sysfs_device
    for _ in range(8):
        vid_path = os.path.join(base, "idVendor")
        pid_path = os.path.join(base, "idProduct")
        try:
            with open(vid_path) as f:
                vid = f.read().strip()
            with open(pid_path) as f:
                pid = f.read().strip()
            if vid:
                return vid, pid
        except (FileNotFoundError, PermissionError):
            pass
        parent = os.path.dirname(base)
        if parent == base:
            break
        base = parent
    return "", ""


def _get_usb_driver(device_path: str) -> str:
    """Read the kernel USB-serial driver name for a tty device."""
    dev_name = os.path.basename(os.path.realpath(device_path))
    driver_link = f"/sys/class/tty/{dev_name}/device/driver"
    driver_path = os.path.realpath(driver_link)
    if not os.path.exists(driver_path):
        return ""
    return os.path.basename(driver_path)


def _stop_bridge_process(bridge_proc) -> None:
    """Terminate a failed probe without leaving a child process behind."""
    import subprocess as _sp

    if bridge_proc is None or bridge_proc.poll() is not None:
        return
    bridge_proc.terminate()
    try:
        bridge_proc.wait(timeout=5)
    except _sp.TimeoutExpired:
        bridge_proc.kill()
        bridge_proc.wait()


def _start_psdk_bridge(bridge_bin: str, socket_path: str,
                       psdk_cfg: dict, candidates: list[dict]):
    """Probe candidates by running the real PSDK initialization on each one.

    A serial device being present only proves that Linux enumerated an
    adapter.  The bridge creates its IPC socket only after DjiCore_Init and
    application startup succeed, so socket readiness is the protocol-level
    E-Port selection signal.
    """
    import subprocess as _sp
    import time as _t

    probe_timeout = max(5, int(psdk_cfg.get("probe_timeout_s", 30)))
    for index, candidate in enumerate(candidates, start=1):
        detected_dev = candidate["path"]
        try:
            os.unlink(socket_path)
        except FileNotFoundError:
            pass
        except OSError as exc:
            print(f"[bundle] cannot remove stale bridge socket {socket_path}: {exc}")
            continue

        print(
            f"[bundle] probing E-Port candidate {index}/{len(candidates)}: "
            f"{detected_dev} (driver={candidate['driver'] or 'unknown'} "
            f"VID={candidate['vid']} PID={candidate['pid']})"
        )
        bridge_proc = _sp.Popen(
            [bridge_bin, socket_path,
             psdk_cfg.get("app_id", ""),
             psdk_cfg.get("app_key", ""),
             psdk_cfg.get("app_license", ""),
             detected_dev,
             str(psdk_cfg.get("baud_rate", 921600))],
            stdout=sys.stdout, stderr=sys.stderr,
        )

        ready = False
        for _ in range(probe_timeout * 10):
            if os.path.exists(socket_path):
                ready = bridge_proc.poll() is None
                break
            return_code = bridge_proc.poll()
            if return_code is not None:
                print(
                    f"[bundle] candidate {detected_dev} failed "
                    f"(psdk_bridge exit={return_code})"
                )
                break
            _t.sleep(0.1)

        if ready:
            print(f"[bundle] E-Port PSDK initialization succeeded on {detected_dev}")
            return bridge_proc, detected_dev

        if bridge_proc.poll() is None:
            print(
                f"[bundle] candidate {detected_dev} did not initialize "
                f"within {probe_timeout}s"
            )
        _stop_bridge_process(bridge_proc)
        try:
            os.unlink(socket_path)
        except FileNotFoundError:
            pass
        except OSError:
            pass

    return None, None


# ── Entry point ───────────────────────────────────────────────────────────────

def _configure_usb_gadget():
    """Check USB network status for DJI video/perception support."""
    pass  # Network HAL configured in C bridge based on UDC state


def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with agent-core in a background thread, then heartbeat every 30s."""
    import urllib.request as _urllib
    import ssl as _ssl

    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url": f"http://localhost:{mcp_port}/mcp",
        "category": category,
    }).encode()
    _ctx = _ssl.create_default_context()
    _ctx.check_hostname = False
    _ctx.verify_mode = _ssl.CERT_NONE

    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(
                    f"{agent_core_url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with _urllib.urlopen(req, timeout=3, context=_ctx):
                    pass
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)

    threading.Thread(target=_run, daemon=True, name="register").start()


def main():
    global _bundle

    cfg = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port = int(cfg.get("mcp_port", 15702))
    psdk_cfg = cfg.get("psdk_bridge", {})

    # Resolve the bridge binary once; each candidate probe starts a fresh
    # bridge process so a failed PSDK initialization cannot contaminate the
    # next candidate.
    uart_dev = psdk_cfg.get("uart_dev", "auto")
    bridge_bin = "/usr/local/bin/psdk_bridge"
    if not os.path.exists(bridge_bin):
        bridge_bin = "/work/psdk_bridge/build/psdk_bridge"
    socket_path = "/tmp/psdk_bridge.sock"

    # Configure USB Bulk gadget (host-side setup required first via install script)
    # Container only checks if FFS endpoints are available — does NOT modify configfs.
    _configure_usb_gadget()

    import time as _t
    bridge_proc = None
    detected_dev = None
    while bridge_proc is None:
        if uart_dev == "auto":
            candidates = _detect_uart_devices(timeout=10)
        elif os.path.exists(uart_dev):
            candidates = [_describe_uart_device(uart_dev)]
        else:
            print(f"[bundle] configured UART device is not present: {uart_dev}")
            candidates = []

        if candidates:
            bridge_proc, detected_dev = _start_psdk_bridge(
                bridge_bin, socket_path, psdk_cfg, candidates)

        if bridge_proc is None:
            print("[bundle] no USB serial candidate completed PSDK initialization; retrying in 5s")
            _t.sleep(5)

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port} uart={detected_dev}")
    print(f"[bundle] psdk_bridge started (pid={bridge_proc.pid})")

    # Give bridge a moment to accept connections
    _t.sleep(1)

    # Bridge client — connects to psdk_bridge C process (retry up to 5 times)
    from bridge_client import BridgeClient
    bridge = None
    for attempt in range(5):
        try:
            bridge = BridgeClient(
                socket_path=socket_path,
                mock_mode=False,
            )
            break
        except Exception as e:
            print(f"[bundle] BridgeClient connect attempt {attempt+1} failed: {e}")
            _t.sleep(2)
    if bridge is None:
        print("[bundle] ERROR: cannot connect to psdk_bridge, exiting")
        bridge_proc.terminate()
        sys.exit(1)
    print(f"[bundle] BridgeClient connected (uart={detected_dev})")

    # ROS2
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    _bundle = Mavic3EDeviceBundle(cfg, namespace, executor, bridge)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "DJI Mavic 3E"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        bridge_proc.terminate()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        bridge_proc.terminate()
        bridge_proc.wait(timeout=3)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
