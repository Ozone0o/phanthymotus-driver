#!/usr/bin/env python3
"""Full-stack integration test: mock gRPC server → driver bundle → MCP HTTP server.

Drives the actual MCP JSON-RPC endpoint (tools/list + tools/call) so the whole
path — HTTP → JSON-RPC → bundle.dispatch → plugin → RobotControlClient → gRPC —
is exercised end-to-end without a robot.

Usage:
    python3 tests/test_driver_integration.py
"""

from __future__ import annotations

import json
import threading
import urllib.request
from concurrent import futures
from http.server import ThreadingHTTPServer

from test_grpc_client import _ensure_stubs, _make_mock_servicer  # also adds ROOT to sys.path

_ensure_stubs()

import grpc
import adam_control_pb2 as pb
import adam_control_pb2_grpc as pb_grpc

import main as driver_main
from grpc_client import RobotControlClient


def _rpc(port: int, method: str, params: dict) -> dict:
    body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode()
    req = urllib.request.Request(
        f"http://localhost:{port}/mcp", data=body, headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read())


def main() -> int:
    failures: list[str] = []

    def check(label, cond, detail=""):
        print(f"  [{'ok' if cond else 'FAIL'}] {label} {detail}")
        if not cond:
            failures.append(label)

    # 1. mock gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    servicer = _make_mock_servicer(pb, pb_grpc)
    pb_grpc.add_RobotControlServicer_to_server(servicer, server)
    grpc_port = server.add_insecure_port("localhost:0")
    server.start()

    # 2. driver bundle + MCP HTTP server
    client = RobotControlClient(f"localhost:{grpc_port}", timeout_sec=2.0)
    config = {"plugins": {"state": {"enabled": True}, "loco": {"enabled": True}}}
    driver_main._bundle = driver_main.AdamDeviceBundle(config, "testhost", client)
    driver_main._bundle.start_all()

    httpd = ThreadingHTTPServer(("", 0), driver_main.make_handler())
    mcp_port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    print("== initialize ==")
    r = _rpc(mcp_port, "initialize", {})
    check("initialize", "protocolVersion" in r.get("result", {}),
          f"serverInfo={r.get('result', {}).get('serverInfo')}")

    print("== tools/list ==")
    r = _rpc(mcp_port, "tools/list", {})
    tools = {t["name"]: t for t in r["result"]["tools"]}
    check("tools present", {"robot_state", "stand_list", "capabilities", "loco"} <= set(tools),
          f"names={sorted(tools)}")
    check("loco is actuator", tools["loco"]["type"] == "actuator")
    check("robot_state is sensor", tools["robot_state"]["type"] == "sensor")
    check("capabilities is resource", tools["capabilities"]["type"] == "resource")

    print("== tools/call: sensor (pull state) ==")
    r = _rpc(mcp_port, "tools/call", {"name": "robot_state", "arguments": {}})
    text = r["result"]["content"][0]["text"]
    state = json.loads(text)
    check("robot_state ok", state.get("state") == "ok", f"mode={state.get('fsm_name')}")

    print("== tools/call: resource ==")
    r = _rpc(mcp_port, "tools/call", {"name": "capabilities", "arguments": {}})
    caps = json.loads(r["result"]["content"][0]["text"])
    check("capabilities dof", caps.get("dof") == 31, f"dof={caps.get('dof')}")

    print("== tools/call: actuator (loco mode) ==")
    r = _rpc(mcp_port, "tools/call", {"name": "loco", "arguments": {"action": "mode", "mode": "Walk"}})
    result = json.loads(r["result"]["content"][0]["text"])
    check("loco mode ok", result.get("state") == "ok", f"state={result.get('state')}")

    print("== verify mode actually reached the (mock) robot ==")
    rs = client.get_robot_state()
    check("mode == Walk", rs.get("fsm_name") == "Walk", f"mode={rs.get('fsm_name')}")

    print("== unknown tool → error ==")
    r = _rpc(mcp_port, "tools/call", {"name": "nope", "arguments": {}})
    check("unknown tool errors", r.get("error", {}).get("code") == -32601,
          f"code={r.get('error', {}).get('code')}")

    # teardown
    driver_main._bundle.stop_all()
    client.close()
    httpd.shutdown()
    server.stop(0)

    print(f"\n{'ALL PASSED' if not failures else 'FAILED: ' + ', '.join(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
