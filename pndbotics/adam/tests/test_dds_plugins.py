#!/usr/bin/env python3
"""Unit-test the DDS plugin layer with a fake DdsClient (no real DDS runtime needed).

Verifies tool schemas and dispatch logic for DdsStatePlugin + HandPlugin, and that
the bundle wires them when a DDS client is provided.  The real DdsClient (which
talks to rt/lowstate etc. via the vendored pnd_sdk_python) can only be exercised
on the robot / in the container where cyclonedds is installed.

Usage:
    python3 tests/test_dds_plugins.py
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


class FakeDdsClient:
    """Minimal stand-in for dds_client.DdsClient."""

    def __init__(self):
        self._positions = [500] * 12
        self._error = None

    def status(self):
        return {"state": "running", "error": self._error}

    def joints(self):
        return {
            "position": [0.0] * 31,
            "velocity": [0.1] * 31,
            "torque": [0.2] * 31,
            "mode": [0] * 31,
            "mode_pr": 0,
            "tick": 1,
        }

    def imu(self):
        return {"quaternion_wxyz": [1, 0, 0, 0], "ypr_rad": [0, 0, 0]}

    def battery(self):
        return {"voltage_v": 48.0, "power_w": 10.0}

    def remote(self):
        return {"wireless_remote": [0.0] * 19}

    def hand_state(self):
        return list(self._positions)

    def set_hand(self, positions):
        self._positions = list(positions)
        return {"state": "ok", "position": list(positions)}


def main() -> int:
    from device import DdsStatePlugin, HandPlugin

    dds = FakeDdsClient()
    failures: list[str] = []

    def check(label, cond, detail=""):
        print(f"  [{'ok' if cond else 'FAIL'}] {label} {detail}")
        if not cond:
            failures.append(label)

    print("== DdsStatePlugin ==")
    state = DdsStatePlugin({}, "host", dds)
    tools = {t["name"]: t for t in state.get_tools()}
    check("state tools present", {"joints", "imu", "battery", "remote", "model"} <= set(tools),
          f"names={sorted(tools)}")
    check("joints is skeleton sensor", tools["joints"]["topic_out"][0]["format"] == "sensor/skeleton")
    check("model is resource", tools["model"]["type"] == "resource")

    r = state.dispatch("joints", {"_tool_name": "joints"})
    check("joints payload", len(r.get("joints", [])) == 31 and r["joints"][0]["name"] == "hipPitch_Left",
          f"n={len(r.get('joints', []))} j0={r['joints'][0]['name'] if r.get('joints') else '?'}")
    check("joints q/dq/tau mapped", r["joints"][0]["q"] == 0.0 and r["joints"][0]["tau"] == 0.2)

    r = state.dispatch("imu", {"_tool_name": "imu"})
    check("imu payload", r.get("quaternion_wxyz") == [1, 0, 0, 0])

    r = state.dispatch("model", {"_tool_name": "model"})
    check("model returns urdf", "urdf" in r and "<robot" in r.get("urdf", ""),
          f"urdf_len={len(r.get('urdf', ''))}")

    print("== HandPlugin ==")
    hand = HandPlugin({}, "host", dds)
    tools = {t["name"]: t for t in hand.get_tools()}
    check("hand tools present", {"hand", "hand_state"} <= set(tools), f"names={sorted(tools)}")
    check("hand is actuator", tools["hand"]["type"] == "actuator")

    r = hand.dispatch("set", {"positions": [1000] * 12})
    check("hand set ok", r.get("state") == "ok", f"state={r.get('state')}")

    r = hand.dispatch("open", {})
    check("hand open", r.get("position") == [1000] * 12)

    r = hand.dispatch("close", {})
    check("hand close", r.get("position") == [0] * 12)

    r = hand.dispatch("hand_state", {"_tool_name": "hand_state"})
    check("hand_state reads back", r.get("position") == [0] * 12, f"pos={r.get('position')}")

    r = hand.dispatch("set", {"positions": [500, 1]})
    check("hand set rejects bad length", "error" in r)

    print("== bundle wiring ==")
    import main as driver_main

    config = {
        "plugins": {"state": {"enabled": False}, "dds_state": {"enabled": True},
                    "hand": {"enabled": True}, "loco": {"enabled": False}},
    }
    bundle = driver_main.AdamDeviceBundle(config, "host", None, dds)
    bundle_tools = {t["name"] for t in bundle.get_all_tools()}
    check("bundle wires dds plugins", {"joints", "imu", "battery", "remote", "model", "hand", "hand_state"} <= bundle_tools,
          f"tools={sorted(bundle_tools)}")

    print(f"\n{'ALL PASSED' if not failures else 'FAILED: ' + ', '.join(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
