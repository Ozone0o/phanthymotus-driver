#!/usr/bin/env python3
"""End-to-end test of grpc_client.RobotControlClient against a mock RobotControl server.

This proves the driver's gRPC layer — proto → stubs → client wrapper — works
without a real robot.  The mock implements the OFFICIAL ``adam_control.proto``
(from the PNDbotics "Client" package), so field names match the real server.

Usage:
    python3 tests/test_grpc_client.py     # needs grpcio + grpcio-tools
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from concurrent import futures
from pathlib import Path

HERE = Path(__file__).resolve().parent   # .../adam/tests
ROOT = HERE.parent                        # .../adam
sys.path.insert(0, str(ROOT))


def _ensure_stubs() -> None:
    """Generate adam_control_pb2/_pb2_grpc into a temp dir unless already importable."""
    try:
        import adam_control_pb2_grpc  # noqa: F401
        return
    except ImportError:
        pass
    tmp = tempfile.mkdtemp(prefix="adam-stubs-")
    subprocess.check_call([
        sys.executable, "-m", "grpc_tools.protoc",
        "-I", str(ROOT / "proto"),
        "--python_out", tmp,
        "--grpc_python_out", tmp,
        str(ROOT / "proto" / "adam_control.proto"),
    ])
    sys.path.insert(0, tmp)


def _make_mock_servicer(pb, pb_grpc):
    """A stateful in-memory RobotControl implementation mirroring documented behavior."""

    class MockRobotControl(pb_grpc.RobotControlServicer):
        def __init__(self):
            self.mode = "Stand"
            self.motion = ""
            self.carry_box = ""
            self.pitch = self.roll = self.yaw = self.height = 0.0
            self.x = self.y = self.yaw_speed = 0.0
            self.dynamic = False
            self.com = False
            self.error_cleared = False

        def SetMode(self, request, context):
            self.mode = request.mode
            return pb.SetModeResponse(success=True, message=f"Mode set to {request.mode}")

        def SetStandMotion(self, request, context):
            self.motion = request.motion
            return pb.SetStandMotionResponse(success=True, message=f"Motion: {request.motion}")

        def SetStandCarryBox(self, request, context):
            self.carry_box = request.carry_box
            return pb.SetCarryBoxResponse(success=True, message=f"Carry box: {request.carry_box}")

        def SetStandAction(self, request, context):
            self.pitch, self.roll = request.stand_pitch, request.stand_roll
            self.yaw, self.height = request.stand_yaw, request.stand_height
            return pb.SetActionResponse(success=True, message="Stand action set")

        def SetStandDynamic(self, request, context):
            self.dynamic = request.dynamic_stand
            return pb.SetDynamicStandResponse(success=True, message=f"Dynamic stand: {request.dynamic_stand}")

        def SetSpeed(self, request, context):
            self.x, self.y, self.yaw_speed = request.x_speed, request.y_speed, request.yaw_speed
            self.continuous = request.continous
            return pb.SetSpeedResponse(success=True, message="Speed set")

        def AutoUnigaitCOM(self, request, context):
            self.com = request.unigait_mode_com_x
            return pb.SetUnigaitCOMResponse(success=True, message="COM set")

        def SetErrorClear(self, request, context):
            self.error_cleared = request.error_clear_flag
            return pb.SetErrorClearResponse(success=True, message="Error cleared")

        def CloseProgram(self, request, context):
            self.closed = request.close_flag
            return pb.CloseProgramResponse(success=True, message="Program closed")

        def GetStandList(self, request, context):
            return pb.GetStandListResponse(
                success=True, message="ok",
                mode_list=["Start", "Zero", "Stand", "Walk", "Run", "Stop"],
                motion_list=["Greeting", "Chest Expansion", "Stretching", "Gentleman's Salute"],
                action_list=["Roll", "Pitch", "Yaw", "Base Height"],
                carrybox_list=["Standing to Pick up the Box", "Squatting to Pick up the Box",
                               "Put Down the Box"],
                balance_control="Dynamic Stand",
            )

        def GetRobotState(self, request, context):
            return pb.GetRobotStateResponse(
                success=True, message="ok",
                fsm_name=self.mode,
                current_motion=self.motion,
                current_action_list=["Roll", "Pitch"],
                mode_enable_list=["Walk", "Zero", "Stop"],
                motion_enable_list=["Greeting", "Chest Expansion", "Stretching",
                                    "Gentleman's Salute"],
                action_enable_list=["Roll", "Pitch", "Yaw", "Base Height"],
                carrybox_enable_list=["Standing to Pick up the Box",
                                      "Squatting to Pick up the Box"],
                balance_control_enable="Dynamic Stand",
                stand_pitch=self.pitch, stand_roll=self.roll,
                stand_yaw=self.yaw, stand_height=self.height,
                x_vel=self.x, y_vel=self.y, yaw_vel=self.yaw_speed,
                balance_control_state=self.dynamic,
                motion_files_enable=False,
            )

    return MockRobotControl()


def main() -> int:
    import grpc
    import adam_control_pb2 as pb
    import adam_control_pb2_grpc as pb_grpc

    from grpc_client import RobotControlClient

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    servicer = _make_mock_servicer(pb, pb_grpc)
    pb_grpc.add_RobotControlServicer_to_server(servicer, server)
    port = server.add_insecure_port("localhost:0")
    server.start()

    client = RobotControlClient(f"localhost:{port}", timeout_sec=2.0)
    failures = []

    def check(label, cond, detail):
        status = "ok" if cond else "FAIL"
        print(f"  [{status}] {label}: {detail}")
        if not cond:
            failures.append(label)

    print("== state queries ==")
    rs = client.get_robot_state()
    check("get_robot_state", rs["state"] == "ok" and rs["fsm_name"] == "Stand",
          f"state={rs['state']} mode={rs.get('fsm_name')}")

    sl = client.get_stand_list()
    check("get_stand_list", sl["state"] == "ok" and len(sl.get("mode_list", [])) == 6,
          f"state={sl['state']} modes={len(sl.get('mode_list', []))}")

    print("== control round-trips (server state is echoed back via GetRobotState) ==")
    r = client.set_mode("Walk")
    check("set_mode", r["state"] == "ok", f"state={r['state']} msg={r.get('message')}")

    r = client.set_stand_motion("Greeting")
    check("set_stand_motion", r["state"] == "ok", f"state={r['state']}")

    r = client.set_stand_carry_box("Put Down the Box")
    check("set_stand_carry_box", r["state"] == "ok", f"state={r['state']}")

    r = client.set_stand_action(0.05, -0.02, 0.0, -0.1)
    check("set_stand_action", r["state"] == "ok", f"state={r['state']}")

    r = client.set_stand_dynamic(True)
    check("set_stand_dynamic", r["state"] == "ok", f"state={r['state']}")

    r = client.set_speed(0.5, 0.0, 0.3, continuous=True)
    check("set_speed", r["state"] == "ok", f"state={r['state']}")

    r = client.auto_unigait_com(True)
    check("auto_unigait_com", r["state"] == "ok", f"state={r['state']}")

    r = client.set_error_clear(True)
    check("set_error_clear", r["state"] == "ok", f"state={r['state']}")

    r = client.close_program(True)
    check("close_program", r["state"] == "ok", f"state={r['state']}")

    print("== echo verification (values written → read back) ==")
    rs = client.get_robot_state()
    check("mode echoed", rs.get("fsm_name") == "Walk", f"mode={rs.get('fsm_name')}")
    check("velocity echoed", abs(rs.get("x_vel", 0.0) - 0.5) < 1e-5 and abs(rs.get("yaw_vel", 0.0) - 0.3) < 1e-5,
          f"vx={rs.get('x_vel')} vyaw={rs.get('yaw_vel')}")
    check("dynamic echoed", rs.get("balance_control_state") is True,
          f"dynamic={rs.get('balance_control_state')}")
    check("action echoed", abs(rs.get("stand_pitch", 0.0) - 0.05) < 1e-6,
          f"pitch={rs.get('stand_pitch')}")

    print("== connection probe ==")
    check("connected()", client.connected() is True, f"connected={client.connected()}")

    client.close()
    server.stop(0)

    print(f"\n{'ALL PASSED' if not failures else 'FAILED: ' + ', '.join(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    _ensure_stubs()
    sys.exit(main())
