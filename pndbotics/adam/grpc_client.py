"""Defensive gRPC client for the PNDbotics Adam RobotControl service.

This module is the gRPC analogue of the native-SDK layer used by the other
drivers (``highcontrol_py`` for bumi, the ROS2 bridge for t800).  It owns the
channel lifecycle, timeouts and error normalization so the device plugins in
``device.py`` stay transport-agnostic.

Every method returns a plain ``dict`` with a ``state`` key (``ok``, ``rejected``
or ``error``) rather than raising, so a missing/unreachable robot degrades to a
readable tool result instead of a traceback.
"""

from __future__ import annotations

import threading


def _now_ms() -> int:
    import time

    return int(time.time() * 1000)


class RobotControlClient:
    """Thin wrapper over ``adam_control.RobotControlStub`` (port 6666)."""

    def __init__(self, address: str = "localhost:6666", timeout_sec: float = 2.0):
        self._address = address
        self._timeout = float(timeout_sec)
        self._channel = None
        self._stub = None
        self._lock = threading.Lock()

    # ── connection lifecycle ────────────────────────────────────────────────

    def _ensure_stub(self):
        """Lazily build the gRPC channel + stub (imports generated modules on first use)."""
        with self._lock:
            if self._stub is not None:
                return self._stub
            import grpc
            import adam_control_pb2_grpc

            self._channel = grpc.insecure_channel(self._address)
            self._stub = adam_control_pb2_grpc.RobotControlStub(self._channel)
            return self._stub

    def connected(self) -> bool:
        try:
            return self.get_robot_state().get("state") != "error"
        except Exception:
            return False

    def close(self) -> None:
        with self._lock:
            if self._channel is not None:
                self._channel.close()
                self._channel = None
                self._stub = None

    @property
    def address(self) -> str:
        return self._address

    # ── primitive RPC wrappers ───────────────────────────────────────────────

    def _simple(self, build_request, rpc_method):
        """Shared path for the eight Set*/Auto RPCs: build a request, send, normalize."""
        import adam_control_pb2

        try:
            stub = self._ensure_stub()
            request = build_request(adam_control_pb2)
            response = rpc_method(stub, request)
            return {
                "state": "ok" if response.success else "rejected",
                "success": bool(response.success),
                "message": getattr(response, "message", ""),
            }
        except Exception as exc:  # noqa: BLE001 — normalize any transport/proto error
            return {"state": "error", "error": str(exc)}

    def set_mode(self, mode: str) -> dict:
        def _build(pb):
            return pb.SetModeRequest(mode=str(mode))

        return self._simple(_build, lambda stub, req: stub.SetMode(req, timeout=self._timeout))

    def set_stand_motion(self, motion: str) -> dict:
        def _build(pb):
            return pb.SetStandMotionRequest(motion=str(motion))

        return self._simple(_build, lambda stub, req: stub.SetStandMotion(req, timeout=self._timeout))

    def set_stand_carry_box(self, carry_box: str) -> dict:
        def _build(pb):
            return pb.SetCarryBoxRequest(carry_box=str(carry_box))

        return self._simple(_build, lambda stub, req: stub.SetStandCarryBox(req, timeout=self._timeout))

    def set_stand_action(self, pitch: float, roll: float, yaw: float, height: float) -> dict:
        def _build(pb):
            return pb.SetActionRequest(
                stand_pitch=float(pitch),
                stand_roll=float(roll),
                stand_yaw=float(yaw),
                stand_height=float(height),
            )

        return self._simple(_build, lambda stub, req: stub.SetStandAction(req, timeout=self._timeout))

    def set_stand_dynamic(self, enable: bool) -> dict:
        def _build(pb):
            return pb.SetDynamicStandRequest(dynamic_stand=bool(enable))

        return self._simple(_build, lambda stub, req: stub.SetStandDynamic(req, timeout=self._timeout))

    def set_speed(self, x: float, y: float, yaw: float, continuous: bool = False) -> dict:
        def _build(pb):
            return pb.SetSpeedRequest(
                x_speed=float(x), y_speed=float(y), yaw_speed=float(yaw),
                continous=bool(continuous),
            )

        return self._simple(_build, lambda stub, req: stub.SetSpeed(req, timeout=self._timeout))

    def auto_unigait_com(self, enable: bool) -> dict:
        def _build(pb):
            return pb.SetUnigaitCOMRequest(unigait_mode_com_x=bool(enable))

        return self._simple(_build, lambda stub, req: stub.AutoUnigaitCOM(req, timeout=self._timeout))

    def set_error_clear(self, flag: bool) -> dict:
        def _build(pb):
            return pb.SetErrorClearRequest(error_clear_flag=bool(flag))

        return self._simple(_build, lambda stub, req: stub.SetErrorClear(req, timeout=self._timeout))

    def close_program(self, flag: bool = True) -> dict:
        def _build(pb):
            return pb.CloseProgramRequest(close_flag=bool(flag))

        return self._simple(_build, lambda stub, req: stub.CloseProgram(req, timeout=self._timeout))

    # ── state queries ────────────────────────────────────────────────────────

    def get_stand_list(self) -> dict:
        import adam_control_pb2

        try:
            stub = self._ensure_stub()
            request = adam_control_pb2.GetStandListRequest(mode_list_req=True)
            response = stub.GetStandList(request, timeout=self._timeout)
            return {
                "state": "ok" if response.success else "rejected",
                "success": bool(response.success),
                "message": getattr(response, "message", ""),
                "mode_list": list(getattr(response, "mode_list", [])),
                "motion_list": list(getattr(response, "motion_list", [])),
                "action_list": list(getattr(response, "action_list", [])),
                "carrybox_list": list(getattr(response, "carrybox_list", [])),
                "balance_control": getattr(response, "balance_control", ""),
            }
        except Exception as exc:  # noqa: BLE001
            return {"state": "error", "error": str(exc)}

    def get_robot_state(self) -> dict:
        import adam_control_pb2

        try:
            stub = self._ensure_stub()
            request = adam_control_pb2.GetRobotStateRequest(get_state_flag=True)
            response = stub.GetRobotState(request, timeout=self._timeout)
            return {
                "state": "ok" if response.success else "rejected",
                "success": bool(response.success),
                "message": getattr(response, "message", ""),
                "fsm_name": getattr(response, "fsm_name", ""),
                "current_motion": getattr(response, "current_motion", ""),
                "current_action_list": list(getattr(response, "current_action_list", [])),
                "mode_enable_list": list(getattr(response, "mode_enable_list", [])),
                "motion_enable_list": list(getattr(response, "motion_enable_list", [])),
                "action_enable_list": list(getattr(response, "action_enable_list", [])),
                "carrybox_enable_list": list(getattr(response, "carrybox_enable_list", [])),
                "balance_control_enable": getattr(response, "balance_control_enable", ""),
                "stand_pitch": float(getattr(response, "stand_pitch", 0.0)),
                "stand_roll": float(getattr(response, "stand_roll", 0.0)),
                "stand_yaw": float(getattr(response, "stand_yaw", 0.0)),
                "stand_height": float(getattr(response, "stand_height", 0.0)),
                "x_vel": float(getattr(response, "x_vel", 0.0)),
                "y_vel": float(getattr(response, "y_vel", 0.0)),
                "yaw_vel": float(getattr(response, "yaw_vel", 0.0)),
                "balance_control_state": bool(getattr(response, "balance_control_state", False)),
                "motion_files_enable": bool(getattr(response, "motion_files_enable", False)),
                "timestamp_ms": _now_ms(),
            }
        except Exception as exc:  # noqa: BLE001
            return {"state": "error", "error": str(exc), "timestamp_ms": _now_ms()}
