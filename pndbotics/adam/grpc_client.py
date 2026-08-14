"""Official Adam RobotControl gRPC client wrapper."""

from __future__ import annotations

import sys
from pathlib import Path

import grpc

sys.path.insert(0, str(Path(__file__).parent / "proto"))
import adam_control_pb2 as pb2  # noqa: E402
import adam_control_pb2_grpc as pb2_grpc  # noqa: E402


class AdamGrpcClient:
    """Client for the official ``adam_control.RobotControl`` service."""

    def __init__(self, host: str = "localhost", port: int = 6666):
        self._addr = f"{host}:{port}"
        self._channel = None
        self._stub = None

    def connect(self) -> None:
        self._channel = grpc.insecure_channel(self._addr)
        self._stub = pb2_grpc.RobotControlStub(self._channel)

    def close(self) -> None:
        if self._channel is not None:
            self._channel.close()
        self._channel = None
        self._stub = None

    def _ensure_connected(self) -> None:
        if self._stub is None:
            self.connect()

    @staticmethod
    def _rpc_error(error: grpc.RpcError) -> dict:
        return {
            "success": False,
            "message": str(error.details() or error),
            "error_code": error.code().name,
        }

    @staticmethod
    def _result(response) -> dict:
        return {
            "success": bool(response.success),
            "message": str(response.message),
        }

    def set_mode(self, mode: str) -> dict:
        self._ensure_connected()
        try:
            response = self._stub.SetMode(
                pb2.SetModeRequest(mode=str(mode)), timeout=5
            )
            return self._result(response)
        except grpc.RpcError as error:
            return self._rpc_error(error)

    def set_speed(
        self,
        x_speed: float,
        y_speed: float,
        yaw_speed: float,
        continuous: bool = True,
    ) -> dict:
        self._ensure_connected()
        try:
            response = self._stub.SetSpeed(
                pb2.SetSpeedRequest(
                    x_speed=float(x_speed),
                    y_speed=float(y_speed),
                    yaw_speed=float(yaw_speed),
                    continous=bool(continuous),
                ),
                timeout=5,
            )
            return self._result(response)
        except grpc.RpcError as error:
            return self._rpc_error(error)

    def get_robot_state(self) -> dict:
        """Return every successful response field using the proto names."""
        self._ensure_connected()
        try:
            response = self._stub.GetRobotState(
                pb2.GetRobotStateRequest(get_state_flag=True), timeout=5
            )
            return {
                "success": bool(response.success),
                "message": str(response.message),
                "fsm_name": str(response.fsm_name),
                "current_motion": str(response.current_motion),
                "current_action_list": list(response.current_action_list),
                "mode_enable_list": list(response.mode_enable_list),
                "motion_enable_list": list(response.motion_enable_list),
                "action_enable_list": list(response.action_enable_list),
                "carrybox_enable_list": list(response.carrybox_enable_list),
                "balance_control_enable": str(response.balance_control_enable),
                "stand_pitch": float(response.stand_pitch),
                "stand_roll": float(response.stand_roll),
                "stand_yaw": float(response.stand_yaw),
                "stand_height": float(response.stand_height),
                "x_vel": float(response.x_vel),
                "y_vel": float(response.y_vel),
                "yaw_vel": float(response.yaw_vel),
                "balance_control_state": bool(response.balance_control_state),
                "motion_files_enable": bool(response.motion_files_enable),
            }
        except grpc.RpcError as error:
            return self._rpc_error(error)

    # Keep the remaining official service calls available without leaking the
    # old, guessed proto field names into the driver.
    def set_stand_motion(self, motion: str) -> dict:
        self._ensure_connected()
        try:
            response = self._stub.SetStandMotion(
                pb2.SetStandMotionRequest(motion=str(motion)), timeout=5
            )
            return self._result(response)
        except grpc.RpcError as error:
            return self._rpc_error(error)

    def set_carry_box(self, carry_box: str) -> dict:
        self._ensure_connected()
        try:
            response = self._stub.SetStandCarryBox(
                pb2.SetCarryBoxRequest(carry_box=str(carry_box)), timeout=5
            )
            return self._result(response)
        except grpc.RpcError as error:
            return self._rpc_error(error)

    def set_stand_action(
        self, stand_pitch: float, stand_roll: float,
        stand_yaw: float, stand_height: float,
    ) -> dict:
        self._ensure_connected()
        try:
            response = self._stub.SetStandAction(
                pb2.SetActionRequest(
                    stand_pitch=float(stand_pitch),
                    stand_roll=float(stand_roll),
                    stand_yaw=float(stand_yaw),
                    stand_height=float(stand_height),
                ),
                timeout=5,
            )
            return self._result(response)
        except grpc.RpcError as error:
            return self._rpc_error(error)

    def set_stand_dynamic(self, dynamic_stand: bool) -> dict:
        self._ensure_connected()
        try:
            response = self._stub.SetStandDynamic(
                pb2.SetDynamicStandRequest(dynamic_stand=bool(dynamic_stand)),
                timeout=5,
            )
            return self._result(response)
        except grpc.RpcError as error:
            return self._rpc_error(error)

    def get_stand_list(self) -> dict:
        self._ensure_connected()
        try:
            response = self._stub.GetStandList(
                pb2.GetStandListRequest(mode_list_req=True), timeout=5
            )
            return {
                "success": bool(response.success),
                "message": str(response.message),
                "mode_list": list(response.mode_list),
                "motion_list": list(response.motion_list),
                "action_list": list(response.action_list),
                "carrybox_list": list(response.carrybox_list),
                "balance_control": str(response.balance_control),
            }
        except grpc.RpcError as error:
            return self._rpc_error(error)

    def set_error_clear(self, error_clear_flag: bool = True) -> dict:
        self._ensure_connected()
        try:
            response = self._stub.SetErrorClear(
                pb2.SetErrorClearRequest(error_clear_flag=bool(error_clear_flag)),
                timeout=5,
            )
            return self._result(response)
        except grpc.RpcError as error:
            return self._rpc_error(error)
