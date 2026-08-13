#!/usr/bin/env python3
"""PNDbotics Adam Pro (31-DOF humanoid) device plugins.

The robot exposes two interfaces:

  - gRPC  ``adam_control.RobotControl`` (port 6666) — upper-level motion control.
  - DDS   ``rt/lowstate`` / ``rt/handstate`` / ``rt/handcmd`` — low-level state
          (31-DOF joints + IMU + battery) and dexterous-hand control.

Plugins:

  - StatePlugin   (gRPC): robot_state, stand_list, capabilities
  - DdsStatePlugin (DDS): joints (skeleton), imu, battery, remote, model (URDF)
  - HandPlugin     (DDS): hand (actuator), hand_state (sensor)
  - LocoPlugin    (gRPC): loco (mode / stand_motion / carry_box / stand_action /
                          stand_dynamic / speed / unigait_com / error_clear)

State is polled and returned via ``tools/call``.  Low-level state is additionally
republished to ROS2 topics by ``ros_bridge.py`` (loaded from ``main.py``) so the
dashboard can render live streams (skeleton / imu / battery / remote / hand).
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

from control import (
    ACTIONS,
    ADAM_PRO_JOINT_NAMES,
    CARRY_BOXES,
    DOF,
    MODES,
    MOTIONS,
    action_schema,
    array_property,
    joint_payload,
    sensor_tool,
)
from dds_client import DdsClient
from grpc_client import RobotControlClient

_RESOURCE_DIR = Path(__file__).parent / "resource"


def _now_ms() -> int:
    return int(time.time() * 1000)


# ── StatePlugin (gRPC) ────────────────────────────────────────────────────────

class StatePlugin:
    """High-level robot state via GetRobotState / GetStandList."""

    def __init__(self, plugin_config: dict, namespace: str, client: RobotControlClient):
        self._client = client
        self._ns = namespace
        self._poll_interval = float(plugin_config.get("poll_interval_sec", 0.5))
        self._running = False
        self._lock = threading.RLock()
        self._robot_state: dict = {"state": "no_data"}
        self._stand_list: dict = {"state": "no_data"}
        self._thread = None

    def get_tools(self) -> list[dict]:
        ns = self._ns
        return [
            sensor_tool("robot_state", "Adam 当前高层状态（gRPC GetRobotState）",
                        topic=f"/{ns}/state/robot", fmt="data/json"),
            sensor_tool("stand_list", "Adam 站立模式固定动作/姿态列表（gRPC GetStandList）"),
            {
                "name": "capabilities",
                "type": "resource",
                "multiInstance": False,
                "description": "PNDbotics Adam Pro 驱动能力、接口与限制说明",
                "inputSchema": {"type": "object", "properties": {}},
            },
        ]

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="adam-grpc-state-poll"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            name = args.get("_tool_name", "robot_state")
            if name == "robot_state":
                return {"state": "running", "topic_out": [{"topic": f"/{self._ns}/state/robot", "format": "data/json"}]}
            return {"state": "running", "tool": name}
        if action == "robot_state":
            with self._lock:
                return dict(self._robot_state)
        if action == "stand_list":
            with self._lock:
                return dict(self._stand_list)
        if action == "capabilities":
            return self._capabilities()
        return None

    def _poll_loop(self) -> None:
        while self._running:
            with self._lock:
                self._robot_state = self._client.get_robot_state()
            with self._lock:
                self._stand_list = self._client.get_stand_list()
            time.sleep(self._poll_interval)

    def _capabilities(self) -> dict:
        return {
            "robot": "PNDbotics Adam Pro",
            "dof": DOF,
            "interfaces": {
                "gRPC": "adam_control.RobotControl (port 6666) — 高层运控",
                "DDS": "rt/lowstate + rt/handstate + rt/handcmd — 底层状态 + 灵巧手",
            },
            "joints": list(ADAM_PRO_JOINT_NAMES),
            "modes": MODES,
            "stand_motions": MOTIONS,
            "stand_actions": ACTIONS,
            "carry_boxes": CARRY_BOXES,
            "control": [
                "set_mode", "stand_motion", "carry_box", "stand_action",
                "stand_dynamic", "speed", "unigait_com", "error_clear", "hand",
            ],
            "feedback": ["robot_state", "stand_list", "joints", "imu", "battery", "remote", "hand_state"],
            "limitations": [
                "低层关节力矩控制 rt/lowcmd 未暴露（高风险，默认关闭）",
                "实时流依赖 ROS2 重发布（ros_bridge.py）；无 ROS 环境时传感器仅 tools/call 拉取",
            ],
        }


# ── DdsStatePlugin (DDS) ──────────────────────────────────────────────────────

class DdsStatePlugin:
    """Low-level state via DDS rt/lowstate (joints / imu / battery / remote) + URDF."""

    def __init__(self, plugin_config: dict, namespace: str, dds: DdsClient):
        self._dds = dds
        self._ns = namespace

    def get_tools(self) -> list[dict]:
        ns = self._ns
        return [
            sensor_tool(
                "joints",
                f"Adam Pro 31DOF 关节位置/速度/力矩，骨架渲染。topic {ns}/state/joints",
                topic=f"/{ns}/state/joints", fmt="sensor/skeleton",
            ),
            sensor_tool("imu", "Adam IMU 四元数/角速度/加速度/姿态角", topic=f"/{ns}/state/imu", fmt="data/json"),
            sensor_tool("battery", "Adam 电池电压/电流/功率/累计功耗", topic=f"/{ns}/state/battery", fmt="data/json"),
            sensor_tool("remote", "Adam 无线遥控器手柄数据（19 通道）", topic=f"/{ns}/state/remote", fmt="data/json"),
            {
                "name": "model",
                "type": "resource",
                "multiInstance": False,
                "description": "Adam Pro URDF 骨架模型（用于 3D 骨架渲染）",
                "inputSchema": {"type": "object", "properties": {}},
            },
        ]

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return self._info(args.get("_tool_name", "joints"))
        if action == "joints":
            data = self._dds.joints()
            if data is None:
                return {"state": "no_data", "error": self._dds.status().get("error")}
            return joint_payload(data["position"], data["velocity"], data["torque"])
        if action == "imu":
            data = self._dds.imu()
            return data if data is not None else {"state": "no_data", "error": self._dds.status().get("error")}
        if action == "battery":
            data = self._dds.battery()
            return data if data is not None else {"state": "no_data", "error": self._dds.status().get("error")}
        if action == "remote":
            data = self._dds.remote()
            return data if data is not None else {"state": "no_data", "error": self._dds.status().get("error")}
        if action == "model":
            urdf = _RESOURCE_DIR / "adam_sp_pro.urdf"
            if urdf.exists():
                return {"urdf": urdf.read_text(encoding="utf-8")}
            return {"error": "URDF model file not found"}
        return None

    def _info(self, name: str) -> dict:
        ns = self._ns
        topics = {
            "joints": ("sensor/skeleton", f"/{ns}/state/joints"),
            "imu": ("data/json", f"/{ns}/state/imu"),
            "battery": ("data/json", f"/{ns}/state/battery"),
            "remote": ("data/json", f"/{ns}/state/remote"),
        }
        fmt, topic = topics.get(name, ("data/json", f"/{ns}/state/{name}"))
        return {"state": "running", "topic_out": [{"topic": topic, "format": fmt}]}


# ── HandPlugin (DDS) ──────────────────────────────────────────────────────────

class HandPlugin:
    """Dexterous-hand control via DDS rt/handcmd + state via rt/handstate."""

    def __init__(self, plugin_config: dict, namespace: str, dds: DdsClient):
        self._dds = dds
        self._ns = namespace

    def get_tools(self) -> list[dict]:
        return [
            {
                "name": "hand",
                "type": "actuator",
                "multiInstance": False,
                "description": "Adam 灵巧手控制（12 维手指位置 0-1000，1000 伸直 / 0 弯曲）",
                "inputSchema": action_schema(
                    {
                        "set": (["positions"], "设置 12 维手指位置"),
                        "open": ([], "手指完全伸直（全 1000）"),
                        "close": ([], "手指完全弯曲（全 0）"),
                        "status": ([], "查询当前手指位置"),
                    },
                    {
                        "positions": array_property("12 维手指位置，范围 0-1000", item_type="integer"),
                    },
                    "灵巧手动作",
                ),
            },
            sensor_tool("hand_state", "Adam 灵巧手 12 维手指实际位置", topic=f"/{self._ns}/state/hand", fmt="data/json"),
        ]

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": f"/{self._ns}/state/hand", "format": "data/json"}]}
        if action == "hand_state" or action == "status":
            pos = self._dds.hand_state()
            if pos is None:
                return {"state": "no_data", "error": self._dds.status().get("error")}
            return {"state": "ok", "position": pos}
        if action == "set":
            positions = args.get("positions")
            if not isinstance(positions, (list, tuple)) or len(positions) != 12:
                return {"error": "positions must be 12 integers in [0, 1000]"}
            return self._dds.set_hand([int(p) for p in positions])
        if action == "open":
            return self._dds.set_hand([1000] * 12)
        if action == "close":
            return self._dds.set_hand([0] * 12)
        return None


# ── LocoPlugin (gRPC) ─────────────────────────────────────────────────────────

class LocoPlugin:
    """High-level motion control actuator over the RobotControl service."""

    def __init__(self, plugin_config: dict, namespace: str, client: RobotControlClient):
        self._client = client
        self._ns = namespace

    def get_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": "PNDbotics Adam 高层运动控制：模式切换、站立动作、搬箱子、姿态、平衡、速度与错误复位",
            "inputSchema": action_schema(
                {
                    "mode": (["mode"], "切换机器人模式（Start/Zero/Stand/Walk/Run/Stop）"),
                    "stand_motion": (["motion"], "站立模式下执行预定义动捕动作"),
                    "carry_box": (["carry_box"], "站立模式下执行搬箱子动作"),
                    "stand_action": (["stand_pitch", "stand_roll", "stand_yaw", "stand_height"], "站立模式下实时调节姿态与蹲起高度"),
                    "stand_dynamic": (["enable"], "开关 Dynamic Stand 平衡"),
                    "speed": (["x", "y", "yaw", "continuous"], "设置移动速度（Walk/Run 模式）"),
                    "unigait_com": (["enable"], "开关原地步态 COM 偏置平衡"),
                    "error_clear": (["flag"], "清除错误状态，可不关电继续运行"),
                    "close_program": (["flag"], "关闭演示程序"),
                    "status": ([], "查询当前模式、动作与可执行列表"),
                },
                {
                    "mode": {"type": "string", "enum": MODES},
                    "motion": {"type": "string", "enum": MOTIONS},
                    "carry_box": {"type": "string", "enum": CARRY_BOXES},
                    "stand_pitch": {"type": "number", "description": "站立俯仰角 rad"},
                    "stand_roll": {"type": "number", "description": "站立横滚角 rad"},
                    "stand_yaw": {"type": "number", "description": "站立偏航角 rad"},
                    "stand_height": {"type": "number", "description": "站立高度偏移 m（负值为下蹲）"},
                    "enable": {"type": "boolean"},
                    "x": {"type": "number", "description": "前向速度 m/s"},
                    "y": {"type": "number", "description": "侧向速度 m/s"},
                    "yaw": {"type": "number", "description": "偏航角速度 rad/s"},
                    "continuous": {"type": "boolean", "description": "持续发送速度（false 则持续 5s）"},
                    "flag": {"type": "boolean"},
                },
                "高层运控动作",
            ),
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "status"):
            return self._status()

        client = self._client
        if action == "mode":
            result = client.set_mode(str(args.get("mode", "")))
        elif action == "stand_motion":
            result = client.set_stand_motion(str(args.get("motion", "")))
        elif action == "carry_box":
            result = client.set_stand_carry_box(str(args.get("carry_box", "")))
        elif action == "stand_action":
            result = client.set_stand_action(
                float(args.get("stand_pitch", 0.0)),
                float(args.get("stand_roll", 0.0)),
                float(args.get("stand_yaw", 0.0)),
                float(args.get("stand_height", 0.0)),
            )
        elif action == "stand_dynamic":
            result = client.set_stand_dynamic(bool(args.get("enable", False)))
        elif action == "speed":
            result = client.set_speed(
                float(args.get("x", 0.0)),
                float(args.get("y", 0.0)),
                float(args.get("yaw", 0.0)),
                bool(args.get("continuous", False)),
            )
        elif action == "close_program":
            result = client.close_program(bool(args.get("flag", True)))
        elif action == "unigait_com":
            result = client.auto_unigait_com(bool(args.get("enable", False)))
        elif action == "error_clear":
            result = client.set_error_clear(bool(args.get("flag", True)))
        else:
            return {"error": f"unknown loco action: {action}"}

        result["action"] = action
        return result

    def _status(self) -> dict:
        state = self._client.get_robot_state()
        return {
            "state": "ready",
            "fsm_name": state.get("fsm_name", ""),
            "current_motion": state.get("current_motion", ""),
            "current_action_list": state.get("current_action_list", []),
            "mode_enable_list": state.get("mode_enable_list", []),
            "motion_enable_list": state.get("motion_enable_list", []),
            "action_enable_list": state.get("action_enable_list", []),
            "carrybox_enable_list": state.get("carrybox_enable_list", []),
            "balance_control_state": state.get("balance_control_state", False),
        }
