"""PNDbotics Adam device plugins.

The public cards deliberately map to the interfaces documented by PNDbotics:

* ``robot_state`` / ``switch_mode`` / ``loco`` use the official gRPC client;
* ``hand_state`` / ``hand`` use ``rt/handstate`` and ``rt/handcmd``;
* ``arm`` uses the documented periodic ``rt/lowcmd`` body command stream.

The old implementation published arm commands to a generic ROS2
``joint_states`` topic. Adam does not subscribe to that topic, so those
commands could never reach a motor controller.
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

try:
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    HAS_ROS2 = True
except ImportError:  # Allows static/unit checks on a non-ROS development host.
    HAS_ROS2 = False

    class Node:  # type: ignore[no-redef]
        pass

    class String:  # type: ignore[no-redef]
        pass

try:
    from pndbotics_sdk_py.core.channel import ChannelPublisher, ChannelSubscriber
    from pndbotics_sdk_py.idl.default import (
        pnd_adam_msg_dds__HandCmd_,
        pnd_adam_msg_dds__LowCmd_,
    )

    HAS_PND_SDK = True
except ImportError:
    HAS_PND_SDK = False
    ChannelPublisher = None  # type: ignore[assignment,misc]
    ChannelSubscriber = None  # type: ignore[assignment,misc]
    pnd_adam_msg_dds__HandCmd_ = None  # type: ignore[assignment]
    pnd_adam_msg_dds__LowCmd_ = None  # type: ignore[assignment]


# ---------------------------------------------------------------------------
# Adam joint order from the official body_joint_motor page.
# ---------------------------------------------------------------------------

ADAM_LITE_JOINTS = [
    "hipPitch_Left", "hipRoll_Left", "hipYaw_Left",
    "kneePitch_Left", "anklePitch_Left", "ankleRoll_Left",
    "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right",
    "waistRoll", "waistPitch", "waistYaw",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
]

ADAM_SP_JOINTS = [
    *ADAM_LITE_JOINTS[:15],
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "wristYaw_Left", "wristPitch_Left", "wristRoll_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
    "wristYaw_Right", "wristPitch_Right", "wristRoll_Right",
]

ADAM_PRO_JOINTS = [
    *ADAM_LITE_JOINTS[:15], "neckYaw", "neckPitch",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "wristYaw_Left", "wristPitch_Left", "wristRoll_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
    "wristYaw_Right", "wristPitch_Right", "wristRoll_Right",
]

VARIANT_JOINTS = {
    "lite": ADAM_LITE_JOINTS,
    "sp": ADAM_SP_JOINTS,
    "pro": ADAM_PRO_JOINTS,
}
VARIANT_DOF = {name: len(joints) for name, joints in VARIANT_JOINTS.items()}

ARM_JOINT_NAMES = {
    "waistRoll", "waistPitch", "waistYaw", "neckYaw", "neckPitch",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "wristYaw_Left", "wristPitch_Left", "wristRoll_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
    "wristYaw_Right", "wristPitch_Right", "wristRoll_Right",
}


def _gain_profile(variant: str):
    """Conservative position gains matching the SDK's Adam Pro example."""
    joints = VARIANT_JOINTS[variant]
    kp = []
    kd = []
    for joint in joints:
        if joint.startswith("hip") or joint.startswith("knee"):
            kp.append(305.0 if joint.startswith("hipPitch") or joint.startswith("knee") else 405.0)
            kd.append(6.1)
        elif joint.startswith("ankle"):
            kp.append(30.0)
            kd.append(2.25)
        elif joint.startswith("waist"):
            kp.append(205.0 if joint == "waistRoll" else 405.0)
            kd.append(4.1 if joint == "waistRoll" else 6.1)
        elif joint.startswith("neck"):
            kp.append(40.0)
            kd.append(1.0)
        elif joint.startswith("shoulderPitch"):
            kp.append(18.0)
            kd.append(0.9)
        else:
            kp.append(9.0)
            kd.append(0.9)
    return kp, kd


def _best_effort_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


# ---------------------------------------------------------------------------
# DDS state publishing
# ---------------------------------------------------------------------------

class _StatePublisherNode(Node):
    def __init__(self, namespace: str, variant: str, publish_rate_hz: float):
        super().__init__("adam_state_publisher")
        self._joints = VARIANT_JOINTS[variant]
        qos = _best_effort_qos()
        self.topic_skeleton = f"/{namespace}/state/joints"
        self.topic_imu = f"/{namespace}/state/imu"
        self.topic_battery = f"/{namespace}/state/battery"
        self.topic_motor_status = f"/{namespace}/state/motor_status"
        self.topic_remote = f"/{namespace}/state/remote"
        self._skeleton_pub = self.create_publisher(String, self.topic_skeleton, qos)
        self._imu_pub = self.create_publisher(String, self.topic_imu, qos)
        self._battery_pub = self.create_publisher(String, self.topic_battery, qos)
        self._motor_status_pub = self.create_publisher(String, self.topic_motor_status, qos)
        self._remote_pub = self.create_publisher(String, self.topic_remote, qos)
        self._state = None
        self._lock = threading.Lock()
        self.create_timer(1.0 / max(1.0, publish_rate_hz), self._publish)

    def update_state(self, state):
        with self._lock:
            self._state = state

    def _publish(self):
        with self._lock:
            state = self._state
        if state is None:
            return

        motors = list(getattr(state, "motor_state", []))
        skeleton = {
            "joints": [
                {"idx": index, "name": name, "q": float(motors[index].q)}
                for index, name in enumerate(self._joints)
                if index < len(motors)
            ]
        }
        imu = getattr(state, "imu_state", None)
        imu_data = {
            "quaternion": list(getattr(imu, "quaternion", [])),
            "gyroscope": list(getattr(imu, "gyroscope", [])),
            "accelerometer": list(getattr(imu, "accelerometer", [])),
            "ypr": list(getattr(imu, "ypr", [])),
            "temperature": int(getattr(imu, "temperature", 0)),
        }
        battery = getattr(state, "battery_data", None)
        battery_data = {
            "timestamp_ms": int(getattr(battery, "timestamp_ms", 0)),
            "voltage": float(getattr(battery, "voltage", 0.0)),
            "current": float(getattr(battery, "current", 0.0)),
            "power": float(getattr(battery, "power", 0.0)),
            "wh_accumulated": float(getattr(battery, "wh_accumulated", 0.0)),
            "status": str(getattr(battery, "status", "")),
        }

        # Motor status (per-joint mode & state)
        motors = list(getattr(state, "motor_state", []))
        motor_list = []
        enabled = 0
        errors = 0
        for idx, name in enumerate(self._joints):
            if idx < len(motors):
                ms = motors[idx]
                mode = int(getattr(ms, "mode", 0))
                state_val = int(getattr(ms, "state", 0))
                motor_list.append({
                    "idx": idx,
                    "name": name,
                    "mode": mode,
                    "state": state_val,
                })
                if mode != 0:
                    enabled += 1
                if state_val != 0:
                    errors += 1
        motor_status_data = {
            "motors": motor_list,
            "summary": {
                "total": len(motor_list),
                "enabled": enabled,
                "error_count": errors,
            },
        }

        # Wireless remote (raw 19-channel float array)
        wireless_remote = list(getattr(state, "wireless_remote", []))
        remote_data = {
            "channels": [float(v) for v in wireless_remote],
        }

        for publisher, payload in (
            (self._skeleton_pub, skeleton),
            (self._imu_pub, imu_data),
            (self._battery_pub, battery_data),
            (self._motor_status_pub, motor_status_data),
            (self._remote_pub, remote_data),
        ):
            message = String()
            message.data = json.dumps(payload)
            publisher.publish(message)


class StatePlugin:
    PREFIX = "state"

    def __init__(self, plugin_config, namespace, executor, variant, lowstate_sub=None, **_kwargs):
        self._running = False
        self._stop_event = threading.Event()
        self._thread = None
        self._lowstate_sub = lowstate_sub
        self._node = _StatePublisherNode(
            namespace, variant, float(plugin_config.get("publish_rate_hz", 50))
        )
        executor.add_node(self._node)

    def get_tools(self):
        return [
            {
                "name": "joints", "type": "sensor", "multiInstance": False,
                "description": "Adam body joint state from DDS rt/lowstate",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._node.topic_skeleton, "format": "sensor/skeleton"}],
            },
            {
                "name": "imu", "type": "sensor", "multiInstance": False,
                "description": "Adam IMU from DDS rt/lowstate",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._node.topic_imu, "format": "data/json"}],
            },
            {
                "name": "battery", "type": "sensor", "multiInstance": False,
                "description": "Adam battery state from DDS rt/lowstate",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._node.topic_battery, "format": "data/json"}],
            },
            {
                "name": "motor_status", "type": "sensor", "multiInstance": False,
                "description": "Adam motor status — per-joint mode (enabled/disabled) and state/error code",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._node.topic_motor_status, "format": "data/json"}],
            },
            {
                "name": "remote", "type": "sensor", "multiInstance": False,
                "description": "Adam wireless remote — raw 19-channel controller input",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._node.topic_remote, "format": "data/json"}],
            },
        ]

    def _poll(self):
        while not self._stop_event.is_set():
            if self._lowstate_sub is None:
                self._stop_event.wait(1.0)
                continue
            try:
                message = self._lowstate_sub.Read(timeout=1)
                if message is not None:
                    self._node.update_state(message)
            except Exception:
                # The SDK can raise while a reader is being closed during shutdown.
                if not self._stop_event.is_set():
                    time.sleep(0.05)

    def start(self):
        self._running = True
        if self._thread is None and self._lowstate_sub is not None:
            self._thread = threading.Thread(target=self._poll, daemon=True, name="adam_lowstate")
            self._thread.start()

    def stop(self):
        self._running = False
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def dispatch(self, action, args):
        topics = {
            "joints": (self._node.topic_skeleton, "sensor/skeleton"),
            "imu": (self._node.topic_imu, "data/json"),
            "battery": (self._node.topic_battery, "data/json"),
            "motor_status": (self._node.topic_motor_status, "data/json"),
            "remote": (self._node.topic_remote, "data/json"),
        }
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        topic, fmt = topics.get(args.get("_tool_name", "joints"), topics["joints"])
        return {"state": "running" if self._running else "idle", "topic_out": [{"topic": topic, "format": fmt}]}


# ---------------------------------------------------------------------------
# gRPC cards
# ---------------------------------------------------------------------------

class RobotStatePlugin:
    PREFIX = "robot_state"

    def __init__(self, _plugin_config, _namespace, _executor, grpc_client, **_kwargs):
        self._grpc = grpc_client

    def get_tool(self):
        return {
            "name": "robot_state",
            "type": "sensor",
            "multiInstance": False,
            "description": "Adam high-level mode/state from RobotControl.GetRobotState; response fields follow the official proto",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, _args):
        if action in ("robot_state", "get_state"):
            return self._grpc.get_robot_state()
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        return None


class SwitchModePlugin:
    PREFIX = "switch_mode"
    OFFICIAL_MODES = ["Start", "Zero", "Stand", "Walk", "Run", "Stop"]

    def __init__(self, _plugin_config, _namespace, _executor, grpc_client, **_kwargs):
        self._grpc = grpc_client

    def get_tool(self):
        return {
            "name": "switch_mode",
            "type": "actuator",
            "multiInstance": False,
            "description": "Switch Adam's official high-level mode through RobotControl.SetMode",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "mode": {
                        "type": "string",
                        "enum": self.OFFICIAL_MODES,
                        "description": "Mode must be one of the modes currently allowed by robot_state.mode_enable_list",
                    }
                },
                "required": ["mode"],
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action in ("switch_mode", "set_mode"):
            mode = args.get("mode")
            if mode not in self.OFFICIAL_MODES:
                return {"success": False, "message": f"Unsupported Adam mode: {mode!r}"}
            return self._grpc.set_mode(mode)
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        return None


class LocoPlugin:
    PREFIX = "loco"

    def __init__(self, _plugin_config, _namespace, _executor, grpc_client, **_kwargs):
        self._grpc = grpc_client

    def get_tool(self):
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": "Adam locomotion through RobotControl.SetSpeed; switch to Walk or Run first",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move", "stop_move"]},
                    "vx": {"type": "number", "description": "Linear x speed (m/s)"},
                    "vy": {"type": "number", "description": "Linear y speed (m/s)"},
                    "vyaw": {"type": "number", "description": "Yaw speed (rad/s)"},
                    "continuous": {"type": "boolean", "default": True, "description": "Keep sending the speed command"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "continuous"], "description": "Set linear and turning speed in Walk/Run mode"},
                    "stop_move": {"params": [], "description": "Send zero speed through SetSpeed"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "move":
            return self._grpc.set_speed(
                args.get("vx", 0.0), args.get("vy", 0.0), args.get("vyaw", 0.0),
                args.get("continuous", True),
            )
        if action == "stop_move":
            return self._grpc.set_speed(0.0, 0.0, 0.0, True)
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return self._grpc.set_speed(0.0, 0.0, 0.0, True)
        return None


# ---------------------------------------------------------------------------
# DDS hand state and hand command cards
# ---------------------------------------------------------------------------

def _hand_payload(message):
    positions = [int(value) for value in list(getattr(message, "position", []))[:12]]
    positions += [0] * (12 - len(positions))
    return {
        "position": positions,
        "reserve": int(getattr(message, "reserve", 0)),
        "left": positions[:6],
        "right": positions[6:12],
    }


class _HandStatePublisherNode(Node):
    def __init__(self, namespace: str, publish_rate_hz: float):
        super().__init__("adam_hand_state_publisher")
        self.topic = f"/{namespace}/state/hand"
        self._publisher = self.create_publisher(String, self.topic, _best_effort_qos())
        self._state = None
        self._lock = threading.Lock()
        self.create_timer(1.0 / max(1.0, publish_rate_hz), self._publish)

    def update_state(self, state):
        with self._lock:
            self._state = state

    def snapshot(self):
        with self._lock:
            state = self._state
        return None if state is None else _hand_payload(state)

    def _publish(self):
        payload = self.snapshot()
        if payload is None:
            return
        message = String()
        message.data = json.dumps(payload)
        self._publisher.publish(message)


class HandStatePlugin:
    PREFIX = "hand_state"

    def __init__(self, plugin_config, namespace, executor, handstate_sub=None, **_kwargs):
        self._running = False
        self._stop_event = threading.Event()
        self._thread = None
        self._handstate_sub = handstate_sub
        self._node = _HandStatePublisherNode(
            namespace, float(plugin_config.get("publish_rate_hz", 50))
        )
        executor.add_node(self._node)

    def get_tool(self):
        return {
            "name": "hand_state",
            "type": "sensor",
            "multiInstance": False,
            "description": "Adam DDS rt/handstate — 12 finger positions: left[0:6], right[6:12]",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._node.topic, "format": "data/json"}],
        }

    def _poll(self):
        while not self._stop_event.is_set():
            if self._handstate_sub is None:
                self._stop_event.wait(1.0)
                continue
            try:
                message = self._handstate_sub.Read(timeout=1)
                if message is not None:
                    self._node.update_state(message)
            except Exception:
                if not self._stop_event.is_set():
                    time.sleep(0.05)

    def start(self):
        self._running = True
        if self._thread is None and self._handstate_sub is not None:
            self._thread = threading.Thread(target=self._poll, daemon=True, name="adam_handstate")
            self._thread.start()

    def stop(self):
        self._running = False
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def dispatch(self, action, _args):
        if action in ("hand_state", "read"):
            payload = self._node.snapshot()
            if payload is None:
                return {"state": "unknown", "message": "No rt/handstate sample received yet"}
            return payload
        if action == "info":
            return {"state": "running" if self._running else "idle", "topic_out": [{"topic": self._node.topic, "format": "data/json"}]}
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        return None


class HandPlugin:
    PREFIX = "hand"

    def __init__(self, plugin_config, _namespace, _executor, hand_pub=None, **_kwargs):
        self._hand_type = str(plugin_config.get("hand_type", "inspire")).lower()
        self._max_value = 1000 if self._hand_type == "pnd" else 1800
        self._rate_hz = float(plugin_config.get("control_rate_hz", 400))
        self._hand_pub = hand_pub
        self._target = [self._max_value] * 12
        self._active = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._command = pnd_adam_msg_dds__HandCmd_() if HAS_PND_SDK else None

    def get_tool(self):
        return {
            "name": "hand",
            "type": "actuator",
            "multiInstance": False,
            "description": f"Adam hand control through DDS rt/handcmd (0=closed, {self._max_value}=open)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["open", "close", "set_fingers"]},
                    "left": {"type": "array", "items": {"type": "integer"}, "minItems": 6, "maxItems": 6, "description": "[pinky, ring, middle, index, thumb, lateral]"},
                    "right": {"type": "array", "items": {"type": "integer"}, "minItems": 6, "maxItems": 6, "description": "[pinky, ring, middle, index, thumb, lateral]"},
                },
                "required": ["action"],
                "x-action-params": {
                    "open": {"params": [], "description": "Open all 12 fingers"},
                    "close": {"params": [], "description": "Close all 12 fingers"},
                    "set_fingers": {"params": ["left", "right"], "description": "Set all 12 finger positions"},
                },
            },
        }

    def _write_once(self):
        if self._hand_pub is None or self._command is None:
            return False
        with self._lock:
            values = list(self._target)
        for index, value in enumerate(values):
            self._command.position[index] = int(value)
        self._command.reserve = 0
        try:
            self._hand_pub.Write(self._command)
            return True
        except Exception:
            return False

    def _publish_loop(self):
        interval = 1.0 / max(1.0, self._rate_hz)
        while not self._stop_event.is_set():
            started = time.monotonic()
            with self._lock:
                active = self._active
            if active:
                self._write_once()
            self._stop_event.wait(max(0.0, interval - (time.monotonic() - started)))

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._publish_loop, daemon=True, name="adam_handcmd")
            self._thread.start()

    def stop(self):
        with self._lock:
            self._active = False
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def _set_target(self, values):
        if self._hand_pub is None or self._command is None:
            return {"success": False, "message": "DDS rt/handcmd publisher is unavailable"}
        try:
            normalized = [
                max(0, min(self._max_value, int(round(float(value)))))
                for value in values
            ]
        except (TypeError, ValueError):
            return {"success": False, "message": "Finger positions must be numeric"}
        if len(normalized) != 12:
            return {"success": False, "message": "Exactly 12 finger positions are required"}
        with self._lock:
            self._target = normalized
            self._active = True
        self._write_once()
        return {"success": True, "position": normalized, "left": normalized[:6], "right": normalized[6:]}

    def dispatch(self, action, args):
        if action == "open":
            result = self._set_target([self._max_value] * 12)
            result["message"] = "All fingers opened" if result.get("success") else result.get("message")
            return result
        if action == "close":
            result = self._set_target([0] * 12)
            result["message"] = "All fingers closed" if result.get("success") else result.get("message")
            return result
        if action == "set_fingers":
            left = args.get("left")
            right = args.get("right")
            if not isinstance(left, list) or not isinstance(right, list):
                return {"success": False, "message": "left and right must each be six-value arrays"}
            return self._set_target(left[:6] + right[:6])
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            with self._lock:
                active = self._active
            return {"state": "active" if active else "idle", "hand_type": self._hand_type}
        return None


# ---------------------------------------------------------------------------
# Direct body command controller for arm card
# ---------------------------------------------------------------------------

class _ArmDdsController:
    def __init__(self, variant: str, rate_hz: float, lowstate_sub=None, lowcmd_pub=None):
        self._variant = variant
        self._joints = VARIANT_JOINTS[variant]
        self._dof = VARIANT_DOF[variant]
        self._joint_index = {name: index for index, name in enumerate(self._joints)}
        self._control_indices = {
            name: index for name, index in self._joint_index.items()
            if name in ARM_JOINT_NAMES
        }
        self._kp, self._kd = _gain_profile(variant)
        self._rate_hz = float(rate_hz)
        self._lowstate_sub = lowstate_sub
        self._lowcmd_pub = lowcmd_pub
        self._latest_state = None
        self._target = {}
        self._active = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._state_thread = None
        self._command_thread = None

    def start(self):
        if self._state_thread is None and self._lowstate_sub is not None:
            self._state_thread = threading.Thread(target=self._poll_state, daemon=True, name="adam_arm_state")
            self._state_thread.start()
        if self._command_thread is None and self._lowcmd_pub is not None:
            self._command_thread = threading.Thread(target=self._publish_loop, daemon=True, name="adam_lowcmd")
            self._command_thread.start()

    def stop(self):
        with self._lock:
            self._active = False
        self._stop_event.set()
        for thread in (self._state_thread, self._command_thread):
            if thread is not None:
                thread.join(timeout=1.5)

    def _poll_state(self):
        while not self._stop_event.is_set():
            try:
                state = self._lowstate_sub.Read(timeout=1)
                if state is not None:
                    with self._lock:
                        self._latest_state = state
            except Exception:
                if not self._stop_event.is_set():
                    time.sleep(0.05)

    def _make_command(self, state):
        motors = list(getattr(state, "motor_state", []))
        if len(motors) < self._dof or not HAS_PND_SDK:
            return None
        command = pnd_adam_msg_dds__LowCmd_(self._dof)
        command.mode_pr = 0
        with self._lock:
            targets = dict(self._target)
        for index, motor in enumerate(motors[:self._dof]):
            output = command.motor_cmd[index]
            output.mode = 1
            output.q = float(targets.get(self._joints[index], motor.q))
            output.dq = 0.0
            output.tau = 0.0
            output.kp = float(self._kp[index])
            output.kd = float(self._kd[index])
            output.ki = 0.0
            output.reserve = 0
        command.reserve = 0
        return command

    def _publish_loop(self):
        interval = 1.0 / max(1.0, self._rate_hz)
        while not self._stop_event.is_set():
            started = time.monotonic()
            with self._lock:
                active = self._active
                state = self._latest_state
            if active and state is not None and self._lowcmd_pub is not None:
                command = self._make_command(state)
                if command is not None:
                    try:
                        self._lowcmd_pub.Write(command)
                    except Exception:
                        pass
            self._stop_event.wait(max(0.0, interval - (time.monotonic() - started)))

    def enable(self):
        if self._lowcmd_pub is None:
            return {"success": False, "message": "DDS rt/lowcmd publisher is unavailable"}
        with self._lock:
            self._active = True
        return {"success": True, "state": "active", "message": "DDS lowcmd arm control enabled"}

    def disable(self):
        with self._lock:
            self._active = False
        return {"success": True, "state": "idle"}

    def set_joints(self, values):
        if not isinstance(values, dict):
            return {"success": False, "message": "joints must be an object of joint name to radians"}
        unknown = []
        normalized = {}
        for name, value in values.items():
            if name not in self._control_indices:
                unknown.append(name)
                continue
            try:
                number = float(value)
            except (TypeError, ValueError):
                return {"success": False, "message": f"Invalid angle for joint {name!r}"}
            if not math.isfinite(number):
                return {"success": False, "message": f"Non-finite angle for joint {name!r}"}
            normalized[name] = number
        if unknown:
            return {"success": False, "message": "Unknown or non-arm joints", "unknown_joints": unknown}
        with self._lock:
            self._target.update(normalized)
            self._active = True
        return {"success": True, "state": "active", "joints_set": len(normalized), "joints": normalized}

    def zero(self):
        return self.set_joints({name: 0.0 for name in self._control_indices})

    def info(self):
        with self._lock:
            return {
                "state": "active" if self._active else "idle",
                "state_received": self._latest_state is not None,
                "controlled_joints": sorted(self._control_indices),
            }


class ArmPlugin:
    PREFIX = "arm"

    def __init__(self, plugin_config, _namespace, _executor, variant, arm_lowstate_sub=None, lowcmd_pub=None, **_kwargs):
        self._controller = _ArmDdsController(
            variant,
            float(plugin_config.get("control_rate_hz", 400)),
            lowstate_sub=arm_lowstate_sub,
            lowcmd_pub=lowcmd_pub,
        )

    def get_tool(self):
        return {
            "name": "arm",
            "type": "actuator",
            "multiInstance": False,
            "description": "Adam waist/arm/wrist control through periodic DDS rt/lowcmd; use while high-level robot is standing",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["enable", "disable", "set_joints", "zero"]},
                    "joints": {"type": "object", "description": "Joint name to radian value, e.g. shoulderPitch_Left"},
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {"params": [], "description": "Enable periodic rt/lowcmd arm control"},
                    "disable": {"params": [], "description": "Release arm lowcmd control"},
                    "set_joints": {"params": ["joints"], "description": "Set one or more arm/waist joint targets in radians"},
                    "zero": {"params": [], "description": "Set controlled upper-body joints to zero"},
                },
            },
        }

    def start(self):
        self._controller.start()

    def stop(self):
        self._controller.stop()

    def dispatch(self, action, args):
        if action == "enable":
            return self._controller.enable()
        if action == "disable":
            return self._controller.disable()
        if action == "set_joints":
            return self._controller.set_joints(args.get("joints", {}))
        if action == "zero":
            return self._controller.zero()
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return self._controller.disable()
        if action == "info":
            return self._controller.info()
        return None


# ---------------------------------------------------------------------------
# Resource card and bundle
# ---------------------------------------------------------------------------

class ModelPlugin:
    PREFIX = "model"
    URDFS = {"lite": "adam_lite.urdf", "sp": "adam_sp.urdf", "pro": "adam_pro.urdf"}

    def __init__(self, _plugin_config, _namespace, _executor, variant, **_kwargs):
        self._variant = variant
        self._path = Path(__file__).parent / "resource" / self.URDFS.get(variant, "adam_pro.urdf")

    def get_tool(self):
        return {
            "name": "model", "type": "resource", "multiInstance": False,
            "description": f"Adam {self._variant} URDF model",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, _args):
        if action in ("start", "stop"):
            return {"state": "running" if action == "start" else "idle"}
        if self._path.exists():
            return {"urdf": self._path.read_text(encoding="utf-8")}
        return {"error": f"No URDF found for variant {self._variant!r}"}


class AdamDeviceBundle:
    def __init__(
        self, config, namespace, executor, grpc_client,
        lowstate_sub=None, arm_lowstate_sub=None, handstate_sub=None,
        lowcmd_pub=None, hand_pub=None,
    ):
        self._plugins = []
        self._tool_map = {}
        variant = config.get("variant", "pro")
        plugin_config = config.get("plugins", {})

        def enabled(name):
            return plugin_config.get(name, {}).get("enabled", True)

        if enabled("state"):
            self._plugins.append(StatePlugin(plugin_config.get("state", {}), namespace, executor, variant, lowstate_sub=lowstate_sub))
        if enabled("robot_state"):
            self._plugins.append(RobotStatePlugin(plugin_config.get("robot_state", {}), namespace, executor, grpc_client))
        if enabled("switch_mode"):
            self._plugins.append(SwitchModePlugin(plugin_config.get("switch_mode", {}), namespace, executor, grpc_client))
        if enabled("loco"):
            self._plugins.append(LocoPlugin(plugin_config.get("loco", {}), namespace, executor, grpc_client))
        if enabled("arm"):
            self._plugins.append(ArmPlugin(plugin_config.get("arm", {}), namespace, executor, variant, arm_lowstate_sub=arm_lowstate_sub, lowcmd_pub=lowcmd_pub))
        if enabled("hand_state"):
            self._plugins.append(HandStatePlugin(plugin_config.get("hand_state", {}), namespace, executor, handstate_sub=handstate_sub))
        if enabled("hand"):
            self._plugins.append(HandPlugin(plugin_config.get("hand", {}), namespace, executor, hand_pub=hand_pub))
        if enabled("model"):
            self._plugins.append(ModelPlugin(plugin_config.get("model", {}), namespace, executor, variant))

        for plugin in self._plugins:
            tools = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            for tool in tools:
                self._tool_map[tool["name"]] = plugin

    def start_all(self):
        for plugin in self._plugins:
            plugin.start()

    def stop_all(self):
        for plugin in reversed(self._plugins):
            plugin.stop()

    def get_all_tools(self):
        tools = []
        for plugin in self._plugins:
            tools.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
        return tools

    def dispatch(self, tool_name, args):
        plugin = self._tool_map.get(tool_name)
        if plugin is None:
            return {"error": f"Unknown tool: {tool_name}"}
        call_args = dict(args or {})
        action = call_args.pop("action", tool_name)
        call_args["_tool_name"] = tool_name
        result = plugin.dispatch(action, call_args)
        if result is None:
            return {"error": f"Unknown action {action!r} for tool {tool_name!r}"}
        return result
