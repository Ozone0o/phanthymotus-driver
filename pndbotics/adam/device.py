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
import io
import math
import os
import struct
import sys
import threading
import time
import zlib
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
# Direct body command controller
# ---------------------------------------------------------------------------

class _LowCmdStream:
    """Single owner of the periodic ``rt/lowcmd`` publisher.

    The lowcmd-backed cards (``arm``, ``lowcmd``, ``ankle_mode``) all write
    per-joint command fields and the ankle mode into this shared stream; it
    composes and publishes ``LowCmd_`` at a fixed rate. Sharing one publisher
    guarantees the cards never fight over the ``rt/lowcmd`` topic.

    Per-joint ``q`` targets are optional: ``None`` means "hold the latest
    ``rt/lowstate`` position", so enabling the stream is safe mid-motion.
    ``kp``/``kd`` are also optional and fall back to the variant gain profile.
    """

    def __init__(self, variant: str, rate_hz: float, lowstate_sub=None, lowcmd_pub=None):
        self._joints = VARIANT_JOINTS[variant]
        self._dof = VARIANT_DOF[variant]
        self._joint_index = {name: index for index, name in enumerate(self._joints)}
        self._default_kp, self._default_kd = _gain_profile(variant)
        self._rate_hz = float(rate_hz)
        self._lowstate_sub = lowstate_sub
        self._lowcmd_pub = lowcmd_pub

        # Full per-joint MotorCmd_ command state (all DOF).
        self._mode = [1] * self._dof      # 1=enable, 0=disable
        self._q = [None] * self._dof      # None -> hold latest lowstate position
        self._dq = [0.0] * self._dof
        self._tau = [0.0] * self._dof
        self._kp = [None] * self._dof     # None -> default gain profile
        self._kd = [None] * self._dof
        self._ki = [0.0] * self._dof
        self._mode_pr = 0                 # 0=PR (pitch/roll), 1=AB (parallel A/B)

        self._latest_state = None
        self._active = False
        self._started = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._state_thread = None
        self._command_thread = None

    # -- lifecycle -----------------------------------------------------------

    def start(self):
        if self._started:
            return
        self._started = True
        if self._lowstate_sub is not None:
            self._state_thread = threading.Thread(
                target=self._poll_state, daemon=True, name="lowcmd_state"
            )
            self._state_thread.start()
        if self._lowcmd_pub is not None:
            self._command_thread = threading.Thread(
                target=self._publish_loop, daemon=True, name="lowcmd_publish"
            )
            self._command_thread.start()

    def stop(self):
        with self._lock:
            self._active = False
        self._stop_event.set()
        for thread in (self._state_thread, self._command_thread):
            if thread is not None:
                thread.join(timeout=1.5)
        self._state_thread = None
        self._command_thread = None
        self._stop_event = threading.Event()
        self._started = False

    # -- lowstate feed (position hold) ----------------------------------------

    def _poll_state(self):
        while not self._stop_event.is_set():
            try:
                sample = self._lowstate_sub.Read(timeout=1)
                if sample is not None:
                    with self._lock:
                        self._latest_state = sample
            except Exception:
                if not self._stop_event.is_set():
                    time.sleep(0.05)

    # -- command mutation (used by cards) -------------------------------------

    def set_active(self, active):
        if active and self._lowcmd_pub is None:
            return {"success": False, "message": "DDS rt/lowcmd publisher is unavailable"}
        with self._lock:
            self._active = bool(active)
        return {"success": True, "state": "active" if active else "idle"}

    def set_mode_pr(self, mode_pr):
        if mode_pr not in (0, 1):
            return {"success": False, "message": "mode_pr must be 0 (PR) or 1 (AB)"}
        with self._lock:
            self._mode_pr = mode_pr
        return {"success": True, "mode_pr": mode_pr, "mode_name": "PR" if mode_pr == 0 else "AB"}

    def set_joint_fields(self, updates):
        """Set per-joint fields. ``updates``: {joint_name: {field: value}}.

        Fields: q (rad), dq (rad/s), tau (Nm), kp, kd, ki, mode (0/1).
        """
        if not isinstance(updates, dict) or not updates:
            return {"success": False, "message": "joints must be a non-empty mapping"}
        unknown = []
        applied = {}
        numeric = ("q", "dq", "tau", "kp", "kd", "ki")
        with self._lock:
            for name, fields in updates.items():
                index = self._joint_index.get(name)
                if index is None or not isinstance(fields, dict):
                    unknown.append(name)
                    continue
                per = {}
                for field, value in fields.items():
                    try:
                        if field in numeric:
                            number = float(value)
                            if not math.isfinite(number):
                                unknown.append(f"{name}.{field}")
                                continue
                        if field == "q":
                            self._q[index] = number
                        elif field == "dq":
                            self._dq[index] = number
                        elif field == "tau":
                            self._tau[index] = number
                        elif field == "kp":
                            self._kp[index] = number
                        elif field == "kd":
                            self._kd[index] = number
                        elif field == "ki":
                            self._ki[index] = number
                        elif field == "mode":
                            self._mode[index] = 1 if int(value) != 0 else 0
                        else:
                            unknown.append(f"{name}.{field}")
                            continue
                        per[field] = value
                    except (TypeError, ValueError):
                        unknown.append(f"{name}.{field}")
                        continue
                if per:
                    applied[name] = per
        if not applied:
            return {"success": False, "message": "No valid joint fields", "unknown": unknown}
        return {"success": True, "joints_set": len(applied), "joints": applied,
                "unknown": unknown or None}

    def zero(self, names=None):
        with self._lock:
            targets = names if names is not None else self._joint_index
            zeroed = []
            for name in targets:
                index = self._joint_index.get(name)
                if index is not None:
                    self._q[index] = 0.0
                    self._dq[index] = 0.0
                    self._tau[index] = 0.0
                    zeroed.append(name)
        return {"success": True, "joints_zeroed": len(zeroed)}

    def info(self):
        with self._lock:
            return {
                "state": "active" if self._active else "idle",
                "state_received": self._latest_state is not None,
                "mode_pr": self._mode_pr,
                "mode_name": "PR" if self._mode_pr == 0 else "AB",
                "dof": self._dof,
                "enabled_joints": [self._joints[i] for i in range(self._dof) if self._mode[i]],
            }

    # -- composition + publish ------------------------------------------------

    def _make_command(self):
        if self._lowcmd_pub is None or not HAS_PND_SDK:
            return None
        with self._lock:
            state = self._latest_state
            mode_pr = self._mode_pr
            mode = list(self._mode)
            q = list(self._q)
            dq = list(self._dq)
            tau = list(self._tau)
            kp = list(self._kp)
            kd = list(self._kd)
            ki = list(self._ki)
        motors = list(getattr(state, "motor_state", []))
        command = pnd_adam_msg_dds__LowCmd_(self._dof)
        command.mode_pr = mode_pr
        for index in range(self._dof):
            output = command.motor_cmd[index]
            output.mode = mode[index]
            if q[index] is None:
                output.q = float(motors[index].q) if index < len(motors) else 0.0
            else:
                output.q = q[index]
            output.dq = dq[index]
            output.tau = tau[index]
            output.kp = kp[index] if kp[index] is not None else self._default_kp[index]
            output.kd = kd[index] if kd[index] is not None else self._default_kd[index]
            output.ki = ki[index]
            output.reserve = 0
        command.reserve = 0
        return command

    def _publish_loop(self):
        interval = 1.0 / max(1.0, self._rate_hz)
        while not self._stop_event.is_set():
            started = time.monotonic()
            with self._lock:
                active = self._active
            if active:
                command = self._make_command()
                if command is not None:
                    try:
                        self._lowcmd_pub.Write(command)
                    except Exception:
                        pass
            self._stop_event.wait(max(0.0, interval - (time.monotonic() - started)))


class _ArmDdsController:
    """Upper-body subset of the shared _LowCmdStream (arm card facade)."""

    def __init__(self, stream):
        self._stream = stream
        self._control_indices = {
            name: index for name, index in stream._joint_index.items()
            if name in ARM_JOINT_NAMES
        }

    def start(self):
        self._stream.start()

    def stop(self):
        self._stream.stop()

    def enable(self):
        return self._stream.set_active(True)

    def disable(self):
        return self._stream.set_active(False)

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
        result = self._stream.set_joint_fields({name: {"q": value} for name, value in normalized.items()})
        result["state"] = "active"
        return result

    def zero(self):
        return self._stream.zero(list(self._control_indices))

    def info(self):
        info = self._stream.info()
        info["controlled_joints"] = sorted(self._control_indices)
        return info


class ArmPlugin:
    PREFIX = "arm"

    def __init__(self, plugin_config, _namespace, _executor, lowcmd_stream=None, **_kwargs):
        self._controller = _ArmDdsController(lowcmd_stream)

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


class LowCmdPlugin:
    """Full-DOF low-level joint control through the shared rt/lowcmd stream."""

    PREFIX = "lowcmd"

    def __init__(self, plugin_config, _namespace, _executor, lowcmd_stream=None, **_kwargs):
        self._stream = lowcmd_stream

    def get_tool(self):
        return {
            "name": "lowcmd",
            "type": "actuator",
            "multiInstance": False,
            "description": "Adam full-DOF low-level control via DDS rt/lowcmd: per-joint q/dq/tau/kp/kd/ki and motor enable. Use while the high-level robot is standing or in the low-level dev pipeline",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["enable", "disable", "set_joints", "zero", "info"]},
                    "joints": {
                        "type": "object",
                        "description": "Joint name → field map. Fields: q (rad), dq (rad/s), tau (Nm), kp, kd, ki, mode (0/1). e.g. {\"hipPitch_Left\": {\"q\": 0.3, \"tau\": 0.0}, \"elbow_Left\": {\"mode\": 0}}",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {"params": [], "description": "Start periodic rt/lowcmd stream"},
                    "disable": {"params": [], "description": "Stop publishing (release control)"},
                    "set_joints": {"params": ["joints"], "description": "Set per-joint q/dq/tau/kp/kd/ki/mode"},
                    "zero": {"params": [], "description": "Zero q/dq/tau for all joints (keeps gains)"},
                    "info": {"params": [], "description": "Stream state, ankle mode_pr, enabled joints"},
                },
            },
        }

    def start(self):
        if self._stream is not None:
            self._stream.start()

    def stop(self):
        if self._stream is not None:
            self._stream.stop()

    def dispatch(self, action, args):
        if self._stream is None:
            return {"success": False, "message": "DDS rt/lowcmd stream is unavailable"}
        if action == "enable":
            return self._stream.set_active(True)
        if action == "disable":
            return self._stream.set_active(False)
        if action == "set_joints":
            return self._stream.set_joint_fields(args.get("joints", {}))
        if action == "zero":
            return self._stream.zero()
        if action == "info":
            return self._stream.info()
        return None


class AnkleModePlugin:
    """Switch ankle control mode PR (0) / AB (1) on the shared rt/lowcmd stream."""

    PREFIX = "ankle_mode"
    MODES = ["PR", "AB"]

    def __init__(self, plugin_config, _namespace, _executor, lowcmd_stream=None, **_kwargs):
        self._stream = lowcmd_stream

    def get_tool(self):
        return {
            "name": "ankle_mode",
            "type": "actuator",
            "multiInstance": False,
            "description": "Adam ankle control mode — PR (series pitch/roll) or AB (parallel virtual A/B joints), carried by LowCmd_.mode_pr on the rt/lowcmd stream",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["set", "get"]},
                    "mode": {"type": "string", "enum": self.MODES, "description": "PR or AB"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set": {"params": ["mode"], "description": "Switch ankle control mode"},
                    "get": {"params": [], "description": "Read current ankle control mode"},
                },
            },
        }

    def start(self):
        if self._stream is not None:
            self._stream.start()

    def stop(self):
        if self._stream is not None:
            self._stream.stop()

    def dispatch(self, action, args):
        if self._stream is None:
            return {"success": False, "message": "DDS rt/lowcmd stream is unavailable"}
        if action == "set":
            mode = str(args.get("mode", "")).upper()
            if mode not in self.MODES:
                return {"success": False, "message": f"mode must be one of {self.MODES}"}
            return self._stream.set_mode_pr(0 if mode == "PR" else 1)
        if action == "get":
            info = self._stream.info()
            return {"success": True, "mode_pr": info["mode_pr"], "mode_name": info["mode_name"]}
        return None


# ---------------------------------------------------------------------------
# Local ZED Mini camera cards
# ---------------------------------------------------------------------------

class ZedCameraPlugin:
    """Publish the Adam ZED Mini through the local ZED Python SDK.

    The camera is physically attached to the Jetson running this container, so
    there is no reason to consume the separate ZED network-stream sender.  One
    capture thread owns the SDK camera and feeds four lightweight ROS2 topics:
    JPEG RGB, zlib-compressed uint16 depth, JSON camera information, and an
    optional point cloud for the native Phanthymotus renderer.
    """

    _FORMATS = {
        "camera_head": "image/jpeg",
        "camera_depth": "image/depth-zlib",
        "camera_info": "data/json",
        "camera_pointcloud": "sensor/pointcloud",
    }

    def __init__(self, plugin_config, namespace, executor):
        self._config = dict(plugin_config or {})
        self._namespace = namespace
        self._topics = {
            "camera_head": f"/{namespace}/camera/head",
            "camera_depth": f"/{namespace}/camera/head/depth",
            "camera_info": f"/{namespace}/camera/head/info",
            "camera_pointcloud": f"/{namespace}/camera/head/points",
        }

        pointcloud_config = self._config.get("pointcloud", {})
        if not isinstance(pointcloud_config, dict):
            pointcloud_config = {}
        self._pointcloud_config = pointcloud_config
        self._pointcloud_enabled = bool(
            pointcloud_config.get(
                "enabled", self._config.get("pointcloud_enabled", False)))
        self._rgb_hz = max(1.0, min(float(self._config.get("rgb_hz", 15)), 30.0))
        self._depth_hz = max(1.0, min(float(self._config.get("depth_hz", 8)), 15.0))
        self._info_hz = max(0.2, min(float(self._config.get("info_hz", 1)), 5.0))
        self._pointcloud_hz = max(
            0.2, min(float(pointcloud_config.get("hz", 2)), 10.0))
        self._jpeg_quality = max(
            20, min(int(self._config.get("jpeg_quality", 70)), 95))
        self._max_points = max(
            1000, min(int(pointcloud_config.get("max_points", 10000)), 40000))
        self._max_point_distance_m = max(
            1.0, min(float(pointcloud_config.get("max_distance_m", 8.0)), 30.0))
        self._resolution_name = str(self._config.get("resolution", "VGA")).upper()
        self._depth_mode_name = str(
            self._config.get("depth_mode", "PERFORMANCE")).upper()
        self._camera_fps = max(1, min(int(self._config.get("fps", 15)), 60))
        # The ZED Mini on Adam's head is physically mounted upside down.  Let
        # the SDK rotate the complete camera data path (RGB, depth and point
        # cloud) together so the three outputs remain pixel/geometry aligned.
        self._camera_flip = bool(self._config.get("camera_flip", True))

        self._running = False
        self._available = False
        self._camera = None
        self._capture_thread = None
        self._info_timer = None
        self._lock = threading.Lock()
        self._state = {
            "state": "idle",
            "available": False,
            "source": "zed-sdk-local",
            "model": None,
            "serial_number": None,
            "error": None,
            "last_frame_ts_ms": None,
            "last_rgb_ts_ms": None,
            "last_depth_ts_ms": None,
            "last_pointcloud_ts_ms": None,
            "pointcloud_enabled": self._pointcloud_enabled,
            "camera_flip": self._camera_flip,
        }

        self._pub_node = Node("adam_zed_camera")
        executor.add_node(self._pub_node)

    @staticmethod
    def _tool(name, description, topic, fmt, input_schema=None):
        return {
            "name": name,
            "type": "sensor",
            "multiInstance": False,
            "description": description,
            "inputSchema": input_schema or {"type": "object", "properties": {}},
            "topic_out": [{"topic": topic, "format": fmt}],
        }

    def get_tools(self):
        return [
            self._tool(
                "camera_head",
                "Adam ZED Mini left RGB image from the local Jetson ZED SDK",
                self._topics["camera_head"], self._FORMATS["camera_head"]),
            self._tool(
                "camera_depth",
                "Adam ZED Mini depth image, zlib-compressed little-endian uint16 millimetres",
                self._topics["camera_depth"], self._FORMATS["camera_depth"]),
            self._tool(
                "camera_info",
                "Adam ZED Mini connection status, stream settings, calibration and intrinsics",
                self._topics["camera_info"], self._FORMATS["camera_info"]),
            self._tool(
                "camera_pointcloud",
                "Adam ZED Mini XYZ point cloud for the Phanthymotus 3D renderer; runtime-toggleable",
                self._topics["camera_pointcloud"], self._FORMATS["camera_pointcloud"],
                {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["start", "stop", "info"],
                            "description": "Enable or disable point-cloud publishing without reopening the camera",
                        },
                    },
                }),
        ]

    def start(self):
        if self._running:
            return
        self._running = True
        try:
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import String, UInt8MultiArray

            qos = _best_effort_qos()
            self._CompressedImage = CompressedImage
            self._UInt8MultiArray = UInt8MultiArray
            self._rgb_pub = self._pub_node.create_publisher(
                CompressedImage, self._topics["camera_head"], qos)
            self._depth_pub = self._pub_node.create_publisher(
                CompressedImage, self._topics["camera_depth"], qos)
            self._info_pub = self._pub_node.create_publisher(
                String, self._topics["camera_info"], qos)
            self._pointcloud_pub = self._pub_node.create_publisher(
                UInt8MultiArray, self._topics["camera_pointcloud"], qos)
            self._info_timer = self._pub_node.create_timer(
                1.0 / self._info_hz, self._publish_info)
        except Exception as exc:
            self._running = False
            self._set_error(f"ROS2 camera publisher setup failed: {exc}")
            return

        self._publish_info()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="adam_zed_capture")
        self._capture_thread.start()

    def stop(self):
        self._running = False
        if self._info_timer is not None:
            try:
                self._info_timer.cancel()
            except Exception:
                pass
            self._info_timer = None
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=3.0)
            self._capture_thread = None
        camera = self._camera
        self._camera = None
        if camera is not None:
            try:
                camera.close()
            except Exception:
                pass
        self._available = False
        with self._lock:
            self._state.update({
                "state": "idle",
                "available": False,
                "pointcloud_enabled": self._pointcloud_enabled,
            })
        self._publish_info()

    def _set_error(self, message):
        with self._lock:
            self._available = False
            self._state.update({
                "state": "error",
                "available": False,
                "error": str(message),
            })
        print(f"[ZedCameraPlugin] {message}", flush=True)

    @staticmethod
    def _enum_name(value):
        name = getattr(value, "name", None)
        return str(name if name is not None else value)

    @staticmethod
    def _resolution_dict(resolution):
        return {
            "width": int(getattr(resolution, "width", 0)),
            "height": int(getattr(resolution, "height", 0)),
        }

    @staticmethod
    def _float_list(value):
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError):
            return []

    def _camera_metadata(self, camera_info, params):
        configuration = camera_info.camera_configuration
        calibration = configuration.calibration_parameters
        left = calibration.left_cam
        right = calibration.right_cam
        translation = calibration.stereo_transform.get_translation().get()
        return {
            "state": "running",
            "available": True,
            "connected": True,
            "source": "zed-sdk-local",
            "sdk": "ZED SDK",
            "model": str(camera_info.camera_model),
            "serial_number": int(camera_info.serial_number),
            "firmware_version": int(configuration.firmware_version),
            "resolution": self._resolution_dict(configuration.resolution),
            "fps": int(configuration.fps),
            "depth_mode": self._enum_name(params.depth_mode),
            "coordinate_units": self._enum_name(params.coordinate_units),
            "calibration": {
                "left": {
                    "fx": float(left.fx), "fy": float(left.fy),
                    "cx": float(left.cx), "cy": float(left.cy),
                    "distortion": self._float_list(left.disto),
                },
                "right": {
                    "fx": float(right.fx), "fy": float(right.fy),
                    "cx": float(right.cx), "cy": float(right.cy),
                    "distortion": self._float_list(right.disto),
                },
                "baseline_m": float(calibration.get_camera_baseline()),
                "stereo_translation_m": self._float_list(translation),
            },
            "error": None,
            "pointcloud_enabled": self._pointcloud_enabled,
            "camera_flip": self._camera_flip,
        }

    @staticmethod
    def _load_zed_module():
        try:
            import pyzed.sl as sl
            return sl
        except ImportError as first_error:
            # The deployment mounts the host's architecture-specific pyzed
            # extension at /opt/pyzed instead of baking a licensed SDK into
            # the driver image.
            candidate = os.environ.get("ZED_PYTHON_PATH", "/opt/pyzed")
            if candidate:
                candidate_path = Path(candidate)
                search_paths = [candidate_path]
                # When the package directory itself is mounted at
                # /opt/pyzed, Python needs its parent (/opt) on sys.path.
                if (candidate_path / "__init__.py").exists() or list(candidate_path.glob("sl*.so")):
                    search_paths.append(candidate_path.parent)
                for search_path in reversed(search_paths):
                    if search_path.exists() and str(search_path) not in sys.path:
                        sys.path.insert(0, str(search_path))
            try:
                import pyzed.sl as sl
                return sl
            except ImportError as second_error:
                raise ImportError(
                    "pyzed.sl is unavailable; mount the Jetson ZED SDK and set "
                    "ZED_PYTHON_PATH (or PYTHONPATH) accordingly"
                ) from second_error
            except Exception:
                raise first_error

    def _capture_loop(self):
        try:
            import numpy as np
            from PIL import Image as PillowImage
            sl = self._load_zed_module()
        except Exception as exc:
            self._running = False
            self._set_error(f"local ZED SDK import failed: {exc}")
            return

        params = sl.InitParameters()
        params.camera_resolution = getattr(
            sl.RESOLUTION, self._resolution_name, sl.RESOLUTION.VGA)
        depth_mode = getattr(sl.DEPTH_MODE, self._depth_mode_name, None)
        if depth_mode is None:
            depth_mode = getattr(sl.DEPTH_MODE, "NEURAL_LIGHT", sl.DEPTH_MODE.PERFORMANCE)
        params.depth_mode = depth_mode
        params.camera_fps = self._camera_fps
        params.coordinate_units = sl.UNIT.METER
        if self._camera_flip:
            if hasattr(params, "camera_image_flip"):
                flip_modes = getattr(sl, "FLIP_MODE", None)
                params.camera_image_flip = getattr(flip_modes, "ON", 1)
            else:
                print(
                    "[ZedCameraPlugin] camera_flip requested but this ZED SDK "
                    "does not expose camera_image_flip",
                    flush=True,
                )
        if hasattr(params, "depth_maximum_distance"):
            params.depth_maximum_distance = self._max_point_distance_m

        camera = sl.Camera()
        try:
            status = camera.open(params)
        except Exception as exc:
            self._running = False
            self._set_error(f"ZED camera open failed: {exc}")
            return
        if status != sl.ERROR_CODE.SUCCESS:
            self._running = False
            self._set_error(f"ZED camera open failed: {status}")
            try:
                camera.close()
            except Exception:
                pass
            return

        self._camera = camera
        self._available = True
        with self._lock:
            self._state = self._camera_metadata(
                camera.get_camera_information(), params)
        self._publish_info()

        runtime = sl.RuntimeParameters()
        image = sl.Mat()
        depth = sl.Mat()
        pointcloud = sl.Mat()
        next_rgb = 0.0
        next_depth = 0.0
        next_pointcloud = 0.0
        last_grab_error = None

        try:
            while self._running:
                status = camera.grab(runtime)
                if status != sl.ERROR_CODE.SUCCESS:
                    if status != last_grab_error:
                        print(f"[ZedCameraPlugin] grab status: {status}", flush=True)
                        last_grab_error = status
                    time.sleep(0.01)
                    continue
                last_grab_error = None
                now = time.monotonic()
                now_ms = int(time.time() * 1000)
                with self._lock:
                    self._state["last_frame_ts_ms"] = now_ms

                if now >= next_rgb:
                    try:
                        camera.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU)
                        jpeg = self._encode_jpeg(
                            np.array(image.get_data(), copy=True), np, PillowImage)
                        msg = self._CompressedImage()
                        msg.format = "jpeg"
                        msg.data = jpeg
                        self._rgb_pub.publish(msg)
                        with self._lock:
                            self._state["last_rgb_ts_ms"] = now_ms
                    except Exception as exc:
                        self._set_error(f"RGB publish failed: {exc}")
                    next_rgb = now + 1.0 / self._rgb_hz

                need_depth = now >= next_depth
                with self._lock:
                    pointcloud_enabled = self._pointcloud_enabled
                need_pointcloud = pointcloud_enabled and now >= next_pointcloud

                if need_depth:
                    try:
                        camera.retrieve_measure(depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
                        depth_mm = self._normalize_depth(
                            np.array(depth.get_data(), copy=True), np)
                        msg = self._CompressedImage()
                        msg.format = "16UC1; compressedDepth zlib"
                        msg.data = zlib.compress(
                            depth_mm.astype("<u2", copy=False).tobytes(), level=1)
                        self._depth_pub.publish(msg)
                        with self._lock:
                            self._state["last_depth_ts_ms"] = now_ms
                    except Exception as exc:
                        self._set_error(f"depth publish failed: {exc}")
                    next_depth = now + 1.0 / self._depth_hz

                if need_pointcloud:
                    try:
                        camera.retrieve_measure(
                            pointcloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
                        payload = self._pack_pointcloud(
                            np.array(pointcloud.get_data(), copy=True), np)
                        if payload is not None:
                            msg = self._UInt8MultiArray()
                            msg.data = list(payload)
                            self._pointcloud_pub.publish(msg)
                            with self._lock:
                                self._state["last_pointcloud_ts_ms"] = now_ms
                    except Exception as exc:
                        self._set_error(f"pointcloud publish failed: {exc}")
                    next_pointcloud = now + 1.0 / self._pointcloud_hz
        finally:
            self._available = False
            if self._running:
                self._running = False
                self._set_error("ZED capture loop stopped unexpectedly")

    def _encode_jpeg(self, image, np, pillow_image):
        if image.ndim == 3 and image.shape[2] >= 3:
            # ZED's default U8_C4 CPU image is BGRA.  The dashboard expects
            # ordinary RGB JPEG bytes, so drop alpha and reverse BGR->RGB.
            rgb = image[:, :, :3][:, :, ::-1]
        elif image.ndim == 2:
            rgb = np.repeat(image[:, :, None], 3, axis=2)
        else:
            raise ValueError(f"unexpected ZED image shape {image.shape}")
        output = io.BytesIO()
        pillow_image.fromarray(np.ascontiguousarray(rgb), "RGB").save(
            output, format="JPEG", quality=self._jpeg_quality, optimize=False)
        return output.getvalue()

    @staticmethod
    def _normalize_depth(depth, np):
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        valid = np.isfinite(depth) & (depth > 0.0)
        millimetres = np.zeros(depth.shape, dtype=np.uint16)
        millimetres[valid] = np.clip(
            depth[valid] * 1000.0, 0.0, 65535.0).astype(np.uint16)

        # The stock Phanthymotus depth renderer consumes a fixed 640x480
        # matrix.  Crop the ZED 16:9 image centrally, then nearest-neighbour
        # resample without introducing an OpenCV dependency.
        height, width = millimetres.shape
        if width * 3 > height * 4:
            crop_width = max(1, (height * 4) // 3)
            left = max(0, (width - crop_width) // 2)
            millimetres = millimetres[:, left:left + crop_width]
        elif width * 3 < height * 4:
            crop_height = max(1, (width * 3) // 4)
            top = max(0, (height - crop_height) // 2)
            millimetres = millimetres[top:top + crop_height, :]
        source_height, source_width = millimetres.shape
        rows = (np.arange(480) * source_height / 480).astype(np.int64)
        cols = (np.arange(640) * source_width / 640).astype(np.int64)
        return millimetres[rows[:, None], cols[None, :]]

    def _pack_pointcloud(self, points, np):
        if points.ndim != 3 or points.shape[2] < 3:
            raise ValueError(f"unexpected ZED point-cloud shape {points.shape}")
        xyz = points[:, :, :3].reshape(-1, 3)
        valid = np.isfinite(xyz).all(axis=1)
        valid &= xyz[:, 2] > 0.05
        valid &= xyz[:, 2] <= self._max_point_distance_m
        xyz = xyz[valid]
        if xyz.size == 0:
            return None
        if xyz.shape[0] > self._max_points:
            stride = int(math.ceil(xyz.shape[0] / self._max_points))
            xyz = xyz[::stride][:self._max_points]

        # The renderer maps packed (a,b,c) to display (b,-c,-a).  Pack the
        # optical ZED frame (right, down, forward) as (forward, right, down)
        # so the displayed frame is (right, up, backward).
        packed_xyz = np.empty((xyz.shape[0], 3), dtype="<f4")
        packed_xyz[:, 0] = xyz[:, 2]
        packed_xyz[:, 1] = xyz[:, 0]
        packed_xyz[:, 2] = xyz[:, 1]
        return struct.pack("<II", 12, int(packed_xyz.shape[0])) + packed_xyz.tobytes()

    def _publish_info(self):
        publisher = getattr(self, "_info_pub", None)
        if publisher is None:
            return
        with self._lock:
            payload = dict(self._state)
            payload["pointcloud_enabled"] = self._pointcloud_enabled
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(message)

    def dispatch(self, action, args):
        tool_name = args.get("_tool_name", action)
        if tool_name == "camera_pointcloud" and action in ("start", "enable"):
            with self._lock:
                self._pointcloud_enabled = True
                self._state["pointcloud_enabled"] = True
            self._publish_info()
            return {"state": "running" if self._available else self._state["state"], "pointcloud_enabled": True}
        if tool_name == "camera_pointcloud" and action in ("stop", "disable"):
            with self._lock:
                self._pointcloud_enabled = False
                self._state["pointcloud_enabled"] = False
            self._publish_info()
            return {"state": "disabled", "pointcloud_enabled": False}
        if action in ("read", "get") and tool_name == "camera_info":
            with self._lock:
                return dict(self._state)
        if action in ("info", "start", "stop", tool_name):
            state = self._state.get("state", "idle")
            if tool_name == "camera_pointcloud" and not self._pointcloud_enabled:
                state = "disabled"
            return {
                "state": state,
                "topic_out": [{
                    "topic": self._topics[tool_name],
                    "format": self._FORMATS[tool_name],
                }],
                **({"pointcloud_enabled": self._pointcloud_enabled}
                   if tool_name == "camera_pointcloud" else {}),
            }
        return {"state": self._state.get("state", "idle")}


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
        if enabled("camera"):
            self._plugins.append(ZedCameraPlugin(
                plugin_config.get("camera", {}), namespace, executor))
        lowcmd_stream = None
        if enabled("arm") or enabled("lowcmd") or enabled("ankle_mode"):
            lowcmd_rate = float(
                plugin_config.get("lowcmd", {}).get(
                    "control_rate_hz",
                    plugin_config.get("arm", {}).get("control_rate_hz", 400),
                )
            )
            lowcmd_stream = _LowCmdStream(
                variant, lowcmd_rate,
                lowstate_sub=arm_lowstate_sub,
                lowcmd_pub=lowcmd_pub,
            )
        if enabled("arm"):
            self._plugins.append(ArmPlugin(plugin_config.get("arm", {}), namespace, executor, lowcmd_stream=lowcmd_stream))
        if enabled("lowcmd"):
            self._plugins.append(LowCmdPlugin(plugin_config.get("lowcmd", {}), namespace, executor, lowcmd_stream=lowcmd_stream))
        if enabled("ankle_mode"):
            self._plugins.append(AnkleModePlugin(plugin_config.get("ankle_mode", {}), namespace, executor, lowcmd_stream=lowcmd_stream))
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
