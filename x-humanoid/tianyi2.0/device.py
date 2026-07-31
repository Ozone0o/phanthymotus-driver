#!/usr/bin/env python3
"""
x-humanoid/tianyi2.0/device.py — 天轶2.0 Pro 设备插件。

设计原则：
  - 一个设备 = 一个 tool (或 multi-tool plugin)
  - sensor：只读，驱动启动时自动 start，数据通过 ROS2 topic 输出 (domain 42)
  - actuator：action 参数分发操作，通过 ROS2 发布指令到天轶 (domain 0)
  - resource：返回静态数据 (如 URDF)
  - 角度对外用度(degrees)，内部转弧度(rad)发送

双 Domain 模式：
  - domain 0 (ros2.ctx_tianyi): 订阅天轶本体话题、发布控制指令
  - domain 42 (ros2.ctx_core): 发布传感器数据给 Agent Core

插件列表：
  StatePlugin      (sensor, multi-tool) — 关节/电池/急停/力传感器/URDF
  CameraPlugin     (sensor)             — Orbbec 头部相机
  AsrPlugin        (sensor)             — 语音识别结果
  NavStatePlugin   (sensor)             — 底盘导航状态
  HeadPlugin       (actuator)           — 头部3DOF控制
  ArmPlugin        (actuator)           — 双臂14DOF控制
  WaistPlugin      (actuator)           — 腰部偏航+腿部升降控制
  HandPlugin       (actuator)           — 灵巧手控制
  GesturePlugin    (actuator)           — 厂商预设上半身表演动作
  TtsPlugin        (actuator)           — 语音合成
  NavPlugin        (actuator)           — 底盘导航控制
  ChatPlugin       (actuator)           — 语音交互开关
"""

import json
import math
from array import array
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)

_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# ── Motor ID → Joint Name 映射 ───────────────────────────────────────────────

_HEAD_JOINTS = {
    1: "head_roll_joint",
    2: "head_pitch_joint",
    3: "head_yaw_joint",
}

_ARM_LEFT_JOINTS = {
    11: "left_shoulder_pitch_joint",
    12: "left_shoulder_roll_joint",
    13: "left_shoulder_yaw_joint",
    14: "left_elbow_pitch_joint",
    15: "left_wrist_yaw_joint",
    16: "left_wrist_pitch_joint",
    17: "left_wrist_roll_joint",
}

_ARM_RIGHT_JOINTS = {
    21: "right_shoulder_pitch_joint",
    22: "right_shoulder_roll_joint",
    23: "right_shoulder_yaw_joint",
    24: "right_elbow_pitch_joint",
    25: "right_wrist_yaw_joint",
    26: "right_wrist_pitch_joint",
    27: "right_wrist_roll_joint",
}

_WAIST_JOINTS = {
    31: "waist_yaw_joint",
    32: "waist_pitch_joint",
}

_LEG_JOINTS = {
    51: "left_hip_pitch_joint",
    52: "left_knee_pitch_joint",
}

_ALL_JOINTS = {**_HEAD_JOINTS, **_ARM_LEFT_JOINTS, **_ARM_RIGHT_JOINTS, **_WAIST_JOINTS, **_LEG_JOINTS}

# ── 关节限位 (deg, rpm, A): motor_id → (min_deg, max_deg, max_spd_rpm, rated_current_a) ─

_JOINT_LIMITS = {
    # 头部
    1:  (-26,    26,    64,  5.0),
    2:  (-25,    25,    64,  5.0),
    3:  (-90,    90,    64,  5.0),
    # 左臂
    11: (-170,   170,   88,  35.0),
    12: (-15,    150,   120, 23.0),
    13: (-170,   170,   73,  8.0),
    14: (-150,   15,    73,  8.0),
    15: (-170,   170,   146, 8.0),
    16: (-45,    60,    72,  5.0),
    17: (-95,    75,    72,  5.0),
    # 右臂
    21: (-170,   170,   88,  35.0),
    22: (-150,   15,    120, 23.0),
    23: (-170,   170,   73,  8.0),
    24: (-150,   15,    73,  8.0),
    25: (-170,   170,   146, 8.0),
    26: (-45,    60,    72,  5.0),
    27: (-75,    95,    72,  5.0),
    # 腰部
    31: (-160,   180,   30,  31.0),
    32: (-45,    120,   37.5, 82.0),
    # 左腿
    51: (-40,    5,     37.5, 5.0),
    52: (-23,    20,    37.5, 5.0),
}


def _rpm2rads(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


def _clamp(val: float, lo: float, hi: float) -> tuple:
    """Clamp value to [lo, hi]; returns (clamped_value, was_clamped)."""
    if val < lo:
        return lo, True
    if val > hi:
        return hi, True
    return val, False


def _resolve_motor_id(identifier, valid_ids: list) -> int | None:
    """将关节标识符（int ID 或 str 名称）解析为 motor ID。"""
    if isinstance(identifier, int):
        return identifier if identifier in valid_ids else None
    if isinstance(identifier, str):
        s = identifier.strip().lower()
        for mid in valid_ids:
            name = _ALL_JOINTS.get(mid, "")
            if s == name or s == str(mid):
                return mid
    return None


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


# ══════════════════════════════════════════════════════════════════════════════
# StatePlugin (sensor, multi-tool)
# ══════════════════════════════════════════════════════════════════════════════

class StatePlugin:
    """关节状态 + 电池 + 急停 + 力传感器 + URDF 模型"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._running = False

        # Cached state
        self._joint_data = {}  # motor_id → {pos, speed, current, temp, error}
        self._battery = {}
        self._estop = {}
        self._force_left = {}
        self._force_right = {}
        self._lock = threading.Lock()

        # Topics for Agent Core (domain 42)
        self._topic_joints = f"/{namespace}/state/joints"
        self._topic_battery = f"/{namespace}/state/battery"
        self._topic_estop = f"/{namespace}/state/estop"
        self._topic_force = f"/{namespace}/state/force"

        # Subscriber node (domain 0 - tianyi)
        self._sub_node = Node("tianyi2_state_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        # Publisher node (domain 42 - agent core)
        self._pub_node = Node("tianyi2_state_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

        self._pub_joints = self._pub_node.create_publisher(String, self._topic_joints, _LOW_LAT_QOS)
        self._pub_battery = self._pub_node.create_publisher(String, self._topic_battery, _LOW_LAT_QOS)
        self._pub_estop = self._pub_node.create_publisher(String, self._topic_estop, _LOW_LAT_QOS)
        self._pub_force = self._pub_node.create_publisher(String, self._topic_force, _LOW_LAT_QOS)

        # URDF path
        self._urdf_path = Path(__file__).parent / "resource" / "tianyi2_model.urdf"

    def get_tools(self) -> list:
        return [
            {
                "name": "joints",
                "type": "sensor",
                "description": "天轶2.0 全身关节状态 — 位置/速度/电流/温度 (头/臂/腰/腿 共21个关节)",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_joints, "format": "sensor/skeleton"}],
            },
            {
                "name": "battery",
                "type": "sensor",
                "description": "天轶2.0 电池状态 — 电压/电流/电量 (大电池 + 小电池)",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_battery, "format": "data/json"}],
            },
            {
                "name": "estop",
                "type": "sensor",
                "description": "天轶2.0 急停和电源状态 — 急停按钮/软急停/电源/工作时间",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_estop, "format": "data/json"}],
            },
            {
                "name": "force_sensor",
                "type": "sensor",
                "description": "天轶2.0 六维力传感器 — 双腕力/力矩 (左/右 各3力+3力矩)",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_force, "format": "data/json"}],
            },
            {
                "name": "model",
                "type": "resource",
                "description": "天轶2.0 URDF 骨架模型 — 用于3D可视化",
                "inputSchema": {"type": "object", "properties": {}},
            },
        ]

    def start(self):
        self._running = True
        try:
            from bodyctrl_msgs.msg import MotorStatusMsg, PowerBatteryStatus, PowerBoardKeyStatus
            from geometry_msgs.msg import WrenchStamped

            # Subscribe to motor status topics
            for topic in ["/head/status", "/arm/status", "/waist/status", "/leg/status"]:
                self._sub_node.create_subscription(
                    MotorStatusMsg, topic, self._on_motor_status, _RELIABLE_QOS)

            # Battery
            self._sub_node.create_subscription(
                PowerBatteryStatus, "/power/battery/status", self._on_battery, _RELIABLE_QOS)

            # E-stop
            self._sub_node.create_subscription(
                PowerBoardKeyStatus, "/power/board/key_status", self._on_estop, _RELIABLE_QOS)

            # Force sensors (100Hz, throttle to 5Hz in callback)
            self._sub_node.create_subscription(
                WrenchStamped, "/arm_6dof_left", self._on_force_left, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                WrenchStamped, "/arm_6dof_right", self._on_force_right, _RELIABLE_QOS)

            print("[StatePlugin] subscriptions created")
        except ImportError as e:
            print(f"[StatePlugin] WARNING: msg import failed ({e}), running in stub mode")

        # Publish timer
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False

    def _on_motor_status(self, msg):
        with self._lock:
            for s in msg.status:
                self._joint_data[s.name] = {
                    "pos": s.pos,
                    "speed": s.speed,
                    "current": s.current,
                    "temp": s.temperature,
                    "error": s.error,
                }

    def _on_battery(self, msg):
        with self._lock:
            self._battery = {
                "master_voltage": msg.master_battery_voltage,
                "master_current": msg.master_battery_current,
                "master_power": msg.master_battery_power,
                "little_voltage": msg.little_battery_voltage,
                "little_current": msg.little_battery_current,
                "little_power": msg.little_battery_power,
                "battery_installed": msg.battery_installed,
                "battery_working": msg.battery_working,
            }

    def _on_estop(self, msg):
        with self._lock:
            self._estop = {
                "work_time": msg.work_time,
                "is_estop": msg.is_estop.data,
                "is_remote_estop": msg.is_remote_estop.data,
                "is_power_on": msg.is_power_on.data,
            }

    _force_last_pub = 0

    def _on_force_left(self, msg):
        now = time.time()
        if now - self._force_last_pub < 0.2:  # 5Hz throttle
            return
        with self._lock:
            self._force_left = {
                "fx": msg.wrench.force.x,
                "fy": msg.wrench.force.y,
                "fz": msg.wrench.force.z,
                "tx": msg.wrench.torque.x,
                "ty": msg.wrench.torque.y,
                "tz": msg.wrench.torque.z,
            }

    def _on_force_right(self, msg):
        with self._lock:
            self._force_right = {
                "fx": msg.wrench.force.x,
                "fy": msg.wrench.force.y,
                "fz": msg.wrench.force.z,
                "tx": msg.wrench.torque.x,
                "ty": msg.wrench.torque.y,
                "tz": msg.wrench.torque.z,
            }

    def _publish_loop(self):
        """Publish aggregated state at 10Hz for joints, 1Hz for battery/estop."""
        joint_counter = 0
        while self._running:
            time.sleep(0.1)  # 10Hz
            joint_counter += 1

            # Publish joints
            with self._lock:
                if self._joint_data:
                    joints = []
                    for motor_id, data in self._joint_data.items():
                        name = _ALL_JOINTS.get(motor_id, f"motor_{motor_id}")
                        joints.append({
                            "idx": motor_id,
                            "name": name,
                            "q": data["pos"],
                            "dq": data["speed"],
                            "current": data["current"],
                            "temp": data["temp"],
                        })
                    payload = json.dumps({"joints": joints})
                    msg = String()
                    msg.data = payload
                    self._pub_joints.publish(msg)

            # 1Hz for battery/estop/force
            if joint_counter % 10 == 0:
                with self._lock:
                    if self._battery:
                        msg = String()
                        msg.data = json.dumps(self._battery)
                        self._pub_battery.publish(msg)
                    if self._estop:
                        msg = String()
                        msg.data = json.dumps(self._estop)
                        self._pub_estop.publish(msg)

            # 5Hz for force
            if joint_counter % 2 == 0:
                with self._lock:
                    if self._force_left or self._force_right:
                        msg = String()
                        msg.data = json.dumps({"left": self._force_left, "right": self._force_right})
                        self._pub_force.publish(msg)

    def dispatch(self, action_or_tool: str, args: dict) -> dict:
        # Resource tool: model
        if action_or_tool == "model":
            try:
                urdf = self._urdf_path.read_text()
                return {"urdf": urdf}
            except FileNotFoundError:
                return {"error": "URDF file not found"}
        # Sensor tools return state
        if action_or_tool == "joints":
            with self._lock:
                return {"joints": list(self._joint_data.values())}
        if action_or_tool == "battery":
            with self._lock:
                return self._battery or {"state": "no_data"}
        if action_or_tool == "estop":
            with self._lock:
                return self._estop or {"state": "no_data"}
        if action_or_tool == "force_sensor":
            with self._lock:
                return {"left": self._force_left, "right": self._force_right}
        # start/stop/info
        if action_or_tool == "start":
            return {"state": "running"}
        if action_or_tool == "stop":
            return {"state": "idle"}
        if action_or_tool == "info":
            tool_name = args.get("_tool_name", "joints")
            topic_map = {
                "joints": self._topic_joints,
                "battery": self._topic_battery,
                "estop": self._topic_estop,
                "force_sensor": self._topic_force,
            }
            topic = topic_map.get(tool_name, self._topic_joints)
            fmt = "sensor/skeleton" if tool_name == "joints" else "data/json"
            return {"state": "running", "topic_out": [{"topic": topic, "format": fmt}]}
        return {"error": f"unknown action: {action_or_tool}"}


# ══════════════════════════════════════════════════════════════════════════════
# CameraPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class CameraPlugin:
    """Orbbec 头部 RGB 相机 — 独立编码线程避免阻塞executor"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/camera/head"
        self._running = False
        self._frame_queue = None  # Will hold latest frame only

        self._sub_node = Node("tianyi2_camera_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_camera_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

    def get_tool(self) -> dict:
        return {
            "name": "camera_head",
            "type": "sensor",
            "description": "天轶2.0 头部相机 (Orbbec RGB) — 彩色图像流",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
        }

    def start(self):
        self._running = True

        # Ensure Orbbec camera service is running
        self._ensure_orbbec_service()

        try:
            from sensor_msgs.msg import Image, CompressedImage
            import numpy as np
            import cv2

            self._np = np
            self._cv2 = cv2
            self._latest_frame = None  # Only keep latest frame
            self._frame_lock = threading.Lock()

            # Publish JPEG as CompressedImage
            self._pub = self._pub_node.create_publisher(CompressedImage, self._topic, _LOW_LAT_QOS)

            # Subscribe - callback just grabs the frame, doesn't encode
            self._sub_node.create_subscription(
                Image, "/ob_camera_head/color/image_raw", self._on_image_grab, _RELIABLE_QOS)

            # Separate encoding thread - avoids blocking executor
            self._encode_thread = threading.Thread(target=self._encode_loop, daemon=True)
            self._encode_thread.start()

            print("[CameraPlugin] subscription + encode thread created")
        except ImportError as e:
            print(f"[CameraPlugin] WARNING: import failed ({e})")

    def _ensure_orbbec_service(self):
        """Configure and start the host's Orbbec service through ``nsenter``.

        The camera runs on the host because it owns the USB device.  Each
        container start therefore makes the vendor startup script idempotently
        request PointCloud2, accelerometer, gyroscope, and a 640x480 depth
        profile before ensuring the service is active. This is deliberately
        runtime setup, not a Docker build step: a Dockerfile cannot alter a
        new machine's systemd service or access its camera.
        """
        import subprocess
        try:
            changed = self._configure_orbbec_startup()
            # Use nsenter to run systemctl on host PID 1's namespace
            result = subprocess.run(
                ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
                 "systemctl", "is-active", "orbbec_head.service"],
                capture_output=True, text=True, timeout=5)
            if result.stdout.strip() == "active" and not changed:
                print("[CameraPlugin] orbbec_head.service already active")
                return
            # Restart applies a changed host startup script; start handles an
            # inactive service without unnecessarily interrupting a live one.
            action = "restart" if result.stdout.strip() == "active" else "start"
            subprocess.run(
                ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
                 "systemctl", action, "orbbec_head.service"],
                check=True, capture_output=True, text=True, timeout=15)
            print(f"[CameraPlugin] orbbec_head.service {action}ed via nsenter")
        except Exception as e:
            print(f"[CameraPlugin] WARNING: could not start orbbec service ({e})")

    @staticmethod
    def _configure_orbbec_startup():
        """Enable the Orbbec streams in the host vendor startup script.

        This only changes the ``headty`` launch command and is idempotent, so
        it is safe to execute every time a freshly-created driver container
        starts.  The host path matches Tianyi's factory service definition.
        """
        import subprocess

        script = "/home/nvidia/data/scripts/start_orbbec_camera.sh"
        launch = "ros2 launch orbbec_camera head_330_ty.launch.py"
        desired = (f"{launch} enable_point_cloud:=true "
                   "enable_accel:=true enable_gyro:=true "
                   "depth_width:=1280 depth_height:=720 depth_fps:=15")
        updater = (
            "from pathlib import Path\n"
            f"path = Path({script!r})\n"
            "lines = path.read_text().splitlines(keepends=True)\n"
            f"launch = {launch!r}\n"
            f"desired = {desired!r}\n"
            "for index, line in enumerate(lines):\n"
            "    if line.lstrip().startswith(launch):\n"
            "        updated = line[:len(line) - len(line.lstrip())] + desired + '\\n'\n"
            "        if line == updated:\n"
            "            print('unchanged')\n"
            "        else:\n"
            "            lines[index] = updated\n"
            "            path.write_text(''.join(lines))\n"
            "            print('changed')\n"
            "        break\n"
            "else:\n"
            "    raise SystemExit('headty Orbbec launch command not found')\n"
        )
        result = subprocess.run(
            ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
             "python3", "-c", updater],
            check=True, capture_output=True, text=True, timeout=10)
        changed = result.stdout.strip() == "changed"
        if changed:
            print("[CameraPlugin] enabled Orbbec point cloud, IMU, and 1280x720@15 depth stream")
        return changed

    def stop(self):
        self._running = False

    def _on_image_grab(self, msg):
        """Callback: just grab the latest frame, don't encode here (non-blocking)."""
        if not self._running:
            return
        with self._frame_lock:
            self._latest_frame = msg

    def _encode_loop(self):
        """Separate thread: encode and publish the latest frame. Always processes newest, skips stale."""
        np = self._np
        cv2 = self._cv2
        from sensor_msgs.msg import CompressedImage

        while self._running:
            # Grab latest frame atomically
            with self._frame_lock:
                msg = self._latest_frame
                self._latest_frame = None  # Mark as consumed
            if msg is None:
                time.sleep(0.005)  # 5ms poll
                continue
            try:
                # Zero-copy: np.frombuffer on array.array directly (no bytes() copy)
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])
                out = CompressedImage()
                out.format = "jpeg"
                out.data = bytes(jpeg)
                self._pub.publish(out)
            except Exception as e:
                print(f"[CameraPlugin] encode error: {e}", flush=True)

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# Additional sensors / indicators
# ══════════════════════════════════════════════════════════════════════════════

class _JsonSensor:
    """Small base class for a domain-0 subscription bridged as JSON on domain 42."""
    _format = "data/json"

    def _tool(self, name, description):
        return {"name": name, "type": "sensor", "description": description,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic, "format": self._format}]}

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": self._format}]}
        return {"state": "running" if self._running else "idle"}


class ImuPlugin(_JsonSensor):
    """Bridge the head Orbbec camera's accelerometer and gyroscope to Agent Core."""
    def __init__(self, plugin_config, namespace, ros2):
        self._running = False
        self._topic = f"/{namespace}/state/imu"
        self._last_pub = 0.0
        self._latest = {
            "available": False,
            "timestamp_ms": int(time.time() * 1000),
            "source": None,
            "reason": "waiting_for_upstream_imu",
        }
        self._subscriptions = []  # Keep rclpy subscriptions alive for the plugin lifetime.
        self._camera_acceleration = None
        self._camera_acceleration_timestamp_ms = None
        self._camera_angular_velocity = None
        self._camera_angular_velocity_timestamp_ms = None
        self._sub_node = Node("tianyi2_imu_sub", context=ros2.ctx_tianyi)
        self._pub_node = Node("tianyi2_imu_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self):
        tool = self._tool("imu", "天轶2.0 头部 Orbbec IMU — 相机实际加速度与角速度")
        tool["multiInstance"] = False
        return tool

    def start(self):
        """Subscribe to the Orbbec streams enabled by ``orbbec_head.service``.

        Orbbec publishes acceleration and angular velocity separately as
        ``sensor_msgs/Imu``.  These sensor publishers use BEST_EFFORT QoS, so
        the domain-0 subscription must use the matching low-latency profile.
        Only fields physically supplied by either stream are forwarded to the
        Agent Core card; no synthetic orientation, Euler angle, or zero values
        are emitted.
        """
        from sensor_msgs.msg import Imu as RosImu
        self._running = True
        self._subscriptions = [
            self._sub_node.create_subscription(
                RosImu, "/ob_camera_head/accel/sample", self._on_camera_accel, _LOW_LAT_QOS),
            self._sub_node.create_subscription(
                RosImu, "/ob_camera_head/gyro/sample", self._on_camera_gyro, _LOW_LAT_QOS),
        ]
        # Match StatePlugin/force_sensor behaviour: publish a snapshot even
        # while the vendor IMU stream is unavailable, so Agent Core renders a
        # useful card state instead of an empty panel.
        self._pub_thread = threading.Thread(target=self._publish_snapshot_loop, daemon=True)
        self._pub_thread.start()
        print("[ImuPlugin] subscribed: /ob_camera_head/accel/sample, /ob_camera_head/gyro/sample")

    def stop(self):
        self._running = False

    def _publish_camera_imu(self):
        """Publish only the samples actually received from the camera."""
        if not self._running:
            return
        timestamps = [ts for ts in (
            self._camera_acceleration_timestamp_ms,
            self._camera_angular_velocity_timestamp_ms,
        ) if ts is not None]
        if not timestamps:
            return
        payload = {"available": True, "timestamp_ms": max(timestamps)}
        if self._camera_acceleration is not None:
            payload["linear_acceleration"] = self._camera_acceleration
        if self._camera_angular_velocity is not None:
            payload["angular_velocity"] = self._camera_angular_velocity
        self._latest = payload

        # The dashboard only needs the latest state.  Bound transport to 30 Hz
        # so a high-rate IMU cannot congest the domain-42 bridge.
        now = time.monotonic()
        if now - self._last_pub < 1.0 / 30.0:
            return
        self._last_pub = now
        out = String()
        out.data = json.dumps(self._latest)
        self._pub.publish(out)

    def _publish_snapshot_loop(self):
        while self._running:
            snapshot = dict(self._latest)
            snapshot["timestamp_ms"] = int(time.time() * 1000)
            out = String()
            out.data = json.dumps(snapshot)
            self._pub.publish(out)
            time.sleep(0.5)

    def _on_camera_accel(self, msg):
        self._camera_acceleration = {
            "x": msg.linear_acceleration.x,
            "y": msg.linear_acceleration.y,
            "z": msg.linear_acceleration.z,
        }
        self._camera_acceleration_timestamp_ms = (
            msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000)
        self._publish_camera_imu()

    def _on_camera_gyro(self, msg):
        self._camera_angular_velocity = {
            "x": msg.angular_velocity.x,
            "y": msg.angular_velocity.y,
            "z": msg.angular_velocity.z,
        }
        self._camera_angular_velocity_timestamp_ms = (
            msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000)
        self._publish_camera_imu()

    def dispatch(self, action, args):
        if action in ("read", "get", "imu"):
            # Keep sensor values at the top level like StatePlugin's
            # force_sensor, which Agent Core renders directly.
            return dict(self._latest)
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": self._format}]}
        return {"state": "running" if self._running else "idle"}


class HandStatePlugin(_JsonSensor):
    """Bridge both Inspire hands' feedback and error arrays."""
    def __init__(self, plugin_config, namespace, ros2):
        self._running, self._lock = False, threading.Lock()
        self._state = {"left": {}, "right": {}}
        self._topic = f"/{namespace}/state/hand"
        self._sub_node = Node("tianyi2_hand_state_sub", context=ros2.ctx_tianyi)
        self._pub_node = Node("tianyi2_hand_state_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node); ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self): return self._tool("hand_state", "天轶2.0 灵巧手状态 — 双手六指位置、速度、电流与错误码")
    def start(self):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import UInt32MultiArray
        self._running = True
        for side in ("left", "right"):
            self._sub_node.create_subscription(JointState, f"/inspire_hand/state/{side}_hand", lambda m, s=side: self._on_state(s, m), _RELIABLE_QOS)
            self._sub_node.create_subscription(UInt32MultiArray, f"/inspire_hand/error/{side}_hand", lambda m, s=side: self._on_error(s, m), _RELIABLE_QOS)
    def stop(self): self._running = False
    def _on_state(self, side, msg):
        with self._lock:
            self._state[side].update({"name": list(msg.name), "position": list(msg.position), "velocity": list(msg.velocity), "effort": list(msg.effort)})
        self._publish()
    def _on_error(self, side, msg):
        with self._lock: self._state[side]["errors"] = list(msg.data)
        self._publish()
    def _publish(self):
        if not self._running: return
        with self._lock: data = json.dumps(self._state)
        out = String(); out.data = data; self._pub.publish(out)


class DepthCameraPlugin:
    """Bridge only the newest Orbbec Z16 frame to Agent Core.

    Depth frames are large and the dashboard only needs the current image.  The
    domain-0 callback therefore never serializes or publishes a frame: it only
    replaces the pending frame.  A domain-42 timer publishes that newest frame,
    so a slow DDS consumer cannot make the input executor drain stale images.
    """
    def __init__(self, plugin_config, namespace, ros2):
        self._running = False; self._topic = f"/{namespace}/camera/head/depth"
        # A 640x480 Z16 frame is 614 KiB.  The dashboard's DDS → WebSocket
        # path is intentionally latest-frame-only, but it still must not be
        # fed faster than the browser can decode and paint it.
        self._max_hz = max(1.0, min(float(plugin_config.get("hz", 8)), 15.0))
        self._last_published_at = 0.0
        self._forwarded_frames = 0
        self._latest_image = None
        self._latest_image_lock = threading.Lock()
        self._subscription = None
        self._publish_timer = None
        self._sub_node = Node("tianyi2_depth_sub", context=ros2.ctx_tianyi)
        self._pub_node = Node("tianyi2_depth_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node); ros2.executor_core.add_node(self._pub_node)
    def get_tool(self):
        return {"name": "camera_depth", "type": "sensor", "description": "天轶2.0 Orbbec 头部深度图（Z16）",
                "inputSchema": {"type": "object", "properties": {}}, "topic_out": [{"topic": self._topic, "format": "image/depth-z16"}]}
    def start(self):
        from sensor_msgs.msg import Image
        import numpy as np
        self._np = np
        latest_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._running = True
        self._pub = self._pub_node.create_publisher(Image, self._topic, latest_qos)
        self._subscription = self._sub_node.create_subscription(
            Image, "/ob_camera_head/depth/image_raw", self._on_image, latest_qos)
        # The timer, rather than the input callback, owns all conversion and
        # publishing work.  At most one source frame can be pending.
        self._publish_timer = self._pub_node.create_timer(1.0 / self._max_hz, self._publish_latest)
        print(f"[DepthCameraPlugin] forwarding newest Z16 frame at <= {self._max_hz:g} Hz")
    def stop(self):
        self._running = False
        with self._latest_image_lock:
            self._latest_image = None

    def _on_image(self, msg):
        if not self._running or msg.encoding not in ("16UC1", "mono16"): return
        # Never block the domain-0 executor on serialization or a slow
        # domain-42 consumer.  Replacing this reference deliberately drops any
        # stale frame that has not yet been published.
        with self._latest_image_lock:
            self._latest_image = msg

    def _publish_latest(self):
        if not self._running:
            return
        with self._latest_image_lock:
            msg, self._latest_image = self._latest_image, None
        if msg is None:
            return
        # Keep the source camera at 1280x720 for its useful field of view, but
        # bridge only the center 640x480 Z16 card.  The slice is a NumPy view;
        # tobytes() below is the sole output-payload copy.
        dashboard = self._to_dashboard_depth(msg)
        if dashboard is None:
            return

        # Rebuild the message for the independent domain-42 context.
        from sensor_msgs.msg import Image
        out = Image()
        out.header = msg.header
        out.height, out.width, out.encoding = 480, 640, "16UC1"
        out.is_bigendian, out.step, out.data = 0, 1280, dashboard.tobytes()
        self._pub.publish(out)
        self._last_published_at = time.monotonic()
        self._forwarded_frames += 1

    def _to_dashboard_depth(self, msg):
        """Return a 640x480 uint16 depth image, or None for malformed input."""
        if msg.encoding not in ("16UC1", "mono16") or msg.is_bigendian:
            return None
        width, height, step = int(msg.width), int(msg.height), int(msg.step)
        if width <= 0 or height <= 0 or step < width * 2:
            return None
        raw = self._np.frombuffer(msg.data, dtype=self._np.uint8)
        needed = height * step
        if raw.size < needed:
            return None
        # Respect source row stride before interpreting its pixels as uint16.
        depth = raw[:needed].reshape(height, step)[:, :width * 2].view(self._np.uint16).reshape(height, width)
        if width == 1280 and height == 720:
            return depth[120:600, 320:960]
        # Retain a no-resize compatibility path for an old 640x480 profile.
        if width == 640 and height == 480:
            return depth
        return None
    def dispatch(self, action, args):
        return {"state": "running" if self._running else "idle", "topic_out": [{"topic": self._topic, "format": "image/depth-z16"}]}


class PointCloudPlugin:
    """Publish the newest native Orbbec PointCloud2 as a sparse renderer cloud."""
    _format = "sensor/pointcloud"
    def __init__(self, plugin_config, namespace, ros2):
        self._running = False; self._topic = f"/{namespace}/camera/head/points"
        self._max_hz = max(1.0, min(float(plugin_config.get("hz", 5)), 8.0))
        self._max_points = max(100, min(int(plugin_config.get("max_points", 10000)), 12000))
        self._min_points = max(1, min(int(plugin_config.get("min_points", 100)), self._max_points))
        self._min_distance_m = max(0.0, float(plugin_config.get("min_distance_m", 0.10)))
        self._max_distance_m = max(self._min_distance_m, float(plugin_config.get("max_distance_m", 8.0)))
        self._latest_cloud = None; self._latest_cloud_seq = 0; self._published_cloud_seq = 0
        self._cloud_lock = threading.Lock(); self._publish_timer = None
        # The renderer's horizontal grid is Y=0.  Gravity alignment fixes the
        # orientation but leaves the camera as the origin; shift upward by the
        # head camera's floor height so the physical floor is at Y=0.
        self._floor_offset_m = max(-3.0, min(float(plugin_config.get("floor_offset_m", 1.50)), 3.0))
        self._gravity_world = None; self._gravity_lock = threading.Lock()
        self._sub_node = Node("tianyi2_points_sub", context=ros2.ctx_tianyi); self._pub_node = Node("tianyi2_points_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node); ros2.executor_core.add_node(self._pub_node)
    def get_tool(self):
        return {"name": "camera_pointcloud", "type": "sensor", "description": "天轶2.0 Orbbec 头部彩色点云（限频、限点）", "inputSchema": {"type": "object", "properties": {}}, "topic_out": [{"topic": self._topic, "format": self._format}]}
    def start(self):
        from sensor_msgs.msg import PointCloud2, Imu
        from std_msgs.msg import UInt8MultiArray
        latest_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1,
                                durability=DurabilityPolicy.VOLATILE)
        self._running = True; self._pub = self._pub_node.create_publisher(UInt8MultiArray, self._topic, latest_qos)
        self._sub_node.create_subscription(PointCloud2, "/ob_camera_head/depth/points", self._on_cloud, latest_qos)
        self._sub_node.create_subscription(Imu, "/ob_camera_head/accel/sample", self._on_accel, latest_qos)
        self._publish_timer = self._pub_node.create_timer(1.0 / self._max_hz, self._publish_latest)
        print(f"[PointCloudPlugin] native newest-frame publisher at {self._max_hz:g} Hz, <= {self._max_points} points")
    def stop(self):
        self._running = False
        with self._cloud_lock: self._latest_cloud = None
    def _on_accel(self, msg):
        """Estimate display-frame up from the camera's stationary IMU vector."""
        # Optical raw coordinates are (right, down, forward); the renderer's
        # world coordinates are (right, up, backward) before leveling.
        g = (msg.linear_acceleration.x, -msg.linear_acceleration.y, -msg.linear_acceleration.z)
        magnitude = math.sqrt(sum(v * v for v in g))
        if not 8.0 <= magnitude <= 11.5: return  # reject dynamic acceleration
        g = tuple(v / magnitude for v in g)
        with self._gravity_lock:
            previous = self._gravity_world
            if previous is None:
                self._gravity_world = g
            else:
                # Low-pass the gravity direction to avoid point-cloud wobble.
                mixed = tuple(0.95 * old + 0.05 * new for old, new in zip(previous, g))
                norm = math.sqrt(sum(v * v for v in mixed))
                self._gravity_world = tuple(v / norm for v in mixed)

    def _gravity_snapshot(self):
        with self._gravity_lock: return self._gravity_world

    @staticmethod
    def _to_renderer_frame(x, y, z, gravity=None, floor_offset_m=0.0):
        """Map optical points to a gravity-levelled, floor-referenced renderer frame."""
        # Renderer map is (input_y, -input_z, -input_x), so this packed form
        # yields world (x, -y, -z): right, up, backward from optical raw XYZ.
        wx, wy, wz = x, -y, -z
        if gravity is not None:
            gx, gy, gz = gravity
            # Rodrigues rotation taking measured up/gravity to world +Y.
            vx, vy, vz = -gz, 0.0, gx  # gravity × (0, 1, 0)
            sine_sq = vx * vx + vy * vy + vz * vz
            cosine = gy
            if sine_sq > 1e-8:
                cross_x = vy * wz - vz * wy
                cross_y = vz * wx - vx * wz
                cross_z = vx * wy - vy * wx
                cross2_x = vy * cross_z - vz * cross_y
                cross2_y = vz * cross_x - vx * cross_z
                cross2_z = vx * cross_y - vy * cross_x
                factor = (1.0 - cosine) / sine_sq
                wx += cross_x + factor * cross2_x
                wy += cross_y + factor * cross2_y
                wz += cross_z + factor * cross2_z
        wy += floor_offset_m
        # Invert the renderer map above to pack the leveled world point.
        return -wz, wx, -wy
    def _on_cloud(self, msg):
        """Ingress is deliberately O(1): overwrite the one pending native frame."""
        if not self._running: return
        with self._cloud_lock:
            self._latest_cloud = msg
            self._latest_cloud_seq += 1

    def _publish_latest(self):
        if not self._running: return
        with self._cloud_lock:
            if self._latest_cloud_seq == self._published_cloud_seq: return
            msg, sequence = self._latest_cloud, self._latest_cloud_seq
        payload = self._pack_cloud(msg)
        # Consume malformed or sparse clouds too.  They must not repeatedly
        # consume timer CPU, and they must not replace the browser's last
        # valid frame with an empty scene.
        self._published_cloud_seq = sequence
        if payload is None: return
        # If a newer frame arrived while this one was packed, its sequence is
        # still pending for the following tick.
        from std_msgs.msg import UInt8MultiArray
        # rclpy expands a ``bytes`` object element-by-element when assigning
        # it to UInt8MultiArray.data.  For a 120 KiB cloud that takes tens of
        # seconds and blocks the shared Domain-42 executor (including depth).
        # array('B') is the generated message's native uint8 container.
        out = UInt8MultiArray(); out.data = array("B", payload); self._pub.publish(out)

    def _pack_cloud(self, msg):
        """Vector-read PointCloud2 XYZ, filter/sample it, then pack bytes once."""
        np = __import__("numpy")
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")) or msg.point_step <= 0: return None
        offsets = [fields[name].offset for name in ("x", "y", "z")]
        if any(offset < 0 or offset + 4 > msg.point_step for offset in offsets): return None
        byte_order = ">" if msg.is_bigendian else "<"
        dtype = np.dtype({"names": ("x", "y", "z"), "formats": (byte_order + "f4",) * 3,
                          "offsets": offsets, "itemsize": msg.point_step})
        count = int(msg.width) * int(msg.height)
        if count <= 0 or len(msg.data) < int(msg.row_step) * int(msg.height): return None
        try:
            cloud = np.ndarray((int(msg.height), int(msg.width)), dtype=dtype, buffer=msg.data,
                               strides=(int(msg.row_step), int(msg.point_step))).reshape(-1)
        except (TypeError, ValueError):
            return None
        xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype("<f4", copy=False)
        distance = np.linalg.norm(xyz, axis=1)
        valid = np.isfinite(xyz).all(axis=1) & (distance >= self._min_distance_m) & (distance <= self._max_distance_m)
        xyz = xyz[valid]
        if len(xyz) < self._min_points: return None
        if len(xyz) > self._max_points:
            xyz = xyz[np.linspace(0, len(xyz) - 1, self._max_points, dtype=np.intp)]
        # Optical XYZ -> renderer packed coordinates, vectorized.  This is
        # equivalent to _to_renderer_frame, including optional gravity level.
        world = np.empty_like(xyz); world[:, 0] = xyz[:, 0]; world[:, 1] = -xyz[:, 1]; world[:, 2] = -xyz[:, 2]
        gravity = self._gravity_snapshot()
        if gravity is not None:
            g = np.asarray(gravity, dtype=np.float32); axis = np.array([-g[2], 0.0, g[0]], dtype=np.float32)
            sine_sq = float(axis @ axis)
            if sine_sq > 1e-8:
                cross = np.cross(axis, world); world += cross + ((1.0 - g[1]) / sine_sq) * np.cross(axis, cross)
        world[:, 1] += self._floor_offset_m
        packed_xyz = np.empty_like(world); packed_xyz[:, 0] = -world[:, 2]; packed_xyz[:, 1] = world[:, 0]; packed_xyz[:, 2] = -world[:, 1]
        return np.asarray((12, len(packed_xyz)), dtype="<u4").tobytes() + np.ascontiguousarray(packed_xyz, dtype="<f4").tobytes()
    def dispatch(self, action, args): return {"state": "running" if self._running else "idle", "topic_out": [{"topic": self._topic, "format": self._format}]}


class LightPlugin:
    """Safe semantic system-light control; no raw vendor command is exposed."""
    _commands = {"standby": 99, "service_wait": 20, "service_ready": 22, "warning": 12, "warning_clear": 13, "error": 10, "error_clear": 11}
    def __init__(self, plugin_config, namespace, ros2):
        self._pub_node = Node("tianyi2_light_pub", context=ros2.ctx_tianyi); ros2.executor_tianyi.add_node(self._pub_node); self._pub = None
    def get_tool(self):
        return {"name": "light", "type": "actuator", "description": "天轶2.0 系统状态灯效", "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": list(self._commands)}}, "required": ["action"], "x-action-params": {k: {"params": [], "description": k} for k in self._commands}}}
    def start(self):
        from bodyctrl_msgs.msg import LightCtrl
        self._pub = self._pub_node.create_publisher(LightCtrl, "/xsys/light/ctrl", _RELIABLE_QOS)
    def stop(self): pass
    def dispatch(self, action, args):
        if action not in self._commands: return {"error": f"unknown action: {action}"}
        from bodyctrl_msgs.msg import LightCtrl
        msg = LightCtrl(); msg.cmd = self._commands[action]; msg.caller_id = "phanthy-motus"; msg.caller_msg = f"Agent Core: {action}"; self._pub.publish(msg)
        return {"state": action}


# ══════════════════════════════════════════════════════════════════════════════
# AsrPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class AsrPlugin:
    """语音识别接口 (lyre ASR) — 覆盖文档 5.9.7: 唤醒词 + 语音识别结果 + ASR事件"""

    # ASR 事件名映射 (文档 5.9.7 AsrEvent 定义)
    _ASR_EVENT_NAMES = {
        2: "error",          # 出错，arg1=错误码
        3: "state",          # 服务状态变更
        4: "wakeup",         # 唤醒
        5: "sleep",          # 休眠
        6: "vad",            # VAD 检测
        10: "pre_sleep",     # 准备休眠
        13: "connected",     # 与服务端建立连接
        14: "disconnected",  # 与服务端断开连接
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/asr"
        self._running = False

        self._sub_node = Node("tianyi2_asr_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_asr_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "asr",
            "type": "sensor",
            "description": (
                "天轶2.0 语音识别接口 (文档 5.9.7) — "
                "唤醒词检测(AsrKeyword) + 语音识别结果(AsrIat) + ASR状态事件(AsrEvent) + "
                "兼容已弃用的 /xunfei/aiui_msg"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        try:
            from lyre_msgs.msg import AsrIat, AsrKeyword, AsrEvent

            # 1. 监听语音识别结果 — /audio_asr/iat (AsrIat: id + text)
            self._sub_node.create_subscription(
                AsrIat, "/audio_asr/iat", self._on_iat, _RELIABLE_QOS)

            # 2. 监听唤醒事件 — /audio_asr/keyword (AsrKeyword: keyword + angle)
            self._sub_node.create_subscription(
                AsrKeyword, "/audio_asr/keyword", self._on_keyword, _RELIABLE_QOS)

            # 3. 监听其它事件 — /audio_asr/event (AsrEvent: event + arg1 + arg2)
            self._sub_node.create_subscription(
                AsrEvent, "/audio_asr/event", self._on_event, _RELIABLE_QOS)

            # 4. 已弃用兼容 — /xunfei/aiui_msg (std_msgs/String)
            self._sub_node.create_subscription(
                String, "/xunfei/aiui_msg", self._on_aiui, _RELIABLE_QOS)

            print("[AsrPlugin] 4 subscriptions: iat, keyword, event, aiui_msg (doc 5.9.7)")
        except ImportError:
            # Fallback: 仅订阅 IAT，使用 String 类型
            self._sub_node.create_subscription(
                String, "/audio_asr/iat", self._on_iat_string, _RELIABLE_QOS)
            print("[AsrPlugin] fallback: String subscription on /audio_asr/iat")

    def stop(self):
        self._running = False

    # ── 发布辅助 ────────────────────────────────────────────────────────────

    def _publish(self, data: dict):
        if not self._running:
            return
        out = String()
        out.data = json.dumps(data)
        self._pub.publish(out)

    # ── 回调: 语音识别结果 ───────────────────────────────────────────────────

    def _on_iat(self, msg):
        """/audio_asr/iat → AsrIat: 实时语音转文字结果"""
        self._publish({
            "type": "iat",
            "id": msg.id,
            "text": msg.text,
        })

    def _on_iat_string(self, msg):
        """Fallback: String 类型的 IAT 消息"""
        self._publish({
            "type": "iat",
            "text": msg.data,
        })

    # ── 回调: 唤醒词 ───────────────────────────────────────────────────────

    def _on_keyword(self, msg):
        """/audio_asr/keyword → AsrKeyword: 唤醒词 + 声源角度"""
        self._publish({
            "type": "keyword",
            "keyword": msg.keyword,
            "angle_deg": msg.angle,
        })

    # ── 回调: ASR 事件 ──────────────────────────────────────────────────────

    def _on_event(self, msg):
        """/audio_asr/event → AsrEvent: ASR 状态事件 (唤醒/休眠/错误/连接等)"""
        self._publish({
            "type": "event",
            "event": msg.event,
            "event_name": self._ASR_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "arg1": msg.arg1,
            "arg2": msg.arg2,
        })

    # ── 回调: 已弃用兼容 ─────────────────────────────────────────────────────

    def _on_aiui(self, msg):
        """/xunfei/aiui_msg → 已弃用，透传原始数据"""
        self._publish({
            "type": "aiui",
            "raw": msg.data,
        })

    # ── dispatch ────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# LyreEventPlugin (sensor) — lyre 语音包事件/状态采集
# ══════════════════════════════════════════════════════════════════════════════

class LyreEventPlugin:
    """lyre 语音包事件采集 — 唤醒词/ASR事件/播放事件/播放进度/TTS事件"""

    _ASR_EVENT_NAMES = {2: "error", 3: "state", 4: "wakeup", 5: "sleep",
                        6: "vad", 10: "pre_sleep", 13: "connected", 14: "disconnected"}
    _PLAY_EVENT_NAMES = {0: "started", 1: "completed", 2: "stopped", 3: "cancelled", 4: "failed"}

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._running = False

        self._sub_node = Node("tianyi2_lyre_event_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_lyre_event_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

        self._topics = {
            "lyre_keyword": f"/{namespace}/lyre/keyword",
            "lyre_asr_event": f"/{namespace}/lyre/asr_event",
            "lyre_play_event": f"/{namespace}/lyre/play_event",
            "lyre_play_progress": f"/{namespace}/lyre/play_progress",
            "lyre_tts_event": f"/{namespace}/lyre/tts_event",
        }
        self._pubs = {}

    def get_tools(self) -> list:
        return [
            {"name": "lyre_keyword", "type": "sensor",
             "description": "天轶2.0 语音唤醒关键词 — 检测到的唤醒词及声源角度",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_keyword"], "format": "data/json"}]},
            {"name": "lyre_asr_event", "type": "sensor",
             "description": "天轶2.0 ASR 状态事件 — 唤醒/休眠/VAD/连接状态",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_asr_event"], "format": "data/json"}]},
            {"name": "lyre_play_event", "type": "sensor",
             "description": "天轶2.0 音频播放事件 — 开始/完成/停止/取消/失败",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_play_event"], "format": "data/json"}]},
            {"name": "lyre_play_progress", "type": "sensor",
             "description": "天轶2.0 音频播放进度 — 当前位置和总时长(秒)",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_play_progress"], "format": "data/json"}]},
            {"name": "lyre_tts_event", "type": "sensor",
             "description": "天轶2.0 TTS 合成事件 — 合成开始/完成/停止/取消/失败",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_tts_event"], "format": "data/json"}]},
        ]

    def start(self):
        self._running = True
        for key, topic in self._topics.items():
            self._pubs[key] = self._pub_node.create_publisher(String, topic, _RELIABLE_QOS)

        try:
            from lyre_msgs.msg import AsrKeyword, AsrEvent, PlayEvent, PlayProgress, TtsEvent
            self._sub_node.create_subscription(
                AsrKeyword, "/audio_asr/keyword", self._on_keyword, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                AsrEvent, "/audio_asr/event", self._on_asr_event, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                PlayEvent, "/audio_play/event", self._on_play_event, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                PlayProgress, "/audio_play/progress", self._on_play_progress, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                TtsEvent, "/audio_tts/event", self._on_tts_event, _RELIABLE_QOS)
            print("[LyreEventPlugin] 5 subscriptions created")
        except ImportError:
            print("[LyreEventPlugin] WARNING: lyre_msgs not available, no subscriptions created")

    def stop(self):
        self._running = False

    def _publish(self, key: str, data: dict):
        if not self._running:
            return
        out = String()
        out.data = json.dumps(data)
        self._pubs[key].publish(out)

    def _on_keyword(self, msg):
        self._publish("lyre_keyword", {"keyword": msg.keyword, "angle": msg.angle})

    def _on_asr_event(self, msg):
        self._publish("lyre_asr_event", {
            "event": msg.event,
            "event_name": self._ASR_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "arg1": msg.arg1, "arg2": msg.arg2,
        })

    def _on_play_event(self, msg):
        self._publish("lyre_play_event", {
            "sid": msg.sid, "seq": msg.seq,
            "event": msg.event,
            "event_name": self._PLAY_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "message": msg.message,
        })

    def _on_play_progress(self, msg):
        self._publish("lyre_play_progress", {
            "sid": msg.sid, "seq": msg.seq,
            "position": msg.position, "duration": msg.duration,
        })

    def _on_tts_event(self, msg):
        self._publish("lyre_tts_event", {
            "sid": msg.sid, "seq": msg.seq,
            "event": msg.event,
            "event_name": self._PLAY_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "message": msg.message,
        })

    def dispatch(self, action: str, args: dict) -> dict:
        tool_name = args.get("_tool_name", "lyre_keyword")
        if tool_name not in self._topics:
            return {"error": f"unknown tool: {tool_name}"}
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topics[tool_name], "format": "data/json"}]}
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# NavStatePlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class NavStatePlugin:
    """底盘导航状态 — 位姿/速度 (轮询 Slamtec HTTP API)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._ns = namespace
        self._ros2 = ros2
        self._slamtec = slamtec_client
        self._topic = f"/{namespace}/nav/state"
        self._running = False

        self._pub_node = Node("tianyi2_nav_state_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "nav_state",
            "type": "sensor",
            "description": "天轶2.0 底盘导航状态 — 位姿(x,y,yaw)/速度 (Slamtec底盘, 2Hz)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        print("[NavStatePlugin] polling started")

    def stop(self):
        self._running = False

    def _poll_loop(self):
        while self._running:
            try:
                pose = self._slamtec.get_pose()
                speed = self._slamtec.get_speed()
                data = {"pose": pose, "speed": speed}
                msg = String()
                msg.data = json.dumps(data)
                self._pub.publish(msg)
            except Exception:
                pass
            time.sleep(0.5)  # 2Hz

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# Lidar2DPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class Lidar2DPlugin:
    """Slamtec 2D laser-scan card.

    The chassis REST endpoint returns polar points as ``angle`` (radians),
    ``distance`` (metres), and ``valid``.  Agent Core's lidar card consumes a
    JSON String topic, therefore this plugin turns them into robot-frame x/y
    metres before publishing into the Domain 42 data bus.
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._slamtec = slamtec_client
        self._topic = f"/{namespace}/sensor/lidar_2d"
        self._hz = max(1.0, min(float(plugin_config.get("hz", 10)), 15.0))
        self._max_points = max(100, min(int(plugin_config.get("max_points", 1440)), 3000))
        self._running = False
        self._last_frame: dict = {"timestamp_ms": 0, "points": [], "point_count": 0}

        self._pub_node = Node("tianyi2_lidar_2d_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "lidar_2d",
            "type": "sensor",
            "description": "思岚底盘二维激光雷达：实时极坐标扫描转换为底盘坐标系 x/y 点云",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "sensor/lidar-2d"}],
        }

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="tianyi-lidar-2d")
        self._thread.start()
        print(f"[Lidar2DPlugin] polling {self._topic} at {self._hz:g}Hz")

    def stop(self):
        self._running = False

    def _to_frame(self, scan: dict) -> dict:
        raw_points = scan.get("laser_points", []) if isinstance(scan, dict) else []
        valid_points = []
        for point in raw_points:
            if not isinstance(point, dict) or not point.get("valid", False):
                continue
            try:
                angle = float(point["angle"])
                distance = float(point["distance"])
            except (KeyError, TypeError, ValueError):
                continue
            if not (math.isfinite(angle) and math.isfinite(distance) and distance > 0.0):
                continue
            valid_points.append({"x": round(distance * math.cos(angle), 4),
                                 "y": round(distance * math.sin(angle), 4)})

        # Preserve the complete angular field-of-view while bounding payload and UI work.
        if len(valid_points) > self._max_points:
            stride = math.ceil(len(valid_points) / self._max_points)
            valid_points = valid_points[::stride]

        return {
            "timestamp_ms": int(time.time() * 1000),
            "frame_id": "laser",
            "source": "slamtec_rest:/api/core/system/v1/laserscan",
            "points": valid_points,
            "point_count": len(valid_points),
            "raw_point_count": len(raw_points),
        }

    def _poll_loop(self):
        interval = 1.0 / self._hz
        while self._running:
            started = time.monotonic()
            try:
                frame = self._to_frame(self._slamtec.get_laser_scan())
                self._last_frame = frame
                msg = String()
                msg.data = json.dumps(frame, separators=(",", ":"))
                self._pub.publish(msg)
            except Exception as e:
                print(f"[Lidar2DPlugin] scan read failed: {e}")
            time.sleep(max(0.0, interval - (time.monotonic() - started)))

    def dispatch(self, action: str, args: dict):
        if action == "info":
            # Older Agent Core releases discover and register topic streams
            # exclusively from the result of an ``info`` call.  Do not return
            # the bare point list here, otherwise the topic is invisible to
            # the dashboard's DDS/WebSocket bridge.
            return {
                "state": "running" if self._running else "idle",
                "data": self._last_frame,
                "topic_out": [{"topic": self._topic, "format": "sensor/lidar-2d"}],
            }
        if action in ("start", "read", "get", "lidar_2d"):
            # The deployed Agent Core lidar renderer only consumes
            # ``mcp_result.payload.result`` when it is a bare [{x, y}, ...]
            # array.  Keep the richer metadata on the DDS topic, but return
            # the renderer-compatible snapshot for the card's MCP action.
            return list(self._last_frame["points"])
        if action == "stop":
            return {"state": "idle"}
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# HeadPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class HeadPlugin:
    """头部3DOF位置控制 (roll/pitch/yaw)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_head_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None  # Lazy init

    def get_tool(self) -> dict:
        return {
            "name": "head",
            "type": "actuator",
            "description": "天轶2.0 头部控制 — 3DOF (yaw±90°, pitch±25°, roll±26°)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "look_at"],
                               "description": "控制动作"},
                    "yaw": {"type": "number", "description": "偏航角(度), 左正右负, 范围[-90, 90]"},
                    "pitch": {"type": "number", "description": "俯仰角(度), 下正上负, 范围[-25, 25]"},
                    "roll": {"type": "number", "description": "翻滚角(度), 范围[-26, 26]"},
                    "target": {"type": "string", "enum": ["forward", "left", "right", "up", "down"],
                               "description": "预设方向"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["yaw", "pitch", "roll"],
                                 "description": "移动头部到指定角度(度)"},
                    "look_at": {"params": ["target"],
                                "description": "看向预设方向"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/head/cmd_pos", _RELIABLE_QOS)
            print("[HeadPlugin] publisher created")
        except ImportError as e:
            print(f"[HeadPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_pos":
            yaw = args.get("yaw", 0)
            pitch = args.get("pitch", 0)
            roll = args.get("roll", 0)
            return self._send_head_pos(roll, pitch, yaw)
        elif action == "look_at":
            target = args.get("target", "forward")
            presets = {
                "forward": (0, 0, 0),
                "left": (45, 0, 0),
                "right": (-45, 0, 0),
                "up": (0, -20, 0),
                "down": (0, 20, 0),
            }
            yaw, pitch, roll = presets.get(target, (0, 0, 0))
            return self._send_head_pos(roll, pitch, yaw)
        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _send_head_pos(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> dict:
        if not self._publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            cmds = []
            for motor_id, deg in [(1, roll_deg), (2, pitch_deg), (3, yaw_deg)]:
                cmd = SetMotorPosition()
                cmd.name = motor_id
                cmd.pos = _deg2rad(deg)
                cmd.spd = 1.0  # rad/s
                cmd.cur = 3.0  # A (max current)
                cmds.append(cmd)
            msg.cmds = cmds
            self._publisher.publish(msg)
            return {"state": "moving", "yaw": yaw_deg, "pitch": pitch_deg, "roll": roll_deg}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# ArmPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class ArmPlugin:
    """双臂14DOF控制 (位置模式 / 力位混合)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_arm_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._pos_publisher = None
        self._ctrl_publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "description": "天轶2.0 双臂控制 — 每臂7DOF (肩3+肘1+腕3), 位置/力位混合模式",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "move_ctrl"],
                               "description": "控制模式"},
                    "side": {"type": "string", "enum": ["left", "right", "both"],
                             "description": "控制哪只手臂"},
                    "positions": {"type": "array", "items": {"type": "number"},
                                  "description": "7个关节角度(度): [肩pitch, 肩roll, 肩yaw, 肘pitch, 腕yaw, 腕pitch, 腕roll]"},
                    "speed": {"type": "number", "description": "运动速度(rad/s), 默认1.0"},
                    "kp": {"type": "array", "items": {"type": "number"},
                           "description": "位置增益(7个), 范围[0,2000]"},
                    "kd": {"type": "array", "items": {"type": "number"},
                           "description": "速度增益(7个), 范围[0,300]"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["side", "positions", "speed"],
                                 "description": "位置模式: 移动手臂关节到指定角度(度)"},
                    "move_ctrl": {"params": ["side", "positions", "kp", "kd"],
                                  "description": "力位混合模式: 指定位置+增益"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, CmdMotorCtrl
            self._pos_publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/arm/cmd_pos", _RELIABLE_QOS)
            self._ctrl_publisher = self._pub_node.create_publisher(
                CmdMotorCtrl, "/arm/cmd_ctrl", _RELIABLE_QOS)
            print("[ArmPlugin] publishers created")
        except ImportError as e:
            print(f"[ArmPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_pos":
            side = args.get("side", "left")
            positions = args.get("positions", [])
            speed = args.get("speed", 1.0)
            if len(positions) != 7:
                return {"error": "positions must have exactly 7 values (degrees)"}
            return self._send_pos(side, positions, speed)
        elif action == "move_ctrl":
            side = args.get("side", "left")
            positions = args.get("positions", [])
            kp = args.get("kp", [200] * 7)
            kd = args.get("kd", [20] * 7)
            if len(positions) != 7:
                return {"error": "positions must have exactly 7 values (degrees)"}
            return self._send_ctrl(side, positions, kp, kd)
        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _send_pos(self, side: str, positions_deg: list, speed: float) -> dict:
        if not self._pos_publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            cmds = []
            sides = []
            if side in ("left", "both"):
                sides.append(("left", 11))
            if side in ("right", "both"):
                sides.append(("right", 21))

            for side_name, base_id in sides:
                for i, deg in enumerate(positions_deg):
                    cmd = SetMotorPosition()
                    cmd.name = base_id + i
                    cmd.pos = _deg2rad(deg)
                    cmd.spd = speed
                    cmd.cur = 5.0
                    cmds.append(cmd)

            msg.cmds = cmds
            self._pos_publisher.publish(msg)
            return {"state": "moving", "side": side, "joints": len(cmds)}
        except Exception as e:
            return {"error": str(e)}

    def _send_ctrl(self, side: str, positions_deg: list, kp: list, kd: list) -> dict:
        if not self._ctrl_publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdMotorCtrl, MotorCtrl
            msg = CmdMotorCtrl()
            cmds = []
            sides = []
            if side in ("left", "both"):
                sides.append(("left", 11))
            if side in ("right", "both"):
                sides.append(("right", 21))

            for side_name, base_id in sides:
                for i, deg in enumerate(positions_deg):
                    cmd = MotorCtrl()
                    cmd.name = base_id + i
                    cmd.pos = _deg2rad(deg)
                    cmd.spd = 0.0
                    cmd.tor = 0.0
                    cmd.kp = kp[i] if i < len(kp) else 200.0
                    cmd.kd = kd[i] if i < len(kd) else 20.0
                    cmds.append(cmd)

            msg.cmds = cmds
            self._ctrl_publisher.publish(msg)
            return {"state": "moving", "side": side, "mode": "force_position"}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# WaistPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class WaistPlugin:
    """腰部偏航 + 腿部升降 (yaw + knee, 俯仰角已禁用)

    调用格式:
      - 腰偏航: {"action": "move_yaw", "yaw": 30, "speed": 0.5}
      - 膝升降: {"action": "move_knee", "knee": 10, "speed": 0.5}
      - 归零:   {"action": "set_zero", "target": "waist|knee|both"}
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_waist_cmd", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._pub_waist = None
        self._pub_leg = None

    def get_tool(self) -> dict:
        return {
            "name": "waist",
            "type": "actuator",
            "description": "天轶2.0 腰部偏航+腿部升降 — yaw (-160°~180°), knee (-23°~20°), 俯仰角已禁用",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_yaw", "move_knee", "set_zero"],
                               "description": "控制模式"},
                    "yaw": {"type": "number", "description": "腰偏航角(度), 范围[-160, 180], 默认0"},
                    "knee": {"type": "number", "description": "膝关节俯仰角(度), 范围[-23, 20], 默认0"},
                    "target": {"type": "string", "enum": ["waist", "knee", "both"],
                               "description": "归零目标: waist=腰归零, knee=腿归零, both=同时归零"},
                    "speed": {"type": "number", "description": "运动速度(rad/s), 默认0.5"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_yaw": {"params": ["yaw", "speed"],
                                 "description": "腰部偏航: 控制yaw角度(度)"},
                    "move_knee": {"params": ["knee", "speed"],
                                  "description": "腿部升降: 控制knee角度(度)"},
                    "set_zero": {"params": ["target"],
                                 "description": "归零: target=waist 腰归零, knee 腿归零, both 同时归零"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._pub_waist = self._pub_node.create_publisher(CmdSetMotorPosition, "/waist/cmd_pos", _RELIABLE_QOS)
            self._pub_leg   = self._pub_node.create_publisher(CmdSetMotorPosition, "/leg/cmd_pos", _RELIABLE_QOS)
            print("[WaistPlugin] publishers created (/waist/cmd_pos, /leg/cmd_pos)")
        except ImportError as e:
            print(f"[WaistPlugin] WARNING: {e}")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_yaw":
            return self._send_yaw(args.get("yaw", 0), args.get("speed", 0.5))
        if action == "move_knee":
            return self._send_knee(args.get("knee", 0), args.get("speed", 0.5))
        if action == "set_zero":
            target = args.get("target", "both")
            applied = []
            if target in ("waist", "both"):
                r = self._send_yaw(0)
                if not r.get("ok"):
                    return r
                applied += r.get("applied", [])
            if target in ("knee", "both"):
                r = self._send_knee(-20.0)
                if not r.get("ok"):
                    return r
                applied += r.get("applied", [])
            return {"ok": True, "card": "waist", "action": "set_zero", "target": target, "applied": applied}
        if action in ("start", "info"):
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        return {"ok": False, "code": "INVALID_ARGUMENT", "message": f"unknown action: {action}"}

    def _send_yaw(self, yaw_deg: float, speed_rad_s: float = 0.5) -> dict:
        if not self._pub_waist:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": "publisher not ready"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            mid = 31
            lim = _JOINT_LIMITS[mid]
            pos_deg, clamped = _clamp(yaw_deg, lim[0], lim[1])
            spd, _ = _clamp(speed_rad_s, 0, _rpm2rads(lim[2]))
            cmd = SetMotorPosition()
            cmd.name = mid; cmd.pos = _deg2rad(pos_deg); cmd.spd = spd; cmd.cur = 5.0
            msg.cmds.append(cmd)
            if clamped:
                return {"ok": False, "code": "JOINT_LIMIT_VIOLATION",
                        "message": f"waist yaw out of range [{lim[0]}°, {lim[1]}°]"}
            self._pub_waist.publish(msg)
            return {"ok": True, "card": "waist", "action": "move_yaw",
                    "applied": [{"name": _ALL_JOINTS[mid], "pos_deg": pos_deg, "spd_rad_s": spd}]}
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}

    def _send_knee(self, knee_deg: float, speed_rad_s: float = 0.5) -> dict:
        if not self._pub_leg:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": "publisher not ready"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            mid = 52
            lim = _JOINT_LIMITS[mid]
            pos_deg, clamped = _clamp(knee_deg, lim[0], lim[1])
            spd, _ = _clamp(speed_rad_s, 0, _rpm2rads(lim[2]))
            cmd = SetMotorPosition()
            cmd.name = mid; cmd.pos = _deg2rad(pos_deg); cmd.spd = spd; cmd.cur = 5.0
            msg.cmds.append(cmd)
            if clamped:
                return {"ok": False, "code": "JOINT_LIMIT_VIOLATION",
                        "message": f"leg knee out of range [{lim[0]}°, {lim[1]}°]"}
            self._pub_leg.publish(msg)
            return {"ok": True, "card": "waist", "action": "move_knee",
                    "applied": [{"name": _ALL_JOINTS[mid], "pos_deg": pos_deg, "spd_rad_s": spd}]}
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}



# ══════════════════════════════════════════════════════════════════════════════
# HandPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class HandPlugin:
    """Inspire 灵巧手控制 — set_fingers / gesture / clear_error.

    set_fingers: 选择 side 后逐指输入 0-100 百分比，未填默认 0（张开）。
    gesture: 选择 side + 预设手势(thumbs_up/fist/victory/open_palm)。
    clear_error: 清除指定手(left/right)所有手指关节错误锁（文档 5.7.7）。
    """

    # 手指ID: 1=小指, 2=无名指, 3=中指, 4=食指, 5=拇指弯曲, 6=拇指旋转
    _FINGER_NAMES = ["little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]

    # 0 表示张开，100 表示弯曲到握紧。顺序见 _FINGER_NAMES。
    _GESTURE_PRESETS = {
        "thumbs_up": [100, 100, 100, 100, 0, 0],
        "fist": [100, 100, 100, 100, 100, 0],
        "victory": [100, 100, 0, 0, 100, 0],
        "open_palm": [0, 0, 0, 0, 0, 0],
    }

    _GESTURE_LABELS = {
        "thumbs_up": "点赞",
        "fist": "握拳",
        "victory": "比耶",
        "open_palm": "张开手掌",
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_hand_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._left_pub = None
        self._right_pub = None
        self._left_clear_error = None
        self._right_clear_error = None
        self._srv_timeout = plugin_config.get("call_timeout", 3.0)

    def get_tool(self) -> dict:
        return {
            "name": "hand",
            "type": "actuator",
            "description": "天轶2.0 Inspire 灵巧手 — 单独手指控制 + 预设手势",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["set_fingers", "gesture", "clear_error"],
                               "description": "控制模式: set_fingers=单独控指, gesture=预设手势, clear_error=清除手指错误"},
                    "side": {"type": "string", "enum": ["left", "right", "both"],
                             "description": "控制哪只手"},
                    "little": {"type": "number",
                               "description": "小指 (0=张开, 100=握紧)"},
                    "ring": {"type": "number",
                             "description": "无名指 (0=张开, 100=握紧)"},
                    "middle": {"type": "number",
                               "description": "中指 (0=张开, 100=握紧)"},
                    "index": {"type": "number",
                              "description": "食指 (0=张开, 100=握紧)"},
                    "thumb_bend": {"type": "number",
                                   "description": "拇指弯曲 (0=张开, 100=握紧)"},
                    "thumb_rotation": {"type": "number",
                                       "description": "拇指旋转"},
                    "gesture": {"type": "string",
                                "enum": list(self._GESTURE_PRESETS),
                                "description": "预设手势名称"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_fingers": {
                        "params": ["side", "little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"],
                        "description": "单独控制每根手指 (0=张开, 100=握紧, 不填默认0)",
                    },
                    "gesture": {
                        "params": ["side", "gesture"],
                        "description": "执行预设手势",
                    },
                    "clear_error": {
                        "params": ["side"],
                        "description": "清除手指关节错误锁",
                    },
                },
            },
        }

    def start(self):
        try:
            from sensor_msgs.msg import JointState
            self._left_pub = self._pub_node.create_publisher(
                JointState, "/inspire_hand/ctrl/left_hand", _RELIABLE_QOS)
            self._right_pub = self._pub_node.create_publisher(
                JointState, "/inspire_hand/ctrl/right_hand", _RELIABLE_QOS)
            print("[HandPlugin] publishers created")
        except ImportError as e:
            print(f"[HandPlugin] WARNING: msg import failed ({e})")

        try:
            from bodyctrl_msgs.srv import SetClearError
            self._left_clear_error = self._pub_node.create_client(
                SetClearError, "/inspire_hand/set_clear_error/left_hand")
            self._right_clear_error = self._pub_node.create_client(
                SetClearError, "/inspire_hand/set_clear_error/right_hand")
            print("[HandPlugin] clear_error clients created")
        except ImportError as e:
            print(f"[HandPlugin] WARNING: clear_error service import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "set_fingers":
            side = args.get("side", "both")
            if side not in ("left", "right", "both"):
                return {"error": "side must be left, right, or both"}
            # 从 args 读取每个手指的百分比，不填默认 0（张开）
            keys = ["little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]
            angles = []
            for k in keys:
                v = args.get(k)
                if v is None:
                    angles.append(0)
                else:
                    angles.append(max(0, min(100, int(v))))
            result = self._send_angles(side, angles)
            if "error" not in result:
                result["mode"] = "set_fingers"
                result["angles"] = {k: a for k, a in zip(keys, angles)}
            return result

        elif action == "gesture":
            side = args.get("side", "both")
            if side not in ("left", "right", "both"):
                return {"error": "side must be left, right, or both"}
            gesture_name = args.get("gesture", "")
            if gesture_name not in self._GESTURE_PRESETS:
                return {"error": f"unknown gesture: {gesture_name}, valid: {list(self._GESTURE_PRESETS)}"}
            result = self._send_angles(side, self._GESTURE_PRESETS[gesture_name])
            if "error" not in result:
                result["mode"] = "gesture"
                result["gesture"] = gesture_name
                result["gesture_label"] = self._GESTURE_LABELS[gesture_name]
            return result

        elif action == "clear_error":
            side = args.get("side", "both")
            if side not in ("left", "right", "both"):
                return {"error": "side must be left, right, or both"}
            return self._clear_error(side)

        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _send_angles(self, side: str, angles: list) -> dict:
        if not self._left_pub or not self._right_pub:
            return {"error": "publishers not initialized"}
        try:
            from sensor_msgs.msg import JointState
            # Angles are in percentage (0=open, 100=closed).
            # Hardware maps position 1.0 → open, 0.0 → closed, so invert.
            positions = [(100 - a) / 100.0 for a in angles]

            pubs = []
            if side in ("left", "both"):
                pubs.append(self._left_pub)
            if side in ("right", "both"):
                pubs.append(self._right_pub)

            for pub in pubs:
                msg = JointState()
                msg.name = [str(i + 1) for i in range(6)]
                msg.position = positions
                pub.publish(msg)

            return {"state": "moving", "side": side, "angles": angles}
        except Exception as e:
            return {"error": str(e)}

    def _clear_error(self, side: str) -> dict:
        """清除指定手的所有手指关节错误锁（文档 5.7.7）。"""
        sides = ["left", "right"] if side == "both" else [side]
        results = {}
        ok = True
        for s in sides:
            client = self._left_clear_error if s == "left" else self._right_clear_error
            if not client:
                results[s] = {"ok": False, "message": "client not initialized"}
                ok = False
                continue
            try:
                if not client.wait_for_service(timeout_sec=self._srv_timeout):
                    results[s] = {"ok": False, "message": "service not available"}
                    ok = False
                    continue
                req = client.srv_type.Request()
                resp = client.call(req)
                results[s] = {"ok": True, "accepted": resp.setclear_error_accepted}
            except Exception as e:
                results[s] = {"ok": False, "message": str(e)}
                ok = False
        return {"ok": ok, "card": "hand", "action": "clear_error", "results": results}


# ══════════════════════════════════════════════════════════════════════════════
# GesturePlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class GesturePlugin:
    """调用天轶运控已封装的上半身预设动作。"""

    _MOTION_IDS = {
        "wave": 1,
        "handshake": 2,
        "group_photo": 3,
        "dance": 4,
    }
    _MOTION_LABELS = {
        "wave": "挥手",
        "handshake": "握手",
        "group_photo": "合影",
        "dance": "舞蹈",
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ros2 = ros2
        self._node = Node("tianyi2_gesture", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._node)
        self._client = None

    def get_tool(self) -> dict:
        return {
            "name": "gesture",
            "type": "actuator",
            "description": "天轶2.0 上半身预设动作 — 挥手、握手、合影、舞蹈（调用厂商运控预设，不生成关节轨迹）",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": list(self._MOTION_IDS),
                        "description": "要执行的预设上半身动作",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    name: {"params": [], "description": label}
                    for name, label in self._MOTION_LABELS.items()
                },
            },
        }

    def start(self):
        try:
            from hric_msgs.srv import SetMotionNumber
            self._client = self._node.create_client(
                SetMotionNumber, "/hric/motion/set_motion_number")
            print("[GesturePlugin] client created (/hric/motion/set_motion_number)")
        except ImportError as e:
            print(f"[GesturePlugin] WARNING: motion service import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action not in self._MOTION_IDS:
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": f"unknown action: {action}"}
        if not self._client:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": "motion service client not initialized"}
        if not self._client.service_is_ready():
            return {
                "ok": False,
                "code": "SERVICE_UNAVAILABLE",
                "message": "天轶预设动作服务未就绪；请确认机器人处于厂商动作控制模式",
            }
        try:
            from hric_msgs.srv import SetMotionNumber
            request = SetMotionNumber.Request()
            request.is_motion = True
            request.motion_number = self._MOTION_IDS[action]
            future = self._client.call_async(request)
            future.add_done_callback(lambda f: self._log_result(action, f))
            return {
                "ok": True,
                "card": "gesture",
                "action": action,
                "label": self._MOTION_LABELS[action],
                "motion_number": request.motion_number,
                "state": "requested",
            }
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}

    def _log_result(self, action: str, future) -> None:
        try:
            response = future.result()
            if not response.sucess:
                self._node.get_logger().warning(
                    f"preset gesture {action} rejected by motion controller")
        except Exception as e:
            self._node.get_logger().error(f"preset gesture {action} service call failed: {e}")


# ══════════════════════════════════════════════════════════════════════════════
# TtsPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class TtsPlugin:
    """语音合成 (lyre TTS)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._srv_node = Node("tianyi2_tts", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._srv_node)
        self._play_client = None
        self._stop_client = None
        self._pause_client = None
        self._resume_client = None

    def get_tool(self) -> dict:
        return {
            "name": "tts",
            "type": "actuator",
            "description": "天轶2.0 语音合成 (TTS) — 文字转语音播放",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["speak", "stop", "pause", "resume"],
                               "description": "控制动作"},
                    "text": {"type": "string", "description": "要播放的文本"},
                    "force": {"type": "boolean", "description": "是否强制播放(打断当前播放)", "default": False},
                },
                "required": ["action"],
                "x-action-params": {
                    "speak": {"params": ["text", "force"], "description": "合成并播放文本"},
                    "stop": {"params": [], "description": "停止播放"},
                    "pause": {"params": [], "description": "暂停播放"},
                    "resume": {"params": [], "description": "恢复播放"},
                },
            },
        }

    def start(self):
        try:
            from lyre_msgs.srv import PlayText, PlayStop, PlayPause, PlayResume
            self._play_client = self._srv_node.create_client(PlayText, "/audio_play/play_text")
            self._stop_client = self._srv_node.create_client(PlayStop, "/audio_play/stop")
            self._pause_client = self._srv_node.create_client(PlayPause, "/audio_play/pause")
            self._resume_client = self._srv_node.create_client(PlayResume, "/audio_play/resume")
            print("[TtsPlugin] service clients created")
        except ImportError as e:
            print(f"[TtsPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "speak":
            text = args.get("text", "")
            force = args.get("force", False)
            if not text:
                return {"error": "text is required"}
            return self._speak(text, force)
        elif action == "stop":
            return self._call_empty_service(self._stop_client, "stop")
        elif action == "pause":
            return self._call_empty_service(self._pause_client, "pause")
        elif action == "resume":
            return self._call_empty_service(self._resume_client, "resume")
        elif action in ("start", "info"):
            return {"state": "ready"}
        return {"error": f"unknown action: {action}"}

    def _speak(self, text: str, force: bool) -> dict:
        if not self._play_client:
            return {"error": "service client not initialized"}
        try:
            from lyre_msgs.srv import PlayText
            req = PlayText.Request()
            req.text = text
            req.force = force
            req.last = True
            future = self._play_client.call_async(req)
            # Non-blocking, just return immediately
            return {"state": "speaking", "text": text[:50]}
        except Exception as e:
            return {"error": str(e)}

    def _call_empty_service(self, client, action_name: str) -> dict:
        if not client:
            return {"error": f"{action_name} service client not initialized"}
        try:
            req = type(client.srv_type.Request)()
            client.call_async(req)
            return {"state": action_name}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# AudioPlugin (actuator) — 音频播放 (lyre PlayBinary/PlayText/PlayFile/PlayUrl)
# ══════════════════════════════════════════════════════════════════════════════

class AudioPlugin:
    """音频播放 — 覆盖文档 5.9.6 全部音频播放接口 (PlayText/PlayFile/PlayUrl/PlayBinary + 控制)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._srv_node = Node("tianyi2_audio", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._srv_node)

        # Service clients — lazy init in start()
        self._text_client = None   # PlayText
        self._file_client = None   # PlayFile
        self._url_client = None    # PlayUrl
        self._binary_client = None # PlayBinary
        self._stop_client = None   # PlayStop
        self._pause_client = None  # PlayPause
        self._resume_client = None # PlayResume

        # Timeout for sync service calls (seconds)
        self._call_timeout = plugin_config.get("call_timeout", 5.0)

    def get_tool(self) -> dict:
        return {
            "name": "audio",
            "type": "actuator",
            "description": "天轶2.0 音频播放 — PlayText/PlayFile/PlayUrl/PlayBinary + 停止/暂停/恢复 (文档 5.9.6)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["play_text", "play_file", "play_url", "play_binary",
                                        "stop", "pause", "resume"],
                               "description": "控制动作"},
                    "text": {"type": "string",
                             "description": "要合成播放的文本 (play_text)"},
                    "path": {"type": "string",
                             "description": "本地音频文件绝对路径 (play_file)"},
                    "url": {"type": "string",
                             "description": "远程音频文件 URL (play_url)"},
                    "data_b64": {"type": "string",
                                 "description": "Base64 编码的二进制音频数据 (play_binary)"},
                    "force": {"type": "boolean",
                              "description": "是否强制播放 (打断当前播放任务)", "default": False},
                    "sid": {"type": "string",
                            "description": "流标识符, 留空则自动生成"},
                },
                "required": ["action"],
                "x-action-params": {
                    "play_text":   {"params": ["text", "force", "sid"],
                                    "description": "合成文本并播放 (TTS → 音频输出)"},
                    "play_file":   {"params": ["path", "force", "sid"],
                                    "description": "播放本地文件系统中的音频文件"},
                    "play_url":    {"params": ["url", "force", "sid"],
                                    "description": "播放远程 URL 指向的音频文件"},
                    "play_binary": {"params": ["data_b64", "force", "sid"],
                                    "description": "播放二进制音频数据 (Base64)"},
                    "stop":   {"params": [], "description": "停止当前播放 (不可恢复)"},
                    "pause":  {"params": [], "description": "暂停当前播放 (可恢复)"},
                    "resume": {"params": [], "description": "恢复已暂停的播放"},
                },
            },
        }

    def start(self):
        try:
            from lyre_msgs.srv import (PlayText, PlayFile, PlayUrl, PlayBinary,
                                        PlayStop, PlayPause, PlayResume)
            self._text_client   = self._srv_node.create_client(PlayText,   "/audio_play/play_text")
            self._file_client   = self._srv_node.create_client(PlayFile,   "/audio_play/play_file")
            self._url_client    = self._srv_node.create_client(PlayUrl,    "/audio_play/play_url")
            self._binary_client = self._srv_node.create_client(PlayBinary, "/audio_play/play_binary")
            self._stop_client   = self._srv_node.create_client(PlayStop,   "/audio_play/stop")
            self._pause_client  = self._srv_node.create_client(PlayPause,  "/audio_play/pause")
            self._resume_client = self._srv_node.create_client(PlayResume, "/audio_play/resume")
            print("[AudioPlugin] 7 service clients created (text/file/url/binary + stop/pause/resume)")
        except ImportError as e:
            print(f"[AudioPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    # ── dispatch ────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "play_text":
            text = args.get("text", "")
            if not text:
                return {"ok": False, "code": "INVALID_ARGUMENT", "message": "text is required"}
            return self._call_play(self._text_client, "PlayText",
                                   text=text, force=args.get("force", False),
                                   sid=args.get("sid", ""))

        elif action == "play_file":
            path = args.get("path", "")
            if not path:
                return {"ok": False, "code": "INVALID_ARGUMENT", "message": "path is required"}
            return self._call_play(self._file_client, "PlayFile",
                                   path=path, force=args.get("force", False),
                                   sid=args.get("sid", ""))

        elif action == "play_url":
            url = args.get("url", "")
            if not url:
                return {"ok": False, "code": "INVALID_ARGUMENT", "message": "url is required"}
            return self._call_play(self._url_client, "PlayUrl",
                                   url=url, force=args.get("force", False),
                                   sid=args.get("sid", ""))

        elif action == "play_binary":
            data_b64 = args.get("data_b64", "")
            if not data_b64:
                return {"ok": False, "code": "INVALID_ARGUMENT", "message": "data_b64 is required"}
            import base64
            try:
                raw = base64.b64decode(data_b64)
            except Exception:
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "message": "data_b64 is not valid base64"}
            # PlayBinary data field is uint8[]; ROS 2 Python expects list[int]
            data_list = list(raw)
            return self._call_play(self._binary_client, "PlayBinary",
                                   data=data_list, force=args.get("force", False),
                                   sid=args.get("sid", ""))

        elif action == "stop":
            return self._call_empty(self._stop_client, "stop")

        elif action == "pause":
            return self._call_empty(self._pause_client, "pause")

        elif action == "resume":
            return self._call_empty(self._resume_client, "resume")

        elif action in ("start", "info"):
            return {"state": "ready", "card": "audio"}

        return {"ok": False, "code": "INVALID_ARGUMENT",
                "message": f"unknown action: {action}"}

    # ── helpers ─────────────────────────────────────────────────────────────

    def _call_play(self, client, svc_name: str, **kwargs) -> dict:
        """Send a play request synchronously and return the service response."""
        if not client:
            return {"ok": False, "code": "COMMUNICATION_ERROR",
                    "message": f"{svc_name} service client not initialized"}

        # Wait for service to become available
        if not client.wait_for_service(timeout_sec=self._call_timeout):
            return {"ok": False, "code": "COMMUNICATION_ERROR",
                    "message": f"{svc_name} service not available (timeout {self._call_timeout}s)"}

        try:
            req = client.srv_type.Request()

            # Common fields
            sid   = kwargs.pop("sid", "")
            force = kwargs.pop("force", False)
            req.sid   = sid
            req.seq   = 0
            req.last  = True
            req.force = force

            # Payload field — the remaining kwarg
            for key, val in kwargs.items():
                setattr(req, key, val)

            # Synchronous call — avoids spin_until_future_complete
            # compatibility issues with MultiThreadedExecutor.
            resp = client.call(req)
            return {
                "ok": True,
                "card": "audio",
                "action": svc_name,
                "sid": resp.sid,
                "code": resp.code,
                "message": resp.message,
            }
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}

    def _call_empty(self, client, action_name: str) -> dict:
        """Call a no-request / no-response service (stop/pause/resume)."""
        if not client:
            return {"ok": False, "code": "COMMUNICATION_ERROR",
                    "message": f"{action_name} service client not initialized"}

        if not client.wait_for_service(timeout_sec=self._call_timeout):
            return {"ok": False, "code": "COMMUNICATION_ERROR",
                    "message": f"{action_name} service not available"}

        try:
            req = client.srv_type.Request()
            client.call_async(req)
            return {"ok": True, "card": "audio", "action": action_name}
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# NavPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class NavPlugin:
    """底盘导航控制 — 自主导航/遥控/旋转/回桩"""

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._ns = namespace
        self._ros2 = ros2
        self._slamtec = slamtec_client

        # cmd_vel publisher for direct velocity control (domain 0)
        self._vel_node = Node("tianyi2_nav_vel", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._vel_node)
        self._vel_pub = None

    def get_tool(self) -> dict:
        return {
            "name": "nav",
            "type": "actuator",
            "description": "天轶2.0 底盘导航 — 自主导航到目标点/方向遥控/旋转/回桩充电 (Slamtec轮式底盘)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["move_to", "move_by", "rotate", "rotate_to", "go_home", "stop", "get_pose"],
                               "description": "导航动作"},
                    "x": {"type": "number", "description": "目标x坐标(米)"},
                    "y": {"type": "number", "description": "目标y坐标(米)"},
                    "direction": {"type": "string",
                                  "enum": ["forward", "backward", "left", "right"],
                                  "description": "移动方向(move_by)"},
                    "angle": {"type": "number", "description": "旋转角度(度), 正=逆时针"},
                    "speed": {"type": "number", "description": "速度比例(0-1), 默认0.5"},
                    "vx": {"type": "number", "description": "前后速度(m/s), 正=前进"},
                    "vy": {"type": "number", "description": "左右速度(m/s), 正=左移"},
                    "vyaw": {"type": "number", "description": "旋转速度(rad/s), 正=逆时针"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_to": {"params": ["x", "y", "speed"],
                                "description": "自主导航到目标点(带避障)"},
                    "move_by": {"params": ["direction", "speed"],
                                "description": "方向遥控移动(不避障, 持续500ms)"},
                    "rotate": {"params": ["angle"],
                               "description": "原地旋转指定角度(度)"},
                    "rotate_to": {"params": ["angle"],
                                  "description": "原地旋转到绝对角度(度)"},
                    "go_home": {"params": [],
                                "description": "自主导航回充电桩"},
                    "stop": {"params": [],
                             "description": "停止当前导航动作"},
                    "get_pose": {"params": [],
                                 "description": "获取当前位姿(x, y, yaw)"},
                },
            },
        }

    def start(self):
        try:
            from geometry_msgs.msg import Twist
            self._vel_pub = self._vel_node.create_publisher(Twist, "/cmd_vel", _RELIABLE_QOS)
            print("[NavPlugin] cmd_vel publisher created")
        except ImportError as e:
            print(f"[NavPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_to":
            x = args.get("x", 0)
            y = args.get("y", 0)
            speed = args.get("speed")
            result = self._slamtec.move_to(x, y, speed_ratio=speed)
            return {"state": "navigating", "target": {"x": x, "y": y}, "api_result": result}

        elif action == "move_by":
            direction = args.get("direction", "forward")
            dir_map = {"forward": 0, "backward": 1, "right": 2, "left": 3}
            d = dir_map.get(direction, 0)
            result = self._slamtec.move_by(d)
            return {"state": "moving", "direction": direction, "api_result": result}

        elif action == "rotate":
            angle_deg = args.get("angle", 0)
            angle_rad = _deg2rad(angle_deg)
            result = self._slamtec.rotate(angle_rad)
            return {"state": "rotating", "angle": angle_deg, "api_result": result}

        elif action == "rotate_to":
            angle_deg = args.get("angle", 0)
            angle_rad = _deg2rad(angle_deg)
            result = self._slamtec.rotate_to(angle_rad)
            return {"state": "rotating_to", "angle": angle_deg, "api_result": result}

        elif action == "go_home":
            result = self._slamtec.go_home()
            return {"state": "going_home", "api_result": result}

        elif action == "stop":
            result = self._slamtec.cancel_current_action()
            # Also stop cmd_vel
            if self._vel_pub:
                try:
                    from geometry_msgs.msg import Twist
                    self._vel_pub.publish(Twist())  # zero velocity
                except Exception:
                    pass
            return {"state": "stopped", "api_result": result}

        elif action == "get_pose":
            pose = self._slamtec.get_pose()
            return {"pose": pose}

        elif action in ("start", "info"):
            return {"state": "ready"}
        return {"error": f"unknown action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# ChatPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class ChatPlugin:
    """语音交互开关"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_chat_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "chat",
            "type": "actuator",
            "description": "天轶2.0 语音交互模式 — 开启/关闭内置语音对话功能",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["enable", "disable"],
                               "description": "开启或关闭"},
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {"params": [], "description": "开启语音交互"},
                    "disable": {"params": [], "description": "关闭语音交互"},
                },
            },
        }

    def start(self):
        self._publisher = self._pub_node.create_publisher(Bool, "/audio_chat/enable", _RELIABLE_QOS)
        print("[ChatPlugin] publisher created")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("enable", "disable"):
            if self._publisher:
                msg = Bool()
                msg.data = (action == "enable")
                self._publisher.publish(msg)
                return {"state": action + "d"}
            return {"error": "publisher not initialized"}
        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# DialoguePlugin (actuator) — 语音对话 (文档 5.9.8: 交互开关 + LlmAsk)
# ══════════════════════════════════════════════════════════════════════════════

class DialoguePlugin:
    """语音对话 — 覆盖文档 5.9.8 语音对话接口 (交互开关 + LLM 提问)"""

    _LLM_EVENT_NAMES = {0: "started", 1: "completed", 2: "stopped", 3: "cancelled", 4: "failed"}

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._running = False

        # Service node (domain 0) — LlmAsk
        self._srv_node = Node("tianyi2_dialogue_srv", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._srv_node)
        self._ask_client = None

        # Pub node (domain 0) — chat enable/disable
        self._pub_node = Node("tianyi2_dialogue_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._chat_pub = None

        # Sub/Pub nodes — for LLM result and event streaming (domain 0 → domain 42)
        self._sub_node = Node("tianyi2_dialogue_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)
        self._bridge_node = Node("tianyi2_dialogue_bridge", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._bridge_node)

        # Output topic (domain 42) — 合并 rst + event 到单 topic
        self._topic_stream = f"/{namespace}/dialogue/stream"
        self._pub_stream = None

        # Timeout for sync service calls (seconds)
        self._call_timeout = plugin_config.get("call_timeout", 5.0)

    def get_tools(self) -> list:
        return [
            {"name": "dialogue", "type": "actuator",
             "description": "天轶2.0 语音对话 — 交互开关 + LLM 提问 (文档 5.9.8)",
             "inputSchema": {
                 "type": "object",
                 "properties": {
                     "action": {"type": "string",
                                "enum": ["enable", "disable", "ask"],
                                "description": "控制动作"},
                     "text": {"type": "string",
                              "description": "向 LLM 发送的问题文本 (ask)"},
                     "id": {"type": "string",
                            "description": "可选标识符，关联 ASR 识别 ID (ask)"},
                 },
                 "required": ["action"],
                 "x-action-params": {
                     "enable":  {"params": [], "description": "开启语音对话交互"},
                     "disable": {"params": [], "description": "关闭语音对话交互"},
                     "ask":     {"params": ["text", "id"],
                                 "description": "向 LLM 大模型提问 (同步等待回复)"},
                 },
             }},
            {"name": "dialogue_stream", "type": "sensor",
             "description": "天轶2.0 LLM 对话反馈流 — 结果文本 + 生命周期事件 (文档 5.9.8)",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topic_stream, "format": "data/json"}]},
        ]

    def start(self):
        self._running = True

        # Actuator: chat enable/disable publisher (domain 0)
        self._chat_pub = self._pub_node.create_publisher(Bool, "/audio_chat/enable", _RELIABLE_QOS)

        # Sensor bridge (domain 0 → domain 42) — 单 topic 合并 rst + event
        self._pub_stream = self._bridge_node.create_publisher(String, self._topic_stream, _RELIABLE_QOS)

        try:
            from lyre_msgs.srv import LlmAsk
            from lyre_msgs.msg import LlmRst, LlmEvent

            self._ask_client = self._srv_node.create_client(LlmAsk, "/audio_llm/ask")
            self._sub_node.create_subscription(
                LlmRst, "/audio_llm/rst", self._on_llm_rst, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                LlmEvent, "/audio_llm/event", self._on_llm_event, _RELIABLE_QOS)
            print("[DialoguePlugin] service client + chat pub + 2 subscriptions created")
        except ImportError as e:
            print(f"[DialoguePlugin] WARNING: lyre_msgs import failed ({e})")

    def stop(self):
        self._running = False

    # ── dispatch ────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict:
        tool_name = args.get("_tool_name", "dialogue")

        if tool_name == "dialogue":
            if action == "enable":
                if not self._chat_pub:
                    return {"ok": False, "code": "COMMUNICATION_ERROR",
                            "message": "chat publisher not initialized"}
                msg = Bool()
                msg.data = True
                self._chat_pub.publish(msg)
                return {"ok": True, "card": "dialogue", "action": "enable",
                        "message": "语音对话已开启"}

            elif action == "disable":
                if not self._chat_pub:
                    return {"ok": False, "code": "COMMUNICATION_ERROR",
                            "message": "chat publisher not initialized"}
                msg = Bool()
                msg.data = False
                self._chat_pub.publish(msg)
                return {"ok": True, "card": "dialogue", "action": "disable",
                        "message": "语音对话已关闭"}

            elif action == "ask":
                text = args.get("text", "")
                qid = args.get("id", "")
                if not text:
                    return {"ok": False, "code": "INVALID_ARGUMENT",
                            "message": "text is required"}
                return self._ask_llm(text, qid)

            elif action in ("start", "info"):
                return {"state": "ready", "card": "dialogue"}
            return {"ok": False, "code": "INVALID_ARGUMENT",
                    "message": f"unknown action: {action}"}

        elif tool_name == "dialogue_stream":
            if action in ("start", "stop", "info"):
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._topic_stream, "format": "data/json"}]}
            return {"state": "running"}

        return {"ok": False, "code": "INVALID_ARGUMENT",
                "message": f"unknown tool: {tool_name}"}

    # ── LLM ask with sync response ──────────────────────────────────────────

    def _ask_llm(self, text: str, qid: str) -> dict:
        """Send question to LLM and wait for the first response."""
        if not self._ask_client:
            return {"ok": False, "code": "COMMUNICATION_ERROR",
                    "message": "LlmAsk service client not initialized"}

        if not self._ask_client.wait_for_service(timeout_sec=self._call_timeout):
            return {"ok": False, "code": "COMMUNICATION_ERROR",
                    "message": f"LlmAsk service not available (timeout {self._call_timeout}s)"}

        try:
            from lyre_msgs.srv import LlmAsk
            req = LlmAsk.Request()
            req.text = text
            req.id = qid

            # Use synchronous call() — avoids spin_until_future_complete
            # compatibility issues with MultiThreadedExecutor.
            resp = self._ask_client.call(req)
            return {
                "ok": True,
                "card": "dialogue",
                "action": "ask",
                "sid": resp.sid,
                "code": resp.code,
                "message": resp.message,
                "text": text[:100],
            }
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}

    # ── LLM result / event callbacks ────────────────────────────────────────

    def _on_llm_rst(self, msg):
        if not self._running:
            return
        out = String()
        out.data = json.dumps({
            "type": "rst",
            "sid": msg.sid, "seq": msg.seq, "last": msg.last, "text": msg.text,
        })
        self._pub_stream.publish(out)

    def _on_llm_event(self, msg):
        if not self._running:
            return
        out = String()
        out.data = json.dumps({
            "type": "event",
            "sid": msg.sid, "seq": msg.seq,
            "event": msg.event,
            "event_name": self._LLM_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "message": msg.message,
        })
        self._pub_stream.publish(out)


# ══════════════════════════════════════════════════════════════════════════════
# LlmPlugin (actuator + sensor) — LLM 语音对话
# ══════════════════════════════════════════════════════════════════════════════

class LlmPlugin:
    """LLM 语音对话 — 向大模型提问并获取回复"""

    _LLM_EVENT_NAMES = {0: "started", 1: "completed", 2: "stopped", 3: "cancelled", 4: "failed"}

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._running = False

        self._srv_node = Node("tianyi2_llm_srv", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._srv_node)
        self._ask_client = None

        self._sub_node = Node("tianyi2_llm_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)
        self._pub_node = Node("tianyi2_llm_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

        self._topic_rst = f"/{namespace}/lyre/llm_rst"
        self._topic_event = f"/{namespace}/lyre/llm_event"
        self._pub_rst = None
        self._pub_event = None

    def get_tools(self) -> list:
        return [
            {"name": "llm_ask", "type": "actuator",
             "description": "天轶2.0 LLM 语音对话 — 向大模型发送文本问题",
             "inputSchema": {
                 "type": "object",
                 "properties": {
                     "action": {"type": "string", "enum": ["ask"], "description": "发送问题"},
                     "text": {"type": "string", "description": "要发送的问题文本"},
                     "id": {"type": "string", "description": "可选标识符，关联 ASR 识别 ID"},
                 },
                 "required": ["action", "text"],
                 "x-action-params": {
                     "ask": {"params": ["text", "id"], "description": "向 LLM 提问"},
                 },
             }},
            {"name": "llm_rst", "type": "sensor",
             "description": "天轶2.0 LLM 语音对话结果 — 大模型返回的文本回复",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topic_rst, "format": "data/json"}]},
            {"name": "llm_event", "type": "sensor",
             "description": "天轶2.0 LLM 语音对话事件 — 开始/完成/停止/取消/失败",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topic_event, "format": "data/json"}]},
        ]

    def start(self):
        self._running = True
        self._pub_rst = self._pub_node.create_publisher(String, self._topic_rst, _RELIABLE_QOS)
        self._pub_event = self._pub_node.create_publisher(String, self._topic_event, _RELIABLE_QOS)

        try:
            from lyre_msgs.srv import LlmAsk
            from lyre_msgs.msg import LlmRst, LlmEvent
            self._ask_client = self._srv_node.create_client(LlmAsk, "/audio_llm/ask")
            self._sub_node.create_subscription(
                LlmRst, "/audio_llm/rst", self._on_llm_rst, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                LlmEvent, "/audio_llm/event", self._on_llm_event, _RELIABLE_QOS)
            print("[LlmPlugin] service client + 2 subscriptions created")
        except ImportError as e:
            print(f"[LlmPlugin] WARNING: lyre_msgs import failed ({e})")

    def stop(self):
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict:
        tool_name = args.get("_tool_name", "llm_ask")

        if tool_name == "llm_ask":
            if action == "ask":
                text = args.get("text", "")
                qid = args.get("id", "")
                if not text:
                    return {"error": "text is required"}
                return self._ask(text, qid)
            elif action in ("start", "info"):
                return {"state": "ready"}
            return {"error": f"unknown action: {action}"}

        elif tool_name == "llm_rst":
            if action in ("start", "stop", "info"):
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._topic_rst, "format": "data/json"}]}
            return {"state": "running"}

        elif tool_name == "llm_event":
            if action in ("start", "stop", "info"):
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._topic_event, "format": "data/json"}]}
            return {"state": "running"}

        return {"error": f"unknown tool: {tool_name}"}

    def _ask(self, text: str, qid: str) -> dict:
        if not self._ask_client:
            return {"error": "llm_ask service client not initialized"}
        try:
            from lyre_msgs.srv import LlmAsk
            req = LlmAsk.Request()
            req.text = text
            req.id = qid
            future = self._ask_client.call_async(req)
            return {"state": "asking", "text": text[:100]}
        except Exception as e:
            return {"error": str(e)}

    def _on_llm_rst(self, msg):
        if not self._running:
            return
        out = String()
        out.data = json.dumps({"sid": msg.sid, "seq": msg.seq, "last": msg.last, "text": msg.text})
        self._pub_rst.publish(out)

    def _on_llm_event(self, msg):
        if not self._running:
            return
        out = String()
        out.data = json.dumps({
            "sid": msg.sid, "seq": msg.seq,
            "event": msg.event,
            "event_name": self._LLM_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "message": msg.message,
        })
        self._pub_event.publish(out)
