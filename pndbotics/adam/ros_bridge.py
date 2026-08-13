#!/usr/bin/env python3
"""ROS2 republish bridge for the PNDbotics Adam driver.

Polls the :class:`DdsClient` accessors and republishes low-level state
(joints / imu / battery / remote / hand_state) to ROS2 topics so the Agent
Core dashboard can render live streams (3D skeleton, IMU curves, battery
panel, …).

This module is imported lazily from ``main.py``'s ROS path only — ``device.py``
stays free of any rclpy dependency, so the unit tests keep running without ROS.
The raw ``pnd_sdk_python`` (CycloneDDS, domain 1) and this ROS2 layer
(rmw_fastrtps, domain 42) coexist in the same process the same way bumi/tianyi
do: different middleware stacks on different DDS domains.
"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from control import joint_payload


_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


class AdamStateNode(Node):
    """Polls DdsClient and republishes to ROS2 topics for live dashboard streams."""

    _JOINTS_INTERVAL = 0.1    # 10 Hz
    _IMU_INTERVAL = 0.05      # 20 Hz
    _BATTERY_INTERVAL = 1.0   # 1 Hz
    _REMOTE_INTERVAL = 0.1    # 10 Hz
    _HAND_INTERVAL = 0.1      # 10 Hz

    def __init__(self, namespace: str, dds, grpc_client=None):
        super().__init__("adam_state")
        self._dds = dds
        self._grpc = grpc_client

        self._joints_pub = self.create_publisher(String, f"/{namespace}/state/joints", _LOW_LAT_QOS)
        self._imu_pub = self.create_publisher(String, f"/{namespace}/state/imu", _LOW_LAT_QOS)
        self._battery_pub = self.create_publisher(String, f"/{namespace}/state/battery", _LOW_LAT_QOS)
        self._remote_pub = self.create_publisher(String, f"/{namespace}/state/remote", _LOW_LAT_QOS)
        self._hand_pub = self.create_publisher(String, f"/{namespace}/state/hand", _LOW_LAT_QOS)
        self._robot_pub = self.create_publisher(String, f"/{namespace}/state/robot", _LOW_LAT_QOS)

        self._running = False
        self._thread: threading.Thread | None = None
        self._grpc_thread: threading.Thread | None = None

    def start_polling(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="adam_dds_poll")
        self._thread.start()
        if self._grpc is not None:
            self._grpc_thread = threading.Thread(target=self._grpc_poll_loop, daemon=True, name="adam_grpc_poll")
            self._grpc_thread.start()

    def stop_polling(self) -> None:
        self._running = False

    def _poll_loop(self) -> None:
        last_joints = 0.0
        last_imu = 0.0
        last_battery = 0.0
        last_remote = 0.0
        last_hand = 0.0

        while self._running:
            try:
                now = time.monotonic()

                if now - last_imu >= self._IMU_INTERVAL:
                    last_imu = now
                    data = self._dds.imu()
                    if data is not None:
                        self._imu_pub.publish(String(data=json.dumps(data)))

                if now - last_joints >= self._JOINTS_INTERVAL:
                    last_joints = now
                    j = self._dds.joints()
                    if j is not None:
                        payload = joint_payload(j["position"], j["velocity"], j["torque"])
                        # Base orientation for the skeleton renderer ([w, x, y, z]).
                        imu = self._dds.imu()
                        if imu is not None and "quaternion_wxyz" in imu:
                            payload["imu_quat"] = imu["quaternion_wxyz"]
                        self._joints_pub.publish(String(data=json.dumps(payload)))

                if now - last_battery >= self._BATTERY_INTERVAL:
                    last_battery = now
                    data = self._dds.battery()
                    if data is not None:
                        self._battery_pub.publish(String(data=json.dumps(data)))

                if now - last_remote >= self._REMOTE_INTERVAL:
                    last_remote = now
                    data = self._dds.remote()
                    if data is not None:
                        self._remote_pub.publish(String(data=json.dumps(data)))

                if now - last_hand >= self._HAND_INTERVAL:
                    last_hand = now
                    pos = self._dds.hand_state()
                    if pos is not None:
                        self._hand_pub.publish(String(data=json.dumps({"position": pos})))

                time.sleep(0.02)
            except Exception as exc:  # noqa: BLE001 — keep the poll loop alive
                self.get_logger().warn(f"adam state poll error: {exc}")
                time.sleep(0.5)

    _ROBOT_INTERVAL = 0.5  # 2 Hz — high-level state (mode/motion/velocity) changes slowly

    def _grpc_poll_loop(self) -> None:
        """Separate thread: gRPC GetRobotState is blocking, so keep it off the DDS path."""
        last = 0.0
        while self._running:
            try:
                now = time.monotonic()
                if now - last >= self._ROBOT_INTERVAL:
                    last = now
                    state = self._grpc.get_robot_state()
                    if state.get("state") != "error":
                        self._robot_pub.publish(String(data=json.dumps(state)))
                time.sleep(0.05)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"adam robot-state poll error: {exc}")
                time.sleep(0.5)
