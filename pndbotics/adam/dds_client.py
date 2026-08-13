"""Defensive DDS client for the PNDbotics Adam low-level interface.

This is the low-level half of the driver (the gRPC ``RobotControl`` service is
the high-level half).  It wraps the official ``pnd_sdk_python`` (vendored under
``pnd_sdk_python/``) to:

  - subscribe ``rt/lowstate``  (31-DOF motor state + IMU + battery)
  - subscribe ``rt/handstate`` (12-DOF dexterous-hand state)
  - publish   ``rt/handcmd``   (12-DOF dexterous-hand command)

All SDK imports are lazy, so the driver still boots (and the gRPC tools still
work) when ``cyclonedds`` is not installed — DDS tools then report an error.

The SDK requires a native CycloneDDS runtime; see the Dockerfile.
"""

from __future__ import annotations

import threading
import time


class DdsClient:
    def __init__(self, config: dict):
        self._domain_id = int(config.get("domain_id", 1))
        self._interface = config.get("network_interface", "lo") or None
        self._timeout = float(config.get("stale_timeout_sec", 1.0))
        self._num_motor = int(config.get("num_motor", 31))

        self._lock = threading.RLock()
        self._lowstate = None
        self._handstate = None
        self._last_lowstate_at = None
        self._last_handstate_at = None

        self._sub_low = None
        self._sub_hand = None
        self._pub_hand = None
        self._started = False
        self._error = None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._started:
            return
        try:
            self._init_channels()
            self._started = True
            self._error = None
        except Exception as exc:  # noqa: BLE001 — degrade gracefully without DDS
            self._error = str(exc)

    def _init_channels(self) -> None:
        from pndbotics_sdk_py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import HandCmd_, HandState_, LowState_

        ChannelFactoryInitialize(self._domain_id, self._interface)

        self._sub_low = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub_low.Init(self._on_lowstate, 1)

        self._sub_hand = ChannelSubscriber("rt/handstate", HandState_)
        self._sub_hand.Init(self._on_handstate, 1)

        self._pub_hand = ChannelPublisher("rt/handcmd", HandCmd_)
        self._pub_hand.Init()

    def stop(self) -> None:
        for channel in (self._sub_low, self._sub_hand, self._pub_hand):
            try:
                if channel is not None:
                    channel.Close()
            except Exception:  # noqa: BLE001
                pass
        self._sub_low = self._sub_hand = self._pub_hand = None
        self._started = False

    # ── status ───────────────────────────────────────────────────────────────

    def status(self) -> dict:
        with self._lock:
            return {
                "state": "running" if self._started else "error" if self._error else "stopped",
                "started": self._started,
                "error": self._error,
                "domain_id": self._domain_id,
                "network_interface": self._interface or "auto",
                "lowstate_age_sec": self._age(self._last_lowstate_at),
                "handstate_age_sec": self._age(self._last_handstate_at),
            }

    def connected(self) -> bool:
        with self._lock:
            if self._last_lowstate_at is None:
                return False
            return self._age(self._last_lowstate_at) <= self._timeout

    def _age(self, ts) -> float | None:
        if ts is None:
            return None
        return max(0.0, time.monotonic() - ts)

    # ── DDS callbacks (run on the SDK listener thread) ────────────────────────

    def _on_lowstate(self, msg) -> None:
        with self._lock:
            self._lowstate = msg
            self._last_lowstate_at = time.monotonic()

    def _on_handstate(self, msg) -> None:
        with self._lock:
            self._handstate = msg
            self._last_handstate_at = time.monotonic()

    # ── state accessors ───────────────────────────────────────────────────────

    def joints(self) -> dict | None:
        with self._lock:
            msg = self._lowstate
            if msg is None:
                return None
        try:
            n = min(self._num_motor, len(msg.motor_state))
            return {
                "position": [float(msg.motor_state[i].q) for i in range(n)],
                "velocity": [float(msg.motor_state[i].dq) for i in range(n)],
                "torque": [float(msg.motor_state[i].tau_est) for i in range(n)],
                "mode": [int(msg.motor_state[i].mode) for i in range(n)],
                "mode_pr": int(msg.mode_pr),
                "tick": int(msg.tick),
            }
        except Exception:  # noqa: BLE001
            return None

    def imu(self) -> dict | None:
        with self._lock:
            msg = self._lowstate
            if msg is None:
                return None
        try:
            imu = msg.imu_state
            return {
                "quaternion_wxyz": [float(imu.quaternion[0]), float(imu.quaternion[1]),
                                    float(imu.quaternion[2]), float(imu.quaternion[3])],
                "gyroscope_rad_s": [float(imu.gyroscope[i]) for i in range(3)],
                "accelerometer_m_s2": [float(imu.accelerometer[i]) for i in range(3)],
                "ypr_rad": [float(imu.ypr[i]) for i in range(3)],
                "temperature_c": int(imu.temperature),
            }
        except Exception:  # noqa: BLE001
            return None

    def battery(self) -> dict | None:
        with self._lock:
            msg = self._lowstate
            if msg is None:
                return None
        try:
            b = msg.battery_data
            return {
                "voltage_v": float(b.voltage),
                "current_a": float(b.current),
                "power_w": float(b.power),
                "wh_accumulated": float(b.wh_accumulated),
                "status": str(b.status),
                "timestamp_ms": int(b.timestamp_ms),
            }
        except Exception:  # noqa: BLE001
            return None

    def remote(self) -> dict | None:
        with self._lock:
            msg = self._lowstate
            if msg is None:
                return None
        try:
            return {"wireless_remote": [float(x) for x in msg.wireless_remote]}
        except Exception:  # noqa: BLE001
            return None

    def hand_state(self) -> list[int] | None:
        with self._lock:
            msg = self._handstate
            if msg is None:
                return None
        try:
            return [int(msg.position[i]) for i in range(12)]
        except Exception:  # noqa: BLE001
            return None

    # ── hand control ──────────────────────────────────────────────────────────

    def set_hand(self, positions: list[int]) -> dict:
        if not self._started or self._pub_hand is None:
            return {"state": "error", "error": self._error or "DDS hand publisher not initialized"}
        if len(positions) != 12 or any(p < 0 or p > 1000 for p in positions):
            return {"state": "error", "error": "hand position must be 12 ints in [0, 1000]"}
        try:
            from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import HandCmd_

            cmd = HandCmd_(list(positions), 0)
            ok = self._pub_hand.Write(cmd)
            return {"state": "ok" if ok else "error", "position": list(positions)}
        except Exception as exc:  # noqa: BLE001
            return {"state": "error", "error": str(exc)}
