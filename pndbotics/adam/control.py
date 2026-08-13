"""Pure control helpers for the PNDbotics Adam Pro driver.

This module intentionally has no gRPC / DDS / ROS dependency.  It owns the joint
layout (from the official DDS ``ADAMJointIndex`` order mapped to the URDF joint
names in ``resource/adam_sp_pro.urdf``), the gRPC mode/motion enums, and the
MCP tool-schema builders shared by the plugins in ``device.py``.
"""

from __future__ import annotations

import math
import time

DOF = 31

# ── Joint layout ──────────────────────────────────────────────────────────────
# Index order follows the official DDS ``ADAMJointIndex`` enum (31 motors).
# Names match the URDF joint names exactly (required by the skeleton renderer).

ADAM_PRO_JOINT_NAMES = (
    # 0-5: left leg
    "hipPitch_Left", "hipRoll_Left", "hipYaw_Left",
    "kneePitch_Left", "anklePitch_Left", "ankleRoll_Left",
    # 6-11: right leg
    "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right",
    # 12-14: waist
    "waistYaw", "waistRoll", "waistPitch",
    # 15-21: left arm
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left",
    "elbow_Left", "wristRoll_Left", "wristPitch_Left", "wristYaw_Left",
    # 22-28: right arm
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right",
    "elbow_Right", "wristRoll_Right", "wristPitch_Right", "wristYaw_Right",
    # 29-30: head/neck
    "neckYaw", "neckPitch",
)

ADAM_PRO_JOINT_GROUPS = {
    "left_leg": tuple(range(0, 6)),
    "right_leg": tuple(range(6, 12)),
    "legs": tuple(range(0, 12)),
    "waist": (12, 13, 14),
    "left_arm": tuple(range(15, 22)),
    "right_arm": tuple(range(22, 29)),
    "arms": tuple(range(15, 29)),
    "head": (29, 30),
    "all": tuple(range(DOF)),
}

# ── gRPC contract constants ───────────────────────────────────────────────────
# From https://wiki.pndbotics.com/robot/motion_control_interface_GRPC/

MODES = ["Start", "Zero", "Stand", "Walk", "Run", "Stop"]
MOTIONS = ["Greeting", "Chest Expansion", "Stretching", "Gentleman's Salute"]
ACTIONS = ["Roll", "Pitch", "Yaw", "Floating Base Height"]
CARRY_BOXES = [
    "Standing to Pick up the Box",
    "Squatting to Pick up the Box",
    "Put Down the Box",
]


def _now_ms() -> int:
    return int(time.time() * 1000)


def clamp(value: float, lower: float, upper: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("value must be finite")
    return max(lower, min(upper, value))


def float_list(value, name, *, size=None, allow_empty=False) -> list[float]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{name} must be an array")
    result = [float(item) for item in value]
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite numbers")
    if not result and not allow_empty:
        raise ValueError(f"{name} cannot be empty")
    if size is not None and len(result) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    return result


def joint_payload(position, velocity=None, torque=None) -> dict:
    """Normalize a 31-motor state into the skeleton-renderer joint payload."""
    n = len(position)
    velocity = velocity or [0.0] * n
    torque = torque or [0.0] * n
    joints = [
        {
            "idx": i,
            "name": ADAM_PRO_JOINT_NAMES[i],
            "q": float(position[i]),
            "dq": float(velocity[i]),
            "tau": float(torque[i]),
        }
        for i in range(n)
    ]
    return {"joints": joints, "timestamp_ms": _now_ms()}


# ── MCP tool-schema builders ──────────────────────────────────────────────────

def action_schema(actions: dict, properties: dict, description: str) -> dict:
    return {
        "type": "object",
        "properties": {
            "action": {"type": "string", "enum": list(actions), "description": description},
            **properties,
        },
        "required": ["action"],
        "x-action-params": {
            name: {"params": params, "description": desc}
            for name, (params, desc) in actions.items()
        },
    }


def array_property(description: str, *, item_type: str = "number") -> dict:
    return {"type": "array", "items": {"type": item_type}, "description": description}


def sensor_tool(name: str, description: str, *, topic=None, fmt=None) -> dict:
    tool = {
        "name": name,
        "type": "sensor",
        "multiInstance": False,
        "readOnly": True,
        "description": description,
        "inputSchema": {"type": "object", "properties": {}},
    }
    if topic is not None:
        tool["topic_out"] = [{"topic": topic, "format": fmt or "data/json"}]
    return tool
