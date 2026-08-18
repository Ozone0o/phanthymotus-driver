"""Read-only Bumi firmware/SDK compatibility card.

This module deliberately performs no control calls.  Vendor firmware and
serial-number APIs vary between SDK builds, so values are reported only when
an SDK attribute or an explicitly documented read-only getter is available.
"""

from __future__ import annotations

import importlib
import os
import platform
from typing import Any


CARD = "bumi_firmware_info"
DRIVER_SDK_VERSION = "3.1.0"
PROTOCOL_NAME = "Noetix HighController / ControlCmd"
TEACHING_COMMANDS = ("STARTTEACH", "SAVETEACH", "PLAYTEACH", "WALK")


def _clean(value: Any) -> str | None:
    if value is None or callable(value):
        return None
    text = str(value).strip()
    return text or None


def _read_value(obj: Any, names: tuple[str, ...]) -> str | None:
    for name in names:
        try:
            value = getattr(obj, name, None)
            if callable(value):
                # Only call explicitly read-only-looking getters.
                if not name.lower().startswith(("get", "read", "query")):
                    continue
                value = value()
            result = _clean(value)
            if result is not None:
                return result
        except Exception:
            continue
    return None


def _enum_names() -> set[str]:
    try:
        module = importlib.import_module("highcontrol_py")
        enum = getattr(module, "ControlCmd", None)
        names = set(dir(enum)) if enum is not None else set()
        return {name for name in names if name.isupper()}
    except Exception:
        return set()


def _compatibility(sdk_version: str | None, commands: set[str]) -> dict:
    sdk_match = sdk_version == DRIVER_SDK_VERSION if sdk_version else None
    missing = sorted(set(TEACHING_COMMANDS) - commands) if commands else list(TEACHING_COMMANDS)
    return {
        "driver_sdk_required": DRIVER_SDK_VERSION,
        "sdk_compatible": sdk_match,
        "sdk_reason": (
            "SDK version matches the driver requirement" if sdk_match is True else
            "SDK version differs from the driver requirement" if sdk_match is False else
            "SDK version could not be read"
        ),
        "teaching_supported": bool(commands) and not missing,
        "teaching_reason": (
            "All required teaching commands are exposed by ControlCmd" if commands and not missing else
            f"Missing ControlCmd commands: {', '.join(missing)}" if commands else
            "ControlCmd enum could not be inspected"
        ),
        "supported_control_commands": sorted(commands),
        "teaching_commands_required": list(TEACHING_COMMANDS),
    }


class FirmwareInfoPlugin:
    """MCP sensor card that reports only read-only firmware information."""

    def __init__(self, high_ctrl=None):
        self._high_ctrl = high_ctrl

    def get_tool(self) -> dict:
        return {
            "name": CARD,
            "type": "sensor",
            "multiInstance": False,
            "description": "Read-only Bumi firmware, SDK, protocol and compatibility information. No commands are sent to the robot.",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action not in (CARD, "info", "read", "get"):
            return None

        sdk_module = None
        try:
            sdk_module = importlib.import_module("highcontrol_py")
        except Exception:
            pass

        sdk_version = _read_value(sdk_module, ("SDK_VERSION", "__version__")) if sdk_module else None
        if sdk_version is None:
            sdk_version = os.environ.get("BUMI_SDK_VERSION")

        hardware_model = _read_value(self._high_ctrl, (
            "get_hardware_model", "get_model", "hardware_model", "model"))
        serial_number = _read_value(self._high_ctrl, (
            "get_serial_number", "get_device_serial", "serial_number", "device_serial"))
        firmware = _read_value(self._high_ctrl, (
            "get_firmware_version", "get_main_firmware_version", "firmware_version"))
        motor_firmware = _read_value(self._high_ctrl, (
            "get_motor_control_version", "get_motor_firmware_version", "motor_control_version"))
        protocol_version = _read_value(self._high_ctrl, (
            "get_protocol_version", "protocol_version"))

        commands = _enum_names()
        result = {
            "state": "completed",
            "read_only": True,
            "main_firmware_version": firmware or "unknown",
            "motor_control_version": motor_firmware or "unknown",
            "hardware_model": hardware_model or os.environ.get("BUMI_HARDWARE_MODEL", "unknown"),
            "sdk_version": sdk_version or "unknown",
            "protocol_version": protocol_version or "unknown",
            "device_serial_number": serial_number or "unknown",
            "sdk_runtime": platform.python_version(),
        }
        result["compatibility"] = _compatibility(sdk_version, commands)
        return result
