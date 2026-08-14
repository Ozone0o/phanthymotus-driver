"""Python protobuf bindings for PNDbotics' official Adam proto.

This module is intentionally kept self-contained.  The production image has
``protobuf`` (through grpcio), but does not need the code-generation tool at
runtime.  The descriptor below is the descriptor defined by
``adam_control.proto`` from the PNDbotics gRPC client SDK v1.0.0.
"""

from google.protobuf import descriptor_pb2 as _descriptor_pb2
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf.internal import builder as _builder


_file = _descriptor_pb2.FileDescriptorProto()
_file.name = "adam_control.proto"
_file.package = "adam_control"
_file.syntax = "proto3"


def _message(name, fields):
    message = _file.message_type.add()
    message.name = name
    for field_name, number, field_type, repeated in fields:
        field = message.field.add()
        field.name = field_name
        field.number = number
        field.type = field_type
        field.label = (
            _descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED
            if repeated
            else _descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
        )


_F_BOOL = _descriptor_pb2.FieldDescriptorProto.TYPE_BOOL
_F_DOUBLE = _descriptor_pb2.FieldDescriptorProto.TYPE_DOUBLE
_F_STRING = _descriptor_pb2.FieldDescriptorProto.TYPE_STRING


_message("SetModeRequest", [("mode", 1, _F_STRING, False)])
_message("SetModeResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])

_message("SetStandMotionRequest", [("motion", 1, _F_STRING, False)])
_message("SetStandMotionResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])

_message("SetCarryBoxRequest", [("carry_box", 1, _F_STRING, False)])
_message("SetCarryBoxResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])

_message(
    "SetActionRequest",
    [
        ("stand_pitch", 1, _F_DOUBLE, False),
        ("stand_roll", 2, _F_DOUBLE, False),
        ("stand_yaw", 3, _F_DOUBLE, False),
        ("stand_height", 4, _F_DOUBLE, False),
    ],
)
_message("SetActionResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])

_message("SetDynamicStandRequest", [("dynamic_stand", 1, _F_BOOL, False)])
_message("SetDynamicStandResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])

_message(
    "SetSpeedRequest",
    [
        ("x_speed", 1, _F_DOUBLE, False),
        ("y_speed", 2, _F_DOUBLE, False),
        ("yaw_speed", 3, _F_DOUBLE, False),
        # The spelling ``continous`` is part of the official proto and is
        # kept verbatim for wire/API compatibility.
        ("continous", 4, _F_BOOL, False),
    ],
)
_message("SetSpeedResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])

_message("SetUnigaitCOMRequest", [("unigait_mode_com_x", 1, _F_BOOL, False)])
_message("SetUnigaitCOMResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])

_message("SetErrorClearRequest", [("error_clear_flag", 1, _F_BOOL, False)])
_message("SetErrorClearResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])

_message("GetStandListRequest", [("mode_list_req", 1, _F_BOOL, False)])
_message(
    "GetStandListResponse",
    [
        ("success", 1, _F_BOOL, False),
        ("message", 2, _F_STRING, False),
        ("mode_list", 3, _F_STRING, True),
        ("motion_list", 4, _F_STRING, True),
        ("action_list", 5, _F_STRING, True),
        ("carrybox_list", 6, _F_STRING, True),
        ("balance_control", 7, _F_STRING, False),
    ],
)

_message("GetRobotStateRequest", [("get_state_flag", 1, _F_BOOL, False)])
_message(
    "GetRobotStateResponse",
    [
        ("success", 1, _F_BOOL, False),
        ("message", 2, _F_STRING, False),
        ("fsm_name", 3, _F_STRING, False),
        ("current_motion", 4, _F_STRING, False),
        ("current_action_list", 5, _F_STRING, True),
        ("mode_enable_list", 6, _F_STRING, True),
        ("motion_enable_list", 7, _F_STRING, True),
        ("action_enable_list", 8, _F_STRING, True),
        ("carrybox_enable_list", 9, _F_STRING, True),
        ("balance_control_enable", 10, _F_STRING, False),
        ("stand_pitch", 11, _F_DOUBLE, False),
        ("stand_roll", 12, _F_DOUBLE, False),
        ("stand_yaw", 13, _F_DOUBLE, False),
        ("stand_height", 14, _F_DOUBLE, False),
        ("x_vel", 15, _F_DOUBLE, False),
        ("y_vel", 16, _F_DOUBLE, False),
        ("yaw_vel", 17, _F_DOUBLE, False),
        ("balance_control_state", 18, _F_BOOL, False),
        ("motion_files_enable", 19, _F_BOOL, False),
    ],
)

_message("CloseProgramRequest", [("close_flag", 1, _F_BOOL, False)])
_message("CloseProgramResponse", [("success", 1, _F_BOOL, False), ("message", 2, _F_STRING, False)])


_service = _file.service.add()
_service.name = "RobotControl"
for _name, _request, _response in (
    ("SetMode", "SetModeRequest", "SetModeResponse"),
    ("SetStandMotion", "SetStandMotionRequest", "SetStandMotionResponse"),
    ("SetStandCarryBox", "SetCarryBoxRequest", "SetCarryBoxResponse"),
    ("SetStandAction", "SetActionRequest", "SetActionResponse"),
    ("SetStandDynamic", "SetDynamicStandRequest", "SetDynamicStandResponse"),
    ("SetSpeed", "SetSpeedRequest", "SetSpeedResponse"),
    ("AutoUnigaitCOM", "SetUnigaitCOMRequest", "SetUnigaitCOMResponse"),
    ("SetErrorClear", "SetErrorClearRequest", "SetErrorClearResponse"),
    ("GetStandList", "GetStandListRequest", "GetStandListResponse"),
    ("GetRobotState", "GetRobotStateRequest", "GetRobotStateResponse"),
    ("CloseProgram", "CloseProgramRequest", "CloseProgramResponse"),
):
    _method = _service.method.add()
    _method.name = _name
    _method.input_type = f".adam_control.{_request}"
    _method.output_type = f".adam_control.{_response}"


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(_file.SerializeToString())
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, "adam_control_pb2", globals())
