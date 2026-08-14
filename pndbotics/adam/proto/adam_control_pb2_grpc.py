"""gRPC bindings for the official Adam RobotControl service."""

import grpc

import adam_control_pb2 as adam_control__pb2


class RobotControlStub:
    def __init__(self, channel):
        self.SetMode = channel.unary_unary(
            "/adam_control.RobotControl/SetMode",
            request_serializer=adam_control__pb2.SetModeRequest.SerializeToString,
            response_deserializer=adam_control__pb2.SetModeResponse.FromString,
        )
        self.SetStandMotion = channel.unary_unary(
            "/adam_control.RobotControl/SetStandMotion",
            request_serializer=adam_control__pb2.SetStandMotionRequest.SerializeToString,
            response_deserializer=adam_control__pb2.SetStandMotionResponse.FromString,
        )
        self.SetStandCarryBox = channel.unary_unary(
            "/adam_control.RobotControl/SetStandCarryBox",
            request_serializer=adam_control__pb2.SetCarryBoxRequest.SerializeToString,
            response_deserializer=adam_control__pb2.SetCarryBoxResponse.FromString,
        )
        self.SetStandAction = channel.unary_unary(
            "/adam_control.RobotControl/SetStandAction",
            request_serializer=adam_control__pb2.SetActionRequest.SerializeToString,
            response_deserializer=adam_control__pb2.SetActionResponse.FromString,
        )
        self.SetStandDynamic = channel.unary_unary(
            "/adam_control.RobotControl/SetStandDynamic",
            request_serializer=adam_control__pb2.SetDynamicStandRequest.SerializeToString,
            response_deserializer=adam_control__pb2.SetDynamicStandResponse.FromString,
        )
        self.SetSpeed = channel.unary_unary(
            "/adam_control.RobotControl/SetSpeed",
            request_serializer=adam_control__pb2.SetSpeedRequest.SerializeToString,
            response_deserializer=adam_control__pb2.SetSpeedResponse.FromString,
        )
        self.AutoUnigaitCOM = channel.unary_unary(
            "/adam_control.RobotControl/AutoUnigaitCOM",
            request_serializer=adam_control__pb2.SetUnigaitCOMRequest.SerializeToString,
            response_deserializer=adam_control__pb2.SetUnigaitCOMResponse.FromString,
        )
        self.SetErrorClear = channel.unary_unary(
            "/adam_control.RobotControl/SetErrorClear",
            request_serializer=adam_control__pb2.SetErrorClearRequest.SerializeToString,
            response_deserializer=adam_control__pb2.SetErrorClearResponse.FromString,
        )
        self.GetStandList = channel.unary_unary(
            "/adam_control.RobotControl/GetStandList",
            request_serializer=adam_control__pb2.GetStandListRequest.SerializeToString,
            response_deserializer=adam_control__pb2.GetStandListResponse.FromString,
        )
        self.GetRobotState = channel.unary_unary(
            "/adam_control.RobotControl/GetRobotState",
            request_serializer=adam_control__pb2.GetRobotStateRequest.SerializeToString,
            response_deserializer=adam_control__pb2.GetRobotStateResponse.FromString,
        )
        self.CloseProgram = channel.unary_unary(
            "/adam_control.RobotControl/CloseProgram",
            request_serializer=adam_control__pb2.CloseProgramRequest.SerializeToString,
            response_deserializer=adam_control__pb2.CloseProgramResponse.FromString,
        )


class RobotControlServicer:
    """Base class retained for compatibility with generated gRPC modules."""

    def SetMode(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def SetStandMotion(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def SetStandCarryBox(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def SetStandAction(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def SetStandDynamic(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def SetSpeed(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def AutoUnigaitCOM(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def SetErrorClear(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def GetStandList(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def GetRobotState(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def CloseProgram(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")


def add_RobotControlServicer_to_server(servicer, server):
    rpc_method_handlers = {
        "SetMode": grpc.unary_unary_rpc_method_handler(
            servicer.SetMode,
            request_deserializer=adam_control__pb2.SetModeRequest.FromString,
            response_serializer=adam_control__pb2.SetModeResponse.SerializeToString,
        ),
        "SetStandMotion": grpc.unary_unary_rpc_method_handler(
            servicer.SetStandMotion,
            request_deserializer=adam_control__pb2.SetStandMotionRequest.FromString,
            response_serializer=adam_control__pb2.SetStandMotionResponse.SerializeToString,
        ),
        "SetStandCarryBox": grpc.unary_unary_rpc_method_handler(
            servicer.SetStandCarryBox,
            request_deserializer=adam_control__pb2.SetCarryBoxRequest.FromString,
            response_serializer=adam_control__pb2.SetCarryBoxResponse.SerializeToString,
        ),
        "SetStandAction": grpc.unary_unary_rpc_method_handler(
            servicer.SetStandAction,
            request_deserializer=adam_control__pb2.SetActionRequest.FromString,
            response_serializer=adam_control__pb2.SetActionResponse.SerializeToString,
        ),
        "SetStandDynamic": grpc.unary_unary_rpc_method_handler(
            servicer.SetStandDynamic,
            request_deserializer=adam_control__pb2.SetDynamicStandRequest.FromString,
            response_serializer=adam_control__pb2.SetDynamicStandResponse.SerializeToString,
        ),
        "SetSpeed": grpc.unary_unary_rpc_method_handler(
            servicer.SetSpeed,
            request_deserializer=adam_control__pb2.SetSpeedRequest.FromString,
            response_serializer=adam_control__pb2.SetSpeedResponse.SerializeToString,
        ),
        "AutoUnigaitCOM": grpc.unary_unary_rpc_method_handler(
            servicer.AutoUnigaitCOM,
            request_deserializer=adam_control__pb2.SetUnigaitCOMRequest.FromString,
            response_serializer=adam_control__pb2.SetUnigaitCOMResponse.SerializeToString,
        ),
        "SetErrorClear": grpc.unary_unary_rpc_method_handler(
            servicer.SetErrorClear,
            request_deserializer=adam_control__pb2.SetErrorClearRequest.FromString,
            response_serializer=adam_control__pb2.SetErrorClearResponse.SerializeToString,
        ),
        "GetStandList": grpc.unary_unary_rpc_method_handler(
            servicer.GetStandList,
            request_deserializer=adam_control__pb2.GetStandListRequest.FromString,
            response_serializer=adam_control__pb2.GetStandListResponse.SerializeToString,
        ),
        "GetRobotState": grpc.unary_unary_rpc_method_handler(
            servicer.GetRobotState,
            request_deserializer=adam_control__pb2.GetRobotStateRequest.FromString,
            response_serializer=adam_control__pb2.GetRobotStateResponse.SerializeToString,
        ),
        "CloseProgram": grpc.unary_unary_rpc_method_handler(
            servicer.CloseProgram,
            request_deserializer=adam_control__pb2.CloseProgramRequest.FromString,
            response_serializer=adam_control__pb2.CloseProgramResponse.SerializeToString,
        ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
        "adam_control.RobotControl", rpc_method_handlers
    )
    server.add_generic_rpc_handlers((generic_handler,))
