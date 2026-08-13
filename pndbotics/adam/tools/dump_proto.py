#!/usr/bin/env python3
"""Dump the exact adam_control.RobotControl proto from a running demo via gRPC reflection.

This is the authoritative way to obtain the real field names of the PNDbotics
Adam upper-level motion-control gRPC service — it reads the FileDescriptorSet
straight from the running server, so there is no guessing.

Requirements: grpcio + grpcio-reflection
    pip install grpcio grpcio-reflection

Usage (run on the machine that can reach the demo, default port 6666):
    python3 dump_proto.py                 # -> localhost:6666
    python3 dump_proto.py 10.10.20.127:6666
"""

from __future__ import annotations

import sys


def _fmt_proto(fd) -> str:
    """Render a FileDescriptorProto back into a readable .proto snippet."""
    lines: list[str] = []
    lines.append(f'syntax = "{fd.syntax}";')
    lines.append(f"package {fd.package};")
    lines.append("")
    for dep in fd.dependency:
        lines.append(f'import "{dep}";')
    if fd.dependency:
        lines.append("")
    for service in fd.service:
        lines.append(f"service {service.name} {{")
        for method in service.method:
            lines.append(
                f"  rpc {method.name} ({_type_name(method.input_type)}) "
                f"returns ({_type_name(method.output_type)});"
            )
        lines.append("}")
        lines.append("")
    for message in fd.message_type:
        lines.extend(_fmt_message(message, indent=""))
        lines.append("")
    return "\n".join(lines)


def _fmt_message(message, indent: str) -> list[str]:
    lines = [f"{indent}message {message.name} {{"]
    for field in message.field:
        card = "repeated " if field.label == field.LABEL_REPEATED else ""
        lines.append(
            f"{indent}  {card}{_field_type(field)} "
            f"{field.name} = {field.number};"
        )
    for nested in message.nested_type:
        lines.extend(_fmt_message(nested, indent + "  "))
    lines.append(f"{indent}}}")
    return lines


_FIELD_TYPE_NAMES = {
    1: "double", 2: "float", 3: "int64", 4: "uint64", 5: "int32",
    6: "fixed64", 7: "fixed32", 8: "bool", 9: "string", 11: "message",
    12: "bytes", 13: "uint32", 14: "enum", 15: "sfixed32", 16: "sfixed64",
    17: "sint32", 18: "sint64",
}


def _field_type(field) -> str:
    if field.type_name:
        return _type_name(field.type_name)
    return _FIELD_TYPE_NAMES.get(field.type, f"type{field.type}")


def _type_name(full: str) -> str:
    # Strip the leading package dot for readability.
    return full.lstrip(".") if full else ""


def main() -> int:
    try:
        from grpc_reflection.v1alpha import reflection_pb2, reflection_pb2_grpc
        import grpc
    except ImportError as exc:
        print(f"missing dependency: {exc}", file=sys.stderr)
        print("install with: pip install grpcio grpcio-reflection", file=sys.stderr)
        return 1

    address = sys.argv[1] if len(sys.argv) > 1 else "localhost:6666"
    channel = grpc.insecure_channel(address)
    stub = reflection_pb2_grpc.ServerReflectionStub(channel)

    def request(payload):
        yield from payload

    try:
        # 1. list services
        resp = list(
            stub.ServerReflectionInfo(
                request([reflection_pb2.ServerReflectionRequest(list_services="")]),
                timeout=5,
            )
        )
        services = [
            service.name
            for item in resp
            if item.list_services_response
            for service in item.list_services_response.service
        ]
        print(f"== services on {address} ==")
        for name in services:
            print("  -", name)
        print()

        # 2. dump the adam_control file descriptor (and any service containing 'RobotControl')
        targets = [s for s in services if "RobotControl" in s or "adam" in s.lower()]
        if not targets:
            targets = [s for s in services if s and not s.startswith("grpc.")]
        for service in targets:
            print(f"== file descriptor for {service} ==")
            resp = list(
                stub.ServerReflectionInfo(
                    request(
                        [
                            reflection_pb2.ServerReflectionRequest(
                                file_containing_symbol=service
                            )
                        ]
                    ),
                    timeout=5,
                )
            )
            for item in resp:
                if not item.file_descriptor_response:
                    continue
                for raw in item.file_descriptor_response.file_descriptor_proto:
                    from google.protobuf import descriptor_pb2

                    fd = descriptor_pb2.FileDescriptorProto()
                    fd.ParseFromString(raw)
                    print(_fmt_proto(fd))
            print()
        return 0
    except grpc.RpcError as exc:
        print(
            f"reflection failed: {exc.code()} {exc.details()}\n"
            "(the demo may not be running, or reflection is not enabled on the server)",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
