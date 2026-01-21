# scripts/gen_proto_msg.py
"""
Generate *_pb2.py from .proto files into common/proto using grpc_tools.protoc.

Usage (from repo root, e.g. RynnRCP/):

    uv pip install grpcio-tools  # or: pip install grpcio-tools
    python scripts/gen_proto_msg.py
"""

from __future__ import annotations
from pathlib import Path

try:
    from grpc_tools import protoc
except ImportError as e:
    print(
        "ERROR: grpcio-tools is not installed.\n"
        "Install it first, e.g.:\n"
        "    uv pip install grpcio-tools\n"
        "or: pip install grpcio-tools\n"
    )
    raise SystemExit(1) from e


def main() -> int:
    root = Path(__file__).resolve().parents[1]

    proto_dir = root / "comm_plugin" / "rynnbot_plugin" / "proto"

    out_dir = root / "comm_plugin" / "rynnbot_plugin" / "proto"
    out_dir.mkdir(parents=True, exist_ok=True)

    init_file = out_dir / "__init__.py"
    if not init_file.exists():
        init_file.write_text("# Generated proto package\n", encoding="utf-8")

    proto_files = [
        "RobotServerData.proto",
        "RobotServerCombinePacket.proto",
        "RobotServerTransportPacket.proto",
        "TunnelTransportService.proto",
    ]

    args = [
        "protoc",
        f"-I{proto_dir}",
        f"--python_out={out_dir}",
    ] + [str(proto_dir / f) for f in proto_files]

    print("Running protoc with args:")
    print("  ", " ".join(args))

    ret = protoc.main(args)
    if ret != 0:
        print(f"protoc failed with exit code {ret}")
    else:
        print("proto generation finished successfully.")

    return ret


if __name__ == "__main__":
    raise SystemExit(main())
