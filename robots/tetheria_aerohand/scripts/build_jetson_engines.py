#!/usr/bin/env python3
"""Build TensorRT engines for the mediapipe_lite_jetson backend on THIS device.

TensorRT engines are not portable: they bind to the TensorRT version and GPU
SM of the machine that built them. Run this script once on every fresh
ORIN NX device (or let the backend build engines lazily on first use).

  python scripts/build_jetson_engines.py            # build missing engines
  python scripts/build_jetson_engines.py --force    # rebuild unconditionally
  python scripts/build_jetson_engines.py --no-fp16  # FP32 engines

Engines are cached under rynnrcp_robot_aero_hand/models/engine/ with a
versioned name (*_trt<version>_sm<cc>.engine) and verified with a dummy
inference before being accepted. If this script fails, the backend still
works via its onnxruntime CUDA/CPU fallback.
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time
import types

_REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _import_jetson_backend():
    """Load the accelerator module without executing the package __init__,
    which depends on rynnrcp; the engine builder only needs cv2/numpy/TRT."""
    if "rynnrcp_robot_aero_hand" not in sys.modules:
        pkg_root = os.path.join(_REPO_DIR, "rynnrcp_robot_aero_hand")
        pkg = types.ModuleType("rynnrcp_robot_aero_hand")
        pkg.__path__ = [pkg_root]
        sys.modules["rynnrcp_robot_aero_hand"] = pkg
        acc = types.ModuleType("rynnrcp_robot_aero_hand.accelerators")
        acc.__path__ = [os.path.join(pkg_root, "accelerators")]
        sys.modules["rynnrcp_robot_aero_hand.accelerators"] = acc
    sys.path.insert(0, _REPO_DIR)
    import importlib

    return importlib.import_module(
        "rynnrcp_robot_aero_hand.accelerators.mediapipe_lite_jetson"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--force", action="store_true",
                        help="rebuild even if a valid cached engine exists")
    parser.add_argument("--no-fp16", action="store_true",
                        help="build FP32 engines instead of FP16")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    jetson = _import_jetson_backend()

    print(f"engine tag: {jetson.engine_tag()}")
    print(f"engine dir: {jetson.ENGINE_DIR}")
    started = time.time()
    try:
        built = jetson.build_default_engines(force=args.force,
                                             fp16=not args.no_fp16)
    except Exception as exc:
        print(f"\nERROR: engine build failed: {exc}", file=sys.stderr)
        print("The backend will fall back to onnxruntime (CUDA EP when the "
              "Jetson onnxruntime-gpu wheel is installed, else CPU EP).",
              file=sys.stderr)
        return 1
    print(f"\nOK in {time.time() - started:.1f}s:")
    for path in built:
        size_mb = os.path.getsize(path) / 1e6
        print(f"  {path}  ({size_mb:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
