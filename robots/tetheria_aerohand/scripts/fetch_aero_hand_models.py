#!/usr/bin/env python3
"""Fetch Aero Hand inference models from the Rynn model zoo (gitee).

Weights are not git-tracked anymore; this script materializes them under the
repo-root ``models/`` tree exactly where the accelerator backends look.
Backends also do this automatically on first use, so pre-fetching is only
needed for offline boards or provisioning images.

Examples:

  python fetch_aero_hand_models.py                    # current platform
  python fetch_aero_hand_models.py --platform rk3588
  python fetch_aero_hand_models.py --platform all --dest /opt/rynn/models
"""
from __future__ import annotations

import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from rynnrcp_robot_aero_hand import model_store  # noqa: E402
    from rynnrcp_robot_aero_hand.platform_detect import (  # noqa: E402
        PLATFORM_JETSON,
        PLATFORM_MTNN,
        PLATFORM_RK3566,
        PLATFORM_RK3588,
        detect_platform,
    )
except ImportError:
    # Bare checkout without rynnrcp installed: the package __init__ pulls the
    # robot controller, so load the two self-contained modules directly.
    import importlib.util
    import types

    _pkg_dir = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "rynnrcp_robot_aero_hand")
    _pkg = types.ModuleType("rynnrcp_robot_aero_hand")
    _pkg.__path__ = [_pkg_dir]
    sys.modules["rynnrcp_robot_aero_hand"] = _pkg
    for _name in ("platform_detect", "model_store"):
        _spec = importlib.util.spec_from_file_location(
            f"rynnrcp_robot_aero_hand.{_name}",
            os.path.join(_pkg_dir, f"{_name}.py"),
        )
        _mod = importlib.util.module_from_spec(_spec)
        sys.modules[_spec.name] = _mod
        setattr(_pkg, _name, _mod)
        _spec.loader.exec_module(_mod)
    model_store = sys.modules["rynnrcp_robot_aero_hand.model_store"]
    _pd = sys.modules["rynnrcp_robot_aero_hand.platform_detect"]
    PLATFORM_JETSON = _pd.PLATFORM_JETSON
    PLATFORM_MTNN = _pd.PLATFORM_MTNN
    PLATFORM_RK3566 = _pd.PLATFORM_RK3566
    PLATFORM_RK3588 = _pd.PLATFORM_RK3588
    detect_platform = _pd.detect_platform

_CHOICES = ("auto", PLATFORM_RK3566, PLATFORM_RK3588, PLATFORM_MTNN,
            PLATFORM_JETSON, "all")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--platform", default="auto", choices=_CHOICES,
        help="which model set to fetch (default: detect this device)",
    )
    parser.add_argument("--dest", default=None,
                        help="models root (default: <repo root>/models)")
    parser.add_argument("--force", action="store_true",
                        help="re-download even if files already verify")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    platform = args.platform
    if platform == "auto":
        platform = detect_platform()
        print(f"detected platform: {platform}")

    if platform == "all":
        fetched: list[str] = []
        for subdir in model_store.MANIFEST:
            fetched.extend(model_store.ensure_subdir(
                subdir, args.dest, force=args.force))
    else:
        fetched = model_store.ensure_models(platform, args.dest,
                                            force=args.force)
    for path in fetched:
        print(f"  {path}")
    print(f"done: {len(fetched)} model file(s) verified under "
          f"{args.dest or model_store.models_root()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
