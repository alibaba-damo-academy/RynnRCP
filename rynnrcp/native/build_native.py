# Copyright 2026 RynnRCP Authors. All rights reserved.
"""Build the optional C++ acceleration module for the active Python env."""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build rynnrcp_core_native with CMake")
    parser.add_argument(
        "--build-dir",
        default=None,
        help="CMake build directory. Defaults to <repo>/build/native-<system>-<machine>.",
    )
    parser.add_argument(
        "--target-dir",
        default=None,
        help="Where to copy the built extension. Defaults to the RynnRCP repo root.",
    )
    parser.add_argument("--debug", action="store_true", help="Build Debug instead of Release")
    parser.add_argument("--no-copy", action="store_true", help="Do not copy the extension after build")
    args = parser.parse_args(argv)

    repo_root = Path(__file__).resolve().parents[2]
    system = platform.system().lower() or "unknown"
    machine = platform.machine() or "unknown"
    py_tag = f"py{sys.version_info.major}{sys.version_info.minor}"
    build_dir = Path(args.build_dir) if args.build_dir else repo_root / "build" / f"native-{system}-{machine}-{py_tag}"
    target_dir = Path(args.target_dir) if args.target_dir else repo_root
    build_type = "Debug" if args.debug else "Release"

    try:
        import pybind11  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "pybind11 is required to build the C++ extension. "
            "Install it in the current env, for example: uv pip install pybind11"
        ) from exc

    cmake_args = [
        "cmake",
        "-S",
        str(repo_root),
        "-B",
        str(build_dir),
        "-DRYNNRCP_BUILD_PYTHON_BINDINGS=ON",
        "-DPYBIND11_FINDPYTHON=ON",
        f"-Dpybind11_DIR={pybind11.get_cmake_dir()}",
        f"-DPython_EXECUTABLE={sys.executable}",
        f"-DPython3_EXECUTABLE={sys.executable}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
    ]
    if platform.system() == "Darwin" and machine in {"arm64", "x86_64"}:
        cmake_args.append(f"-DCMAKE_OSX_ARCHITECTURES={machine}")

    _run(cmake_args, cwd=repo_root)
    _run(
        [
            "cmake",
            "--build",
            str(build_dir),
            "--config",
            build_type,
            "--target",
            "rynnrcp_core_native",
            "-j",
            str(max(1, os.cpu_count() or 1)),
        ],
        cwd=repo_root,
    )

    built = _find_extension(build_dir)
    if args.no_copy:
        print(f"Built native extension: {built}")
        return 0

    target_dir.mkdir(parents=True, exist_ok=True)
    copied = target_dir / built.name
    for stale in target_dir.glob("rynnrcp_core_native*.so"):
        if stale.name != built.name:
            stale.unlink()
    shutil.copy2(built, copied)
    print(f"Built native extension: {built}")
    print(f"Copied native extension to: {copied}")
    return 0


def _run(cmd: list[str], *, cwd: Path) -> None:
    print("+ " + " ".join(cmd))
    subprocess.run(cmd, cwd=cwd, check=True)


def _find_extension(build_dir: Path) -> Path:
    candidates = sorted(
        path for path in build_dir.rglob("rynnrcp_core_native*")
        if path.suffix in {".so", ".pyd", ".dll", ".dylib"}
    )
    if not candidates:
        raise SystemExit(f"Could not find built rynnrcp_core_native extension under {build_dir}")
    expected_suffix = sysconfig.get_config_var("EXT_SUFFIX")
    if expected_suffix:
        matches = [path for path in candidates if path.name.endswith(expected_suffix)]
        if matches:
            return matches[0]
    return candidates[0]


if __name__ == "__main__":
    raise SystemExit(main())
