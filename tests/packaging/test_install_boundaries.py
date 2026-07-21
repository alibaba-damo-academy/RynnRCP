"""Tests for public package dependency boundaries."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib  # type: ignore[no-redef]


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _pyproject(path: str) -> dict:
    with (PROJECT_ROOT / path / "pyproject.toml").open("rb") as fh:
        return tomllib.load(fh)


def _dependency_names(config: dict) -> set[str]:
    dependencies = config.get("project", {}).get("dependencies", [])
    names = set()
    for dependency in dependencies:
        head = str(dependency).split(";", 1)[0].strip()
        for separator in ("[", "<", ">", "=", "~", "!", " "):
            head = head.split(separator, 1)[0]
        names.add(head.lower())
    return names


def test_core_package_keeps_robot_app_and_cloud_dependencies_out() -> None:
    core = _dependency_names(_pyproject("."))

    assert {"numpy", "pyyaml", "msgpack", "lcm", "psutil"}.issubset(core)
    assert core.isdisjoint({
        "flask",
        "flask-socketio",
        "opencv-python",
        "fastmcp",
        "lerobot",
        "pyserial",
        "paho-mqtt",
        "websocket-client",
        "protobuf",
        "alibabacloud-oss-v2",
        "pillow",
        "requests",
    })


def test_app_packages_own_their_optional_dependencies() -> None:
    common = _dependency_names(_pyproject("apps/common"))
    mcp = _dependency_names(_pyproject("apps/mcp"))
    rynnbot = _dependency_names(_pyproject("apps/rynnbot"))
    teleop = _dependency_names(_pyproject("apps/teleop"))

    assert {"rynnrcp", "opencv-python-headless"}.issubset(common)
    assert {"rynnrcp", "fastmcp"}.issubset(mcp)
    assert {
        "rynnrcp",
        "rynnrcp-app-common",
        "paho-mqtt",
        "websocket-client",
        "protobuf",
        "alibabacloud-oss-v2",
        "opencv-python-headless",
        "pillow",
        "requests",
    }.issubset(rynnbot)
    assert {
        "rynnrcp",
        "rynnrcp-app-common",
        "flask",
        "pyyaml",
    }.issubset(teleop)


def test_rynnkit_owns_hardware_capability_dependencies() -> None:
    config = _pyproject("rynnkit")
    dependencies = _dependency_names(config)
    realsense = config["project"]["optional-dependencies"]["realsense"]

    assert {"numpy", "opencv-python-headless"}.issubset(dependencies)
    assert "pyrealsense2" in {str(item).split(";", 1)[0].strip() for item in realsense}


def test_noetix_bumi_package_declares_robot_layer_dependencies_and_assets() -> None:
    config = _pyproject("robots/noetix_bumi")
    dependencies = _dependency_names(config)
    robot_entry_points = config["project"]["entry-points"]["rynnrcp.robots"]
    package_data = config["tool"]["setuptools"]["package-data"]["rynnrcp_robot_bumi"]

    assert {"onnxruntime", "rynnrcp", "rynnkit"}.issubset(dependencies)
    assert robot_entry_points == {"bumi": "rynnrcp_robot_bumi"}
    assert "policies/*/*.onnx" in package_data


def test_so101_package_declares_robot_layer_dependencies_and_entry_points() -> None:
    config = _pyproject("robots/lerobot_so101")
    dependencies = _dependency_names(config)
    scripts = config["project"]["scripts"]
    robot_entry_points = config["project"]["entry-points"]["rynnrcp.robots"]

    assert {
        "rynnrcp",
        "feetech-servo-sdk",
        "pyserial",
        "opencv-python-headless",
        "flask",
    }.issubset(dependencies)
    assert "lerobot" not in dependencies
    assert "lerobot_so101*" in config["tool"]["setuptools"]["packages"]["find"]["include"]
    assert set(scripts) == {
        "rynnrcp-so101-configure",
    }
    assert robot_entry_points == {"so101": "rynnrcp_robot_so101"}


def test_aero_hand_package_declares_camera_master_dependencies_and_assets() -> None:
    config = _pyproject("robots/tetheria_aerohand")
    dependencies = _dependency_names(config)
    package_data = config["tool"]["setuptools"]["package-data"]["rynnrcp_robot_aero_hand"]

    assert {"rynnrcp", "pyserial", "mediapipe", "numpy", "opencv-python", "pyyaml"}.issubset(dependencies)
    assert "model/*.task" in package_data


def test_core_import_smoke_does_not_pull_robot_or_teleop_modules() -> None:
    code = """
import sys
import rynnrcp
import rynnkit

blocked = [
    name for name in (
        "rynnrcp_robot_so101",
        "rynnrcp_app_teleop",
        "cv2",
        "flask",
        "lerobot",
        "serial",
    )
    if name in sys.modules
]
if blocked:
    raise SystemExit(f"unexpected imports: {blocked}")
print(rynnrcp.__name__, rynnkit.__name__)
"""
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(PROJECT_ROOT),
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )

    assert result.stdout.strip() == "rynnrcp rynnkit"
