# rcp_core/device_monitor_server/device_monitor_server.py

"""
Device/robot info server.
~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.device_monitor_server.device_monitor_server.DeviceMonitorServer`,
a :class:`~rcp_core.common.server.base_server.BaseServer` subclass that exposes a bus tool
to retrieve basic robot identity and host resource information.

What it provides
----------------
A single bus tool, ``get_device_info``, returning a unified result dict containing:
- robot/device identity:
  - ``arch``: CPU architecture (via :mod:`platform`)
  - ``arm_info``: robot name/model (from config ``device_monitor_server.robot_info.robot_name``)
  - ``distrib_desc``: OS description (Linux via ``distro`` if available; otherwise platform fallback)
  - ``kernel``: kernel/release version
- resource usage:
  - ``cpu_load``: current CPU utilization (via :mod:`psutil`)
  - ``mem``: total memory in KB (as a string)
  - ``mem_used``: used memory in KB (as a string)
- camera_info:
  - gathered indirectly by calling the bus tool ``get_image_info`` (expected to be provided
    by the sensor server), then normalizing it into a list of camera descriptors
    ``{id, name, width, height, brand}``.

How it works
------------
- On each ``get_device_info`` call, the server rebuilds ``self._device_info`` using
  helper methods and returns it wrapped by :meth:`RcpBus.make_result`.
- Camera configuration is not read directly from YAML here; it is obtained by calling
  ``self.bus.call_tool("get_image_info")`` and transforming the returned mapping.

Bus registration
----------------
:meth:`DeviceMonitorServer.bind_bus` registers ``get_device_info`` with a descriptive
output schema so other components can discover and call it via :class:`~rcp_core.common.bus.rcp_bus.RcpBus`.
"""

from __future__ import annotations

import platform
from typing import Any, Dict, List

import psutil

from ..common.server.base_server import BaseServer
from ..common.bus.rcp_bus import RcpBus


class DeviceMonitorServer(BaseServer):
    """Device monitoring server providing basic robot info and system resource usage.

    Camera information is sourced from:
      - Parsed from the sensor_server.inputs in the entire config:
        * protocol: module
        * adapter: ImageModuleInputAdapter
        * params.init_args: {device_id, width, height, encoding, fps, brand}
        * params.out_key: observation.images.xxx
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize the server and load robot info from configuration."""
        # BaseServer will place this server's configuration in self.server_config
        super().__init__(config, "device_monitor_server")

        robot_info = self.server_config.get("robot_info", {})
        self._robot_name = robot_info.get("robot_name", "robot")

        self._device_info: Dict[str, Any] = {}

    def _parse_cameras_from_config(self) -> List[Dict[str, Any]]:
        """
        Call the sensor_server's get_image_info tool to retrieve camera information.
        """

        resp: Dict[str, Any] = self.bus.call_tool("get_image_info")
        if not resp.get("success"):
            raise RuntimeError(
                f"[DeviceMonitorServer] get_image_info call failed: {resp.get('message')}"
            )

        image_info: Dict[str, Any] = resp.get("result", {})

        cameras: List[Dict[str, Any]] = []

        for cam_id, (camera_name, cam_params) in enumerate(image_info.items()):
            cameras.append(
                {
                    "id": cam_id,
                    "name": camera_name,
                    "width": cam_params.get("width", 0),
                    "height": cam_params.get("height", 0),
                    "brand": cam_params.get("brand", "unknown"),
                }
            )
        return cameras

    def _update_device_info(self) -> Dict[str, Any]:
        """Refresh and return the current device information dictionary."""
        self._device_info = {
            "arch": self._get_architecture(),
            "arm_info": self._robot_name,
            "distrib_desc": self._get_distrib_desc(),
            "kernel": self._get_kernel_version(),
            "mem": self._get_total_memory(),
            "camera_info": self._parse_cameras_from_config(),
            "cpu_load": self._get_cpu_load(),
            "mem_used": self._get_mem_used(),
        }

        return self._device_info

    def get_device_info(self) -> Dict[str, Any]:
        """Return current device and robot info including resource usage."""
        info = self._update_device_info()

        return self.bus.make_result(
            success=True,
            result=info,
            message="OK",
        )

    def _get_architecture(self) -> str:
        try:
            return platform.machine() or "unknown"
        except Exception:
            return "unknown"

    def _get_cpu_load(self) -> str:
        try:
            return f"{psutil.cpu_percent():.2f}%"
        except Exception:
            return "unknown"

    def _get_kernel_version(self) -> str:
        try:
            return platform.release() or "unknown"
        except Exception:
            return "unknown"

    def _get_total_memory(self) -> str:
        try:
            return f"{psutil.virtual_memory().total / 1024:.0f}"
        except Exception:
            return "unknown"

    def _get_mem_used(self) -> str:
        try:
            vm = psutil.virtual_memory()
            used = (vm.total - vm.available) / 1024
            return f"{used:.0f}"
        except Exception:
            return "unknown"

    def _get_distrib_desc(self) -> str:
        try:
            system = platform.system()
            if system == "Linux":
                try:
                    import distro

                    return distro.name(pretty=True) or "Linux"
                except Exception:
                    return platform.platform()
            if system == "Darwin":
                return f"macOS {platform.mac_ver()[0]}"
            if system == "Windows":
                return f"Windows {platform.version()}"
            return system
        except Exception:
            return "unknown"

    def bind_bus(self, bus: RcpBus) -> None:
        """Register get_device_info on the bus for external calls."""
        super().bind_bus(bus)
        bus.add_tool(
            "get_device_info",
            self.get_device_info,
            input_schema=None,
            output_schema={
                "success": "bool",
                "message": "str",
                "result": {
                    "arch": "str  # CPU architecture, e.g. 'x86_64'/'aarch64'",
                    "arm_info": "str  # robot name/model, e.g. 'robot'",
                    "distrib_desc": "str  # OS description, e.g. 'Ubuntu 22.04 LTS'",
                    "kernel": "str  # kernel version, e.g. '5.15.0-...'",
                    "mem": "str  # total memory (KB) as string",
                    "cpu_load": "str  # current CPU usage, e.g. '12.34%'",
                    "mem_used": "str  # used memory (KB) as string",
                    "camera_info": [
                        {
                            "id": "int | str  # camera id",
                            "name": "str  # camera name",
                            "brand": "str | None  # camera brand",
                            "width": "int | None  # image width",
                            "height": "int | None  # image height",
                            "encoding": "str | None  # image encoding, e.g. 'jpeg'/'png'",
                        }
                    ],
                },
            },
            description=(
                "Get basic device info (OS/CPU/memory) and camera config from sensor_server configuration."
            ),
        )
