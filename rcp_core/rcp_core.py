# rcp_core/rcp_core.py

"""
rcp_core top-level runtime entrypoint.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.rcp_core.RcpCore`, a convenience wrapper that
encapsulates the system’s initialization and exposes a simple tool-oriented API.

Initialization workflow
-----------------------
:meth:`RcpCore.__init__` performs the following steps given a YAML config file path:

1) Load config
   - Reads YAML via :meth:`load_config`.
   - Exits the process if the file cannot be found.

2) Initialize logging identity
   - Calls :func:`~rcp_core.common.utils.logger.init_process_logging` using the config file
     to derive robot_name and allocate a unique APPID.

3) Validate configuration
   - Uses :class:`~rcp_core.common.config_manager.validator.ConfigValidator` to validate
     the loaded configuration structure and adapter/protocol fields.

4) Create the tool bus and protocol layer
   - Instantiates :class:`~rcp_core.common.bus.rcp_bus.RcpBus`.
   - Instantiates :class:`~rcp_core.common.protocol.protocol_factory.ProtocolFactory`,
     which creates protocol adapters (ROS2/LCM/module/port) based on config usage and
     runtime availability.

5) Initialize servers and bind them
   - Creates:
     - :class:`~rcp_core.sensor_server.sensor_server.SensorServer`
     - :class:`~rcp_core.action_server.action_server.ActionServer`
     - :class:`~rcp_core.device_monitor_server.device_monitor_server.DeviceMonitorServer`
   - Binds each server to the bus (registering tools).
   - Binds protocol adapters for SensorServer and ActionServer (subscribing to inputs).

Tool-oriented API
-----------------
- :meth:`tool_list` returns the bus tool registry (same as ``bus.list_tool()``).
- :meth:`tool_call` invokes a tool by name (same as ``bus.call_tool(...)``), intended as
  the external-facing entrypoint for interacting with the runtime.

Shutdown
--------
- :meth:`stop` stops protocol adapters via ``protocol_factory.stop()`` (ROS 2 shutdown,
  LCM loop stop, module/port polling stop, etc.).
"""

import json
import yaml
from typing import Dict, Any, List

from .common.bus.rcp_bus import RcpBus
from .common.config_manager.validator import ConfigValidator
from .common.protocol.protocol_factory import ProtocolFactory
from .sensor_server.sensor_server import SensorServer
from .action_server.action_server import ActionServer
from .device_monitor_server.device_monitor_server import DeviceMonitorServer
from .data_server.data_server import DataServer
from .common.utils.logger import init_process_logging
from .common.utils.logger import server_logger


class RcpCore:
    """
    Core framework that encapsulates the original main initialization logic.

    It exposes:
      - tool_list(): query a list of available tools
      - tool_call(): invoke a tool (internally calls bus.call_tool)
    """

    def __init__(self, config_file: str):
        """
        Initialize RcpCore from a YAML configuration file.

        Steps:
        - Load and validate configuration.
        - Create the bus and middleware adapter.
        - Initialize all servers and bind them to the bus and middleware.
        """
        try:
            self.config = RcpCore.load_config(config_file)
        except FileNotFoundError:
            print("[ERR] Config file not found")
            raise SystemExit(1)

        init_process_logging(robot_config_path=config_file, appid_default=0)

        self.logger = server_logger()

        # 1. Validate configuration
        validator = ConfigValidator()
        validator.validate_config(self.config)

        # 2. Bus & protocol
        self.bus = RcpBus()
        self.protocol_factory = ProtocolFactory(config=self.config)

        # 3. Servers
        self.sensor_server = SensorServer(self.config)
        self.action_server = ActionServer(self.config)
        self.device_monitor_server = DeviceMonitorServer(self.config)
        self.data_server = DataServer(self.config)

        # 4. Bind servers to bus
        self.sensor_server.bind_bus(self.bus)
        self.action_server.bind_bus(self.bus)
        self.device_monitor_server.bind_bus(self.bus)
        self.data_server.bind_bus(self.bus)

        # 5. Bind middleware (subscribe to ROS2 / LCM)
        self.sensor_server.bind_adapter(self.protocol_factory)
        self.action_server.bind_adapter(self.protocol_factory)

        self.logger.info("[OK] RcpCore initialized, available tools:")
        # print(json.dumps(self.bus.list_tool(), indent=2, ensure_ascii=False))

    @staticmethod
    def load_config(path: str) -> Dict[str, Any]:
        """Load configuration from a YAML file and return it as a dictionary."""
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def tool_list(self) -> List[Dict[str, Any]]:
        """
        Return the list of available tools exposed by the bus.

        This is equivalent to self.bus.list_tool().

        Example return structure:
        {
          "get_state": {
            "description": "...",
            "input": {...},   # input schema
            "output": {...},  # output schema
          },
          "get_image": {
            "description": "...",
            "input": {...},
            "output": {...},
          },
          ...
        }
        """
        return self.bus.list_tool()

    def tool_call(self, tool_name: str, *args, **kwargs) -> Any:
        """
        Call a bus tool by name.

        External callers use this method instead of calling bus.call_tool directly.

        Example:
            rcp_core.tool_call("get_state")
            rcp_core.tool_call(
                "get_image",
                {
                    "observation.images.wrist": {
                        "encoding": "png",
                        "width": 320,
                        "height": 240,
                    }
                },
            )
        """
        return self.bus.call_tool(tool_name, *args, **kwargs)

    def stop(self):
        """Cleanly shut down middleware and any other resources that need explicit cleanup."""
        if hasattr(self, "protocol_factory") and self.protocol_factory is not None:
            try:
                self.protocol_factory.stop()
            except Exception as e:
                self.logger.error(f"[RcpCore] protocol_factory.stop() exception: {e}")
