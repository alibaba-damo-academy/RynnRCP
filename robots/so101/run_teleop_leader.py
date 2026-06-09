#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teleop leader script with Web UI.

This script starts the leader arm with a web-based control interface.
Open the browser at http://127.0.0.1:5001 to control teleop and data collection.

Usage:
    python run_teleop_leader.py
"""

from rynnrcp import RynnRCP, RcpCore
from comm_plugin.teleop_plugin import TeleopPlugin

teleop = TeleopPlugin(
    config_file="./config/teleop_config.yaml",
    role="leader",
    enable_web_ui=True,
    web_port=5001,
    open_browser=True,
)

app = RynnRCP(
    rcp_core=RcpCore(config_file="./config/so101_leader_config.yaml"),
    plugins=[teleop],
)

print("=" * 60)
print("遥操 Leader 已启动 (Web UI 模式)")
print("请在浏览器中访问: http://127.0.0.1:5001")
print("=" * 60)

app.start()
