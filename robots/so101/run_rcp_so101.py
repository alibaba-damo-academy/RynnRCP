#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from rynnrcp import RynnRCP, RcpCore, RynnBot, McpPlugin

app = RynnRCP(
    rcp_core=RcpCore(config_file="./config/so101_config.yaml"),
    plugins=[
        RynnBot(config_file="./config/rynnbot_config.yaml"),
        McpPlugin(config_file="./config/mcp_config.yaml"),
    ],
)

print("[OK] RynnRCP 启动，接入乐云平台")
app.start()
