#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from rynnrcp import RynnRCP, RcpCore, TeleopPlugin

app = RynnRCP(
    rcp_core=RcpCore(config_file="./config/so101_config.yaml"),
    plugins=[
        TeleopPlugin(config_file="./config/teleop_config.yaml", role="follower"),
    ],
)

print("✅ 遥操 Follower 已启动")
app.start()
