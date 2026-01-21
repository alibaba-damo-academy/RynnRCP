#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from rynnrcp import RynnRCP, RcpCore, RynnBot

app = RynnRCP(
    rcp_core=RcpCore(config_file="./config/so100_config.yaml"),
    plugins=[
        RynnBot(config_file="./config/rynnbot_config.yaml"),
    ],
)

print("✅ 启动 RynnRCP 启动，接入乐云平台")
app.start()
