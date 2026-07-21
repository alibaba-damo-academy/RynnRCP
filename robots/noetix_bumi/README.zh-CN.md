# Noetix Bumi

[English](README.md)

Noetix Bumi 的 RynnRCP 接入包，支持 high-level 动作控制、low-level 关节控制和本地行走策略。

## 前置条件

先按 [Noetix 官方文档](https://web.noetixrobotics.com/docs) 安装并编译 Bumi SDK，确保
`highcontrol_py` 和 `lowcontrol_py` 可以导入。

## 安装

先进入 Noetix Bumi 目录，再安装并激活环境：

```bash
cd robots/noetix_bumi
export BUMI_SDK_ROOT=<noetix_sdk_bumi>
bash setup_bumi.sh
source venv/bin/activate
```

安装脚本使用本地已编译的厂商 SDK，并安装 RynnBot、MCP、Protocol Debug 和 Teleop App。

## 启动

High-level：

```bash
rynnrcp-server --config rynnrcp_robot_bumi/config/bumi_high_server.yaml
```

Low-level：

```bash
rynnrcp-server --config rynnrcp_robot_bumi/config/bumi_low_server.yaml
```

High-level 和 low-level 选择其中一个 Server 启动。Server 启动后，可按需启动 App：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_bumi/config/bumi_rynnbot_app.yaml --server-config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-teleop-app
```

每个 Server 启动后都会打印 `Debug UI` 地址。需要查看状态、Action、相机图像或实时曲线时，手动在浏览器打开该地址；端口被占用时，使用对应 Server 终端打印的新地址。

启动 RynnBot App 前，在 `bumi_rynnbot_app.yaml` 中填写设备凭据。

## 能力

- 机器人状态、相机和动作库
- Low-level 关节状态、IMU、电池和关节控制
- ONNX `walk` 本地策略

Low-level 首次测试必须可靠支撑机器人。SDK 验证、动作顺序和策略调试见
[调试参考](DEBUGGING.zh-CN.md)。
