# Booster T1

[English](README.md)

Booster T1 的 RynnRCP 接入包，同时支持 high-level 运控和 low-level `LowCmd` 策略控制。

## 前置条件

- Booster T1 运控板
- Python 环境可安装 `booster_robotics_sdk_python==1.5.6`
- Booster motion/loco 服务正常运行

## 安装

先进入 Booster T1 目录，再安装并激活环境：

```bash
cd robots/booster_t1
bash setup_booster_t1.sh
source venv/bin/activate
```

安装脚本会安装 `booster_robotics_sdk_python==1.5.6`、RynnBot、MCP 和 Protocol Debug App。

## 启动

High-level：

```bash
rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
```

Low-level：

```bash
rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_low_server.yaml
```

High-level 和 low-level 选择其中一个 Server 启动。Server 启动后，可按需启动 App：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_booster_t1/config/booster_t1_rynnbot_app.yaml --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
```

每个 Server 启动后都会打印 `Debug UI` 地址。需要查看状态、Action 或实时曲线时，手动在浏览器打开该地址；端口被占用时，使用对应 Server 终端打印的新地址。

启动 RynnBot App 前，在 `booster_t1_rynnbot_app.yaml` 中填写设备凭据。

## 能力

- High-level 模式切换和移动控制
- Low-level 关节状态、IMU 和 50 Hz `LowCmd`
- ONNX `walk` 本地策略

Low-level 首次测试必须可靠支撑机器人，并先执行 `action.robot.enter_low`。模式切换、策略运行和故障排查见
[调试参考](DEBUGGING.zh-CN.md)。
