# Atom01

[English](README.md)

Atom01 人形机器人的 RynnRCP 接入包，提供关节控制、状态观测和本地策略服务。

## 前置条件

- Linux 控制机
- Atom01 硬件和通信链路可用
- C++ 编译依赖；安装脚本可通过 `apt-get` 安装

## 安装

先进入 Atom01 目录，再安装并激活环境：

```bash
cd robots/roboparty_atom01
bash setup_atom01.sh
source venv/bin/activate
rynnrcp-atom01-configure
```

安装脚本会编译控制绑定，并安装 RynnBot、MCP 和 Protocol Debug App。

## 启动

```bash
rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

Server 启动后，可在其他终端按需启动 App：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_atom01/config/atom01_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_atom01/config/atom01_rynnbot_app.yaml --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
```

Server 启动后会打印 `Debug UI` 地址。需要查看状态、Action 或实时曲线时，手动在浏览器打开该地址；端口被占用时，使用 Server 终端打印的新地址。

## 能力

- 关节状态和 IMU 观测
- 关节位置、回零和阻尼动作
- `stand`、`walk` 等本地策略

首次下发动作前应固定机器人并确认零位和健康状态。许可证说明、标定和策略调试见
[调试参考](DEBUGGING.zh-CN.md)。
