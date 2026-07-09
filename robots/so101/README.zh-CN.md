# SO101 机器人包

[English](README.md)

这个包是 RynnRCP Python 标准实现中的 SO101 机器人接入包。它把轻量 SO101
驱动映射到 RCP 标准设备模型，包括机器人状态、相机观测、动作执行、健康状态、
数据录制、遥操、MCP 接入和 RynnBot 云端工作流。


## 提供什么

- SO101 follower/leader controller 封装。
- 长期运行的 RynnRCP server 配置，以及独立 MCP、Teleop、RynnBot App 配置。
- 用于串口、相机、标定和 Server/App 设置的浏览器配置工具。
- 启动 SO101 Server 和 App 的命令行入口。

## 安装

安装脚本固定使用 Python 3.10。Windows 用户请在 Git Bash 中运行。
从仓库根目录进入 SO101 包目录执行：

```bash
cd robots/so101
./setup_so101.sh
source venv/bin/activate
```

脚本会创建 `robots/so101/venv`，把 RynnRCP 作为本地库安装，同时安装官方 App
和 SO101 机器人包，全部使用源码 editable 模式。

一般用户只需要使用 `setup_so101.sh`。不要先在仓库根目录手动安装一遍，再进入 SO101
目录重复安装。

安装完成后，当前环境会获得 RynnRCP、官方 App 和 SO101 配置工具命令：

```bash
rynnrcp-server
rynnrcp-teleop-app
rynnrcp-mcp-app
rynnrcp-rynnbot-app
rynnrcp-so101-configure
```

每个命令都可以用 `-h` 查看参数。

## 配置

SO101 配置文件位于：

```text
robots/so101/rynnrcp_robot_so101/config/
```

重要文件：

| 文件 | 用途 |
| --- | --- |
| `robot_integration.yaml` | 机器人包维护的接入定义。 |
| `so101_follower_server.yaml` | SO101 从臂 RynnRCP Server 配置，包含 `manifest.robot_id`、机械臂串口和相机。 |
| `so101_leader_server.yaml` | SO101 主臂 RynnRCP Server 配置，包含 `manifest.robot_id` 和机械臂串口。 |
| `so101_rynnbot_app.yaml` | RynnBot App 云端设备凭据。 |

使用配置工具编辑常见硬件和 Server/App 设置：

```bash
rynnrcp-so101-configure
```

配置工具支持串口扫描、USB 相机扫描和预览、SO101 标定、Server ID、
RynnBot 凭据填写和配置校验。
默认会打开本机浏览器页面 `http://127.0.0.1:28401`；如果默认端口被占用，
工具会自动选择下一个可用端口并在终端打印实际地址。

第一次使用 SO101 时，推荐按这个顺序完成：

1. 运行 `rynnrcp-so101-configure`，填写 follower/leader 串口和相机。
2. 分别完成 follower 和 leader 标定。
3. 保存配置并启动 follower Server。
4. 需要遥操时，再启动 leader Server 和 Teleop App。

## 运行命令

后续运行命令也建议从 SO101 包目录执行：

```bash
cd robots/so101
source venv/bin/activate
```

先按目标选择要启动的程序：

| 你想做什么 | 需要启动 |
| --- | --- |
| 只检查从臂状态、相机、动作或给 MCP/RynnBot 使用 | 从臂 Server |
| 遥操从臂 | 从臂 Server + 主臂 Server + Teleop App |
| 只用 MCP 调试协议工具 | 从臂 Server + MCP App |
| 接入 RynnBot 云端 | 从臂 Server + RynnBot App |
| 修改串口、相机、标定或 RynnBot 凭据 | 配置工具 |

启动 SO101 从臂 Server：

```bash
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

启动 SO101 主臂 Server：

```bash
rynnrcp-server --config rynnrcp_robot_so101/config/so101_leader_server.yaml
```

启动 Teleop App Web UI：

```bash
rynnrcp-teleop-app
```

启动 MCP App：

```bash
rynnrcp-mcp-app --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

启动 RynnBot App：

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_so101/config/so101_rynnbot_app.yaml --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

打开配置工具：

```bash
rynnrcp-so101-configure
```

Teleop App 会启动本地 Web UI，访问：

```text
http://<teleop_web_host>:28402
```

本机双终端测试时访问：

```text
http://127.0.0.1:28402
```

需要指定自定义 SO101 Server 配置时：

```bash
rynnrcp-server --config <path-to-so101-server.yaml>
```

## 跟随响应观察

SO101 follower 使用 LeRobot 原始的 Feetech 电机配置。原始实现会把 follower 电机的
`P_Coefficient` 设为 `16`，注释说明这是为了降低抖动；电机默认值是 `32`。本包默认
保留这个行为，不主动调高 PID。

直接电机测试显示，发送/读回是毫秒级，但读回位置仍有 `115-140ms` 延迟，实际幅值只有目标的
`0.65-0.84`。瓶颈是 follower 电机自身的位置环响应，不是 gRPC、RCP、Python 或串口调用。

可以用独立脚本绕开 Teleop/RCP 测电机本体响应：

```bash
python -m lerobot_so101.motor_response_test \
  --port /dev/cu.usbmodem5AE70441561 \
  --joint shoulder_lift \
  --amplitude 0.20 \
  --frequency 0.5 \
  --duration 8 \
  --control-hz 60
```

脚本会输出 CSV、SVG 和摘要。重点看：

- `lag_ms`：实际位置相对目标的延迟。
- `amplitude_ratio`：实际运动幅值 / 目标幅值。
- `send_p95_ms`、`read_p95_ms`：如果仍是毫秒级，说明上层发送和读回不是主要瓶颈。

遥操时也可以用环境变量记录 controller 内部的 action/state trace：

```bash
export RYNNRCP_SO101_TRACE_POSITIONS=1
export RYNNRCP_SO101_TRACE_DIR=/tmp/so101_trace

rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

停止 follower Server 时会保存：

- `so101_trace_*.csv`：位置采样，包含 `action`、`sent`、`state`。
- `so101_trace_*.timing.csv`：`read_state`、`send_action`、`worker_tick` 耗时。
- `so101_trace_*.svg`：曲线图，红色是目标 `action`，绿色是实际发送 `sent`，蓝色是读回 `state`。


## Controller 约定

- 前五个机械臂关节内部使用弧度。
- 夹爪使用 `[0, 1]` 归一化比例。
- 第一次真机检查时，先使用保持当前位置或小幅关节空间动作。
- 发送运动指令前，先确认机器人状态和相机流是新鲜的。
