# LeKiwi

LeKiwi 的 RynnRCP 接入包，通过 9 维状态控制六轴机械臂和三轮全向底盘，并采集前置、腕部相机图像。

[English](README.md)

## 安装与配置

准备 Python 3.10，在 Bash 中执行：

```bash
cd robots/lerobot_lekiwi
bash setup_lekiwi.sh
```

脚本结束时会打印当前平台的激活命令。macOS/Linux 使用 `source venv/bin/activate`，Windows Git Bash 使用 `source venv/Scripts/activate`。激活后运行 `rynnrcp-lekiwi-configure-web`。

安装脚本会安装 Protocol Debug、MCP、RynnBot 和 Teleop App。

在配置页中依次完成 Robot ID、两路相机、从臂串口与标定、主臂串口与标定。保存时，从臂 Robot ID、主臂 Robot ID 和 RynnBot App ID 会自动使用同一个基于本机生成的 8 位稳定后缀；同一台机器重复配置时后缀保持稳定。从臂使用 1–9 号电机，主臂使用 1–6 号电机。标定时缓慢推动每个关节走完整范围，确认 Min 和 Max 持续变化后保存。

## 哪些云端字段需要填写

| 使用方式 | Product Key / Device Name / Device Secret |
| --- | --- |
| 本地主从遥操和本地数采 | 保留 Robot ID 后继续设备配置 |
| MCP / Protocol Debug | 使用本地 Robot ID |
| 让 LeKiwi 从臂接入 RynnBot | 填写一套从臂执行端凭据 |

从臂和主臂 Robot ID 是两个本地 RCP Server 的身份，请为二者设置不同值。App ID 是从臂 RynnBot App 的标识，由配置工具自动生成。主臂通过本地 Teleop App 控制，因此页面仅提供从臂的 Product Key、Device Name 和 Device Secret。

HTTP URL 通常保持默认。“图片上传编码”用于 RynnBot 云端图片上传；本地遥操直接使用相机输出。

## 本地主从遥操与数采

以下命令需要在已激活 LeKiwi 虚拟环境的终端中执行。每个新终端都先进入 `robots/lerobot_lekiwi`，再执行适合当前平台的激活命令。

使用三个终端。终端 1 在 LeKiwi 开发板上运行，可以通过 SSH 或无桌面环境启动从臂 Server：

```bash
# 终端 1：LeKiwi 开发板
rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
```

终端 2 在连接主臂的笔记本本地桌面中启动主臂 Server：

```bash
# 终端 2：连接主臂的笔记本
rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_leader_server.yaml
```

两个 Server 都打印就绪信息后，在笔记本的终端 3 启动 Teleop App：

```bash
# 终端 3：连接主臂的笔记本
rynnrcp-teleop-app
```

每个 Server 都会打印自己的 `Debug UI` 地址。需要查看关节状态、Action、相机图像和实时曲线时手动打开该地址；默认端口 `8092` 被占用时，使用对应 Server 终端打印的回退地址。页面打开期间会轮询机器人数据，完成查看后直接关闭页面。

在 Teleop 页面中选择控制端 `lekiwi_leader`、执行端 `lekiwi`，并使用：

```text
observation.robot.joint_state -> action.robot.joint_position
```

主臂 Server 使用 `pynput` 监听键盘。推荐 Windows、macOS 本地桌面或 Linux X11/Xorg；macOS 需要为终端开启“输入监控”权限。Linux SSH、无桌面环境和拦截全局按键的 Wayland 会话用于运行从臂 Server。

| 按键 | 控制 |
| --- | --- |
| `W/S` | 前进/后退 |
| `A/D` | 左移/右移 |
| `Q/E` | 左转/右转 |
| `[/]` | 降低/提高速度档位 |
| `Space` | 输出零速度 |

## 从臂本体接入 RynnBot

先在配置页面填写一套从臂执行端凭据。使用两个终端，先启动从臂 Server，再启动 RynnBot App：

```bash
# 终端 1：LeKiwi 开发板
rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml

# 终端 2：LeKiwi 开发板
rynnrcp-rynnbot-app --config rynnrcp_robot_lekiwi/config/lekiwi_rynnbot_app.yaml --server-config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
```

RynnBot App 使用从臂 Server 提供的 9 维状态、两路相机和 Action。

## Protocol Debug

Protocol Debug 使用 LeKiwi 从臂配置启动或连接从臂 Server，并打开浏览器调试页面：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
```

## MCP

MCP App 连接已经启动的从臂 Server，因此在同一台机器上使用两个终端：

```bash
# 终端 1：LeKiwi 开发板
rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml

# 终端 2：LeKiwi 开发板
rynnrcp-mcp-app --server-config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
```

## 数据

| 对象 | 频率 | 内容 |
| --- | --- | --- |
| `observation.robot.joint_state` | 60 Hz | 从臂实际 9 维状态 |
| `action.robot.joint_position` | 60 Hz | 主臂与键盘产生的 9 维命令 |
| `observation.front.image` | 30 Hz | `640 × 360` JPEG |
| `observation.wrist.image` | 30 Hz | `640 × 360` JPEG |

9 维顺序为：

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper,
base_x_velocity, base_y_velocity, base_yaw_velocity
```

前五维单位为 rad，夹爪范围为 `0–1`，底盘后三维单位依次为 m/s、m/s、rad/s。底盘超过 0.5 秒未收到新命令时自动停车。

## 验收

- 移动主臂时，action 与 observation 的前六维应跟随变化。
- 按住底盘按键时，后三维应按方向变化，松开后回到零。
- 两路图像应持续更新且尺寸为 `640 × 360`。
- 电机、标定或相机异常时，回到 Web 配置页重新检查对应设备。

## 配置文件

| 用途 | 配置文件 |
| --- | --- |
| 从臂 Server | `lekiwi_server.yaml` |
| 主臂 Server | `lekiwi_leader_server.yaml` |
| 从臂 RynnBot App | `lekiwi_rynnbot_app.yaml` |
