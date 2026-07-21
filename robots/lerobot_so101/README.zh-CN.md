# LeRobot SO101

SO101 的 RynnRCP 真机接入包，支持单臂和双臂、USB 相机、主从遥操、动作测试、数据采集、MCP 与 RynnBot。

[English](README.md)

## 选择构型

| 构型 | 机械臂 | 相机 | 状态与动作 |
| --- | --- | --- | --- |
| `single` | 1 个从臂 + 1 个主臂 | `front` + `wrist` | 6 维 |
| `dual` | 左右各 1 个从臂和主臂 | `front` + `left_wrist` + `right_wrist` | 12 维，左 6 + 右 6 |

双臂在 RynnRCP 中表现为一个机器人。左主臂控制左从臂，右主臂控制右从臂。

## 快速开始

准备 Python 3.10，并连接需要使用的机械臂和相机。在仓库根目录执行：

```bash
cd robots/lerobot_so101
bash setup_so101.sh
source venv/bin/activate
rynnrcp-so101-configure
```

Windows 在 Git Bash 中执行安装；Git Bash 使用 `source venv/Scripts/activate`，PowerShell 使用 `.\venv\Scripts\Activate.ps1`。后续每个终端都先进入 `robots/lerobot_so101`，再激活虚拟环境。

配置页面会自动打开。按页面顺序完成：

1. 选择单臂或双臂构型。
2. 绑定相机。
3. 为每条机械臂选择不同的串口。
4. 分别标定页面列出的每条机械臂。
5. 用小幅动作测试确认关节方向。
6. 保存配置。

双臂需要四个串口和四份独立标定，分别对应左从臂、右从臂、左主臂和右主臂。

## RynnBot 凭据怎么填

判断方法只有一条：**看这次运行中哪个本体要接入 RynnBot，填写该端的一套设备凭据。** “从臂执行端”用于从臂本体接入云端；“主臂控制端”用于真实主臂控制仿真从臂。

| 使用方式 | 需要填写的 Product Key / Device Name / Device Secret |
| --- | --- |
| 配置页面动作测试 | 0 套 |
| 本地主从臂 Teleop | 0 套 |
| MCP / Protocol Debug | 0 套 |
| 从臂本体接入 RynnBot | 1 套：只填从臂执行端 |
| 真实主臂通过 RynnBot 控制仿真从臂 | 1 套：主臂控制端 |

“主臂控制端凭据”用于把主臂 Server 注册为 RynnBot controller。每一组的 App ID 都由配置工具自动生成，HTTP URL 通常保持默认。

## 运行前准备

以下每个终端都先进入 SO101 目录并激活虚拟环境：

```bash
cd robots/lerobot_so101
source venv/bin/activate
```

Windows Git Bash 使用 `source venv/Scripts/activate`，PowerShell 使用 `.\venv\Scripts\Activate.ps1`。

每个 Server 启动后都会打印 `Debug UI` 地址。需要查看状态、Action、相机图像或实时曲线时，打开该地址；默认端口被占用时，使用对应 Server 终端打印的新地址。

## 本地主从臂 Teleop 与数采

### 单臂本地 Teleop

使用三个终端。先启动从臂和主臂 Server，两个 Server 都打印就绪信息后再启动 Teleop App。

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml

# 终端 2
rynnrcp-server --config rynnrcp_robot_so101/config/so101_leader_server.yaml

# 终端 3
rynnrcp-teleop-app
```

### 双臂本地 Teleop

同样使用三个终端：

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml

# 终端 2
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_leader_server.yaml

# 终端 3
rynnrcp-teleop-app
```

打开 Teleop 终端打印的 Web UI 地址，选择对应的主臂和从臂后开始遥操；数据采集也在同一页面启动和停止。停止服务时，在对应终端按 `Ctrl+C`。

## 从臂本体接入 RynnBot

先在配置页面选择对应构型，并填写“从臂执行端”凭据。每种构型使用两个终端：终端 1 启动从臂 Server，等待就绪后在终端 2 启动 RynnBot App。

### 单臂从臂接入 RynnBot

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml

# 终端 2
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

### 双臂从臂接入 RynnBot

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml

# 终端 2
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_bimanual_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

## 主臂作为 RynnBot 控制端控制仿真

先在配置页面选择对应构型，并填写“主臂控制端”凭据。仿真环境负责启动仿真从臂；连接主臂的电脑使用两个终端，先启动主臂 Server，再启动主臂控制端 RynnBot App。

### 单臂主臂控制仿真从臂

```bash
# 终端 1：启动 6 维单臂主臂 Server
rynnrcp-server --config rynnrcp_robot_so101/config/so101_leader_server.yaml

# 终端 2：把单臂主臂注册为 RynnBot controller
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_master_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_leader_server.yaml
```

仿真任务选择 6 维 SO101 接口。主臂动作通过 `observation.robot.joint_state` 发送为云端 `action`。

### 双臂主臂控制仿真从臂

```bash
# 终端 1：启动 12 维双臂主臂 Server
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_leader_server.yaml

# 终端 2：把双臂主臂注册为 RynnBot controller
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_bimanual_master_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_bimanual_leader_server.yaml
```

仿真任务选择 12 维双臂接口，动作顺序为左臂 6 维 + 右臂 6 维。

## Protocol Debug

Protocol Debug 会使用指定配置启动或连接对应从臂 Server，并打开浏览器调试页面。

### 单臂 Protocol Debug

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

### 双臂 Protocol Debug

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

## MCP

MCP App 连接已经启动的从臂 Server，因此每种构型使用两个终端。

### 单臂 MCP

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml

# 终端 2
rynnrcp-mcp-app --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

### 双臂 MCP

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml

# 终端 2
rynnrcp-mcp-app --server-config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

## 验证结果

打开从臂 Server 的 `Debug UI` 并确认：

- 单臂关节状态和动作各包含 6 个值；双臂各包含 12 个值。
- 单臂显示 `front`、`wrist`；双臂显示 `front`、`left_wrist`、`right_wrist`。
- 双臂遥操时，左右主臂分别驱动对应的从臂。

## 双臂数据约定

```json
{
  "n_dof": 12,
  "task_keys": [
    "observation.state",
    "observation.images.front",
    "observation.images.left_wrist",
    "observation.images.right_wrist",
    "action"
  ]
}
```

`observation.state` 和 `action` 都按左臂 6 维、右臂 6 维排列。每条机械臂的前五个关节使用弧度，夹爪使用 `[0, 1]` 归一化比例。

## 配置文件

配置文件位于 `rynnrcp_robot_so101/config/`：

| 构型 | Follower Server | Leader Server | 从臂 RynnBot App | 主臂控制端 RynnBot App |
| --- | --- | --- | --- | --- |
| 单臂 | `so101_follower_server.yaml` | `so101_leader_server.yaml` | `so101_rynnbot_app.yaml` | `so101_master_rynnbot_app.yaml` |
| 双臂 | `so101_bimanual_follower_server.yaml` | `so101_bimanual_leader_server.yaml` | `so101_bimanual_rynnbot_app.yaml` | `so101_bimanual_master_rynnbot_app.yaml` |

修改串口、相机、标定或凭据时，重新运行 `rynnrcp-so101-configure`。

串口占用、标定失败、相机读帧和动作跟随问题见[调试参考](DEBUGGING.zh-CN.md)。
