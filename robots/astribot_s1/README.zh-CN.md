# Astribot S1

[English](README.md)

Astribot S1 的 RynnRCP 接入包，提供 22 维上半身关节、双夹爪、移动底盘和三路相机
观测与控制，并提供浏览器配置和实机测试页面。

> **真机安全**
>
> 连接机器人、接管控制权或发送 Action 前，清空运动范围并保持急停可触达。接管控制权
> 会立即停止原控制端的当前运动。当前只运行一个机器人控制进程。

## 前置条件

- 在 Astribot S1 开发机上操作。
- 开发机运行 Ubuntu 22.04 ARM64、ROS 2 Humble 和 Python 3.10。
- Astribot SDK 已安装在 `/home/astribot/astribot_sdk_aarch64`。
- 开发机通过有线网络访问机器人 `192.168.0.10`。
- 已进入 RynnRCP 仓库根目录。

每个新终端先设置标准 SDK 目录：

```bash
export ASTRIBOT_SDK_ROOT=/home/astribot/astribot_sdk_aarch64
```

## 登录开发机

电脑连接 Astribot S1 开发机所在的有线网络后执行：

```bash
ssh astribot@<开发机IP>
cd <RynnRCP仓库目录>
```

使用开发机的实际 IP 和仓库目录。登录后，后续命令均在开发机上执行。

## 机器人上电与驱动启动

每次连接 SDK、打开配置页面或启动 RynnRCP Server 前，先在机器人控制页面完成：

1. 清空机器人运动范围，确认急停可触达并已复位。
2. 在浏览器打开 [机器人控制页面](http://192.168.0.10:5141)。
3. 点击 **上电**，等待电源状态显示为已上电。
4. 点击 **启动机器人驱动**，等待驱动状态显示为运行中，各身体部件无故障提示。
5. 长按 **进入初始姿势**，保持按住直到初始化动作完成、机器人稳定停在初始姿势。
6. 检查页面告警；状态正常后再连接 SDK 或 RynnRCP。

使用 SDK 只读示例确认机器人已准备好：

```bash
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
python "$ASTRIBOT_SDK_ROOT/examples/100-get_robot_properties.py"
```

输出中应显示接口存活、机器人已连接，并列出 chassis、torso、双臂、双夹爪和 head。

## 安装

在仓库根目录执行：

```bash
cd robots/astribot_s1
bash setup_astribot_s1.sh
source venv/bin/activate
```

安装脚本会加载 SDK 环境，创建使用系统 ROS 2 包的独立虚拟环境，并安装 RynnRCP 与
Astribot S1 接入包。

看到 `Astribot SDK import OK`、`RynnBot App import OK`、`RynnRCP controller import OK` 和
`Astribot S1 setup completed.` 表示安装成功。

## 配置和实机测试

完成上电、驱动启动和初始姿势后，在 `robots/astribot_s1` 目录执行：

```bash
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
source venv/bin/activate
rynnrcp-astribot-s1-configure
```

使用终端打印的配置页面地址打开浏览器，然后：

1. 填写并保存 Astribot SDK 目录。
2. 需要接入云端时，填写 Product Key、Device Name 和 Device Secret 并保存。
3. 点击 **连接并读取状态（不抢占）**，确认 22 维关节、底盘和夹爪状态持续更新。
4. 需要实机控制时，确认工作空间安全并勾选页面中的安全确认。
5. 页面已持有控制权时点击 **解锁运动控制**；其他客户端持有控制权时点击
   **接管并解锁控制**。
6. 从单个夹爪或小幅度单关节目标开始测试。拖动滑块时，后台会以 100 Hz 平滑更新
   机器人目标。

普通连接会自动进入只读状态，控制权选择在配置页面完成，启动终端不需要输入。
控制权丢失后，页面会立即锁定关节和夹爪输入。

SDK 路径会保存到本机 Server 配置中。提交代码前检查配置差异，确保提交内容适用于
目标部署环境。

## 启动 Server

完成配置后，在 `robots/astribot_s1` 目录执行：

```bash
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
source venv/bin/activate
rynnrcp-server --config rynnrcp_robot_astribot_s1/config/astribot_s1_server.yaml
```

首次启动保持前台运行并观察日志。终端打印就绪信息和 `Debug UI` 地址后，先确认
`observation.robot.joint_state` 持续更新，再发送小幅度 Action。

使用 `Ctrl+C` 停止 Server。等待机器人稳定后，再关闭驱动或断开机器人电源。

## 接入 RynnBot

先在配置页面填写并保存 RynnBot 云端凭据。保持 Server 运行，在开发机的另一个终端
执行：

```bash
cd <RynnRCP仓库目录>/robots/astribot_s1
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
source venv/bin/activate
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_astribot_s1/config/astribot_s1_rynnbot_app.yaml \
  --server-config rynnrcp_robot_astribot_s1/config/astribot_s1_server.yaml
```

终端显示设备已连接后，即可通过 RynnBot 使用 Astribot S1 的观测与 Action。

## 打开 Protocol Debug

保持 Server 运行，在开发机的另一个终端执行：

```bash
cd <RynnRCP仓库目录>/robots/astribot_s1
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
source venv/bin/activate
rynnrcp-protocol-debug \
  --config rynnrcp_robot_astribot_s1/config/astribot_s1_server.yaml
```

使用终端打印的 Protocol Debug 地址打开页面。先检查 Observation 和 `get_health`，
再从单个小幅度 Action 开始测试。

## 接口

`joint_state`、`joint_position` 和 `joint_velocity` 的 22 维顺序如下：

1. torso：4 维
2. left arm：7 维
3. left gripper：1 维
4. right arm：7 维
5. right gripper：1 维
6. head：2 维

躯干、双臂和头部的位置单位为弧度。第 11、19 位夹爪值标准化为 `0–1`，适配器负责
与 SDK 的 `0–100` 范围互转。独立夹爪状态同时保留 SDK 原始值，便于排障。

主要 Action：

- `joint_position`：22 维上半身位置控制，第 11、19 位夹爪命令使用 `0–1`。
- `joint_velocity`：22 维上半身速度控制。
- `base_velocity`：底盘局部坐标系的 x、y 和 yaw 速度。
- `left_gripper` / `right_gripper`：标准化位置 `0–1`，可附带最大夹持力 `force`（N）。
- `home`：调用 SDK 的碰撞检查回零动作。
- `stop` / `restart`：停止当前运动或恢复控制。

`chassis_state` 提供底盘 `[x, y, yaw]` 位置和速度。三路相机分别发布为
`head_camera.image`、`left_wrist_camera.image` 和 `right_wrist_camera.image`。

## 控制权

Configure 页面统一处理控制权：普通连接保持只读，接管操作需要现场安全确认。

`high_control_rights` 只控制 RynnRCP Server 的启动行为：

- `false`：连接时由 SDK 交互式确认是否接管。
- `true`：Server 启动时直接接管现有控制权。

有人现场监护且明确需要接管时再启用 `high_control_rights: true`。

## 排障

出现 `not alive` 或 `No simulation or real robot is started` 时：

1. 回到机器人控制页面，依次确认 **上电**、**启动机器人驱动** 和
   **进入初始姿势** 已完成。
2. 确认急停已复位，页面中没有未处理的故障或告警。
3. 确认当前只有一个 Astribot SDK 或 RynnRCP 控制进程。
4. 执行 `ping 192.168.0.10`，确认有线网络可达。
5. 在同一 SDK 环境中运行 `examples/100-get_robot_properties.py` 和
   `examples/101-get_joint_states.py`。
6. 检查 `sdk_root`、`ROS_DOMAIN_ID=25` 和开发机的 `192.168.0.x` 网络地址。
