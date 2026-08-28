# Franka FR3

Franka Research 3 和原厂 Franka Hand 的 RynnRCP 真机接入包。控制数据统一为 `[7 个机械臂关节, 夹爪]` 共 8 维关节位置。

[English](README.md)

## 快速开始

准备 Ubuntu、Python 3.10–3.12，并在 Franka Desk 中解锁机械臂、启用 FCI。在仓库根目录执行：

```bash
cd robots/franka_fr3
bash setup_franka_fr3.sh
source venv/bin/activate
rynnrcp-franka-fr3-configure
```

安装脚本会安装编译依赖，拉取并编译官方 `libfranka 0.13.3` 和 `Ruckig 0.15.3`，安装 RealSense Python 驱动，然后安装 RynnRCP 和 Franka FR3 接入包。系统依赖已经准备完成时，使用 `--skip-apt`；Robot System Version 需要其他 libfranka 版本时，使用 `--libfranka-version`。

配置页面会自动打开。按页面顺序完成：

1. 填写机械臂 IP。
2. 保存配置并连接机械臂。
3. 确认页面显示 7 个机械臂关节和 1 个夹爪状态。
4. 清空工作空间并确认急停可用。
5. 点击“回到 Home”，确认机械臂能够安全到达配置的 Home。
6. 拖动一个关节滑块进行单关节测试。
7. 机械臂位于现场认可的安全姿态时，将当前状态保存为 Home。
8. 页面会自动打开所有已连接 RealSense 的低带宽实时预览。在每个画面下选择绑定为 `cam_arm`、`cam_main` 或 `cam_side`，然后保存配置。

机械臂关节滑块使用弧度，夹爪滑块范围为 `[0, 1]`。页面以 60 Hz 持续发送最新关节目标并更新状态；拖动事件会合并为最新目标，同一时刻最多只有一个 HTTP 请求在途。机械臂的状态与控制流量统一由一个 native 循环管理：1 kHz libfranka 回调缓存实测状态，并使用 Ruckig 生成满足速度、加速度和 jerk 限制的控制指令。夹爪状态由独立后台线程更新缓存，8 维 `joint_state` 只合并机械臂和夹爪缓存，因此夹爪网络读取不会降低 60 Hz observation 帧率；action 只更新控制目标。

默认关节限制为速度 `0.25 rad/s`、加速度 `0.5 rad/s²`、jerk `2.5 rad/s³`。需要调整时修改 Server YAML 中对应参数。

## Home 位置

默认 Home 使用 libfranka 示例常用初始姿态：

```text
[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```

7 个值的单位均为弧度。Home 只移动机械臂，夹爪保持独立控制。该姿态是初始配置值。首次使用时在配置页面确认现场空间、末端负载和安装方向，再将认可的当前姿态保存为本机 Home。

配置保存在：

```text
rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

三路摄像头默认关闭。配置页使用 640 × 480、15 FPS 预览所有已连接设备，便于根据实际画面完成对应关系；同一个角色只能绑定一台设备。保存后，Server 会启用已绑定的相机，并按 1280 × 720、30 FPS 发布 JPEG 画面。启动 Server 前请在配置页终端按 `Ctrl+C`，以释放预览占用的 RealSense。

## 启动 Server

每个终端先进入 Franka FR3 目录并激活环境：

```bash
cd robots/franka_fr3
source venv/bin/activate
```

启动 Server：

```bash
rynnrcp-server --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

Server 连接成功后，native 控制循环会保持启动时的实测姿态，直到收到目标。终端会打印 `Debug UI` 地址，打开该地址查看状态和发送 Action。启动前清空机械臂工作空间并将急停放在手边；停止服务时在对应终端按 `Ctrl+C`。

## 启动应用

Server 运行后，在新的终端进入 `robots/franka_fr3` 并激活 `venv`，再按用途启动一个应用。

查看协议状态并手动发送 Action：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

打开遥操与数据采集页面：

```bash
rynnrcp-teleop-app
```

使用 Meta Quest 3 右手柄时，另一个终端启动主臂 Server：

```bash
rynnrcp-server \
  --config rynnrcp_robot_meta_quest3/config/meta_quest3_franka_fr3_right_server.yaml
```

在 Teleop 中将 `meta_quest3_franka_fr3_right` 的 `joint_state` 映射到 `franka_fr3` 的 `joint_position`。这条数据固定为 `[7 个机械臂关节, 夹爪]` 共 8 维。按 A 标定后，首个关节目标为配置的 Home；到达 Home 后按住 grip 操作机械臂。

启动 MCP 接口：

```bash
rynnrcp-mcp-app --server-config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

接入 RynnBot 前，先填写 `franka_fr3_rynnbot_app.yaml` 中的设备凭据，然后执行：

```bash
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_franka_fr3/config/franka_fr3_rynnbot_app.yaml \
  --server-config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

## 接口

- `observation.robot.joint_state`：以 60 Hz 发布 8 维缓存状态，顺序为 7 个机械臂关节和 1 个归一化夹爪开口。
- `observation.cam_arm.image`：机械臂视角 RealSense 彩色图像。
- `observation.cam_main.image`：主视角 RealSense 彩色图像。
- `observation.cam_side.image`：侧视角 RealSense 彩色图像。
- `action.robot.joint_position`：以 60 Hz 接收 `[7 个弧度制关节目标, 夹爪 0–1]` 共 8 维。
- `action.robot.home`：使用配置中的 `home_joint_positions`。

## 通信验证

配置页连接测试会读取机械臂和夹爪状态。排查通信质量时执行：

```bash
rynnrcp-franka-fr3-check --robot-ip 192.168.0.110
.deps/libfranka-build/examples/communication_test 192.168.0.110
```

正式运动控制使用 Franka 支持的实时内核，并在配置页面启用实时调度检查。

控制目标超过 `target_timeout_s` 未更新时，控制循环会取消尚未完成的旧目标，并通过 Ruckig 平滑减速、保持当前关节位置。控制循环保持运行，恢复目标流后可继续接收新目标。健康状态中的 `franka.target_timeout` 用于提示上层目标流已经中断。
