# Atom01

[English](README.md)

Atom01 人形机器人的 RynnRCP 接入包，提供关节和 IMU 观测、关节动作、本地策略、
Protocol Debug、MCP 与 RynnBot 接入。

> **真机安全**
>
> 启动配置程序、测试策略或 RynnRCP Server 前，可靠支撑机器人并准备好急停。
> RynnRCP Server 会连接 CAN、初始化电机，并让机器人进入默认姿态。当前只运行一个
> 电机控制进程。

## 前置条件

- 在 Atom01 Linux 主控上操作。
- 电机 CAN 和 IMU 通信链路已按机器人硬件说明配置。
- 已进入 RynnRCP 仓库根目录。

硬件接口和系统环境的诊断方法见 [调试参考](DEBUGGING.zh-CN.md)。

## 登录主控

参考部署中，Atom01 主控会开启 Wi-Fi 热点：

- 热点名称：`atom`
- 默认密码：`jujujuju`

电脑连接该热点后执行：

```bash
ssh orangepi@192.168.12.1
```

`orangepi` 是默认用户名，`192.168.12.1` 是热点网络中的主控地址。如果机器人接入
其他局域网，电脑连接同一局域网，并使用主控的实际用户名和 IP 地址。登录后进入
主控上的 RynnRCP 仓库根目录。

## 安装

在仓库根目录执行：

```bash
cd robots/roboparty_atom01
bash setup_atom01.sh
source venv/bin/activate
```

安装脚本会安装编译依赖，创建独立虚拟环境，安装 RynnRCP、Protocol Debug、MCP、
RynnBot 和 Atom01 包，并编译 `atom01_py`。

看到 `Imports OK` 和 `Atom01 setup completed.` 表示安装成功。

## 配置和按需标定

在 `robots/roboparty_atom01` 目录执行：

```bash
source venv/bin/activate
rynnrcp-atom01-configure
```

使用终端打印的 `Atom01 configure UI LAN` 地址打开配置页面，然后：

1. 保存 `Robot ID`。
2. 接入 RynnBot 时填写 Product Key、Device Name 和 Device Secret。
3. 查看 CAN、IMU 和 23 个关节诊断。

零位标定仅在以下情况执行：

- 机器人首次组装完成。
- 机器人站立不稳定。

执行标定前，对照零位参考图检查 23 个关节，并确认机器人已可靠支撑、急停可用，
且当前只有配置程序访问电机。标定完成后查看 CAN、IMU 和关节诊断，确认标定状态
已保存。已经完成标定且站立稳定时，保留当前标定结果。

凭据保存在主控本地配置中。提交代码前检查配置差异，避免把设备凭据提交到仓库。

## 启动 Server

完成必要配置后，在 `robots/roboparty_atom01` 目录执行：

```bash
source venv/bin/activate
rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

首次启动保持前台运行并观察日志。终端打印就绪信息和 `Debug UI` 地址后，再进行协议
调试或接入 App。Server 端口由系统动态分配，以终端打印值为准。

使用 `Ctrl+C` 停止 Server，等待电机进入安全状态后再断开主控电源。

## 部署和运行 Policy

### 打开 Protocol Debug

保持 Server 运行，在主控的另一个终端执行：

```bash
cd robots/roboparty_atom01
source venv/bin/activate
rynnrcp-protocol-debug --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

使用终端打印的 Protocol Debug 地址打开页面。先检查 Observation 和 `get_health`，
再进入 Policy 区域。

### 安装平台下发的 Policy ZIP

Server 从以下目录扫描 Policy：

```text
robots/roboparty_atom01/rynnrcp_robot_atom01/policies/
```

先查看 ZIP 内容：

```bash
unzip -l <平台下发的Policy包.zip>
```

ZIP 应包含一个 Policy 目录，目录内至少包含 `policy.yaml` 和 `policy.py`，例如：

```text
my_policy/
├── policy.yaml
├── policy.py
└── model.onnx
```

在仓库根目录解压：

```bash
unzip <平台下发的Policy包.zip> \
  -d robots/roboparty_atom01/rynnrcp_robot_atom01/policies/
```

确认 `policy.yaml` 位于 Policy 根目录的下一层：

```bash
find robots/roboparty_atom01/rynnrcp_robot_atom01/policies \
  -mindepth 2 -maxdepth 2 -name policy.yaml -print
```

每个 `policy_id` 必须唯一。解压期间保持机器人处于安全状态；完成后先停止当前
Policy，再使用 `Ctrl+C` 停止并重新启动 Server。Server 重启时会重新扫描目录，
Protocol Debug 的 `list_policies` 会显示新 Policy。

### 启动 Policy

在 Protocol Debug 的 Policy 区域：

1. 点击 `list_policies`，确认目标 Policy 已出现。
2. 检查 `runtime_inputs JSON`。
3. 可靠支撑机器人并准备好急停。
4. 点击“启动 policy”。

启动成功后，Policy 会立即循环读取关节和 IMU Observation，并持续向
`action.robot.joint_position` 下发目标，机器人会开始执行 Policy。启动操作是真机
执行，不是 dry-run。

`stand` 和 `walk` 使用 `cmd_vel: [vx, vy, wz]`。默认值为 `[0, 0, 0]`；输入超过
500 ms 未更新时会自动回到默认值。修改 JSON 后点击“更新输入”，新值会应用到当前
Policy。

### 切换 Policy

目标 Policy 加载成功后，服务会停止当前 Policy，再启动目标 Policy。切换期间电机
保持上一个目标位置；新 Policy 的第一帧会直接接管控制，服务不会在两个 Policy
姿态之间自动插值。

切换前确认两个 Policy 的关节顺序、默认姿态和第一帧目标兼容。首次切换采用以下
流程：

1. 停止当前 Policy。
2. 让机器人回到目标 Policy 的预期起始姿态。
3. 检查目标 Policy 的输入。
4. 启动目标 Policy。

### 停止 Policy

点击“停止 policy”后，Policy 推理循环和后续 Action 下发会停止，电机继续保持最后
一次下发的目标位置。停止 Policy 不会关闭 Server，也不会自动进入阻尼模式。

需要结束真机控制时：

1. 先停止 Policy。
2. 确认机器人姿态稳定并保持可靠支撑。
3. 回到 Server 终端按 `Ctrl+C`。
4. 等待控制器完成关闭流程后再断开主控电源。

Policy 包加载失败、切换异常、Health 告警和 dry-run 检查见
[调试参考](DEBUGGING.zh-CN.md)。

## 接入 RynnBot

先在配置页面填写云端设备凭据。保持 Server 运行，在主控的另一个终端执行：

```bash
cd robots/roboparty_atom01
source venv/bin/activate
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_atom01/config/atom01_rynnbot_app.yaml \
  --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
```

## 接入 MCP

保持 Server 运行，在主控的另一个终端执行：

```bash
cd robots/roboparty_atom01
source venv/bin/activate
rynnrcp-mcp-app \
  --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
```

构建、硬件、标定、端口、Health 和策略问题见
[调试参考](DEBUGGING.zh-CN.md)。
