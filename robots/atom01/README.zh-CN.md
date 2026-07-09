# Atom01 机器人包

这个包把 Atom01 人形机器人接入 RynnRCP。它包含 Atom01 C++ 控制绑定、RCP
controller、Server/RynnBot 配置、零位标定页面和示例 policy。

## 许可协议

本包随仓库发布，仓库根协议为 Apache License 2.0。其中
`rynnrcp_robot_atom01/atom_control/` 下的 C++ 控制绑定源码包含 GPL-3.0 文件；
这些文件以其 SPDX 文件头标注的 GPL-3.0 为准。

## 提供什么

- `Atom01Controller`：把 `atom01_py` 底层控制接口映射为 RCP Observation、Action 和 Health。
- `robot_integration.yaml`：声明 23 自由度人形机器人、关节状态、IMU、关节位置动作、回零和阻尼模式。
- `atom01_server.yaml`：Atom01 RynnRCP Server 配置。
- `atom01_rynnbot_app.yaml`：RynnBot App 云端设备配置。
- `rynnrcp-atom01-configure`：浏览器配置与零位标定页面。
- `rynnrcp-atom01-test-policy`：本地 policy 效果检查入口。

## 安装

推荐在 Atom01 机器人本机运行安装脚本。脚本会创建 `robots/atom01/venv`，安装
RynnRCP、RynnBot App、Atom01 包，并编译 `atom01_py`。

从仓库根目录执行：

```bash
cd robots/atom01
./setup_atom01.sh
source venv/bin/activate
```

如需跳过 apt 依赖安装或重建环境：

```bash
./setup_atom01.sh --skip-apt --recreate
```

安装完成后会得到这些命令：

```bash
rynnrcp-atom01-configure
rynnrcp-atom01-test-policy
rynnrcp-protocol-debug
rynnrcp-server
rynnrcp-rynnbot-app
```

## 配置和标定

打开配置页面：

```bash
rynnrcp-atom01-configure
```

默认地址：

```text
http://127.0.0.1:28421
```

页面用于：

- 保存 `Robot ID`。
- 填写 RynnBot Product Key、Device Name、Device Secret。
- 查看 URDF 零位参考图。
- 在机器人被可靠支撑、policy/server 控制已断开时，手动标定全部电机零位。
- 读取当前关节、IMU 和 CAN 诊断状态。

## 运行 Server

标定并确认配置后启动 Server：

```bash
rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

Server 对外暴露：

- `observation.robot.joint_state`
- `observation.robot.imu`
- `action.robot.joint_position`
- `action.robot.home_position`
- `action.robot.damping_mode`

同时启用 `policy_service`，并从包内 `policies/` 扫描本地策略。`list_policies`
默认可看到：

- `stand`：站立策略，运行时输入 `cmd_vel`，形状 `[3]`，500 ms 未更新后回到默认值。
- `walk`：行走策略，运行时输入 `cmd_vel`，形状 `[3]`，500 ms 未更新后回到默认值。
- `sim2simdance`：舞蹈示例策略，无运行时输入。

真机运行策略前，先用默认 dry-run 检查输出：

```bash
rynnrcp-atom01-test-policy --policy stand
```

只有确认机器人被可靠支撑、零位和输出幅度都安全后，才使用
`--apply --i-understand-risk` 把策略目标下发到真机。

## 接入 RynnBot

先在配置页面填写云端凭据，再启动：

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_atom01/config/atom01_rynnbot_app.yaml --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
```

## 调试

协议级调试可以使用：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

真机调试前先读 `get_health`，确认没有 `atom01.zero_calibration_unconfirmed`、
`atom01.motors_not_enabled` 等警告后再下发动作。

## 开发者手动安装

一般用户不需要这一段；优先使用 `setup_atom01.sh`。

```bash
python -m pip install -e .
python -m pip install -e apps/common -e apps/rynnbot
python -m pip install -e robots/atom01
```
