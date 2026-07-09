# Aero Hand（绳肌妙算）机器人包

这个包把 Aero Hand Open SDK 接入 RynnRCP，作为 Aero Hand（绳肌妙算）单手和双手两个云端构型放在 `robocore/robots/aero_hand`。

## 提供什么

- `AeroHandController`：把 Aero SDK 的 compact hand joints 接成 RCP `joint_state` / `joint_position`。
- `robot_integration.yaml`：两个构型共享的 RCP 映射，始终声明 `observation.robot.joint_state` 和 `action.robot.joint_position`。
- `aero_hand_single_server.yaml`：单手 7 维云端构型，不区分左/右手。
- `aero_hand_dual_server.yaml`：双手 14 维云端构型，顺序是 left 7 维 + right 7 维。
- `aero_hand_single_rynnbot_app.yaml`：单手云端设备的 RynnBot App 配置。
- `aero_hand_dual_rynnbot_app.yaml`：双手云端设备的 RynnBot App 配置。
- `rynnrcp-aero-hand-configure`：浏览器配置/调试页面。

## 安装

推荐使用安装脚本。它会创建 `robots/aero_hand/venv`，并以 editable 模式安装 RynnRCP、官方 App 和 Aero Hand 包。

从项目根目录执行：

```bash
cd robots/aero_hand
./setup_aero_hand.sh
source venv/bin/activate
```

如需指定 Python 源或重建环境：

```bash
./setup_aero_hand.sh --pip-index-url https://mirrors.aliyun.com/pypi/simple/ --recreate
```

安装完成后会得到这些命令：

```bash
rynnrcp-aero-hand-configure
rynnrcp-server
rynnrcp-teleop-app
rynnrcp-mcp-app
rynnrcp-rynnbot-app
```

## 配置和调试

打开配置页面：

```bash
rynnrcp-aero-hand-configure
```

默认地址：

```text
http://127.0.0.1:28411
```

页面按顺序完成：

- 选择配置单手还是双手。
- 通过“记录当前端口 -> 插入手 -> 识别新插入的手”写入串口。
- 保存 `Robot ID` / `Robot Name`。
- 分别填写并保存单手/双手 RynnBot 云端凭据。
- 测试连接、读取状态。
- 手动执行 homing。
- 发送安全调试姿态。

Linux 通常是 `/dev/ttyUSB0`、`/dev/ttyACM0` 或 `/dev/serial/by-id/...`。

## 云端构型和数据集

云端下发/采集的是 RynnBot 格式的 `observation.state` 和 `action`，RynnBot App 会在 RCP 里映射为：

```text
observation.robot.joint_state
action.robot.joint_position
```

单手数据集是 7 维，不区分左手或右手，控制方式相同。

双手数据集是 14 维，字段和向量顺序为：

```text
left_thumb_cmc_abd, left_thumb_cmc_flex, left_thumb_mcp_ip, left_index_finger, left_middle_finger, left_ring_finger, left_pinky_finger,
right_thumb_cmc_abd, right_thumb_cmc_flex, right_thumb_mcp_ip, right_index_finger, right_middle_finger, right_ring_finger, right_pinky_finger
```

## 运行 Server

先在配置页面识别并保存端口，再启动对应构型。

单手：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

双手：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

## 接入 RynnBot

先在配置页面选择单手或双手构型，分别填写对应云端设备的 Product Key、Device Name、Device Secret。

单手：

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

双手：

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

## 标定说明

Aero Hand 日常调试只需要在安全位置手动执行 homing。

首次装机、换电机、调整电机 ID、限流或 trim 时，才需要使用厂商 SDK 的维护能力。

## Controller 约定

- RCP 对外单位是 radians。
- Aero Hand SDK 内部单位是 degrees。
- 单手 7 维关节顺序：`thumb_cmc_abd`、`thumb_cmc_flex`、`thumb_mcp_ip`、`index_finger`、`middle_finger`、`ring_finger`、`pinky_finger`。
- 双手 14 维关节顺序：左手 7 维在前，右手 7 维在后。
