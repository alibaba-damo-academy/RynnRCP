# Aero Hand 调试参考

这个包把 Aero Hand Open SDK 接入 RynnRCP，提供单手、双手真机执行端，以及摄像手势控制端构型。摄像手势控制端读取所选摄像头并输出手势动作；摄像头参数在配置页面的“手势摄像头与采集画面”中设置。

## 最短接入

```bash
cd robots/tetheria_aerohand
bash setup_aero_hand.sh
source venv/bin/activate
rynnrcp-aero-hand-configure
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

setup 会安装 RynnBot、MCP、Protocol Debug 和 Teleop App。Server 启动后按需另开终端：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

双手构型把 `single` 换成 `dual`。云端设备凭据保存在本机配置中；公开或分享配置文件时保留 `YOUR_*` 占位值。

## 提供什么

- `AeroHandController`：把 Aero SDK 的 compact hand joints 接成 RCP `joint_state` / `joint_position`。
- `robot_integration.yaml`：两个构型共享的 RCP 映射，始终声明 `observation.robot.joint_state` 和 `action.robot.joint_position`。
- `aero_hand_single_server.yaml`：单手 7 维真机执行端，使用单手 compact 关节顺序。
- `aero_hand_dual_server.yaml`：双手 14 维真机执行端，顺序是 left 7 维 + right 7 维。
- `aero_hand_single_rynnbot_app.yaml`：单手云端设备的 RynnBot App 配置。
- `aero_hand_dual_rynnbot_app.yaml`：双手云端设备的 RynnBot App 配置。
- `aero_hand_single_hand_master_server.yaml`：单手摄像手势控制端，输出 7 维关节状态。
- `aero_hand_dual_hand_master_server.yaml`：双手摄像手势控制端，输出 left 7 + right 7。
- `aero_hand_*_hand_master_rynnbot_app.yaml`：摄像手势控制端通过 RynnBot 控制仿真目标的配置。
- `rynnrcp-aero-hand-configure`：浏览器配置/调试页面。

## 安装

推荐使用安装脚本。它会创建 `robots/tetheria_aerohand/venv`，并以 editable 模式安装 RynnRCP、官方 App 和 Aero Hand 包。

从项目根目录执行：

```bash
cd robots/tetheria_aerohand
bash setup_aero_hand.sh
source venv/bin/activate
```

如需指定 Python 源或重建环境：

```bash
bash setup_aero_hand.sh --pip-index-url https://mirrors.aliyun.com/pypi/simple/ --recreate
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
- 扫描并选择摄像头；摄像手势控制端通过 RynnBot 控制仿真目标时填写对应凭据。
- 在“真机调试”中启动摄像头手势遥操，通过实时骨架画面验证识别结果和真机控制链路。

Linux 通常是 `/dev/ttyUSB0`、`/dev/ttyACM0` 或 `/dev/serial/by-id/...`。

## 真机执行端构型和数据集

云端下发/采集的是 RynnBot 格式的 `observation.state` 和 `action`，RynnBot App 会在 RCP 里映射为：

```text
observation.robot.joint_state
action.robot.joint_position
```

单手数据集是 7 维，画面中的任意一只手都使用同一组控制方式。

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

判断方法是：为本次启动 RynnBot App 的真实端填写一组凭据。仿真目标由仿真环境提供，因此控制仿真时填写摄像手势控制端。

| 使用方式 | 凭据 |
| --- | --- |
| 本地手势遥操、本地 Teleop、本地数采 | 直接使用本地 Server 配置 |
| 真机本体连接 RynnBot | 填写真机执行端一组 |
| 摄像手势控制端通过 RynnBot 控制仿真目标 | 填写摄像手势控制端一组 |

真机执行端用于 Aero Hand 本体接入 RynnBot。摄像手势控制端读取摄像头并产生控制动作，以 controller 设备身份接入 RynnBot 并控制仿真目标。根据本次用途选择一组凭据。

单手：

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

双手：

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

## 摄像手势控制端

摄像手势控制端使用 MediaPipe 完成手势检测、滤波、归一化和 16 维到 compact 7 维映射。模型文件随 Aero Hand 包安装。

本地单手遥操与数采需要三个终端，依次启动：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
rynnrcp-teleop-app
```

Teleop 控制流选择：

```text
observation.robot.joint_state -> action.robot.joint_position
```

单手摄像手势控制端自动选择画面中的一只手并发布 7 维状态。双手摄像手势控制端按 `left 7 + right 7` 发布，并在左右手都至少识别一次后提供首个完整状态；某只手短暂离开画面时保持最后一个有效姿态。

RynnBot 云端仿真控制模式：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
```

本地双手遥操与数采：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
rynnrcp-teleop-app
```

RynnBot 双手控制端：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
```

每条命令在独立终端中运行。先等待 Server 打印就绪信息，再启动对应 App。摄像手势控制端和真机执行端使用相同构型维度。

出现 `robot_id '...' was not found` 时，依次确认：

1. 对应的 `rynnrcp-server` 进程仍在运行。
2. Server 配置中的 `manifest.robot_id` 与 App 的 `--server-config` 指向同一个文件。
3. Server 已完成摄像头初始化并打印就绪信息；随后重新启动 App。

## 标定说明

Aero Hand 日常调试只需要在安全位置手动执行 homing。

首次装机、换电机、调整电机 ID、限流或 trim 时，才需要使用厂商 SDK 的维护能力。

## Controller 约定

- RCP 对外单位是 radians。
- Aero Hand SDK 内部单位是 degrees。
- 单手 7 维关节顺序：`thumb_cmc_abd`、`thumb_cmc_flex`、`thumb_mcp_ip`、`index_finger`、`middle_finger`、`ring_finger`、`pinky_finger`。
- 双手 14 维关节顺序：左手 7 维在前，右手 7 维在后。
