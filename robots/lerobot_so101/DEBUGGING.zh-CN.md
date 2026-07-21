# SO101 调试参考

用于排查 SO101 单臂和双臂的串口、标定、相机、遥操和动作响应问题。安装、配置和启动流程见 [README](README.zh-CN.md)。

[English](DEBUGGING.md)

## 先定位故障组件

1. 打开 Server 终端打印的 `Debug UI` 地址。
2. 查看 `health`、关节状态和每路相机是否持续更新。
3. 在终端日志中找到第一个 `ERROR` 或重复出现的 `WARNING`。
4. 双臂健康信息中的 `details.side` 会标明 `left` 或 `right`。

完成配置后可单独启动 follower 做检查：

```bash
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml
# 或
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

## 串口连接失败

在配置页面重新扫描串口，并确认每条机械臂绑定不同的设备。双臂需要四个唯一串口。

macOS 可以查看串口是否被其他进程占用：

```bash
lsof /dev/cu.usbmodem*
```

Linux 可以查看设备和占用进程：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
fuser /dev/ttyACM0
```

处理顺序：

1. 停止正在使用同一串口的 Server、标定工具或串口终端。
2. 重新插拔设备并在配置页面刷新。
3. 重新绑定左右、主从关系。
4. 保存配置后重启 Server。

如果左右动作颠倒，交换配置页面中的左右串口绑定，再执行小幅动作测试。

## 标定失败

标定时，同一串口只能由配置工具使用。先停止连接该机械臂的 Server，再开始标定。

双臂分别保存四份标定：

- 左从臂
- 右从臂
- 左主臂
- 右主臂

每条机械臂按以下顺序处理：

1. 确认页面显示的串口和机械臂角色正确。
2. 单独连接并标定这一条机械臂。
3. 完成后保存，再处理下一条。
4. 全部完成后执行小幅动作测试。

出现“标定文件不匹配”或某侧关节方向异常时，为该侧重新选择正确串口并重新标定。保存双臂配置时，左右机械臂应使用不同的 robot ID。

## 相机无法打开或持续读帧失败

先在配置页面重新扫描和预览，确认每个位置绑定不同的相机编号：

- 单臂：`front`、`wrist`
- 双臂：`front`、`left_wrist`、`right_wrist`

如果日志持续出现以下信息：

```text
<device_id>: Failed to read a frame
reopening camera after 5 failed reads
```

日志中的 `<device_id>` 就是失败的相机编号。只有第三路失败、前两路正常时，通常是共享 USB Hub 的带宽或供电不足。

按以下顺序处理：

1. 把至少一路相机移到另一条独立的主机 USB 端口。
2. 使用带独立供电的 Hub，并避免三路相机和四条机械臂都集中在同一个无源 Hub。
3. 暂时断开一路相机，确认剩余两路可以稳定运行。
4. 在配置页面重新扫描并保存相机映射，然后重启 follower Server。

配置中的 `encoding: jpg` 表示发布 JPEG 图像；具体采集后端不一定会让相机在 USB 线上使用压缩格式，因此它不能替代 USB 带宽和供电检查。

腕部图像左右颠倒时，重新绑定 `left_wrist` 和 `right_wrist`，保存后重启 follower Server。

## 遥操只有一侧跟随

确认 follower 和 leader 使用同一种构型：

```bash
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_leader_server.yaml
rynnrcp-teleop-app
```

在 Debug UI 中检查：

- follower 和 leader 的关节状态都是 12 维。
- action 也是 12 维，顺序为左 6 + 右 6。
- `health` 显示左右两侧连接正常。
- 左右主臂分别使用对应且唯一的串口。

首次真机检查使用保持当前位置或小幅关节动作，并确认机器人状态和相机流持续更新。

## 动作响应慢或跟踪幅值不足

使用电机响应测试绕过 Teleop 和 RCP，直接测量单条机械臂：

```bash
python -m lerobot_so101.motor_response_test \
  --port /dev/cu.usbmodem-follower \
  --joint shoulder_lift \
  --amplitude 0.20 \
  --frequency 0.5 \
  --duration 8 \
  --control-hz 60
```

脚本会输出 CSV、SVG 和摘要。重点检查：

- `lag_ms`：实际位置相对目标的延迟。
- `amplitude_ratio`：实际运动幅值除以目标幅值。
- `send_p95_ms`、`read_p95_ms`：发送和读取调用的耗时。

需要观察 controller 内部的 action/state 时，启用位置 trace：

```bash
export RYNNRCP_SO101_TRACE_POSITIONS=1
export RYNNRCP_SO101_TRACE_DIR=/tmp/so101_trace
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

停止 follower Server 后会生成：

- `so101_trace_*.csv`：`action`、`sent`、`state` 位置采样。
- `so101_trace_*.timing.csv`：`read_state`、`send_action`、`worker_tick` 耗时。
- `so101_trace_*.svg`：目标、发送值和读回状态曲线。

控制循环调试参数：

```bash
export RYNNRCP_SO101_CONTROL_LOOP_HZ=100
export RYNNRCP_SO101_STATE_READ_HZ=60
export RYNNRCP_SO101_MAX_JOINT_VELOCITY_RAD_S=30
export RYNNRCP_SO101_MAX_GRIPPER_VELOCITY_PER_S=30
```

修改限制前先完成小幅动作验证。

## RynnBot 连接失败

判断凭据时，看这次运行中哪个本体要接入 RynnBot。让从臂本体接入云端并启动下面的从臂 App，就填写“从臂执行端”一组 `product_key`、`device_name` 和 `device_secret`。本地 Teleop、MCP 和 Protocol Debug 直接使用本地 Server 配置。

真实主臂作为控制源，通过 RynnBot 控制仿真从臂时，填写“主臂控制端”一组凭据。“从臂执行端”和“主臂控制端”分别对应这两种使用方式。

保存后启动：

```bash
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
# 或
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_bimanual_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

App ID 由配置工具生成，HTTP URL 通常保持默认。本地主从遥操直接连接 leader Server，不使用 RynnBot 凭据。

## Controller 约定

- 每条机械臂的前五个关节使用弧度。
- 夹爪使用 `[0, 1]` 归一化比例。
- 双臂状态和动作固定为左 6 + 右 6。
- 双臂健康信息使用 `details.side` 标识故障侧。

## 配置与测试参考

配置文件位于 `rynnrcp_robot_so101/config/`。需要直接检查配置时，重点查看：

| 文件 | 用途 |
| --- | --- |
| `robot_integration.yaml` | 单臂 6 DoF 接入定义 |
| `robot_integration_bimanual.yaml` | 双臂 12 DoF 接入定义 |
| `so101_follower_server.yaml` | 单臂 follower、串口和相机 |
| `so101_leader_server.yaml` | 单臂 leader 串口 |
| `so101_bimanual_follower_server.yaml` | 双臂 follower、左右串口和三路相机 |
| `so101_bimanual_leader_server.yaml` | 双臂 leader 左右串口 |

框架级测试入口见 [tests/README.md](../../tests/README.md)。
