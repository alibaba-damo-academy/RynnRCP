# Bumi 调试参考

这个目录只保留 Bumi 接入 RynnRCP 必需的代码、配置、模型和说明文档。

厂商 SDK、DDS、依赖安装以 Noetix 官方文档为准：

https://web.noetixrobotics.com/docs

## 最短接入

先按“准备 SDK”跑通厂商示例，再进入 Noetix Bumi 目录执行：

```bash
cd robots/noetix_bumi
export BUMI_SDK_ROOT=<noetix_sdk_bumi>
bash setup_bumi.sh
source venv/bin/activate
rynnrcp-server --config rynnrcp_robot_bumi/config/bumi_high_server.yaml
```

setup 脚本会同时安装 RynnBot、MCP 和 Protocol Debug App。Server 启动后按需另开终端：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-rynnbot-app --config <rynnbot-app.yaml> --server-config rynnrcp_robot_bumi/config/bumi_high_server.yaml
```

`<rynnbot-app.yaml>` 是包含云端设备凭据的本机配置，不应提交到仓库。使用 low-level 时把
server config 换成 `bumi_low_server.yaml`，并先阅读下文安全步骤。

## 目录

```text
robots/noetix_bumi/
  pyproject.toml                         # Bumi RCP 包定义
  setup_bumi.sh                          # 本地 venv 安装脚本，不下载/编译厂商 SDK
  test_policy_auto.py                    # 无手柄 ONNX policy 直连 SDK 验证脚本
  README.zh-CN.md
  rynnrcp_robot_bumi/
    controller.py                        # high-level SDK 封装
    low_controller.py                    # low-level SDK 封装
    config/
      bumi_high_server.yaml
      bumi_low_server.yaml
      robot_integration.yaml
      robot_integration_low.yaml
    policies/walk/
      policy.py
      policy.yaml
      model.onnx                         # 需要随包保存，否则 start_policy 找不到模型
```

## 准备 SDK

本包不下载、不编译、不修改 `noetix_sdk_bumi`。先按官方文档准备好 Bumi SDK，并确认目录至少有：

```text
build/
config/dds.xml
lib/
examples_py/
```

设置 SDK 路径：

```bash
export BUMI_SDK_ROOT=<noetix_sdk_bumi>
```

先跑厂商示例：

```bash
cd "$BUMI_SDK_ROOT"
python3 examples_py/test_high.py
python3 examples_py/test_low.py
```

如果 `highcontrol_py`、`lowcontrol_py` import 失败，先修复 SDK 环境，再启动 RCP。

## 安装

在 RynnRCP 仓库里：

```bash
cd robots/noetix_bumi
bash setup_bumi.sh
source venv/bin/activate
```

没提前 export 时：

```bash
bash setup_bumi.sh --sdk-root <noetix_sdk_bumi>
source venv/bin/activate
```

`setup_bumi.sh` 只做三件事：

- 创建 `robots/noetix_bumi/venv`
- 安装 RynnRCP、调试台、MCP app、`rynnkit`、Bumi 包
- 校验 SDK 的 `build/`、`config/dds.xml`、`lib/`

仓库配置不保存个人 SDK 绝对路径、相机序列号、IP 或账号凭据。需要本机参数时，
优先使用环境变量（例如 `BUMI_SDK_ROOT`），或在本地未提交配置副本里填写。

## High-Level

High-level 用于状态、动作库、相机和厂商 `publish_cmd` 行走。

```bash
rynnrcp-server --config rynnrcp_robot_bumi/config/bumi_high_server.yaml
```

另开终端：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_bumi/config/bumi_high_server.yaml
```

常用 observation：

```text
observation.robot.mode
observation.robot.battery
observation.robot.imu
observation.robot.joint_state
observation.head_camera.image
```

推荐动作顺序：

```text
action.robot.enter_walk
等待 observation.robot.mode.mode == 2
action.robot.base_velocity
action.robot.stop
```

实机现象：`base_velocity.linear x`、`angular z` 是 Bumi SDK 归一化量，不是 m/s 或 rad/s。明显移动优先用 `1.0` 或 `-1.0`，`0.3` 这类小值可能不动。

动作库：

```text
action.robot.swing
action.robot.shake
action.robot.cheer
action.robot.dance
action.robot.dance1
action.robot.dance2
action.robot.tear
action.robot.fall_to_stand
action.robot.stand_to_fall
```

动作类命令发送一次即可，代码会自动补 `DEFAULT`。

## Low-Level

Low-level 会直接发关节目标。切换模式前先停掉另一个 Server；实机测试先吊装。

```bash
rynnrcp-server --config rynnrcp_robot_bumi/config/bumi_low_server.yaml
```

调试台：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_bumi/config/bumi_low_server.yaml
```

最小能力：

```text
observation.robot.joint_state
observation.robot.imu
observation.robot.battery
action.robot.joint_position
action.robot.damping
policy: walk
```

先用直连 SDK 脚本确认 ONNX 和站姿能跑：

```bash
python test_policy_auto.py --onnx rynnrcp_robot_bumi/policies/walk/model.onnx --seconds 10 --cmd-vel 0 0 0
```

这个脚本是无手柄版厂商流程：读取 SDK `config/bumi_ac.yaml`，先插值到 `default_joint_pos`，再 500Hz 发送关节命令、50Hz 推理 ONNX，退出时 damping。

RCP policy 启动：

```text
1. 只启动 low server，确认 joint_state/imu/battery 在刷新。
2. 先用 test_policy_auto.py 零速度确认 ONNX 和低层链路。
3. 再 start_policy(policy_id="walk", runtime_inputs={"cmd_vel": [0.0, 0.0, 0.0]})。
4. 零速度稳定后 update_policy_inputs。
5. 停止 policy 后再按需要 damping。
```

```text
start_policy(policy_id="walk", runtime_inputs={"cmd_vel": [0.0, 0.0, 0.0]})
```

更新速度：

```text
update_policy_inputs(policy_id="walk", runtime_inputs={"cmd_vel": [1.0, 0.0, 0.0]})
```

`cmd_vel` 是 `[x, y, yaw]`：

```text
[ 1.0, 0.0,  0.0]  前进
[-1.0, 0.0,  0.0]  后退
[ 0.0, 0.0,  1.0]  左转
[ 0.0, 0.0, -1.0]  右转
```

当前 policy 预处理会忽略 `abs(yaw) < 0.3` 的转向小值，调试时先用 `1.0` 或 `-1.0`。

停止后建议：

```text
run_action_chunk("action.robot.damping", [{}], frame_rate=1)
```

日志判断：

```text
Bumi policy loaded              # ONNX 已加载
Bumi policy infer               # policy.py 正在推理
Bumi low joint_position command # low_controller 已把关节目标送进 SDK
```

## MCP App

MCP App 只把已经启动的 RCP Server 暴露成 MCP 工具，不新增机器人能力。

启动顺序：

```text
先 rynnrcp-server
再 rynnrcp-mcp-app
```

```bash
rynnrcp-mcp-app --server-config rynnrcp_robot_bumi/config/bumi_high_server.yaml
```

默认地址：

```text
http://<host>:28403/mcp
```

调 low-level 时把 config 换成 `rynnrcp_robot_bumi/config/bumi_low_server.yaml`。

## 字段约定

标准字段保持简单：

- `observation.robot.joint_state`: `joint_positions`，可选 `joint_velocities`
- `observation.robot.imu`: `accel`、`gyro`，可选 `orientation_quat_wxyz`
- `observation.robot.battery`: `battery_soc`、`battery_temp`、`battery_soh`、`battery_alarm`
- `action.robot.joint_position`: `joint_positions`

Bumi 的 kp/kd 由 low controller 从 SDK `config/bumi_ac.yaml` 读取。
