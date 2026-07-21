# Booster T1 调试参考

High-level 版本使用 Booster 官方 Python SDK。当前验证版本为
`booster_robotics_sdk_python==1.5.6`。

## 最短接入

先按“准备”安装并确认 Booster SDK 可用，再进入 Booster T1 目录执行：

```bash
cd robots/booster_t1
bash setup_booster_t1.sh
source venv/bin/activate
rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
```

setup 脚本会同时安装 RynnBot、MCP 和 Protocol Debug App。Server 启动后按需另开终端：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
rynnrcp-rynnbot-app --config <rynnbot-app.yaml> --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
```

`<rynnbot-app.yaml>` 是包含云端设备凭据的本机配置，不应提交到仓库。需要 low-level
`LowCmd` 和本地 `walk` policy 时，把上述 server config 换成 `t1_low_server.yaml`，并阅读下文安全步骤。

## 准备

先在机器人运控板安装并验证 Booster SDK：

```bash
python3 -m pip install booster_robotics_sdk_python==1.5.6
python3 - <<'PY'
import importlib.metadata
import booster_robotics_sdk_python as sdk
print("Booster SDK OK:", importlib.metadata.version("booster_robotics_sdk_python"))
print("SDK path:", sdk.__file__)
PY
```

标准版 T1 使用 `booster_robotics_sdk_python==1.5.6`。其他版本可能缺少当前接入依赖的接口。确认实际 import 路径：

```bash
python3 - <<'PY'
import importlib.metadata
import booster_robotics_sdk_python as sdk
version = importlib.metadata.version("booster_robotics_sdk_python")
assert version == "1.5.6", f"expected booster_robotics_sdk_python==1.5.6, got {version}"
print("version:", version)
print(sdk.__file__)
print([x for x in dir(sdk.RobotMode) if not x.startswith("_")])
PY
```

RynnRCP 的 `setup_booster_t1.sh` 也会在 venv 中安装同一版本：

```bash
bash setup_booster_t1.sh --booster-sdk-version 1.5.6
```

如果 import 失败，先确认 pip 源能安装 1.5.6，或使用 Booster 随固件/SDK 分发的同版本 wheel。

开机调试顺序：

```text
1. 机器人吊挂或安全支撑。
2. 松开急停。
3. 等待 Booster motion/loco 服务启动。
4. 确认 SDK WaitForService 为 True。
5. 先测 Damping -> Prepare -> Walking -> Damping。
6. 再启动 RCP high-level server。
```

SDK 服务检查：

```bash
python3 - <<'PY'
from booster_robotics_sdk_python import ChannelFactory, B1LocoClient

ChannelFactory.Instance().Init(0, "127.0.0.1")
c = B1LocoClient()
c.Init()
print("wait:", c.WaitForService(10000))
if c.WaitForService(1000):
    s = c.GetStatus()
    print("status:", s.current_mode, s.current_body_control, s.current_actions)
PY
```

吊挂下的 high-level 最小动作测试：

```bash
python3 - <<'PY'
import time
from booster_robotics_sdk_python import ChannelFactory, B1LocoClient, RobotMode

ChannelFactory.Instance().Init(0, "127.0.0.1")
c = B1LocoClient()
c.Init()
assert c.WaitForService(), "loco service not ready"

try:
    c.ChangeMode(RobotMode.kPrepare)
    time.sleep(2)
    print("prepare:", c.GetStatus().current_mode, c.GetStatus().current_body_control, c.GetStatus().current_actions)

    c.ChangeMode(RobotMode.kWalking)
    time.sleep(1)
    print("walking:", c.GetStatus().current_mode, c.GetStatus().current_body_control, c.GetStatus().current_actions)

    c.Move(0.03, 0.0, 0.0)
    time.sleep(0.3)
    c.Move(0.0, 0.0, 0.0)
finally:
    c.ChangeMode(RobotMode.kPrepare)
    time.sleep(0.5)
    print("after:", c.GetStatus().current_mode, c.GetStatus().current_body_control, c.GetStatus().current_actions)
PY
```

交互等待用独立脚本或 Python REPL；heredoc 会占用 stdin，`input()` 会 EOF。

## 安装

在 Booster T1 目录里：

```bash
bash setup_booster_t1.sh --recreate
source venv/bin/activate
```

setup 脚本会安装 RynnRCP、`robots/booster_t1` 和 `booster_robotics_sdk_python==1.5.6`，并验证 Booster SDK 和 RCP controller 能 import。
同时会安装 protocol_debug、apps/common 和 MCP app。

## 启动

```bash
rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
```

常用 observation：

```text
observation.robot.mode
```

推荐动作顺序：

```text
action.robot.enter_walk
等待 observation.robot.mode.name == "walking"
action.robot.base_velocity
action.robot.stop
action.robot.damping
```

RCP client 最小联调：

```bash
python3 - <<'PY'
from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.protocol_client import connect_to_server

iface = ClientInterface.with_defaults(local_registry=True, mdns=False)
c = connect_to_server(robot_id="booster_t1", interface=iface)
print("obs:", c.get_observations(["observation.robot.mode"], sync=True).payload)
print("actions:", [x["name"] for x in c.list_actions().payload["actions"]])
PY
```

吊挂下的 RCP 最小动作测试：

```bash
python3 - <<'PY'
import time
from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.protocol_client import connect_to_server

iface = ClientInterface.with_defaults(local_registry=True, mdns=False)
c = connect_to_server(robot_id="booster_t1", interface=iface)

print(c.run_action_chunk("action.robot.enter_walk", [{}], frame_rate=1).ok)
time.sleep(1)
print("mode:", c.get_observations(["observation.robot.mode"], sync=True).payload)

print(c.run_action_chunk(
    "action.robot.base_velocity",
    [{"linear_vel": [0.03, 0.0, 0.0], "angular_vel": [0.0, 0.0, 0.0]}],
    frame_rate=20,
).ok)

time.sleep(1)
print(c.run_action_chunk("action.robot.stop", [{}], frame_rate=1).ok)
print(c.run_action_chunk("action.robot.damping", [{}], frame_rate=1).ok)
print("mode:", c.get_observations(["observation.robot.mode"], sync=True).payload)
PY
```

如果 `WaitForService` 超时，先查急停是否松开，再查 motion 服务：

```bash
ps -ef | grep -iE 'loco|motion|booster|dds|cyclone' | grep -v grep
```

重启后推荐顺序是：开机、松急停、等服务、`WaitForService`、再发动作。

## MCP App

MCP App 只把已经启动的 RCP Server 暴露成 MCP 工具，不新增机器人能力。先启动 `rynnrcp-server`，再启动：

```bash
rynnrcp-mcp-app --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
```

默认 MCP 服务：

```text
http://<robot-ip>:28403/mcp
```

## Low-Level

Low-level 直接发布 `LowCmd`。切换前先停掉另一个 Server，实机首测必须吊挂。

启动 low server：

```bash
rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_low_server.yaml
```

Protocol Debug 调 low-level 时，顺序固定：

```text
1. 确认 observation.robot.joint_state 和 observation.robot.imu 都在刷新。
2. 执行 action.robot.enter_low，确认 health 不再提示 enter_low_required。
3. 启动 policy: walk，runtime_inputs 先用 {"cmd_vel": [0.0, 0.0, 0.0]}。
4. 零速度稳定后，再更新 cmd_vel，例如 [0.05, 0.0, 0.0]。
5. 停止 policy。控制流停止超过 0.5 秒后 controller 会回 Prepare；再次推理前重新执行 enter_low。
```

启动 policy 前先执行 `action.robot.enter_low`。未进入 low/custom 时，health 会提示
`booster_t1_low.enter_low_required`，`joint_position` 会拒绝执行。

low server 暴露：

```text
observation.robot.joint_state
observation.robot.imu
action.robot.enter_low
action.robot.joint_position
action.robot.damping
policy.walk
```

推荐低层调试顺序：

```text
1. 只启动 low server，不启动 high server。
2. 先读 observation.robot.joint_state，确认能收到 23 维关节状态。
3. 执行 action.robot.enter_low，让机器人 Prepare -> Custom。
4. 首次 joint_position 只发当前关节值，确认链路没有跳变。
5. 再在单个关节上加很小 delta。
6. 正常退出时回到 Prepare；只有机器人有可靠支撑并且明确需要卸力时才执行 action.robot.damping。
```

不经过 RCP 的直接 policy 测试：

```bash
cd <rynnrcp-root>/robots/booster_t1
source venv/bin/activate
python3 run_policy_direct.py --duration 5 --x 0.0 --y 0.0 --yaw 0.0
```

确认零速度稳定后，再给很小速度：

```bash
python3 run_policy_direct.py --duration 5 --x 0.05 --y 0.0 --yaw 0.0
```

这个脚本会直接使用 `BoosterT1LowController` 和 `policies/walk/policy.py`：启动低层 SDK、把 29 电机状态映射为 23 维 policy 状态、以 50 Hz 持续发送 LowCmd、切 `Prepare -> Custom`、缓慢插值到 policy 默认站姿并运行 `model.onnx` 推理。正常退出和异常收尾默认回到 Prepare；只有显式传入 `--exit-mode damping` 才发送 damping。运行前停止 high/low RCP server，机器人必须可靠支撑。

Policy 或 joint_position 流停止更新超过 0.5 秒时，low controller 会自动回到 Prepare 并停止 LowCmd heartbeat。再次控制前必须重新执行 `action.robot.enter_low`。

Policy 接入链路：

```text
start_policy(walk)
  -> policy.py ONNX Runtime inference
  -> action.robot.joint_position
  -> BoosterT1LowController.set_joint_positions
  -> B1LowCmdPublisher / LowCmd
```

启动 policy 前必须先执行：

```text
action.robot.enter_low
```

然后通过 MCP/RCP 调：

```json
{"policy_id": "walk", "runtime_inputs": {"cmd_vel": [0.0, 0.0, 0.0]}}
```

更新速度命令：

```json
{"policy_id": "walk", "runtime_inputs": {"cmd_vel": [0.2, 0.0, 0.0]}}
```

停止 policy：

```json
{"policy_id": "walk", "reason": "manual stop"}
```

默认 policy 会读取随包提供的 `policies/walk/model.onnx`，也可以通过环境变量指定：

```bash
export BOOSTER_T1_POLICY_ONNX=/path/to/model.onnx
```

policy 使用 Booster 官方 deploy 示例的 T1 模型导出的 ONNX，观测为 47 维，输出 12 个腿部动作，再映射成 23 维 `joint_position`。

`action.robot.joint_position` 输入：

```json
{"joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
```

上面是格式示例。实机先读取当前 `joint_positions`，再发送当前值或很小增量。

## High-Level 动作关系

```text
prepare
  前置：机器人已开机，Booster 服务正常；建议支架/安全支撑。
  后置：mode.name == "prepare"。进入 walking 前先走这一步。

walking
  前置：mode.name == "prepare"，机器人已落地站稳。
  后置：mode.name == "walking"。之后才能发 base_velocity。

enter_walk
  前置：机器人可安全进入 prepare/walking。
  后置：mode.name == "walking"。内部执行 prepare -> 等待 -> walking。

base_velocity
  前置：mode.name == "walking"。
  后置：仍是 walking；当前实现发送短速度脉冲后自动 Move(0,0,0)。

stop
  前置：controller 已启动。
  后置：模式不变；打断正在执行的 base_velocity 脉冲。walking 下发送 Move(0,0,0)，非 walking 下不发 Move。

damping
  前置：机器人保持可靠支撑，进入阻尼时姿态稳定。
  后置：mode.name == "damping"，关节不主动保持站立。

controller shutdown
  默认：先切换到 prepare，再停止 LowCmd 并关闭 SDK 通道。
  只有显式 damping 操作或 `--exit-mode damping` 才保留阻尼退出。

get_up
  前置：机器人姿态和周围空间适合执行厂商起身动作。
  后置：取决于 Booster SDK GetUp 行为；调试时需要人工确认模式。
```

标准版当前实测 `RotateHead` 会返回 `API call failed, code = 400`，所以默认不暴露 `action.robot.look`。
