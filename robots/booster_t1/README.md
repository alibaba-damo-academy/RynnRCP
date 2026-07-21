# Booster T1

RynnRCP integration for Booster T1 high-level motion control and low-level
`LowCmd` policy control.

[简体中文](README.zh-CN.md)

## Requirements

- Booster T1 motion-control computer
- Python environment that can install `booster_robotics_sdk_python==1.5.6`
- Running Booster motion/loco services

## Install

Enter the Booster T1 directory, then install and activate the environment:

```bash
cd robots/booster_t1
bash setup_booster_t1.sh
source venv/bin/activate
```

The setup script installs `booster_robotics_sdk_python==1.5.6`, RynnBot, MCP,
and Protocol Debug.

## Run

Start one Server profile:

```bash
rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_low_server.yaml
```

Choose either the high-level or low-level Server. After the Server is running,
start any required app in another terminal:

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_booster_t1/config/booster_t1_rynnbot_app.yaml --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
```

Each Server prints a `Debug UI` address after startup. Open that address in a
browser to inspect state, Actions, or live charts. If the port is occupied, use
the fallback address printed by that Server terminal.

Fill the RynnBot credentials in `booster_t1_rynnbot_app.yaml` before starting that app.

## Capabilities

- High-level mode switching and movement control
- Low-level joint state, IMU, and 50 Hz `LowCmd`
- Local ONNX `walk` policy

Support the robot securely before first low-level tests, and enter low-level
mode with `action.robot.enter_low`. See the
[debugging reference](DEBUGGING.zh-CN.md) for mode switching and policy
details.
