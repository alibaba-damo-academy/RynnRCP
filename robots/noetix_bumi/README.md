# Noetix Bumi

RynnRCP integration for Noetix Bumi high-level actions, low-level joint
control, and local walking policies.

[简体中文](README.zh-CN.md)

## Requirements

- Built Noetix Bumi SDK
- Importable `highcontrol_py` and `lowcontrol_py`

## Install

Enter the Noetix Bumi directory, then install and activate the environment:

```bash
cd robots/noetix_bumi
export BUMI_SDK_ROOT=<noetix_sdk_bumi>
bash setup_bumi.sh
source venv/bin/activate
```

The setup script uses the locally built vendor SDK and installs RynnBot, MCP,
Protocol Debug, and Teleop.

## Run

Start one Server profile:

```bash
rynnrcp-server --config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-server --config rynnrcp_robot_bumi/config/bumi_low_server.yaml
```

Choose either the high-level or low-level Server. After the Server is running,
start any required app in another terminal:

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_bumi/config/bumi_rynnbot_app.yaml --server-config rynnrcp_robot_bumi/config/bumi_high_server.yaml
rynnrcp-teleop-app
```

Each Server prints a `Debug UI` address after startup. Open that address in a
browser to inspect state, Actions, camera images, or live charts. If the port is
occupied, use the fallback address printed by that Server terminal.

Fill the RynnBot credentials in `bumi_rynnbot_app.yaml` before starting that app.

## Capabilities

- Robot state, cameras, and action library
- Low-level joint state, IMU, battery, and joint control
- Local ONNX `walk` policy

Support the robot securely before first low-level tests. See the
[debugging reference](DEBUGGING.zh-CN.md) for SDK checks, action order, and
policy details.
