# Atom01

RynnRCP integration for the Atom01 humanoid robot, including the Atom01 C++
control binding, Server/RynnBot configs, calibration UI, and local policies.

[简体中文](README.zh-CN.md)

## Requirements

- Atom01 robot machine
- Linux build tools for the packaged C++ binding

## Install

Enter the Atom01 directory, then install and activate the environment:

```bash
cd robots/roboparty_atom01
bash setup_atom01.sh
source venv/bin/activate
rynnrcp-atom01-configure
```

The setup script installs RynnBot, MCP, and Protocol Debug, then builds
`atom01_py`.

## Run

Start the Server after configuration:

```bash
rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

After the Server is running, start any required app in another terminal:

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_atom01/config/atom01_server.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_atom01/config/atom01_rynnbot_app.yaml --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
```

The Server prints a `Debug UI` address after startup. Open that address in a
browser to inspect state, Actions, or live charts. If the port is occupied, use
the fallback address printed by the Server terminal.

## Capabilities

- Robot state and motor control
- Zero-position calibration UI
- Local `stand`, `walk`, and `sim2simdance` policies
- MCP and RynnBot integration

Before sending real actions, check `get_health` and clear calibration or motor
warnings. See the [debugging reference](DEBUGGING.zh-CN.md) for policy and
calibration details.
