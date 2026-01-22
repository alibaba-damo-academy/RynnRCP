# Overview

**RynnRCP** provides a service-oriented capability framework for robot embodiments and an external communication protocol (Robotics Context Protocol). Its goal is to encapsulate robot-side capabilities—such as motion control, sensor acquisition, and device monitoring—into a unified set of **Tools**, and expose them to external applications / cloud platforms via **communication plugins (Plugins)**.

This project mainly consists of two parts: **RCP Core (tool services)** on the robot side and **Comm Plugin (external protocol/transport)**:
- **RCP Core**: integrates robot hardware and sensors, maintains buffers, aligns observations, executes actions, and exposes capabilities as Tools.
- **Comm Plugin**: exposes Tools to external systems via specific protocols and transports such as MQTT, WebSocket, MCP (JSON-RPC), etc. Currently, a plugin for the Leyun cloud platform (**RynnBot**) is implemented.

> With RynnRCP, users can clearly understand the end-to-end pipeline from sensor data acquisition to model inference and robot action execution. Meanwhile, the layered architecture and standardized communication protocol make it easier to migrate the system to specific application scenarios.


# Directory Structure

```bash
.
├── setup_rcp.sh                 # One-click installation/setup (recommended)
├── pyproject.toml               # Python package configuration
├── README.md / README_cn.md     # Project documentation
├── rynnrcp/                     # Top-level entry: RynnRCP
├── rcp_core/                    # RCP Core: tool service core
│   ├── rcp_core.py              # RcpCore initialization entry
│   ├── action_server/           # Action-related servers
│   ├── sensor_server/           # Sensor-related servers
│   ├── device_monitor_server/   # Device monitoring server
│   └── common/                  # bus / adapter / protocol / utils, etc.
├── comm_plugin/                 # Communication plugins: external protocol/transport
│   ├── base_plugin/             # Plugin interface definitions
│   └── rynnbot_plugin/          # RynnBot plugin (MQTT + WebSocket)
├── rcp_sensor/                  # Port devices / sensor implementations
│   └── camera/                  # Camera implementation
├── robots/                      # Minimal runnable projects & docs for each robot
│   ├── so100/
│   └── so101/
├── rcp_motion/                  # Motion control + simulation/toolchain
├── common/                      # Shared files (configs, LCM messages, etc.)
│   ├── config/
│   └── lcm/
├── scripts/                     # Scripts
└── docs/                        # Documentation

```

# Compatibility Support Status


## Robot Configurations
| Robot | Platform/Plugin | Typical Integration | Directory | Notes |
|---|---|---|---|---|
| SO100 | RynnBot platform (MQTT + WebSocket) | Action: module | Sensor: port	 | `robots/so100/` | Minimal runnable example + configuration wizard |
| SO101 | RynnBot platform (MQTT + WebSocket) | Action: module | Sensor: port	 | `robots/so101/` | Minimal runnable example + configuration wizard |

> The SO100/SO101 configurations support zero-code integration: users only need to configure the serial port, cameras, and platform device triplet to run. The robot control logic is provided by the built-in `controller` in `rcp_motion` (no need to write SDK calls). Meanwhile, `rcp_motion` (RCP Motion) converts low-frequency discrete outputs from the cloud/model into high-frequency continuous control signals executable by the robot, and provides tooling such as MuJoCo simulation, real-robot debugging & replay, data collection, and trajectory visualization.

## Services (RCP Core)

| Server | Status | Tools | Description |
|---|---|---|---|
| ActionServer | ✅ | `get_state`、`run_action_chunk` | Observation alignment and action execution; exposes state snapshot and action chunk execution |
| SensorServer | ✅ | `get_image`、`get_image_info` | Sensor data service (currently camera-focused): image capture/encoding and camera capability enumeration |
| DeviceMonitorServer | ✅ | `get_device_info` | Device/system monitoring: CPU/memory/system info and camera list, etc. |
| Skill Server | 🚧 | - | Skill encapsulation and execution |
| Data Manager Server | 🚧 | - | Data collection, replay, and management |
| Infer Server | 🚧  | - | Inference service: supports multiple models (e.g., VLA/RL) for local or remote inference; calls Action/Sensor via BUS at runtime |
| Model Manager Server | 🚧 | - | Model management: provides model management services |
| Agent Server | 🚧 | - | Workflow/task orchestration: parses and executes a unified workflow spec; can call Skills or low-level Tools; will integrate planning models and support cross-robot collaboration after secure pairing |


## Communication Plugins (Comm Plugin)

| Plugin | Status | Protocol | Description |
|---|---|---|---|
| RynnBot Plugin | ✅ | MQTT + WebSocket | RynnBot platform integration: device claim/release, action dispatch, image/state fetching, and device property reporting |
| MCP Plugin | 🚧 | JSON-RPC (MCP-style) | Standardized tools/list / tools/call integration for local/LAN/WAN connectivity with external apps and model services |


# Installation & Usage

## Clone
```bash
git clone https://github.com/alibaba-damo-academy/RynnRCP.git
```

## Quick Setup (Recommended)

We provide a unified setup script with bilingual support (English/Chinese):

```bash
cd RynnRCP
bash setup_rcp.sh
```

It will guide you through:
1. Language selection (English / 中文)
2. Installing uv (if not installed)
3. Creating and activating a virtual environment (venv/)
4. Installing RynnRCP and development dependencies
5. Generating protocol and message code (protobuf + LCM; on Windows, LCM failures will be reported and skipped)
6. Import checks (rynnrcp / rcp_motion / mujoco)


## Manual Installation


### Python Environment
```bash
cd RynnRCP

# Using venv
python3 -m venv venv
source venv/bin/activate
pip install -e .

# Or using Conda
conda create --name venv python=3.10
conda activate venv
pip install -e .

# Generate protocol & message code
python scripts/gen_proto_msg.py
python scripts/gen_lcm_msg.py   # May need to skip on Windows
```
> Python 3.10 is the minimum required version.

## Run

This project supports multiple robot arms/configurations. For robot-specific configuration, calibration, and launch instructions (SO100/SO101, etc.), see:
- robots/<robot_name>/README.md
- robots/<robot_name>/config/*.yaml
- robots/<robot_name>/run_rcp_*.py launch scripts

Typical workflow:
1. Go to the target robot directory (e.g., robots/so100/)
2. Follow its README to finish configuration (camera/serial/plugin settings)
3. Run the example launch script (typically run_rcp_*.py)


## Advanced Development
Advanced development is supported, including:
- Adding a new robot configuration
- Adding a custom `Server`
- Adding a new `Comm Plugin`

For more detailed instructions and examples, please refer to: [RynnRCP Tutorials](https://rynnrcp.github.io/)


## Notes

**Linux：**
- Some setup steps may require sudo privileges
- UDP buffer settings will be adjusted in /etc/sysctl.conf to meet LCM image transmission requirements

**macOS：**
- Supports Apple Silicon (M1/M2/M3/M4) and Intel Macs
- For LCM multicast: sudo route -nv add -net 224.0.0.0/4 -interface lo0

**Windows：**
- Cameras must not be connected through the same USB hub/expander; otherwise, multiple cameras cannot be opened simultaneously.
- It is recommended to use Git Bash as the terminal for running setup_rcp.sh to configure the environment. When installing Git, pay attention to the line-ending configuration and select "Checkout as-is, commit Unix-style line endings" (i.e., use Unix-style line endings).

# Todo
- [x] Release **RynnRCP** 1.0 version
- [ ] Release Technical Report
- [ ] ActionServer and SensorServer support MCP
- [ ] Complete the rest of RynnRCP framework
- [ ] Support for robots with more structural types
