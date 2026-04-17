# Minimal Example: SO101 to RynnBot (RynnRCP)
This directory provides a minimal runnable project to connect the **SO101** robotic arm to **RynnRCP**, including:

- Config files:
  - `config/so101_config.yaml`: **RcpCore** configuration (inputs/outputs for Action/Sensor/DeviceMonitor)
  - `config/rynnbot_config.yaml`: RynnBot (cloud platform) device credentials (triplet/URL)
  - `config/mcp_config.yaml`: **MCP server** configuration (exposes RCP tools over HTTP)
- Launch script:
  - `run_rcp_so101.py`: starts **RcpCore + RynnBot (plugin) + McpPlugin**
    - **RynnBot**: connects to the cloud platform (MQTT/WS)
    - **McpPlugin**: starts an MCP HTTP service and exposes RCP tools
- Configuration wizard:
  - `configure_so101.py`: interactive setup (Chinese/English), which can configure cameras, follower/leader serial ports, follower/leader arm calibration, and RynnBot device triplet
  - `configure_so101_web.py`: web-based configuration tool with graphical interface, featuring camera live preview, serial port auto-detection, and real-time calibration log streaming


## Directory Structure

```text
robots/so101/
├── config/
│   ├── rynnbot_config.yaml         # RynnBot device triplet / URL
│   ├── so101_config.yaml           # RcpCore config: Action/Sensor/DeviceMonitor (follower)
│   ├── so101_leader_config.yaml    # RcpCore config for leader arm (teleop)
│   ├── teleop_config.yaml          # TeleopPlugin network config (hosts / ports / hz)
│   └── mcp_config.yaml             # MCP Server config
├── configure_so101.py              # Interactive configuration wizard (CN/EN)
├── configure_so101_web.py          # Web-based configuration tool (GUI)
├── configure_so101_web_usage.md    # Web configuration tool usage guide
├── run_rcp_so101.py                # Launch: RcpCore + RynnBot(plugin)
├── run_teleop_leader.py            # Launch: leader arm + Web UI
├── run_teleop_follower.py          # Launch: follower arm
├── README.md                       # Documentation (English)
└── README_cn.md                    # Documentation (Chinese)
```


## Prerequisites

1. Project is installed (Python environment ready, dependencies installed, and venv activated)
2. SO101 controller is connected (serial port)
3. Cameras are connected (USB cameras; two cameras recommended: front / wrist)
4. RynnBot device credentials (triplet):
   - product_key
   - device_name
   - device_secret
   - http_url (platform access URL; keep the default value)
 - If you enable MCP, make sure the port is reachable (example: 8001)


## Configure via Environment Variables (Optional)

### RynnBot Environment Variables
Required (device triplet):
- `RYNNBOT_PRODUCT_KEY`
- `RYNNBOT_DEVICE_NAME`
- `RYNNBOT_DEVICE_SECRET`

Optional:
- `RYNNBOT_HTTP_URL` (default: `https://robot-access.damo-academy.com`)

Example:
```bash
export RYNNBOT_PRODUCT_KEY="put_product_key_here"
export RYNNBOT_DEVICE_NAME="put_device_name_here"
export RYNNBOT_DEVICE_SECRET="put_device_secret_here"
# optional
export RYNNBOT_HTTP_URL="https://robot-access.damo-academy.com"
```

### MCP Environment Variables
Required:
- `MCP_SERVER_NAME`
- MCP_HOST
- MCP_PORT


Optional:
- MCP_PATH (default: /mcp)
- MCP_TRANSPORT (default: streamable-http)


Example:
```bash
export MCP_SERVER_NAME="rcp_server"
export MCP_HOST="0.0.0.0"
export MCP_PORT="8000"
# optional
export MCP_PATH="/mcp"
export MCP_TRANSPORT="streamable-http"
```

Note: If you configure via environment variables, you can omit `rynnbot_config.yaml` / `mcp_config.yaml`. When environment variables are set, they take priority.


## Quick Start

Enter this directory and run the configuration wizard:

```bash
cd RynnRCP/robots/so101
python configure_so101.py
```

After selecting a language (Chinese/English), the wizard provides the following menu:
1. Device settings: fill in RynnBot device credentials and save to `config/rynnbot_config.yaml`
2. Camera settings: detect cameras by plug/unplug and write to `config/so101_config.yaml`
3. Follower arm serial settings: detect follower arm serial port, update permissions, and write to `rcp_motion/robots/so101/configs/so101.yaml` → `robot.port`
4. Follower arm calibration: run the follower arm calibration procedure and write output to `~/.cache/huggingface/lerobot/calibration/robots/so101_follower`
5. Configure all: execute steps 1 → 2 → 3 → 4 → 6 → 7 in order
6. Leader arm serial settings: detect leader arm serial port, update permissions, and write to `rcp_motion/robots/so101/configs/so101.yaml` → `teleoperate.port`
7. Leader arm calibration: run the leader arm calibration procedure and write output to `~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader`

> Calibration guide reference: [SO101 Calibration Guide](../../docs/so101_calibrate.md)


## Launch Edge-side Server

After configuration, start the server in this directory:

```bash
python run_rcp_so101.py
```

This script reads:
- ./config/so101_config.yaml
- ./config/rynnbot_config.yaml
- ./config/mcp_config.yaml

Then it starts the edge-side server node and connects to RynnBot / the cloud services, while also starting the MCP service (McpPlugin).


## Teleoperation (Leader / Follower)

The SO101 supports bilateral teleoperation between a **leader arm** (operator) and a **follower arm** (robot) over LAN using the `TeleopPlugin`.

### How connection works

Both sides continuously send heartbeat packets to each other at 1 Hz.  
Data exchange only begins **after both sides are running** and heartbeats are detected within 5 seconds — if only one side is running, no business data is transmitted.

### Config: `config/teleop_config.yaml`

```yaml
teleop:
  leader_host: 127.0.0.1    # IP of the machine running the leader arm
  follower_host: 127.0.0.1  # IP of the machine running the follower arm

  leader_port: 9101
  follower_port: 9102

  control_hz: 30            # joint state send rate (Hz)
  image_hz: 10              # camera image send rate (Hz)
```

> For two separate machines, replace `127.0.0.1` with the actual LAN IPs.

### Launch leader arm (with Web UI)

```bash
cd RynnRCP/robots/so101
python run_teleop_leader.py
```

This reads `config/teleop_config.yaml` and `config/so101_leader_config.yaml`, starts the leader arm, and opens a web control panel at:

```
http://127.0.0.1:5000
```

The Web UI lets you:
- Start / stop teleop streaming
- Start / stop data recording
- Export or discard recorded episodes

### Launch follower arm

```bash
cd RynnRCP/robots/so101
python run_teleop_follower.py
```

This reads `config/teleop_config.yaml` and `config/so101_config.yaml`, starts the follower arm, and begins receiving joint commands from the leader once the connection is established.

### Typical workflow

1. Start the follower arm first (or simultaneously with the leader).
2. Start the leader arm — the Web UI opens automatically.
3. Wait ~5 seconds for the connection indicator to show **connected**.
4. Click **Start Teleop** in the Web UI to begin streaming leader joint states to the follower.
5. Click **Start Recording** to collect an episode; click **Stop Recording** when done.
6. Click **Export** to package all recorded episodes into a ZIP file.
