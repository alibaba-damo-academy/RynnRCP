# Minimal Example: SO100 to RynnBot (RynnRCP)
This directory provides a minimal runnable project to connect the **SO100** robotic arm to **RynnRCP**, including:

- Config files:
  - `config/so100_config.yaml`: **RcpCore** configuration (inputs/outputs for Action/Sensor/DeviceMonitor)
  - `config/rynnbot_config.yaml`: **RynnBot** (cloud platform) device credentials (triplet/URL)
  - `config/mcp_config.yaml`: **MCP server** configuration (exposes RCP tools over HTTP)
- Launch script:
  - `run_rcp_so100.py`: starts **RcpCore + RynnBot (plugin) + McpPlugin**
    - **RynnBot**: connects to the cloud platform (MQTT/WS)
    - **McpPlugin**: starts an MCP HTTP service and exposes RCP tools
- Configuration wizard:
  - `configure_so100.py`: interactive setup (Chinese/English), which can configure cameras, serial port, calibration, and RynnBot device triplet

## Directory Structure

```text
robots/so100/
├── config/
│   ├── rynnbot_config.yaml     # RynnBot device triplet / URL
│   ├── so100_config.yaml       # RcpCore config: Action/Sensor/DeviceMonitor
│   └── mcp_config.yaml         # MCP Server config
├── configure_so100.py          # Interactive configuration wizard (CN/EN)
├── run_rcp_so100.py            # Launch: RcpCore + RynnBot + McpPlugin
├── README.md                   # Documentation (English)
└── README_cn.md                # Documentation (Chinese)
```


## Prerequisites

1. Project is installed (Python environment ready, dependencies installed, and venv activated)
2. SO100 controller is connected (serial port)
3. Cameras are connected (USB cameras; two cameras recommended: front / wrist)
4. RynnBot device credentials (triplet):
   - product_key
   - device_name
   - device_secret
   - http_url (platform access URL; keep the default value)
 - If you enable MCP, make sure the port is reachable (example: 8000)

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
cd RynnRCP/robots/so100
python configure_so100.py
```
After selecting a language (Chinese/English), the wizard provides the following menu:
1. Device settings: fill in RynnBot device credentials and save to config/rynnbot_config.yaml
2. Camera settings: detect cameras by plug/unplug and write to config/so100_config.yaml
3. Robot serial settings: detect serial devices, update serial permissions, and write to the rcp_motion config file (RynnRCP/rcp_motion/robots/so100/configs/config.yaml)
4. Arm calibration: run the calibration procedure and write calibration output (RynnRCP/rcp_motion/robots/so100/.cache/calibration/so100/main_follower.json)
5. Configure all: execute steps 1 → 2 → 3 → 4 in order

> Calibration pose reference: [SO100 Calibration Pose](../../docs/images/so100_calibration_diagram.png)

## Launch Edge-side Server

After configuration, start the server in this directory:

```bash
python run_rcp_so100.py
```
This script reads:
- ./config/so100_config.yaml
- ./config/rynnbot_config.yaml
- ./config/mcp_config.yaml

Then it starts the edge-side server node and connects to RynnBot / the cloud services, while also starting the MCP service (McpPlugin).
