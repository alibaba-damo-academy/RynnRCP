# Minimal Example: lekiwi to RynnBot (RynnRCP)
This directory provides a minimal runnable project to connect the **lekiwi** robotic arm to **RynnRCP**, including:

- Config files:
  - `config/lekiwi_config.yaml`: **RcpCore** configuration (inputs/outputs for Action/Sensor/DeviceMonitor)
  - `config/rynnbot_config.yaml`: RynnBot (cloud platform) device credentials (triplet/URL)
  - `config/mcp_config.yaml`: **MCP server** configuration (exposes RCP tools over HTTP)
- Launch script:
  - `run_rcp_lekiwi.py`: starts **RcpCore + RynnBot (plugin) + McpPlugin**
    - **RynnBot**: connects to the cloud platform (MQTT/WS)
    - **McpPlugin**: starts an MCP HTTP service and exposes RCP tools
- Configuration wizard:
  - `configure_lekiwi.py`: interactive setup (Chinese/English), which can configure cameras, serial port, calibration, and RynnBot device triplet


## Directory Structure

```text
robots/lekiwi/
├── config/
│   ├── rynnbot_config.yaml     # RynnBot device triplet / URL
│   └── lekiwi_config.yaml       # RcpCore config: Action/Sensor/DeviceMonitor
│   └── mcp_config.yaml         # MCP Server configDeviceMonitor
├── configure_lekiwi.py          # Interactive configuration wizard (CN/EN)
├── run_rcp_lekiwi.py            # Launch: RcpCore + RynnBot(plugin)
├── README.md                   # Documentation (English)
└── README_cn.md                # Documentation (Chinese)
```


## Prerequisites

1. Project is installed (Python environment ready, dependencies installed, and venv activated)
2. LeKiwi controller is connected (serial port)
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
cd RynnRCP/robots/lekiwi
python configure_lekiwi.py
```

After selecting a language (Chinese/English), the wizard provides the following menu:
1. Device settings: fill in RynnBot device credentials and save to config/rynnbot_config.yaml
2. Camera settings: detect cameras by plug/unplug and write to config/lekiwi_config.yaml
3. Robot serial settings: detect serial devices, update serial permissions, and write to the rcp_motion config file
4. Arm calibration: run the calibration procedure and write calibration output
5. Configure all: execute steps 1 → 2 → 3 → 4 in order

> Calibration guide reference: [lekiwi Calibration Guide](../../docs/lekiwi_calibrate.md)


## Launch Edge-side Server

After configuration, start the server in this directory:

```bash
python run_rcp_lekiwi.py
```

This script reads:
- ./config/lekiwi_config.yaml
- ./config/rynnbot_config.yaml
- ./config/mcp_config.yaml

Then it starts the edge-side server node and connects to RynnBot / the cloud services, while also starting the MCP service (McpPlugin).
