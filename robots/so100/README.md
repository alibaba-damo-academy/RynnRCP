# Minimal Example: SO100 to RynnBot (RynnRCP)
This directory provides a minimal runnable project to connect the **SO100** robotic arm to **RynnRCP**, including:

- Config files:
  - `config/so100_config.yaml`: **RcpCore** configuration (inputs/outputs for Action/Sensor/DeviceMonitor)
  - `config/rynnbot_config.yaml`: RynnBot (cloud platform) device credentials (triplet/URL)
- Launch script:
  - `run_rcp_so100.py`: starts **RcpCore + RynnBot (plugin)** to connect to RynnBot / the cloud services
- Configuration wizard:
  - `configure_so100.py`: interactive setup (Chinese/English), which can configure cameras, serial port, calibration, and RynnBot device triplet

## Directory Structure

```text
robots/so100/
├── config/
│   ├── rynnbot_config.yaml     # RynnBot device triplet / URL
│   └── so100_config.yaml       # RcpCore config: Action/Sensor/DeviceMonitor
├── configure_so100.py          # Interactive configuration wizard (CN/EN)
├── run_rcp_so100.py            # Launch: RcpCore + RynnBot(plugin)
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

Then it starts the edge-side server node and connects to RynnBot / the cloud services.
