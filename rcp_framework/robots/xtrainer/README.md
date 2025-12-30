# Xtrainer Server

## Overview
This folder contains two core components: `xtrainer_servers` and `camera_node`. The `xtrainer_servers` manages communication and management with the cloud and devices, while the `camera_node` handles data interaction with the robot.

## Directory Structure
```bash
lerobot/
├── config/
│   ├── device_config.yaml # Device authentication information
│   └── cameras.yaml       # Camera configuration
├── motion_node/
│   └── xtrainer.py        # Robot control node
└── xtrainer_servers/
    ├── xtrainer_servers.cpp  # Server node program
    └── CMakeLists.txt     # CMake build configuration file
```

## Configuration File
### device_config.yaml
This file contains device authentication information and communication endpoints, ensuring that the device can interact securely and effectively with other systems.
```yaml
http_url: https://robot-access.damo-academy.com      # URL for authorization token
endpoint_mqtt: /connect/mqtt                         # MQTT communication endpoint for authorization
endpoint_websocket: /connect/webSocket               # WebSocket communication endpoint for authorization
product_key: put_product_key_here                    # Device's product key
device_name: put_device_name_here                    # Device name used to identify a specific physical device
device_secret: put_device_secret_here                # Device secret for authentication
```

### cameras.yaml
This file contains configuration information related to the cameras.
```yaml
cameras:
- name: observation.images.cam_main       # Name of the head camera
  device: 0                               # Device file name of the camera
  brand: Brand Name                       # Brand name of the camera
  format: YUYV                            # Video format, e.g., YUYV
  width: 640                              # Width of the captured image
  height: 480                             # Height of the captured image
  framerate: 30                           # Frames captured per second
- name: observation.images.cam_left_arm   # Name of the left arm camera
  device: 1                               # Device file name of the camera
  brand: Brand Name                       # Brand name of the camera
  format: YUYV                            # Video format, e.g., YUYV
  width: 640                              # Width of the captured image
  height: 480                             # Height of the captured image
  framerate: 30                           # Frames captured per second
- name: observation.images.cam_right_arm  # Name of the right arm camera
  device: 2                               # Device file name of the camera
  brand: Brand Name                       # Brand name of the camera
  format: YUYV                            # Video format, e.g., YUYV
  width: 640                              # Width of the captured image
  height: 480                             # Height of the captured image
  framerate: 30                           # Frames captured per second
```


## Server Node
This server node program is implemented in C++ and communicates with the robot using the RobotServer library.
The server code is located in the `realman_servers` directory, and its main functions include:
1. Log setup: Initialize the logging system according to configuration file
2. Configuration loading: Load device configurations from a YAML file
3. Communication initialization: Set up MQTT and WebSocket communication instances
4. Server startup: Start action, sensor, and device monitoring servers
5. Data interaction: Receive commands from the cloud and communicate locally with sensors and robot arms

### Building the Server Node
Make sure to execute the installation script `deploy_linux.sh` or `deploy_mac.sh`, which compiles the files in this directory and generates the executable `xtrainer_servers`.

### Running the Server Node
Run the server by passing the device configuration file and logging configuration file:
```bash
./xtrainer_servers <path_to_device_config> <path_to_glog_config>
```

### Camera Node
Python script used to process robot data and transmit it via the LCM protocol.
```bash
python3 -m rcp_framework.robots.xtrainer.motion_node.xtrainer
```

## Launch script
This script simplifies the process of starting the server. Run the following commands:
```bash
cd RynnRCP
bash example/xtrainer.sh DEBUG
```

## Notes
- Please ensure to replace the placeholders in the configuration files with valid authentication information.
- The server needs access to the internet to connect to the specified cloud services.
- The control node depends on dobot_xtrainer SDK. Before execution, ensure that the dobot_xtrainer SDK path is included in the environment variable (export PYTHONPATH=/path/to/dobot_xtrainer:$PYTHONPATH).
