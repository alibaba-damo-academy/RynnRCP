# Realman Server

## Overview
This folder contains two core components: `realman_servers` and `motion_node`. The `realman_servers` manages communication and management with the cloud and devices, while the `motion_node` handles data interaction with the robot.

## Directory Structure
```bash
lerobot/
├── config/
│   ├── device_config.yaml  # Device authentication information
│   └── cameras.yaml        # Camera configuration
├── motion_node/
│   ├── action_executor.py    # Action executor
│   ├── realman_controller.py # Realman controller
│   └── realman.py            # Realman node
└── realman_servers/
    ├── realman_servers.cpp  # Server node program
    └── CMakeLists.txt       # CMake build configuration file
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

Please copy the device configuration information from the RynnBot platform to the `config/device_config.yaml` file.

### cameras.yaml
This file contains configuration information related to the cameras.
```yaml
cameras:
- name: observation.images.head_camera                 # Head camera name
  ros2_topic: /camera_head/color/image_raw/compressed  # Camera ros2 topic
  brand: Brand Name                                    # Brand name of the camera
  format: YUYV                                         # Video format, e.g., YUYV
  width: 640                                           # Width of the captured image
  height: 480                                          # Height of the captured image
  framerate: 30                                        # Frames captured per second
- name: observation.images.left_hand_camera            # Left hand camera name
  ros2_topic: /camera_left/color/image_raw/compressed  # Camera ros2 topic
  brand: Brand Name                                    # Brand name of the camera
  format: YUYV                                         # Video format, e.g., YUYV
  width: 640                                           # Width of the captured image
  height: 480                                          # Height of the captured image
  framerate: 30                                        # Frames captured per second
- name: observation.images.right_hand_camera           # Right hand camera name
  ros2_topic: /camera_right/color/image_raw/compressed # Camera ros2 topic
  brand: Brand Name                                    # Brand name of the camera
  format: YUYV                                         # Video format, e.g., YUYV
  width: 640                                           # Width of the captured image
  height: 480                                          # Height of the captured image
  framerate: 30                                        # Frames captured per second
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
Make sure to execute the installation script `deploy_linux.sh` or `deploy_mac.sh`, which compiles the files in this directory and generates the executable `realman_servers`.

### Running the Server Node
Run the server by passing the device configuration file and logging configuration file:
```bash
./realman_servers <path_to_device_config> <path_to_glog_config>
```

### Camera Node
Python script used to process robot data and transmit it via the LCM protocol.
```bash
python3 -m rcp_framework.robots.realman.motion_node.realman
```

## Launch script
This script simplifies the process of starting the server. Run the following commands:
```bash
cd RynnRCP
bash example/realman.sh DEBUG
```

## Notes
- Please ensure to replace the placeholders in the configuration files with valid authentication information.
- The server needs access to the internet to connect to the specified cloud services.
- Update `LEFT_IP`, `LEFT_PORT`, `RIGHT_IP`, and `RIGHT_PORT` in motion_node/realman.py to match the robot arm’s actual IP addresses and ports.

