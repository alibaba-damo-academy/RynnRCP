# Overview
**RynnRCP** is a complete set of robot service agreements and frameworks, mainly consisting of two modules: **RCP framework** and **RobotMotion**.

- **RCP framework** is the overall implementation of the architecture. The current main functions implemented include: providing an abstraction of the capabilities of the robot body and related sensors, offering various functionalities externally and interacting through standard protocols, different transport layers, and model services.
- **RobotMotion** serves as a bridge between cloud inference and robot body control, converting discrete low-frequency inference commands into high-frequency continuous control signals in real time to drive the robot to complete motion tasks. Additionally, it is equipped with a toolkit necessary for motion planning and control, including MuJoCo physics simulation, real machine debugging and playback, data collection, and motion trajectory visualization, facilitating the integration of embodied intelligence into the physical world.

Users can gain a clear and comprehensive understanding of the complete workflow from data acquisition from sensors to model inference and robot action execution through RynnRCP. The well-defined layered structure and standard communication protocols also make it relatively easy for users to adapt the SDK to their own usage scenarios.

In the future, RCP framework will integrate more robust and adaptable link layers for various scenarios to facilitate high-speed and stable interactions with cloud model inference. Additionally, RobotServer will be compatible with more hardware platforms. Furthermore, model inference services may also be integrated into the edge and open-sourced, making the entire RynnRCP even more comprehensive.

RobotMotion will provide robot control interfaces for different forms of inference models, adopting a unified foundational module and development paradigm for robot regulation and control. Additionally, RobotMotion will offer simulation tools based on Mujoco, serving as a secure visualization platform for validating model execution, enabling developers to confirm their robot configurations and quickly build embodied intelligent robot control systems.

# Directory Structure
``` bash
.
├── common                 # Common message definitions and components
│   ├── config             # Configuration files
│   ├── lcm                # LCM message type declarations
│   ├── proto              # Protobuf message type declarations
│   └── third_party        # Third-party dependencies source code
├── docker                 # Docker environment
├── examples               # Deployment preparations and running instructions for different robots
├── robot_motion           # Implementations related to RobotMotion
│   ├── models             # Robotic arm models
│   └── robots             # Supported robots
│       └── lerobot        # Control implementation for lerobot, including simulation tools
├── rcp_framework          # Implementations related to rcp_framework
│   ├── cpp                # Contains the C++ implementations of rcp_framework
│   │   ├── common         # Contains communication capabilities and other common components
│   │   └── robot_server   # Contains implementations related to robot_server
│   └── robots             # Supported robots
│       └── so100          # Minimal project for so100, including camera nodes
└── scripts                # Compilation scripts
```

# Installation and Usage
> Note: For detailed usage instructions, please refer to: [Rynnbot Embodied Intelligence Development Platform User Documentation](https://developer.damo-academy.com/allSpark/document/guide/basic)  

## Download the Source Code
```bash
git clone https://github.com/alibaba-damo-academy/RynnRCP.git
```

## Dependency Installation and Compilation
### Install Python Environment
```bash
cd RynnRCP

# Using venv
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Or using Conda
conda create --name venv python=3.10
conda activate venv
pip install -r requirements.txt

# Install RobotMotion module
cd robot_motion/robots/lerobot
pip install -e .
```
> This will create and activate the required environment and install all dependencies. Python 3.10 is the minimum required version.

### Install and Compile C++ Modules

```bash
cd RynnRCP
bash scripts/deploy.sh
```
> Run the one-click installation script `deploy.sh`. This script will automatically check and install necessary dependencies. It will install CMake, build tools, Python development libraries, GLib, Glog, Protobuf, OpenSSL 3, and more. It will also compile required third-party libraries (such as LCM, libwebsockets, Paho MQTT, libyaml-cpp, etc.) and compile the server for the lerobot sample program.

During the execution of the script, the following operations will also be performed:
- Modify the UDP buffer configuration to accommodate the communication requirements for image transmission via LCM.

Notes:
- Ensure your system is running Ubuntu and is connected to the internet.
- Some steps in the script may prompt for the sudo password to gain administrative privileges.
- If any errors arise, troubleshoot and resolve them based on the output information.
- Ensure that the Python virtual environment is activated before running the script.

## Modify Configuration Files
To simplify the configuration process, we provide an interactive configuration script that can automatically detect devices and guide you through the configuration. You can run the following command to start the configuration tool:

```bash
cd RynnRCP

# Using Conda environment
conda activate venv

# Or using venv environment
source venv/bin/activate

# Run the configuration script
python example/configure_so100.py
```

The script will guide you through the following configurations:
1. Device authentication configuration (MQTT/WebSocket)
2. Camera device configuration (automatic device detection)
3. Robot parameter configuration (automatic serial port detection)
4. Robot arm calibration

For more advanced or specific configuration needs, you can also manually edit the following configuration files:

### RobotServer
- Copy the three parameters (`product_key`, `device_name`, `device_secret`) from the robot device activation page and configure them in the `rcp_framework/robots/so100/config/device_config.yaml` file (the parameters http_url/endpoint_mqtt/endpoint_websocket do not need modification).

### Camera Node
- Insert a front-angle camera, corresponding to the camera name `observation.images.front`, and execute `ls /dev/video*` to check the device number, for example, `/dev/video0`.
- Insert a wrist-angle camera, corresponding to the camera name `observation.images.wrist`, and execute `ls /dev/video*` to check the device number, for example, `/dev/video2`.
- Fill in the device numbers in the camera configuration file `rcp_framework/robots/so100/config/cameras.yaml`.
- It is recommended that **at least one of the two cameras is connected directly to the USB port of the host**. Based on experience, if both cameras are used through a hub, it may lead to operational issues.

### RobotMotion

Configure the relevant parameters for the robot. The configuration process may vary for different robots. For the lerobot, the relevant configurations and the method for calibrating the robotic arm can be found in: [RobtoMotion LeRobot README](robot_motion/robots/lerobot/README.md).


## Run
```bash
cd RynnRCP

# Using Conda environment
conda activate venv

# Or using venv environment
source venv/bin/activate

# Launch nodes
bash example/so100.sh

# Or launch nodes and output debug logs to terminal
bash example/so100.sh DEBUG
```

This script will sequentially start the following components to support the `so100`'s edge functionality:
- RobotMotion: Launches the motion control program.
- RobotServer: Runs the robot server and loads device information and log configurations.
- Camera Node: Starts the camera node, processes camera configuration information, and initiates asynchronous image capture.

Notes:
- Logs are stored in the `$HOME/RynnRcplog/` folder.
- Use the Ctrl+C key combination to terminate the script execution; the system will automatically close all subprocesses.
- Ensure that all configuration file paths and dependencies are correctly set before running the script.
- Ensure that the Python virtual environment is activated before running the script.


# Module Introduction

## RobotServer
RobotServer provides a framework for building robot server modules, primarily responsible for handling communication and data exchange between robot devices and cloud. This module include Action Server (ActionServer), Sensor Server (SensorServer), and Device Monitor Server (DeviceMonitorServer), which communicate via data transmission protocols such as MQTT and WebSocket for controlling robot devices and data collection.
- Multi-protocol Communication: A unified communication middleware is provided to cater to various protocol requirements, supporting MQTT, WebSocket, LCM, etc., achieving high-frequency and stable execution of core functions such as command transmission, data collection, and state monitoring through standardized interfaces.
- Decoupled Server Module Design: Adopting a modular design, it offers basic server node templates for action execution, sensor acquisition, and device monitoring. Developers can extend and customize service nodes based on the CTerminalDeviceServer base class to implement functionalities such as device occupancy control, resource scheduling, and multi-client collaboration.
- Secure Environment and Configuration Center: Configuration files only store device authentication metadata. At runtime, requests are initiated through an HTTPS secure channel to the authentication service, generating time-sensitive access tokens to ensure the security of communication links.

## RobotMotion
Robot Motion provides a middleware that drives robot movement and task completion using different models, along with a Mujoco simulation visualization tool equipped with a physics engine, offering a safe environment for configuring and validating models for your robots.
- Model and Robot Integration: It provides regulatory control middleware that robots can execute based on the disparities in output from different models. For hierarchical inference models of the VLA type or lightweight real-time inference models (such as ACT), RobotMotion supplies middleware that can be executed at high frequency by real robots.
- Simulation Tools with Physics Engine: It offers Mujoco-based simulation tools with a physics engine, providing a secure visual environment for validating robot strategy execution and enabling rapid verification of robot configurations in the simulation environment.
- Additional Auxiliary Tools: It includes tools for data recording and visualization, simulation playback, and more.

## Camera Node

The camera node implements a basic image capture and transmission class, providing efficient and real-time image capture and processing capabilities. This module utilizes the LCM protocol for instantaneous transmission of camera data and optimizes data processing and management through a multithreaded architecture.

- Real-time Image Capture: Utilizes the OpenCV library to achieve real-time image capture for multiple cameras, supporting dynamic adjustment of resolution and frame rate based on configurations.
- Lossless Image Compression: Employs Gzip for lossless compression of raw images to reduce bandwidth consumption during network transmission.
- Asynchronous Processing: Separates the camera capture process from message response using Python's threading capabilities, ensuring system responsiveness and effective utilization of system resources.

# Todo
- [x] Release **RynnRCP** 1.0 version
- [ ] Release Technical Report
- [ ] ActionServer and SensorServer support MCP
- [ ] Complete the rest of **RynnRCP** framework
- [ ] Support for robots with more structural types