# Robot Server SDK
Robot Server SDK is a modular robot server development framework that provides various server modules, such as action execution, device monitoring, and sensor data processing. It supports asynchronous communication and efficient event-driven programming. The design features emphasize low coupling and extensibility, allowing developers to customize and extend based on their needs.

## Directory Structure

```bash
robot_server/
├── CMakeLists.txt          # CMake build configuration file
├── README.md               # SDK documentation
├── README_cn.md            # Chinese SDK documentation
├── action_server/          # Action server module
├── common/                 # Common base components
├── device_monitor_server/  # Device monitoring server module
├── sensor_server/          # Sensor server module
```

## Module Introduction
### common Module
- **Core Files**:
  - **version.hpp**: Version information
  - **robot_server.hpp**: Unified header file for SDK
  - **terminal_device_server/**: Implementation related to terminal device server module
  - **server/**: Implementation related to common server module

### server Module
- **Core Files**:
  - **server.hpp**: Declaration of basic server interface
  - **server.cpp**: Implementation of basic server class
- **Function Description**:
  - Processes messages from cloud
  
### terminal_device_server Module
- **Core Files**:
  - **terminal_device_server.hpp**: Declaration of terminal device server module interface
  - **terminal_device_server.cpp**: Implementation of terminal device server module
- **Function Description**:
  - Inherits from common server module
  - Implements occupation and release of servers
  - Implements classification and distribution of messages
  
### action_server Module
- **Core Files**:
  - **action_server.hpp**: Declaration of action server
  - **action_server.cpp**: Implementation of action server
- **Function Description**:
  - Processes specific action message content
  - Implements message interaction with robot_motion
  - Uploads robot-related status

### sensor_server Module
- **Core Files**:
  - **sensor_server.hpp**: Declaration of sensor server
  - **sensor_server.cpp**: Implementation of sensor server
- **Function Description**:
  - Processes specific sensor message content
  - Implements message interaction with sensor nodes
  - Uploads sensors related configurations

### device_monitor_server Module
- **Core Files**:
  - **device_monitor_server.hpp**: Declaration of device monitoring server
  - **device_monitor_server.cpp**: Implementation of device monitoring server
- **Function Description**:
  - Monitors device status periodically
  - Periodically reports device status

## Design Features
1. **Modular Design**
   The framework features distinct module separation, promoting low coupling between modules to facilitate independent development and maintenance.
2. **Asynchronous Communication Mechanism**
  Utilizing an efficient asynchronous communication model, the framework supports event-driven programming approaches.
3. **Scalability**
   Offers well-defined extension interfaces, enabling the integration of new device types and functional modules with ease.

