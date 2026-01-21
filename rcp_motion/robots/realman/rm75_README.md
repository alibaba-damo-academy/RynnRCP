# RealMan RM75 Robot Control

[中文版](rm75_README_cn.md)

## Overview

This module provides the RynnRCP control interface for the RealMan RM75 7-DOF robotic arm. It supports both MuJoCo simulation and real hardware control via ROS2.

## Architecture

```
RynnRCP Python (RM75Interface)
    ↓ ROS2 Topics
rm_driver (C++ ROS2 Node)
    ↓ SDK calls
libapi_cpp.so
    ↓ TCP/UDP
RM75 Robot (192.168.1.18:8080)
```

## Prerequisites

### For Simulation Only
- Python 3.10+
- MuJoCo
- No additional hardware required

### For Real Hardware
- ROS2 Humble
- [ros2_rm_robot](https://github.com/RealManRobot/ros2_rm_robot) packages:
  - `rm_ros_interfaces` (message types)
  - `rm_driver` (hardware driver)

## Quick Start

### 1. Install RynnRCP

```bash
cd RynnRCP
bash setup_rcp.sh
# Select option 2: RealMan RM75
```

### 2. Build ROS2 Driver (for real hardware)

```bash
# Clone ros2_rm_robot to your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/RealManRobot/ros2_rm_robot.git

# Build (interfaces first, then driver)
cd ~/ros2_ws
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build --packages-select rm_driver
source install/setup.bash
```

### 3. Configure Robot IP

Edit `rcp_motion/robots/realman/configs/rm75_config.yaml` or the rm_driver config:
- Default robot IP: `192.168.1.18`
- Default port: `8080`

## Running

### Simulation Mode

```bash
cd RynnRCP
source venv/bin/activate

# Run with simulated trajectory
python -m rcp_motion.robots.realman.scripts.unified_realman_controller \
    --mode sim --signal_source sim

# Run with policy commands
python -m rcp_motion.robots.realman.scripts.unified_realman_controller \
    --mode sim --signal_source policy
```

### Real Hardware Mode

**Terminal 1: Start rm_driver**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rm_driver rm_75_driver.launch.py
```

**Terminal 2: Run RynnRCP**
```bash
cd RynnRCP
source venv/bin/activate

# Test with simulated trajectory
python -m rcp_motion.robots.realman.scripts.unified_realman_controller \
    --mode real --signal_source sim

# Run with policy commands
python -m rcp_motion.robots.realman.scripts.unified_realman_controller \
    --mode real --signal_source policy
```

## Command Line Options

```
python -m rcp_motion.robots.realman.scripts.unified_realman_controller [options]

Options:
  --mode {sim,real}           Control mode (required)
  --signal_source {sim,policy} Signal source (default: sim)
  --ctrlfreq {30-250}         Control frequency in Hz (default: 100)
  --config PATH               Path to config file
```

## Configuration

### rm75_config.yaml

```yaml
robot:
  inference_rate: 30.0          # Policy inference rate (Hz)
  timeout_seconds: 30.0         # Connection timeout
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]

signal:
  signal_home_position: [0.0, 0.0, 0, 1.57, 0, 0.0, -0.0]
  signal_amplitude: 0.3
  signal_frequency: 0.1

plot:
  enable_joint_plotting: false  # Set true for data recording
```

## ROS2 Topics

### Published by rm_driver
| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Joint positions, velocities |
| `/rm_driver/udp_six_force` | Sixforce | 6-axis force/torque data |
| `/rm_driver/udp_arm_position` | Armstate | Cartesian pose |

### Subscribed by rm_driver
| Topic | Type | Description |
|-------|------|-------------|
| `/joint_command` | sensor_msgs/JointState | Joint position commands |
| `/rm_driver/movej_cmd` | Movej | Joint space motion |
| `/rm_driver/movel_cmd` | Movel | Linear Cartesian motion |

## Data Recording

1. Enable joint plotter in config:
   ```yaml
   plot:
     enable_joint_plotting: true
   ```

2. Logs are saved to: `~/RynnRcplog/robotMotion/`

## Troubleshooting

### Connection Failed
- Check robot IP and ensure it's reachable: `ping 192.168.1.18`
- Verify rm_driver is running: `ros2 topic list | grep rm_driver`
- Check ROS2 network: `ros2 topic echo /joint_states`

### Robot Not Moving
- Ensure robot is powered on and not in error state
- Check emergency stop is released
- Verify joint limits in config match robot

## File Structure

```
rcp_motion/robots/realman/
├── configs/
│   ├── rm75_config.yaml      # RM75 configuration
│   └── eco65_config.yaml     # ECO65 configuration
├── controller/
│   └── rm75_motion.py        # Motion controller
├── interface/
│   └── rm75_interface.py     # ROS2 interface
├── scripts/
│   └── unified_realman_controller.py  # Main entry point
├── rm75_README.md            # This file
└── rm75_README_cn.md         # Chinese version
```

## See Also

- [RealMan ROS2 Driver](https://github.com/RealManRobot/ros2_rm_robot)
- [RynnRCP Main README](../../../README.md)
