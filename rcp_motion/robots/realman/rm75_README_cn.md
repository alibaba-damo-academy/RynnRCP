# RealMan RM75 机械臂控制

[English](rm75_README.md)

## 概述

本模块为 RealMan RM75 七自由度机械臂提供 RynnRCP 控制接口，支持 MuJoCo 仿真和通过 ROS2 控制真实硬件。

## 架构

```
RynnRCP Python (RM75Interface)
    ↓ ROS2 话题
rm_driver (C++ ROS2 节点)
    ↓ SDK 调用
libapi_cpp.so
    ↓ TCP/UDP
RM75 机器人 (192.168.1.18:8080)
```

## 环境要求

### 仅仿真
- Python 3.10+
- MuJoCo
- 无需额外硬件

### 真实硬件
- ROS2 Humble
- [ros2_rm_robot](https://github.com/RealManRobot/ros2_rm_robot) 功能包:
  - `rm_ros_interfaces` (消息类型)
  - `rm_driver` (硬件驱动)

## 快速开始

### 1. 安装 RynnRCP

```bash
cd RynnRCP
bash setup_rcp.sh
# 选择选项 2: RealMan RM75
```

### 2. 编译 ROS2 驱动（真实硬件需要）

```bash
# 将 ros2_rm_robot 克隆到 ROS2 工作空间
cd ~/ros2_ws/src
git clone https://github.com/RealManRobot/ros2_rm_robot.git

# 编译（先编译接口，再编译驱动）
cd ~/ros2_ws
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build --packages-select rm_driver
source install/setup.bash
```

### 3. 配置机器人 IP

编辑 `rcp_motion/robots/realman/configs/rm75_config.yaml` 或 rm_driver 配置:
- 默认机器人 IP: `192.168.1.18`
- 默认端口: `8080`

## 运行

### 仿真模式

```bash
cd RynnRCP
source venv/bin/activate

# 使用仿真轨迹运行
python -m rcp_motion.robots.realman.scripts.unified_realman_controller \
    --mode sim --signal_source sim

# 使用策略命令运行
python -m rcp_motion.robots.realman.scripts.unified_realman_controller \
    --mode sim --signal_source policy
```

### 真实硬件模式

**终端 1: 启动 rm_driver**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rm_driver rm_75_driver.launch.py
```

**终端 2: 运行 RynnRCP**
```bash
cd RynnRCP
source venv/bin/activate

# 使用仿真轨迹测试
python -m rcp_motion.robots.realman.scripts.unified_realman_controller \
    --mode real --signal_source sim

# 使用策略命令运行
python -m rcp_motion.robots.realman.scripts.unified_realman_controller \
    --mode real --signal_source policy
```

## 命令行选项

```
python -m rcp_motion.robots.realman.scripts.unified_realman_controller [选项]

选项:
  --mode {sim,real}           控制模式（必需）
  --signal_source {sim,policy} 信号来源（默认: sim）
  --ctrlfreq {30-250}         控制频率，单位 Hz（默认: 100）
  --config PATH               配置文件路径
```

## 配置文件

### rm75_config.yaml

```yaml
robot:
  inference_rate: 30.0          # 策略推理频率 (Hz)
  timeout_seconds: 30.0         # 连接超时时间
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]

signal:
  signal_home_position: [0.0, 0.0, 0, 1.57, 0, 0.0, -0.0]
  signal_amplitude: 0.3
  signal_frequency: 0.1

plot:
  enable_joint_plotting: false  # 设为 true 以启用数据记录
```

## ROS2 话题

### rm_driver 发布的话题
| 话题 | 类型 | 描述 |
|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 关节位置、速度 |
| `/rm_driver/udp_six_force` | Sixforce | 六维力/力矩数据 |
| `/rm_driver/udp_arm_position` | Armstate | 笛卡尔位姿 |

### rm_driver 订阅的话题
| 话题 | 类型 | 描述 |
|------|------|------|
| `/joint_command` | sensor_msgs/JointState | 关节位置指令 |
| `/rm_driver/movej_cmd` | Movej | 关节空间运动 |
| `/rm_driver/movel_cmd` | Movel | 直线笛卡尔运动 |

## 数据记录

1. 在配置文件中启用关节绘图器:
   ```yaml
   plot:
     enable_joint_plotting: true
   ```

2. 日志保存路径: `~/RynnRcplog/robotMotion/`

## 故障排除

### 连接失败
- 检查机器人 IP 是否可达: `ping 192.168.1.18`
- 确认 rm_driver 正在运行: `ros2 topic list | grep rm_driver`
- 检查 ROS2 网络: `ros2 topic echo /joint_states`

### 机器人不动
- 确保机器人已上电且无错误状态
- 检查急停是否已释放
- 确认配置中的关节限位与机器人一致

## 文件结构

```
rcp_motion/robots/realman/
├── configs/
│   ├── rm75_config.yaml      # RM75 配置
│   └── eco65_config.yaml     # ECO65 配置
├── controller/
│   └── rm75_motion.py        # 运动控制器
├── interface/
│   └── rm75_interface.py     # ROS2 接口
├── scripts/
│   └── unified_realman_controller.py  # 主入口
├── rm75_README.md            # 英文版
└── rm75_README_cn.md         # 本文件
```

## 相关链接

- [RealMan ROS2 驱动](https://github.com/RealManRobot/ros2_rm_robot)
- [RynnRCP 主 README](../../../README_cn.md)
