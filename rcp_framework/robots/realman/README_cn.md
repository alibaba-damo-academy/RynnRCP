# Realman Server

## 概述
本文件夹内包含了 `realman_servers` 和 `motion_node` 两个核心组件，`realman_servers` 负责与云端及设备进行通信和管理， `motion_node` 则处理和机器人本体的数据交互。

## 目录结构
```bash
realman/
├── config/
│   ├── device_config.yaml    # 设备认证信息
│   └── cameras.yaml          # 摄像头配置
├── motion_node/
│   ├── action_executor.py    # 动作执行器
│   ├── realman_controller.py # realman 控制器
│   └── realman.py            # realman 节点
└── realman_servers/
    ├── realman_servers.cpp   # 服务节点程序
    └── CMakeLists.txt        # 构建配置文件
```

## 配置文件说明
### device_config.yaml
该文件包含设备认证信息和通信端点，确保了设备能够安全有效地与其他系统进行交互。
```yaml
http_url: https://robot-access.damo-academy.com       # 设备访问的基础 URL，通常用于获取授权token
endpoint_mqtt: /connect/mqtt                          # MQTT 通信端点，用于授权
endpoint_websocket: /connect/webSocket                # WebSocket 通信端点，用于授权
product_key: put_product_key_here                     # 设备的产品密钥
device_name: put_device_name_here                     # 设备名称，用于标识特定物理设备
device_secret: put_device_secret_here                 # 设备密钥，用于身份验证和安全通信
```

### cameras.yaml
该文件包含摄像头相关的配置信息。
```yaml
cameras:
- name: observation.images.head_camera                 # 固定视角摄像头的名称
  ros2_topic: /camera_head/color/image_raw/compressed  # 摄像头 ros2 话题
  brand: Brand Name                                    # 摄像头的品牌名称
  format: YUYV                                         # 视频格式，例如 YUYV
  width: 640                                           # 捕获图像的宽度
  height: 480                                          # 捕获图像的高度
  framerate: 30                                        # 每秒捕获的帧数
- name: observation.images.left_hand_camera            # 腕部视角摄像头的名称
  ros2_topic: /camera_left/color/image_raw/compressed  # 摄像头 ros2 话题
  brand: Brand Name                                    # 摄像头的品牌名称
  format: YUYV                                         # 视频格式，例如 YUYV
  width: 640                                           # 捕获图像的宽度
  height: 480                                          # 捕获图像的高度
  framerate: 30                                        # 每秒捕获的帧数
- name: observation.images.right_hand_camera           # 腕部视角摄像头的名称
  ros2_topic: /camera_right/color/image_raw/compressed # 摄像头的设备路径
  brand: Brand Name                                    # 摄像头的品牌名称
  format: YUYV                                         # 视频格式，例如 YUYV
  width: 640                                           # 捕获图像的宽度
  height: 480                                          # 捕获图像的高度
  framerate: 30                                        # 每秒捕获的帧数
```

## 服务节点
C++ 实现的服务节点，使用RobotServer与机器人通信。
服务端代码位于 `realman_servers` 目录，主要功能包括：
- 日志设置：通过配置文件初始化日志系统
- 配置加载：从 YAML 文件加载设备配置
- 通信初始化：设置 MQTT 和 WebSocket 通信实例
- 服务模块启动：启动动作、传感器和设备监控服务模块
- 数据交互：接收云端指令，和传感器与机器臂进行本地通信

### 构建服务节点
确保已经执行过一键安装脚本 `deploy_linux.sh` 或者 `deploy_mac.sh` ，该脚本会编译本目录文件并生成可执行程序 `realman_servers` 。

### 修改配置文件
请复制乐云平台的设备配置信息至 `config/device_config.yaml` 文件中。

### 运行服务节点
通过指定设备配置文件和日志配置文件来运行服务：
```bash
./realman_servers <path_to_device_config> <path_to_glog_config>
```

### 运行控制节点
用于处理机器人本体数据并通过 LCM 协议进行传输。启动方式如下：
```bash
python3 -m rcp_framework.robots.realman.motion_node.realman
```

## 一键运行脚本
此脚本简化了启动服务器的过程。请运行以下命令：
```bash
cd RynnRCP
bash example/realman.sh DEBUG
```

## 注意事项
- 确保使用有效的认证信息替换配置文件中的占位符。
- 服务节点需要访问网络以连接到指定的云端服务。
- 修改 `motion_node/realman.py` 文件中的 `LEFT_IP` \ `LEFT_PORT` \ `RIGHT_IP` \ `RIGHT_PORT` 为机器臂具体的 IP 地址。

