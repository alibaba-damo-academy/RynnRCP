# 概述
**RynnRCP** 是一整套机器人服务协议和框架，包含两个主要模块：**RCP framework** 和 **RobotMotion**。

- **RCP framework** 是架构的整体实现。当前主要实现的功能包括，提供机器人本体和相关传感器的能力抽象，对外提供多种能力，并通过标准协议、不同的传输层和模型服务进行交互。
- **RobotMotion** 架起云端推理与机器人本体控制之间的桥梁，实时将离散低频的推理指令转化为高频连续的控制信号，驱动机器人完成运动任务。同时，它配套规划控制所需工具集，包括 MuJoCo 物理仿真、真机调试与回放、数据采集及运动轨迹可视化等，助力具身智能走入物理世界。

用户可以通过 RynnRCP 清晰地了解到从传感器获取数据到模型推理以及机器人动作执行的完整链路，同时，清晰的分层结构和标准的通信协议使得迁移到特定使用场景变得更加容易。

将来 **RCP framework** 将集成更多性能强大以及适配不同场景下的链路层以便和云端模型推理进行高速、稳定的交互，同时RobotServer也将兼容更多的硬件平台。之后模型推理服务也可集成到端侧并开源，使整个 **RynnRCP** 更加完整。

**RobotMotion** 将会为不同推理形式的模型提供机器人控制接口，采用统一的机器人规控基础模块及开发范式；同时 **RobotMotion** 还会提供基于Mujoco的仿真工具，可以作为模型执行验证的一个安全的可视化平台，便于开发者确认自己的机器人的相关配置，快速搭建起具身智能机器人控制系统。

# 目录结构
``` bash
.
├── common                 # 包含公共消息定义和组件
│   ├── config             # 公共配置文件
│   ├── lcm                # 基于lcm的消息类型声明
│   ├── proto              # 基于protobuf的消息类型声明
│   └── third_party        # 共享的第三方依赖库源码
├── examples               # 包含针对不同本体所需要的部署等准备工作，以及运行说明
├── robot_motion           # RobotMotion相关实现
│   ├── models             # 机械臂模型
│   └── robots             # 支持的机器人、机械臂
│       └── lerobot        # lerobot控制相关实现，包含仿真工具
├── rcp_framework          # rcp_framework相关实现
│   ├── cpp                # 包含rcp_framework的cpp实现
│   │   ├── common         # 包含通信能力和其他通用组件
│   │   └── robot_server   # 包含robot_server相关实现
│   └── robots             # 支持的机器人、机械臂
│       └── so100          # 支持so100的最小项目，包含相机节点
└── scripts                # 包含编译脚本
```

# 安装与使用
> Note: 详细使用说明，可阅读：[RynnBot 乐云具身智能开发平台使用文档](https://developer.damo-academy.com/allSpark/document/guide/basic)  

## 下载源码
```bash
git clone https://github.com/alibaba-damo-academy/RynnRCP.git
```

## 依赖安装和编译
### 安装 Python 环境
```bash
cd RynnRCP

# 使用 venv
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# 或使用 Conda
conda create --name venv python=3.10
conda activate venv
pip install -r requirements.txt

# 安装RobotMotion模块
cd robot_motion/robots/lerobot
pip install -e .
```
> 这将创建并激活所需环境，并安装所有依赖项。python 3.10 是最低要求的 Python 版本。

### 安装和编译 C++ 模块

```bash
cd RynnRCP
bash scripts/deploy.sh
```
> 运行一键安装脚本 `deploy.sh`。该脚本将自动检查和安装所需的依赖项。脚本安装 CMake、构建工具、Python 开发库、GLib、Glog、Protobuf、OpenSSL 3 等。并编译所需的第三方库（例如 LCM、libwebsockets、Paho MQTT、libyaml-cpp 等），最后编译 robot server for lerobot 样例程序。

在脚本执行的过程中，还会进行以下操作：
- 修改 UDP 缓冲区配置，以适应 LCM 传输图片的通信需求。

注意事项
- 请确保运行系统为 Ubuntu 并连接到互联网。
- 脚本中的一些步骤可能会请求输入 sudo 密码以获得管理权限。
- 如果出现错误，请根据输出信息进行排查和修复。
- 确保在运行脚本之前，已激活 Python 虚拟环境。

## 修改配置文件
为了简化配置过程，我们提供了一个交互式配置脚本，可以自动检测设备并引导您完成 so100 机械臂的配置。您可以运行以下命令启动配置工具：

```bash
cd RynnRCP

# 使用 Conda 激活 Python 虚拟环境
conda activate venv

# 或使用 Python venv 虚拟环境
source venv/bin/activate

# 运行交互式配置脚本
python3 example/configure_so100.py
```

该脚本将引导您完成以下配置：
1. 设备认证信息配置
2. 相机设备配置（自动检测相机设备）
3. 机器人参数配置（自动检测串口设备）
4. so100 机器臂标定

对于更高级或特定的配置需求，您也可以手动编辑以下配置文件：

### RobotServer
- 复制机器人设备激活页面中三个参数的`product_key`，`device_name`, `device_secret`，并且将这三个参数配置到 RynnRCP 目录`rcp_framework/robots/so100/config/device_config.yaml`文件中（http_url/endpoint_mqtt/endpoint_websocket参数不用修改）

### 相机节点
- 插入固定视角相机，对应相机名称为`observation.images.front`，执行 `ls /dev/video*` 查看设备号，例如`/dev/video0`。
- 插入腕部视角相机，对应相机名称为`observation.images.wrist`，执行 `ls /dev/video*` 查看设备号，例如`/dev/video2`。
- 将设备号填入相机配置文件`rcp_framework/robots/so100/config/cameras.yaml`。
- 推荐两个视角的相机**至少有一个直连主机的USB口**，根据经验在性能较弱的机器上，如果两个相机同时使用拓展坞可能会导致使用异常。

### RobotMotion

配置机器人的相关参数，不同机器人的配置流程可能存在差异，以 so100 为例，相关配置与机械臂标定方法详见：[RobtoMotion LeRobot README](robot_motion/robots/lerobot/README_cn.md)。

## 运行
```bash
cd RynnRCP

# 使用 Conda 激活 Python 虚拟环境
conda activate venv

# 或使用 Python venv 虚拟环境
source venv/bin/activate

# 启动端侧节点
bash example/so100.sh

# 或者启动端侧节点并输出调试日志
bash example/so100.sh DEBUG
```

该脚本将依次启动以下组件以支持 `so100` 端侧功能：
- RobotMotion：启动运动控制程序。
- 服务节点：运行端侧服务节点，并加载设备信息和日志配置。
- 相机节点：启动相机节点，处理摄像头配置信息并启动异步图像抓取。

注意事项
- 日志存储在 `$HOME/RynnRcplog/` 文件夹中。
- 使用 Ctrl+C 组合键来终止脚本执行，系统会自动关闭所有子进程。
- 确保在运行脚本之前，所有配置文件路径和依赖项已正确设置。
- 确保在运行脚本之前，激活了 Python 虚拟环境。

# 模块介绍

## RobotServer
RobotServer提供了一个用于构建机器人服务节点的框架，主要负责处理机器人设备与云端之间的通信和数据交换。该模块的核心部分包括动作服务节点（ActionServer）、传感器服务节点（SensorServer）和设备监控服务节点（DeviceMonitorServer），这些服务节点可通过 MQTT 和 WebSocket 等数据传输协议进行通信，实现对机器人设备的控制和数据采集。
- 多协议通信连接：针对不同通信协议需求提供统一通信中间件，支持MQTT、WebSocket、LCM等协议，通过标准化接口实现指令传输、数据采集、状态监控等核心功能的高频稳定执行；
- 服务节点解耦设计：采用模块化设计，提供动作执行、传感器采集、设备监控等基础服务节点模板，开发者可基于 CTerminalDeviceServer 基类扩展定制化服务节点，实现设备占用控制、资源调度、多客户端协同等自定义功能；
- 安全环境与配置中心：配置文件仅存储设备认证元数据、运行时通过HTTPS安全通道向认证服务发起请求，实时生成具有时效性的访问 Token ，保障通信链路安全。

## RobotMotion
Robot Motion 为你提供一个能够通过不同模型驱动机器人运动及完成任务的规控中间件，同时还提供了带物理引擎的 Mujoco 仿真可视化工具，为你配置自己的机器人或验证自己的模型时提供一个安全的环境。
- 模型与机器人的联结：对于不同模型输出存在差异提供机器人可执行的规控中间件，对于VLA类型的分层推理模型或轻量级（如ACT）实时推理模型， RobotMotion 提供了实际机器人均可高频执行的规控中间件；
- 带物理引擎的仿真工具：提供了基于 Mujoco 的带物理引擎的仿真工具，为验证机器人策略执行提供一个安全的可视化环境，并能在仿真环境中快速验证自己的机器人相关配置是否正常；
- 提供其他辅助工具，如数据记录及可视化，仿真回放等；

## 相机节点

相机节点实现基础的图像捕获和传输功能，提供高效、实时的图像捕获与处理能力。该模块结合了 LCM 协议，实现相机数据的即时传输，并通过多线程架构优化数据处理和管理。
- 实时图像捕获：利用 OpenCV 库实现摄像头的实时图像捕获，支持根据配置动态调整分辨率和帧率。
- 无损图像压缩：使用 Gzip 对原始图像进行无损压缩，以减少在网络上传输时的带宽消耗。
- 异步处理：通过 Python的 线程功能将相机捕获和消息响应过程分离，确保系统的实时性和响应性，同时有效利用系统资源。

# 待办
- [x] 发布**RynnRCP** 1.0版本
- [ ] 发布技术报告
- [ ] ActionServer和SensorServer支持MCP
- [ ] 完善RynnRCP框架剩余部分
- [ ] 支持更多构型的机器人