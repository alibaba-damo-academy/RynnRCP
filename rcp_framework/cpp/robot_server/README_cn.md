# Robot Server SDK
Robot Server SDK是一个模块化的机器人服务开发框架，提供多种服务模块，如动作执行、设备监控和传感器数据处理，支持异步通信与高效的事件驱动编程。设计特点强调低耦合和可扩展性，方便开发者根据需求进行定制和扩展。

## 目录结构

```bash
robot_server/
├── CMakeLists.txt          # CMake构建配置文件
├── README.md               # SDK说明文档
├── README_cn.md            # 中文SDK说明文档
├── action_server/          # 动作服务模块
├── common/                 # 公共基础组件
├── device_monitor_server/  # 设备监控服务模块
├── sensor_server/          # 传感器服务模块
```

## 模块介绍
### common 模块
- **核心文件**:
  - **version.hpp**: 版本信息
  - **robot_server.hpp**: SDK统一头文件
  - **terminal_device_server/**: 终端设备服务模块相关实现
  - **server/**: 通用服务模块相关实现

### server 模块
- **核心文件**:
  - **server.hpp**: 通用服务模块接口声明
  - **server.cpp**: 通用服务模块具体实现
- **功能描述**:
  - 对云端消息进行处理
  
### terminal_device_server 模块
- **核心文件**:
  - **terminal_device_server.hpp**: 端侧设备服务模块接口声明
  - **terminal_device_server.cpp**: 端侧设备服务具体实现
- **功能描述**:
  - 继承通用服务模块
  - 实现模块的占用与释放
  - 实现消息的分类与分发
  
### action_server 模块
- **核心文件**:
  - **action_server.hpp**: 动作服务模块接口声明
  - **action_server.cpp**: 动作服务具体实现
- **功能描述**:
  - 处理具体动作消息内容
  - 实现与动作执行节点的消息交互
  - 上传机器人相关状态

### sensor_server 模块
- **核心文件**:
  - **sensor_server.hpp**: 传感器服务模块接口声明
  - **sensor_server.cpp**: 传感器服务具体实现
- **功能描述**:
  - 处理具体传感器消息内容
  - 实现与传感器节点的消息交互
  - 上传传感器相关配置
  
### device_monitor_server 模块
- **核心文件**:
  - **device_monitor_server.hpp**: 设备监控服务模块接口声明
  - **device_monitor_server.cpp**: 设备监控服务具体实现
- **功能描述**:
  - 周期监控设备状态
  - 周期上报设备状态

## SDK设计特点
1. **模块化设计**
   采用清晰的模块划分，各模块之间保持低耦合，便于独立开发和维护。
2. **异步通信机制**
   基于高效的异步通信模型，支持事件驱动的编程方式。
3. **可扩展性**
   提供清晰的扩展接口，方便集成新的设备类型和功能模块。
