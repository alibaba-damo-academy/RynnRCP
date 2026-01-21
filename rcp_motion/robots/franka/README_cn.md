# Franka - 机器人运动接口

## 第一部分：亮点特性

本软件包为控制 Franka 机械臂提供了对接模型policy的框架，具有以下关键特性：

### 跨平台支持
- **Linux 和 macOS 兼容**：在两个操作系统上无缝运行
- **统一接口**：仿真以及真实硬件使用相同的 API

### 机器人控制模式
- **MuJoCo 仿真**：基于物理的仿真环境，用于安全测试
- **真实机器人接口**：通过 ros2/humble 控制 Franka Research3 机械臂

### 高级功能
- **VLA高频指令生成器**：多种方法将低频 VLA 输出生成高频控制命令
- **离线关节绘图器**：命令位置与实际位置的实时可视化对比
- **轻量级 Python 环境**：最小依赖，快速设置
- **预定义运动模式**：内置测试动作用于机器人验证
- **VLA 集成**：从基础控制到视觉-语言-动作模型的逐步进展

### 开发工作流程
- 从仿真开始（无需硬件）
- 在真实硬件上验证
- 与 VLA 系统集成

## 第二部分：安装和设置

### 2.1 自动环境设置

```bash
# 导航到项目目录
cd robots/franka

# 自动安装依赖和环境
source ./setup_motion.sh
```

设置脚本将：
- 根据需要创建虚拟环境
- 控制实际机械臂时，需要额外安装Franka ros2相关资源
- 从 requirements.txt 安装所有依赖项
- 以开发模式设置包（启用清洁导入）
- 安装 LCM 和其他必需包
- 可选安装开发工具

**手动设置（如果需要）：**
```bash
# 创建并激活虚拟环境
python -m venv .venv
source .venv/bin/activate

# 然后运行设置
./setup_motion.sh
```

### 2.2 实际机械臂配置可参考Franka相关说明

franka官方提供了[franka ros2](https://github.com/frankarobotics/franka_ros2?tab=readme-ov-file#setup)的具体安装说明，可按照官方提供的说明文档安装及验证。


## 第三部分：虚拟机械臂

### 3.1 设定轨迹测试

```bash
cd robots/franka

# 测试设定轨迹驱动虚拟机械臂
python -m scripts.unified_franka_controller --mode sim --signal_source sim 

```
仿真轨迹的相关参数可以在`robots/franka/configs/config.yaml`中修改

### 3.2 模型轨迹测试

```bash
cd robots/franka

# 测试policy信号驱动虚拟机械臂
python -m scripts.unified_franka_controller --mode sim --signal_source policy
```


## 第四部分：实际机械臂测试

### 4.1 设定轨迹测试

```bash
cd robots/franka

# 测试设定轨迹驱动真实机械臂
python -m scripts.unified_franka_controller --mode real --signal_source sim

```

### 4.2 模型轨迹测试

```bash
cd robots/franka

# 测试模型驱动真实机械臂
python -m scripts.unified_franka_controller --mode real --signal_source policy
```

```bash
python -m scripts.unified_franka_controller [选项]

选项:
  --mode {sim,real}                  控制模式（必需）
  --ctrlfreq {30-250}                    控制频率，单位 Hz（默认：100）
  --signal_source {sim,policy}           信号来源（默认：sim）

示例:
  --mode sim                             # MuJoCo 仿真
  --mode real --motion 2                 # 真实机器人，模式
```

