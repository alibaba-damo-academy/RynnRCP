# 概述

**RynnRCP** 提供一套面向机器人本体的“服务化能力框架”与对外通信协议（Robotics Context Protocol）。它的目标是把机器人侧的动作控制、传感器采集、设备信息等能力，封装为一组统一的 Tools，并通过通信插件（Plugin）向外部应用/云端平台暴露。

本项目主要由两部分组成：机器人侧的 **RCP Core（工具服务）** 与 **Comm Plugin（对外协议/传输）**：
- **RCP Core**：负责把机器人本体与传感器接入进来，维护缓冲区、对齐观测、执行动作，并以 Tool 的形式对外提供能力。
- **Comm Plugin**：负责把 Tool 通过特定协议与传输通道暴露给外部系统，例如 MQTT、WebSocket、MCP(JSON-RPC) 等；目前已实现对乐云平台 **RynnBot** 的插件支持。


>用户可以通过 RynnRCP 清晰地了解到从传感器获取数据到模型推理以及机器人动作执行的完整链路，同时，清晰的分层结构和标准的通信协议使得迁移到特定使用场景变得更加容易。



# 目录结构

```bash
.
├── setup_rcp.sh                 # 一键安装/配置（推荐）
├── pyproject.toml               # Python 包配置
├── README.md / README_cn.md     # 项目说明
├── rynnrcp/                     # 顶层入口：RynnRCP
├── rcp_core/                    # RCP Core：工具服务核心
│   ├── rcp_core.py              # RcpCore 初始化入口
│   ├── action_server/           # 动作相关 server
│   ├── sensor_server/           # 传感器相关 server
│   ├── device_monitor_server/   # 设备监控 server
│   └── common/                  # bus / adapter / protocol / utils 等公共组件
├── comm_plugin/                 # 通信插件：对外协议/传输
│   ├── base_plugin/             # 插件接口定义
│   ├── mcp_plugin/              # Mcp 插件
│   └── rynnbot_plugin/          # RynnBot 插件（MQTT + WebSocket）
├── rcp_sensor/                  # 端口设备/传感器实现
│   └── camera/                  # 相机实现
├── robots/                      # 各机器人最小工程与说明
|   ├── lekiwi/
│   ├── so100/
│   └── so101/
├── rcp_motion/                  # 运动控制与仿真/工具链
├── common/                      # 公共文件（配置、lcm 消息等）
├── scripts/                     # 脚本
└── docs/                        # 文档

```

# 能力支持情况

## 构型
| 机器人 | 平台/插件 | 接入方式 | 目录入口 | 说明 |
|---|---|---|---|---|
| SO100 | 乐云平台（MQTT + WebSocket） | Action：module｜Sensor：port | `robots/so100/` | 提供最小可运行工程与配置向导 |
| SO101 | 乐云平台（MQTT + WebSocket） | Action：module｜Sensor：port | `robots/so101/` | 提供最小可运行工程与配置向导 |
| LeKiwi | 乐云平台（MQTT + WebSocket） | Action：module｜Sensor：port | `robots/lekiwi/` | 提供最小可运行工程与配置向导 |

> SO100/SO101/LeKiwi 构型为零代码接入：用户仅需配置串口、相机与平台三元组等信息即可运行；本体控制逻辑由 `rcp_motion` 内置 `controller` 提供（无需自行编写 SDK 调用代码）。同时，`rcp_motion`（RCP Motion）负责将云端/模型的离散低频输出转换为机器人可执行的高频连续控制信号，并提供 MuJoCo 仿真、真机调试与回放、数据采集与轨迹可视化等配套工具。

## 服务（RCP Core）
| Server | 状态 | 提供 Tools| 说明 |
|---|---|---|---|
| ActionServer | ✅ | `get_state`、`run_action_chunk` | 观测对齐与动作执行；对外提供状态快照与动作序列下发能力 |
| SensorServer | ✅ | `get_image`、`get_image_info` | 传感器数据服务（当前以相机为主）：图像获取/编码与相机能力枚举 |
| DeviceMonitorServer | ✅ | `get_device_info` | 设备信息与资源监控：CPU/内存/系统信息与相机列表等 |
| Skill Server | 🚧 | - | 技能封装与执行 |
| Data Manager Server | 🚧 | - | 数据采集、回放与管理 |
| Infer Server | 🚧  | - | 推理服务：支持多种模型（VLA/RL 等）在本地或远程推理，会通过 BUS 调用 Action/Sensor |
| Model Manager Server | 🚧 | - | 模型管理：提供模型管理服务 |
| Agent Server | 🚧 | - | 工作流/任务编排：解析并执行统一工作流协议；可调度 Skill 或底层 Tool；支持未来接入规划模型，并在安全建联后调度其他机器人协作执行任务 |


## 通信插件（Comm Plugin）
| Plugin | 状态 | 传输协议 | 说明 |
|---|---|---|---|
| RynnBot Plugin | ✅ | MQTT + WebSocket | 面向乐云平台接入：设备占用/释放、动作下发、取图/取状态、设备属性上报 |
| MCP Plugin | ✅ | JSON-RPC（MCP 风格） | 标准化 `tools/list` / `tools/call` 接入，便于本地/局域网/公网场景对接外部应用与模型服务 |


# 安装与使用
<!-- > Note: 如需了解乐云平台侧能力与设备管理，可阅读：[RynnBot 乐云具身智能开发平台使用文档](https://developer.damo-academy.com/allSpark/document/guide/basic)   -->

## 下载源码
```bash
git clone https://github.com/alibaba-damo-academy/RynnRCP.git
```

## 快速设置（推荐）

我们提供了一个统一的设置脚本，支持双语（英语/中文）：

```bash
cd RynnRCP
bash setup_rcp.sh
```

该脚本将引导您完成：
1. 语言选择（English / 中文）
2. 安装 uv（若未安装）
3. 创建并激活虚拟环境（venv/）
4. 安装 RynnRCP 与开发依赖
5. 生成协议与消息代码（protobuf + LCM；Windows 上 LCM 失败会提示并继续）
6. 导入验证（rynnrcp / rcp_motion / mujoco）


## 手动安装

### 安装 Python 环境
```bash
cd RynnRCP

# 使用 venv
python3 -m venv venv
source venv/bin/activate
pip install -e .

# 或使用 Conda
conda create --name venv python=3.10
conda activate venv
pip install -e .

# 生成协议与消息代码
python scripts/gen_proto_msg.py
python scripts/gen_lcm_msg.py   # Windows 上可能需要跳过
```
> Python 3.10 是最低要求的 Python 版本。

## 运行

本项目已经接入了多种机械臂机器人/构型。具体机器人（SO100/SO101/LeKiwi等）的 **配置、标定与启动** 方式，请参考：
- robots/<robot_name>/README.md
- robots/<robot_name>/config/*.yaml
- robots/<robot_name>/run_rcp_*.py 等启动脚本

使用流程：
1. 进入对应机器人目录（例如 robots/so100/）
2. 按其 README 完成配置（相机/串口/插件相关配置）
3. 启动示例脚本（一般是 run_rcp_*.py 或对应启动脚本）


## 进阶开发
支持用户进行进阶开发，目前支持以下功能：
- 新增本体构型
- 新增自定义 `Server`
- 新增 `Comm Plugin`

更详细的开发说明与示例请参考：[RynnRCP Tutorials](https://rynnrcp.github.io/)


## 注意事项

**Linux：**
- 脚本中的一些步骤可能会请求输入 sudo 密码以获得管理权限
- UDP 缓冲区配置将在 `/etc/sysctl.conf` 中修改，以适应 LCM 图像传输需求

**macOS：**
- 支持 Apple Silicon (M1/M2/M3/M4) 和 Intel Mac
- 如需配置 LCM 多播：`sudo route -nv add -net 224.0.0.0/4 -interface lo0`

**Windows：**
- 相机必须插在不同的USB控制器上，否则不能同时打开多个相机
- 推荐使用git bash作为终端界面，进行setup_rcp.sh进行环境配置，其中git安装时注意行尾的配置，须选择Unix-style line endings

# 待办
- [x] 发布**RynnRCP** 1.0版本
- [ ] 发布技术报告
- [x] ActionServer和SensorServer支持MCP
- [ ] 完善RynnRCP框架剩余部分
- [ ] 支持更多构型的机器人