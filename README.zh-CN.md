# RynnRCP

[English](README.md)

RynnRCP 是面向 Embodied Agent 的机器人能力协议
（RCP, Robotics Capability Protocol）。这个仓库是该协议的 Python 标准实现。

RCP 标准化机器人如何暴露 Manifest、Observation、Action、Policy、Health、Resource 和
Data Collection。App、Agent、MCP 或云端系统通过这些协议对象调用机器人能力，避开每台
机器人的 SDK、串口、相机编号或内部通道差异。

更简单地说：

> MCP 解决 Agent 怎么调用工具；RCP 解决机器人执行经验怎么变成数据，本地策略怎么变成
> 可复用技能，技能怎么再被 Agent 安全调用。

ROS、机器人 SDK 和底层控制器继续负责硬实时控制、轨迹插补、力控、急停和安全限幅。
RCP 负责把这些机器人能力整理成统一协议接口。

## 这个仓库提供什么

这个仓库提供 RCP 协议的一套具体实现：

- Python Runtime：加载机器人配置，并运行标准机器人服务；
- 官方 App：Teleop、MCP、RynnBot 云端接入、协议调试、录制、编码和导出；
- 机器人接入包：当前包含 SO101、Aero Hand（绳肌妙算）、Atom01、Booster T1、Noetix Bumi 和 Isaac Sim；
- 面向 App 接入、机器人构型接入、策略服务、协议对象和配置编写的文档；
- 覆盖实现边界的测试。

## 面向谁

- 机器人接入方：把真实机器人、相机、ROS2/LCM 系统或 Python SDK 接成 RCP Server。
- App 或 Agent 开发者：通过统一 Interface 调用机器人，不直接依赖每台机器人的 SDK。
- 数据和训练角色：采集原始 Episode、回放执行过程、编码和导出数据集。
- 云端接入方：通过 RynnBot App 把云端指令和 RCP Server 连接起来。

## 从这里开始

RynnRCP Runtime、官方 App 和机器人构型包都在这个仓库内维护。具体安装和运行方式由对应包负责说明；使用某个机器人构型时，请进入该构型目录阅读 README，并使用它提供的安装脚本完成环境配置。

| 你的目标 | 先看 |
| --- | --- |
| 使用 SO101 机器人 | [robots/lerobot_so101/README.zh-CN.md](robots/lerobot_so101/README.zh-CN.md)，按其中的 `bash setup_so101.sh` 自动安装和配置 |
| 使用 Aero Hand（绳肌妙算） | [robots/tetheria_aerohand/README.zh-CN.md](robots/tetheria_aerohand/README.zh-CN.md)，按其中的 `bash setup_aero_hand.sh` 自动安装和配置 |
| 使用 Atom01 人形机器人 | [robots/roboparty_atom01/README.zh-CN.md](robots/roboparty_atom01/README.zh-CN.md)，按其中的 `bash setup_atom01.sh` 自动安装、编译和配置 |
| 使用 Franka Research 3 | [robots/franka_fr3/README.zh-CN.md](robots/franka_fr3/README.zh-CN.md)，拉取并编译兼容版本的官方 libfranka |
| 使用其他机器人 | [robots/README.zh-CN.md](robots/README.zh-CN.md)，选择 Booster T1、Noetix Bumi 或 Isaac Sim |
| 使用 Teleop / MCP / RynnBot App | [apps/README.zh-CN.md](apps/README.zh-CN.md)，再进入对应 App README |
| 查看单个 Server 协议接口 | [apps/protocol_debug/README.zh-CN.md](apps/protocol_debug/README.zh-CN.md) |
| 开发新 App 调用 RCP Server | [docs/RCP App 接入 Server 指南.html](docs/RCP%20App%20接入%20Server%20指南.html) |
| 新增一个机器人构型 | [docs/RCP 机器人构型接入指南.html](docs/RCP%20机器人构型接入指南.html) |
| 接入本地推理策略 | [docs/RCP 策略服务接入指南.html](docs/RCP%20策略服务接入指南.html) |
| 查询配置字段怎么写 | [docs/RCP配置文件说明.md](docs/RCP配置文件说明.md) |
| 按运行链路查看日志和排障 | [docs/RCP日志与排障.md](docs/RCP日志与排障.md) |
| 查询协议对象和接口格式 | [docs/RCP使用场景及协议.md](docs/RCP使用场景及协议.md) |
| 测试实机延迟和 CPU/内存负载 | [tests/benchmarks/live_device/README.zh-CN.md](tests/benchmarks/live_device/README.zh-CN.md) |
| 选择文档入口 | [docs/README.zh-CN.md](docs/README.zh-CN.md) |
| 运行或维护测试 | [tests/README.zh-CN.md](tests/README.zh-CN.md) |

新增机器人时，先用机器人原生 SDK、ROS2/LCM、串口工具或相机工具跑通最小硬件能力，再把这些能力整理成 controller/source，最后写 RCP 配置。

## Server 和 App

RynnRCP 把机器人系统分成两个边界：

- Server：运行在机器人能力端，加载机器人配置，连接 controller、相机、ROS2/LCM 或 Python SDK，并暴露协议工具。
- App：运行在用户或云端侧，通过 RCP Interface 调用 Server，不直接碰机器人 SDK、串口、相机编号或内部 channel。

这种分层让同一个机器人 Server 可以被 Teleop、MCP、RynnBot 或后续自定义 App 复用；同一个 App 也可以连接不同机器人构型，只要这些机器人按 RCP 协议暴露能力。

调用链保持简单：

```text
App / Agent / MCP / Cloud
        |
        v
RCP Interface
        |
        v
RCP Server
        |
        v
controller / camera / ROS2 / LCM / SDK
```

## 安装 Runtime 和官方 App

只安装 RynnRCP Runtime 和官方 App 时，在仓库根目录执行：

```bash
python -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e .
python -m pip install -e apps/common -e apps/protocol_debug -e apps/mcp -e apps/teleop -e apps/rynnbot
```

安装完成后，当前 Python 环境会获得 RynnRCP 库和官方 App 命令：

```bash
rynnrcp-server
rynnrcp-teleop-app
rynnrcp-mcp-app
rynnrcp-rynnbot-app
rynnrcp-protocol-debug
```

每个命令都可以用 `-h` 查看使用方法和参数：

```bash
rynnrcp-server -h
rynnrcp-teleop-app -h
rynnrcp-mcp-app -h
rynnrcp-rynnbot-app -h
rynnrcp-protocol-debug -h
```

### 查看 Server 实时状态

`rynnrcp-server` 会启动内置的只读状态页面，但不会自动打开浏览器。Server 启动完成后，使用终端打印的 `Debug UI` 地址手动访问：

```text
Debug UI:   http://127.0.0.1:8092/
```

页面展示 Observation/state、Action、相机图像和实时曲线，不提供机器人控制。默认端口是 `8092`；端口已占用时，Server 会打印 warning 并选择后续可用端口，因此请始终使用当前终端打印的地址。

没有页面访问时，状态和图像不会被额外读取。关闭页面或切换到后台后，轮询立即停止，Action 快照会在约 3 秒后关闭并清空。

这些包以 editable 模式安装。修改仓库源码后，再次运行命令会直接使用最新代码。

使用具体机器人构型时，优先阅读对应机器人包 README。例如 SO101 使用
[`robots/lerobot_so101/setup_so101.sh`](robots/lerobot_so101/setup_so101.sh)，Aero Hand 使用
[`robots/tetheria_aerohand/setup_aero_hand.sh`](robots/tetheria_aerohand/setup_aero_hand.sh)，Atom01 使用
[`robots/roboparty_atom01/setup_atom01.sh`](robots/roboparty_atom01/setup_atom01.sh)，完成 Runtime、官方 App 和机器人包的安装和配置。
其他机器人使用各自 README 中的 setup 脚本；每个脚本至少安装 RynnBot、MCP 和 Protocol Debug App。

## 包组成

- `rynnrcp`：协议 Runtime 实现。
- `rynnkit`：随 `rynnrcp` 安装的实现辅助工具。
- Python 包 `rynnrcp-app-common`：通用录制、编码和导出辅助逻辑，无独立命令。
- Python 包 `rynnrcp-app-mcp`：面向 MCP 的独立 App，安装后提供 `rynnrcp-mcp-app`。
- Python 包 `rynnrcp-app-teleop`：遥操 Web App，安装后提供 `rynnrcp-teleop-app`。
- Python 包 `rynnrcp-app-rynnbot`：RynnBot 云端工作流 App，安装后提供 `rynnrcp-rynnbot-app`。
- `rynnrcp-protocol-debug`：随 `rynnrcp` 安装的浏览器协议调试台。
- Python 包 `rynnrcp-robot-so101`：SO101 机器人接入包，安装后提供 `rynnrcp-so101-configure`。
- Python 包 `rynnrcp-robot-aero-hand`：Aero Hand（绳肌妙算）接入包，安装后提供 `rynnrcp-aero-hand-configure`。
- Python 包 `rynnrcp-robot-atom01`：Atom01 人形机器人接入包，安装后提供 `rynnrcp-atom01-configure` 和 `rynnrcp-atom01-test-policy`。
- 其他机器人包包括 Booster T1、Noetix Bumi 和 Isaac Sim，硬件前置条件与启动命令见各包 README。

## 目录结构

```text
pyproject.toml          RynnRCP 实现包元数据
README.md               英文 README

rynnrcp/                协议 Runtime 实现
  adapters/             payload 适配
  config/               配置加载和展开
  connectors/           协议和模块连接器
  interface/            gRPC 通信和协议客户端
  ipc/                  通道和共享内存传输
  native/               native 辅助代码
  process/              进程生命周期辅助代码
  protocol/             协议方法定义
  robot/                机器人 controller 抽象
  runtime/              Runtime 编排
  services/             内置服务
  utils/                通用工具

rynnkit/                随 RynnRCP 安装的辅助工具
  cameras/              相机接口和 USB 相机支持

apps/                   官方独立 App 包
  common/               通用录制和导出辅助逻辑
  mcp/                  MCP App
  protocol_debug/       浏览器协议调试台
  rynnbot/              RynnBot 云端 App
  teleop/               遥操 Web App

robots/                 机器人接入包
  lerobot_so101/        SO101 接入
  lerobot_lekiwi/       LeKiwi 接入
  tetheria_aerohand/    Aero Hand（绳肌妙算）接入
  roboparty_atom01/     Atom01 接入
  booster_t1/           Booster T1 high-level / low-level 接入
  noetix_bumi/          Noetix Bumi high-level / low-level 接入
  sim_robot/            Isaac Sim 仿真接入

benchmarks/             可复现的实机性能测试
docs/                   长期维护的实现文档
tests/                  实现测试
```

## 测试

运行实现测试：

```bash
python -m pytest tests -q
```

机器人真机验证需要真实硬件和本机配置，具体启动与检查流程见对应机器人 README。

## 许可协议

除文件头或包 README 另有说明外，RynnRCP 使用 Apache License 2.0。

Atom01 机器人包在 `robots/roboparty_atom01/rynnrcp_robot_atom01/atom_control/` 下包含
GPL-3.0 C++ 控制绑定源码；这些文件以其 SPDX 文件头标注的 GPL-3.0 为准。

第三方代码和资产说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 文档

- [Docs README](docs/README.zh-CN.md)：长期维护文档入口。
- [RCP 协议](docs/RCP使用场景及协议.md)：对外协议契约、标准对象、工具和数据/资源流。
- [机器人构型接入指南](docs/RCP%20机器人构型接入指南.html)：新增机器人包时，如何先跑通硬件能力，再整理 controller/source 和配置。
- [配置指南](docs/RCP配置文件说明.md)：配置字段字典，说明 server config、robot integration、source、codec 每个字段怎么写。
- [策略服务接入指南](docs/RCP%20策略服务接入指南.html)：本地 policy 目录、`policy.yaml`、`policy.py` 和运行时输入/动作输出约定。
- [App 接入 Server 指南](docs/RCP%20App%20接入%20Server%20指南.html)：App 如何通过 Interface 调用 RCP Server。
- [Teleop 使用指南](docs/RCP%20Teleop%20遥操数采使用指南.html)：遥操、数采、回放和导出页面流程。
- [Apps README](apps/README.zh-CN.md)：官方 App 入口。
- [Robots README](robots/README.zh-CN.md)：机器人构型包入口。
- [SO101 README](robots/lerobot_so101/README.zh-CN.md)：SO101 安装、配置、启动命令和硬件说明。
- [Aero Hand README](robots/tetheria_aerohand/README.zh-CN.md)：Aero Hand（绳肌妙算）单手/双手配置、摄像手势遥操数采、启动和 RynnBot 接入说明。
- [Atom01 README](robots/roboparty_atom01/README.zh-CN.md)：Atom01 安装、零位标定、启动和 RynnBot 接入说明。
- [Booster T1 README](robots/booster_t1/README.zh-CN.md)：high-level 运控、low-level `LowCmd` 和本地行走策略说明。
- [Protocol Debug README](apps/protocol_debug/README.zh-CN.md)：浏览器协议调试台，用于查看工具/观测/动作并发送原始请求。
- [测试指南](tests/README.zh-CN.md)：测试归属和推荐测试命令。
