# RynnRCP

[English](README.md)

RynnRCP 是面向 Embodied Agent 的机器人能力协议
（RCP, Robotics Capability Protocol）。这个仓库是该协议的 Python 标准实现。

RCP 标准化机器人如何暴露 Manifest、Observation、Action、Policy、Health、Resource 和
Data Collection。App、Agent、MCP 或云端系统只需要理解这些协议对象，不需要知道每台
机器人的 SDK、串口、相机编号或内部通道实现。

更简单地说：

> MCP 解决 Agent 怎么调用工具；RCP 解决机器人执行经验怎么变成数据，本地策略怎么变成
> 可复用技能，技能怎么再被 Agent 安全调用。

RCP 不替代 ROS、机器人 SDK 或底层控制器，也不要求大模型进入硬实时控制闭环。
硬实时控制、轨迹插补、力控、急停和安全限幅必须由机器人本体侧 controller 或
safety layer 实现。

## 这个仓库提供什么

这个仓库提供 RCP 协议的一套具体实现：

- Python Runtime：加载机器人配置，并运行标准机器人服务；
- 官方 App：Teleop、MCP、RynnBot 云端接入、协议调试、录制、编码和导出；
- 机器人接入包：当前包含 SO101、Aero Hand（绳肌妙算）、Atom01；
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
| 使用 SO101 机器人 | [robots/so101/README.zh-CN.md](robots/so101/README.zh-CN.md)，按其中的 `./setup_so101.sh` 自动安装和配置 |
| 使用 Aero Hand（绳肌妙算） | [robots/aero_hand/README.zh-CN.md](robots/aero_hand/README.zh-CN.md)，按其中的 `./setup_aero_hand.sh` 自动安装和配置 |
| 使用 Atom01 人形机器人 | [robots/atom01/README.zh-CN.md](robots/atom01/README.zh-CN.md)，按其中的 `./setup_atom01.sh` 自动安装、编译和配置 |
| 使用 Teleop / MCP / RynnBot App | [apps/README.zh-CN.md](apps/README.zh-CN.md)，再进入对应 App README |
| 调试单个 Server 协议接口 | [apps/protocol_debug/README.zh-CN.md](apps/protocol_debug/README.zh-CN.md) |
| 开发新 App 调用 RCP Server | [docs/RCP App 接入 Server 指南.html](docs/RCP%20App%20接入%20Server%20指南.html) |
| 新增一个机器人构型 | [docs/RCP 机器人构型接入指南.html](docs/RCP%20机器人构型接入指南.html) |
| 接入本地推理策略 | [docs/RCP 策略服务接入指南.html](docs/RCP%20策略服务接入指南.html) |
| 查询配置字段怎么写 | [docs/RCP配置文件说明.md](docs/RCP配置文件说明.md) |
| 查询协议对象和接口格式 | [docs/RCP使用场景及协议.md](docs/RCP使用场景及协议.md) |
| 不确定该看哪份文档 | [docs/README.zh-CN.md](docs/README.zh-CN.md) |
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

如果你需要 RynnRCP Runtime 和官方 App，但不需要安装具体机器人构型，可以在仓库根目录执行：

```bash
python -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e .
python -m pip install -e apps/common -e apps/mcp -e apps/teleop -e apps/rynnbot
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

这些包以 editable 模式安装。修改仓库源码后，再次运行命令会直接使用最新代码。

使用具体机器人构型时，优先阅读对应机器人包 README。例如 SO101 使用
[`robots/so101/setup_so101.sh`](robots/so101/setup_so101.sh)，Aero Hand 使用
[`robots/aero_hand/setup_aero_hand.sh`](robots/aero_hand/setup_aero_hand.sh)，Atom01 使用
[`robots/atom01/setup_atom01.sh`](robots/atom01/setup_atom01.sh)，完成 Runtime、官方 App 和机器人包的安装和配置。

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
  so101/                SO101 接入
  aero_hand/            Aero Hand（绳肌妙算）接入
  atom01/               Atom01 接入

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

Atom01 机器人包在 `robots/atom01/rynnrcp_robot_atom01/atom_control/` 下包含
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
- [SO101 README](robots/so101/README.zh-CN.md)：SO101 安装、配置、启动命令和硬件说明。
- [Aero Hand README](robots/aero_hand/README.zh-CN.md)：Aero Hand（绳肌妙算）单手/双手配置、启动和 RynnBot 接入说明。
- [Atom01 README](robots/atom01/README.zh-CN.md)：Atom01 安装、零位标定、启动和 RynnBot 接入说明。
- [Protocol Debug README](apps/protocol_debug/README.zh-CN.md)：浏览器协议调试台，用于查看工具/观测/动作并发送原始请求。
- [测试指南](tests/README.zh-CN.md)：测试归属和推荐测试命令。
