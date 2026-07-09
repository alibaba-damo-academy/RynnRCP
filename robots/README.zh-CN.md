# Robots

这个目录存放机器人构型接入包。每个子目录代表一种机器人或一类机器人实现。

机器人构型包负责把具体硬件接入 RCP Server，例如 controller、硬件配置、标定工具和启动配置。
它不实现 RCP 协议层，也不实现 App。

## 当前构型

| 目录 | 说明 | 入口 |
| --- | --- | --- |
| `so101/` | SO101 主臂/从臂接入包 | [`so101/README.zh-CN.md`](so101/README.zh-CN.md) |
| `aero_hand/` | Aero Hand（绳肌妙算）单手/双手接入包 | [`aero_hand/README.zh-CN.md`](aero_hand/README.zh-CN.md) |
| `atom01/` | Atom01 人形机器人接入包 | [`atom01/README.zh-CN.md`](atom01/README.zh-CN.md) |

## 新增机器人

新增机器人时，先用机器人原生 SDK、串口工具、ROS2/LCM 或相机工具跑通最小硬件能力；
再把这些能力整理成 RCP Server 可以加载的 controller、数据源和配置。

详细步骤见：
[`../docs/RCP 机器人构型接入指南.html`](../docs/RCP%20机器人构型接入指南.html)。

## 一个机器人包应该包含什么

- controller：把机器人 SDK 映射到协议动作和观测。
- config：Server 配置和 robot integration 配置。
- policies（可选）：本地 RL / VLA / policy 策略目录，启用 `policy_service` 时由 Server 扫描。
- model assets（可选）：URDF、mesh、标定文件或策略权重等资源。
- setup：该机器人需要的安装脚本。
- README：该机器人如何安装、配置、启动和测试。
