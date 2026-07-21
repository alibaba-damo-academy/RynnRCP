# Robots

这个目录存放机器人构型接入包。每个子目录代表一种机器人或一类机器人实现。

[English](README.md)

机器人构型包负责把具体硬件接入 RCP Server，例如 controller、硬件配置、标定工具和启动配置。
RCP 协议层由仓库根包提供，配套 App 位于 `apps/`。

## 当前构型

| 目录 | 说明 | 入口 |
| --- | --- | --- |
| `lerobot_so101/` | SO101 主臂/从臂接入包 | [`lerobot_so101/README.zh-CN.md`](lerobot_so101/README.zh-CN.md) |
| `tetheria_aerohand/` | Aero Hand（绳肌妙算）单手/双手和摄像手势主端接入包 | [`tetheria_aerohand/README.zh-CN.md`](tetheria_aerohand/README.zh-CN.md) |
| `roboparty_atom01/` | Atom01 人形机器人接入包 | [`roboparty_atom01/README.zh-CN.md`](roboparty_atom01/README.zh-CN.md) |
| `booster_t1/` | Booster T1 high-level、low-level 和本地行走策略接入包 | [`booster_t1/README.zh-CN.md`](booster_t1/README.zh-CN.md) |
| `noetix_bumi/` | Noetix Bumi high-level、low-level 和本地行走策略接入包 | [`noetix_bumi/README.zh-CN.md`](noetix_bumi/README.zh-CN.md) |
| `sim_robot/` | Isaac Sim 仿真机器人接入包 | [`sim_robot/README.zh-CN.md`](sim_robot/README.zh-CN.md) |
| `lerobot_lekiwi/` | LeKiwi 移动操作机器人接入包 | [`lerobot_lekiwi/README.zh-CN.md`](lerobot_lekiwi/README.zh-CN.md) |

## 安装约定

每个机器人都先进入自己的目录，再运行安装脚本并激活本地环境：

```bash
cd robots/<robot>
bash setup_<robot>.sh
source venv/bin/activate
```

具体脚本名以对应机器人的 README 为准，例如 Noetix Bumi 使用
`setup_bumi.sh`，Sim Robot 使用 `setup_sim.sh`。

每个 `rynnrcp-server` 启动后都会在终端打印 `Debug UI` 地址。需要查看关节状态、Action、相机图像或实时曲线时，手动在浏览器打开该地址；端口被占用时，使用当前 Server 终端打印的新地址。

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
