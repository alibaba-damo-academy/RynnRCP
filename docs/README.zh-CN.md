# RynnRCP 文档入口

这个目录提供长期维护的 RynnRCP 使用、接入和协议文档。

## 推荐阅读顺序

| 目标 | 文档 |
| --- | --- |
| 了解 RCP 对外协议 | [RCP使用场景及协议.md](RCP使用场景及协议.md) |
| 新增机器人构型 | [RCP 机器人构型接入指南.html](RCP%20机器人构型接入指南.html) |
| 编写 server config / robot integration 字段 | [RCP配置文件说明.md](RCP配置文件说明.md) |
| 接入本地推理策略服务 | [RCP 策略服务接入指南.html](RCP%20策略服务接入指南.html) |
| 开发 App 调用 RCP Server | [RCP App 接入 Server 指南.html](RCP%20App%20接入%20Server%20指南.html) |
| 使用 Teleop 遥操、数采、回放和导出 | [RCP Teleop 遥操数采使用指南.html](RCP%20Teleop%20遥操数采使用指南.html) |

## 文档定位

- `RCP使用场景及协议.md`：协议契约，定义对外对象、方法、参数和返回值。
- `RCP 机器人构型接入指南.html`：给机器人接入者，说明先跑通硬件，再写 controller/source/config。这里讲接入流程，不当作字段字典使用。
- `RCP配置文件说明.md`：配置字段字典，说明 server config、robot integration、component、source、codec 每个字段怎么写。
- `RCP日志与排障.md`：日志目录、运行关联 ID、日志配置和 App 到设备链路排障步骤。
- `RCP 策略服务接入指南.html`：给策略接入者，说明 policy 目录、`policy.yaml`、`policy.py` 和运行时输入/动作输出约定。
- `RCP App 接入 Server 指南.html`：给 App 开发者，说明发现、连接、调用工具和 Resource 传输。
- `RCP Teleop 遥操数采使用指南.html`：给 Teleop 使用者，说明页面流程和数据采集导出逻辑。
