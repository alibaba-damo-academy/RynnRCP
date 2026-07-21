# Teleop App

Teleop App 用于绑定控制端 Server 和执行端 Server，完成遥操、数据采集、回放和数据导出。

它不直接连接机器人硬件。控制端和执行端都来自已经启动的 RCP Server。

常见用法：

- 控制端 Server：主臂、手柄、键盘控制器或其他能输出 observation 的设备。
- 执行端 Server：从臂或真实执行机器人，通常需要支持 action 和 data collection。

## 启动

在安装好 `rynnrcp-app-teleop` 后运行：

```bash
rynnrcp-teleop-app
```

默认 Web UI 地址：

```text
http://127.0.0.1:28402
```

需要使用自定义 App 配置时：

```bash
rynnrcp-teleop-app --config <teleop-app.yaml>
```

默认配置中，遥操控制频率为 60Hz，图像预览为 10Hz，导出时默认启用 RynnBot key 映射。

## 使用流程

1. 启动控制端 Server 和执行端 Server。
2. 启动 `rynnrcp-teleop-app`，打开 Web UI。
3. 在页面中选择控制端和执行端。
4. 选择控制流，例如 `observation.robot.joint_state -> action.robot.joint_position`。
5. 点击开始遥操；需要采集时，再点击开始数采。
6. 停止数采后，可以回放 episode 或导出数据。

Server 启动后会在终端打印 `Local gRPC` 和 `LAN gRPC` 地址。自动发现不可用或跨网段时，
在 Teleop 页面填写 Server 实际打印的地址。

## 主要功能

- 自动发现或手动绑定 RCP Server。
- 选择控制流，例如 `observation.robot.joint_state -> action.robot.joint_position`。
- 预览执行端图像和控制端/执行端状态。
- 在执行端 Server 采集原始数据。
- 回放指定 episode。
- 导出数据集；需要 RynnBot 等外部平台格式时，在 App 侧做 key 映射和编码。

## 数据边界

- 原始采集数据属于执行端 Server。
- 回放或导出远端 Server 数据时，App 通过 Resource 拉取采集文件；执行端 Server 开启 `resources` 能力后可使用历史采集文件。
- 导出文件保存在 App 本机。
- App 和 Server 之间始终使用 RCP key；导出到外部平台格式时，映射发生在 App 边界。

更多 Interface 和 Resource 交互流程见：
[`../../docs/RCP App 接入 Server 指南.html`](../../docs/RCP%20App%20接入%20Server%20指南.html)。
