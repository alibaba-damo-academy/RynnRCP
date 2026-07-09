# RynnRCP Apps

这个目录存放官方 App。App 是 RCP Server 的客户端，不实现机器人本体能力，也不直接读取机器人
SDK；它们通过 RynnRCP Interface 调用 Server 暴露的协议工具。

使用 App 之前，需要先有一个可连接的 RCP Server。这个 Server 可以运行在本机，也可以运行在局域网内的机器人设备上。

## App 列表

| App | 入口命令 | 用途 |
| --- | --- | --- |
| Teleop | `rynnrcp-teleop-app` | 遥操、数据采集、回放、编码导出 |
| MCP | `rynnrcp-mcp-app` | 把一个 RCP Server 暴露为 MCP 工具 |
| RynnBot | `rynnrcp-rynnbot-app` | 接入 RynnBot 云端，处理属性上报、技能执行、数据回流 |
| Protocol Debug | `rynnrcp-protocol-debug` | 浏览器协议调试台，查看工具/观测/动作并发送原始请求 |
| Common | 无独立命令 | App 内部共用的录制、编码、资源传输和 key 映射工具 |

## 安装入口

普通用户通常不需要在这里单独安装 App。使用具体机器人构型时，优先按该机器人包的
README 执行安装脚本；例如 SO101 会通过 `robots/so101/setup_so101.sh` 安装 Runtime、
官方 App 和 SO101 机器人包。

只安装 Runtime 和官方 App 时，见仓库根目录 [`../README.zh-CN.md`](../README.zh-CN.md)。
只有在单独开发某个 App 时，才进入对应 App 目录查看 README 和安装方式。

安装完成后，可以用 `-h` 确认命令和参数：

```bash
rynnrcp-teleop-app -h
rynnrcp-mcp-app -h
rynnrcp-rynnbot-app -h
rynnrcp-protocol-debug -h
```

## 使用原则

- App 不知道机器人 SDK、串口、相机编号、channel 或 adapter。
- App 只通过协议对象名调用 Server，例如 `observation.robot.joint_state`、`action.robot.joint_position`。
- App 与 Server 的本地或远程数据交换应走 Resource 相关接口；如果目标 Server 关闭 `resources` 能力，App 不应读取该 Server 的文件类资源。
- App 如需对接外部平台格式，应在 App 边界做 key 和数据格式映射，内部和 Server 通信用 RCP 协议格式。

## 继续阅读

- App 如何接入 RCP Server：[`../docs/RCP App 接入 Server 指南.html`](../docs/RCP%20App%20接入%20Server%20指南.html)
- Teleop：[`teleop/README.zh-CN.md`](teleop/README.zh-CN.md)
- MCP：[`mcp/README.zh-CN.md`](mcp/README.zh-CN.md)
- RynnBot：[`rynnbot/README.zh-CN.md`](rynnbot/README.zh-CN.md)
- Protocol Debug：[`protocol_debug/README.zh-CN.md`](protocol_debug/README.zh-CN.md)
