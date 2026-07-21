# Protocol Debug App

Protocol Debug App 是一个浏览器调试台，用来连接一个 RynnRCP Server，直接查看
Manifest、注册工具、Observation、Action、Policy，并发送原始协议请求。

它只用于本地开发、接入验证和问题排查；不替代 Teleop、MCP 或 RynnBot App。

## 启动

安装根包 `rynnrcp` 后会获得命令：

```bash
rynnrcp-protocol-debug --config <server-config.yaml>
```

Atom01 默认示例：

```bash
rynnrcp-protocol-debug --config robots/roboparty_atom01/rynnrcp_robot_atom01/config/atom01_server.yaml
```

默认 Web 地址：

```text
http://127.0.0.1:8091
```

常用参数：

```bash
rynnrcp-protocol-debug --host 127.0.0.1 --port 8091 --no-open
```

## 能做什么

- 启动 `--config` 指定的 Server，或连接本地 registry 里已经启动的 Server。
- 查看 `get_manifest`、`list_tools`、`list_observations`、`list_actions`、`list_policies` 返回。
- 轮询选中的 Observation。
- 对 Action 发送单帧 `run_action_chunk`。
- 用 Raw Request 调试任意协议方法和 payload。

## 边界

- 页面会直接调用真实 Server，Action 会真实下发到机器人或底层控制器。
- 真机调试前先确认机器人处于安全姿态，并优先使用小幅动作或只读 Observation。
- 协议字段以 [`../../docs/RCP使用场景及协议.md`](../../docs/RCP使用场景及协议.md) 为准。
