# MCP App

MCP App 把一个 RCP Server 暴露为 MCP 工具，方便 Agent 或调试工具按协议调用机器人。

它不直接读取机器人配置里的硬件信息，也不新增机器人能力；目标 Server 能暴露什么工具，
MCP App 就暴露什么工具。

## 启动

```bash
rynnrcp-mcp-app --server-config <server-config.yaml>
```

SO101 示例：

```bash
rynnrcp-mcp-app --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

`--server-config` 用来读取 `manifest.robot_id`，MCP App 会根据这个 robot id 连接已经启动的
RCP Server。运行 MCP App 前，请先启动对应 Server。

需要修改 MCP 服务监听地址、端口或 transport 时，可以使用 App 配置：

```bash
rynnrcp-mcp-app --server-config <server-config.yaml> --config <mcp-app.yaml>
```

默认 MCP 服务使用 `streamable-http`，监听 `0.0.0.0:28403`，路径为 `/mcp`。

## 暴露内容

MCP App 不新增机器人能力，只把 Server 已注册的协议工具暴露出去，例如：

- `get_manifest`
- `list_observations`
- `get_observations`
- `list_actions`
- `run_action_chunk`
- `stop_action`
- Policy 相关工具：`list_policies`、`start_policy`、`update_policy_inputs`、`stop_policy`；只有目标 Server 的 `manifest.capabilities.policy_service` 为 `true` 且已加载 policy 时才有实际策略可用
- `get_health`
- Resource 相关工具；只有目标 Server 的 `manifest.capabilities.resources` 为 `true` 时才可用于读取日志、模型、标定或采集文件

协议字段以 [`../../docs/RCP使用场景及协议.md`](../../docs/RCP使用场景及协议.md) 为准。
