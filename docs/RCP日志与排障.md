# RCP 日志与排障

RynnRCP 使用统一的终端和轮转文件日志。排查一次运行时，先找到日志中的
`run_id`，再用同一个 ID 检索 App、Server、子进程和仿真桥接日志。

## 日志位置

默认根目录是 `~/.rynnrcp`，可通过 `RYNNRCP_HOME` 修改。

| 组件 | 默认文件 |
| --- | --- |
| Server | `~/.rynnrcp/<robot_id>/logs/<session_id>/server.log` |
| Server 子进程 | `~/.rynnrcp/<robot_id>/logs/<session_id>/<process>.log` |
| ActionBridge | `~/.rynnrcp/<robot_id>/logs/action_bridge.log` |
| Teleop App | `~/.rynnrcp/apps/<app_id>/logs/teleop.log` |
| MCP App | `~/.rynnrcp/apps/<app_id>/logs/mcp.log` |
| RynnBot App | `~/.rynnrcp/apps/<app_id>/logs/rynnbot.log` |
| Protocol Debug App | `~/.rynnrcp/apps/protocol_debug/logs/protocol_debug.log` |
| Model Debug App | `~/.rynnrcp/apps/model_debug/logs/model_debug.log` |

文件默认按 50 MiB 轮转并保留 5 个备份。

## 关联同一次运行

Server 启动的子进程自动继承同一个 `run_id`。分别启动的 App、Server 和
ActionBridge 使用同一个 `RYNNRCP_RUN_ID` 即可关联。

```bash
export RYNNRCP_RUN_ID=fr3_sim_20260730_01

rynnrcp-server --config path/to/server.yaml
rynnrcp-teleop-app --config path/to/teleop.yaml
python robots/sim_robot/action_bridge.py --robot-id <robot_id>
```

每条日志包含时间、进程、线程、组件标签、logger 名称和上下文字段，例如：

```text
[TAG:SERVICE] [process=server robot_id=fr3_sim run_id=fr3_sim_20260730_01] ...
```

## 配置日志

| 环境变量 | 用途 | 默认值 |
| --- | --- | --- |
| `RYNNRCP_LOG_LEVEL` | `DEBUG`、`INFO`、`WARNING` 或 `ERROR` | `INFO` |
| `RYNNRCP_LOG_TO_FILE` | `1/true/on` 写文件，`0/false/off` 只写终端 | `true` |
| `RYNNRCP_LOG_MAX_BYTES` | 单个日志文件的最大字节数 | `52428800` |
| `RYNNRCP_LOG_MAX_FILES` | 轮转备份数量 | `5` |
| `RYNNRCP_RUN_ID` | 跨独立进程的运行关联 ID | 当前进程生成 |
| `RYNNRCP_HOME` | 运行数据和日志根目录 | `~/.rynnrcp` |

如果日志目录不存在，RynnRCP 会自动创建。目录无写权限或文件无法打开时，
进程不会因此退出，而是降级到终端日志并输出 `FILE_SINK_UNAVAILABLE` 告警。

具体 action 数值、payload 摘要和变化范围记录在 `DEBUG`；运行进度记录在
`INFO`；持续全零、维度变化和可恢复连接问题记录在 `WARNING`；无法继续
执行的故障记录在 `ERROR` 并附带异常堆栈。

## 按链路排查

使用同一个 `run_id` 依次检查：

1. App 是否发起 action 或 collection 请求。
2. Server 的 ActionService 是否接收并发布 action。
3. CollectionService 是否持续写入对应数据流。
4. Server 子进程是否正常读取控制器、相机或 IPC 数据。
5. 仿真场景继续检查 ActionBridge 和 SimZMQ 的 endpoint、robot name、
   接收频率、维度、全零帧和恢复日志。

高频故障采用“首次记录、定期汇总、恢复通知”的形式。看到 `recovered`
表示数据链路已经恢复；持续出现 `still failing` 时，使用同一条日志中的
组件、endpoint、channel 和 `run_id` 定位上游。

日志默认只记录 payload 类型、长度、shape 或脱敏摘要。需要查看 action
数值时将 `RYNNRCP_LOG_LEVEL` 设置为 `DEBUG`，完成排查后恢复为 `INFO`。

## 贡献者日志约定

- 模块使用 `logging.getLogger(__name__)`，由进程入口统一配置 handler。
- 运行时事件使用 `[Component][EVENT]`，并携带 action、channel、endpoint
  或 stream 等定位字段。
- `INFO` 记录生命周期和周期摘要，具体数值放在 `DEBUG`。
- 可恢复故障使用 `WARNING`，无法完成当前操作的故障使用 `ERROR`。
- 捕获未知异常时使用 `logger.exception()` 或 `exc_info=True` 保留堆栈。
- 轮询、设备、网络和 IPC 高频故障使用 `LogGate`，同时记录恢复事件。
- 配置、请求和 payload 使用 `redact()` 或 `describe_payload()`；日志不写入
  token、password、cookie、Authorization、原始图像和二进制数据。
- 面向终端用户的 CLI、配置向导和交互式标定可以使用 `print()`；后台线程、
  控制器、策略和库代码写入 logger。
