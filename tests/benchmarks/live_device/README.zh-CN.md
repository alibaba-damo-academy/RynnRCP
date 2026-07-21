# RynnRCP 实机性能测试

本测试脚本位于 `tests/benchmarks/live_device/`，用于在不同硬件平台上验证指定 RynnRCP 机器人构型。执行者只需提供 Server 配置；脚本自动识别该构型公布的机械臂、相机及其他 Observation，完成固定测试并生成可交付的结果目录。

使用前先完成对应机器人的安装、端口选择和标定，然后激活该机器人的 Python 环境。

## 执行测试

在 RynnRCP 仓库根目录执行一条命令。SO101 从臂示例：

```bash
python tests/benchmarks/live_device/live_benchmark.py \
  --config robots/lerobot_so101/rynnrcp_robot_so101/config/so101_follower_server.yaml
```

其他机器人只替换 `--config` 后的 Server 配置路径。脚本会自动选择机械臂、相机、频率、时长和输出目录。

测试默认持续约 16 分钟，包含 3 次、每次 300 秒的真实负载。终端打印结果目录后表示完成。

## 脚本自动完成的测试

1. 通过标准 `rynnrcp-server` 入口启动指定机器人构型。
2. 读取 Manifest，获取该构型公布的全部 Observation。
3. 逐一执行设备预检；机械臂或相机无数据时停止测试并记录错误。
4. 记录 30 秒 Server、Runner、Client 和整机空闲 CPU/内存基线。
5. 使用固定大小 Payload 执行最小 `ping/echo` 请求，单独测量 Interface 传输基线。
6. 对每个 Observation 预热 30 次、正式采样 300 次，计算 P50/P95/P99 调用延迟和数据鲜度。
7. 机械臂状态按最高 60 Hz、图像按最高 30 Hz 并发读取，且不超过 Manifest 公布的频率。
8. 持续负载 300 秒并重复 3 次，记录实际请求频率、有效数据频率、重复时间戳、失败率和超期率。
9. 汇总空闲与加载阶段的 CPU/RSS 及差值。
10. 停止 Server 和 Runner，回收 Channel 与共享内存。

自动测试不向真实机械臂发送 Action，避免在无人监督的跨平台测试中引发运动。

## 单独测试 Interface 传输性能

传输基线与 `grpc_latency_bench.py` 的固定字节测试思路一致，但直接使用当前实机 RCP Server 的持久连接和 `ping/echo` 方法，无需另外启动回显 Server。它分别统计客户端调用、请求编码、Interface 往返、响应解码、Server 请求解析和最小 Handler 的 P50/P95/P99。

只运行该基线时执行：

```bash
python tests/benchmarks/live_device/live_benchmark.py \
  --config robots/lerobot_so101/rynnrcp_robot_so101/config/so101_follower_server.yaml \
  --label rk3566_transport \
  --transport-only \
  --transport-sizes 0,493,32768,65536 \
  --transport-samples 1000 \
  --transport-warmup 100
```

`summary.json` 的 `transport_baseline` 字段保存汇总结果：`transport_round_trip_ms` 是固定 Payload 请求的基础往返时延，`server_handler_ms` 是最小回显处理时延，`wire_request_bytes` 和 `wire_response_bytes` 是实际编码规模。每次原始请求同时写入 `raw_samples.jsonl`，`record_type` 为 `transport_baseline`。

常规整机测试默认先执行 0 B、493 B、32 KiB 和 64 KiB 四组传输基线，随后继续 Observation 与资源测试。只运行 Observation 与资源测试时使用 `--skip-transport-baseline`。

## 结果目录

脚本在仓库根目录自动创建带时间戳的目录：

```text
benchmark_results/
└── so101_follower_server_20260716_193000/
    ├── summary.json
    ├── raw_samples.jsonl
    ├── resources.jsonl
    ├── environment.json
    ├── manifest.json
    ├── preflight.json
    ├── parameters.json
    └── server.log
```

测试完成后，将整个时间戳目录交付给报告整理人员。

- `summary.json`：可直接用于报告的 Interface 传输基线、Observation 时延、频率、失败率和资源汇总。
- `raw_samples.jsonl`：每次请求的原始时延和返回大小。
- `resources.jsonl`：各阶段的整机、Server、Runner 和 Client CPU/RSS 采样。
- `environment.json`：平台、CPU、内存、Python、依赖及 RynnRCP 代码修订号。
- `manifest.json`：测试时的真实本体能力。
- `preflight.json`：每个 Observation 的就绪状态。
- `server.log`：Server 与 Runner 完整运行日志。

测试失败时，结果目录中会生成 `failure.json`。根据其中的错误信息修复设备配置后，重新执行同一条命令。

## 排查选项

日常正式测试只使用 `--config`。以下参数用于定位设备或环境问题：

- `--list-devices`：打印 Manifest 中的 Observation 和 Action。
- `--observations`：只调试指定 Observation。
- `--endpoint`：连接已启动的 Server。
- `--output`：指定自定义结果目录。
- `--duration`、`--repeat`、`--samples`：缩短排查测试。

Server 进程树 CPU 在使用多个 CPU 核心时可超过 100%，该值表示 Server 与所有 Runner 进程 CPU 使用率之和。跨主机测试数据鲜度前，使用 NTP 或 PTP 同步两端时钟。
