# 测试套件说明

[English](README.md)

这个目录保存 RynnRCP Python 标准实现的测试套件。新增测试时，把用例放到最贴近行为边界的目录；跨多个子系统的用例放到 runtime 或 service 集成测试。

## 命令

运行框架实现测试：

```bash
venv/bin/python -m pytest tests -q
```

IPC 和进程相关测试会创建 POSIX shared memory 和 semaphore，建议在本地开发环境运行。

## 核心测试归属

| 文件 | 覆盖范围 |
| --- | --- |
| `adapters/test_protocol_adapters.py` | 当前协议 input/image/action adapter 行为。 |
| `config/test_core_config.py` | 源配置加载器和校验覆盖。 |
| `config/test_lekiwi_configure_web.py` | LeKiwi Web 配置保存和本机身份 ID。 |
| `config/test_runtime_config.py` | Runtime 直接从 server、core、integration 配置初始化。 |
| `config/test_so101_configure_web.py` | SO101 Web 配置保存和本机身份 ID。 |
| `connectors/test_connector.py` | Connector factory 和 module/port connector 行为。 |
| `connectors/test_ros2_connector_real.py` | 可选 ROS2 connector smoke 覆盖。 |
| `diagnostics/test_logging.py` | 日志初始化和格式。 |
| `interface/test_interface_mock_server.py` | 基于 gRPC 的协议级 mock server。 |
| `interface/test_interface_server.py` | RynnRCP server 和 Interface dispatcher 行为。 |
| `ipc/test_channel.py` | Channel manager 和 publisher/subscriber 行为。 |
| `ipc/test_ring_buffer.py` | Shared-memory ring buffer 正确性。 |
| `ipc/test_transport.py` | Memory/SHM transport 行为和 notifier 语义。 |
| `kit/test_sensor_base.py` | `rynnkit` sensor/camera 基础契约。 |
| `kit/test_sensor_camera.py` | USB camera backend、编码和生命周期行为。 |
| `native/test_event.py` | Native/manual event wrapper 行为。 |
| `native/test_native_config.py` | Native feature 配置开关。 |
| `native/test_notifier.py` | 跨平台 notifier 抽象。 |
| `native/test_shm.py` | Native shared-memory wrapper 行为。 |
| `process/test_process.py` | Process node、launcher、registry 和 multiprocessing 行为。 |
| `packaging/test_install_boundaries.py` | 包依赖边界和 entry-point 声明。 |
| `robot/test_base_controller.py` | 公共 robot controller 接口契约。 |
| `runtime/test_pipeline.py` | Scheduler/channel pipeline 集成。 |
| `runtime/test_scheduler.py` | Scheduler component timing 和生命周期。 |
| `services/test_action_service_snapshot.py` | 只读 Debug UI 按需获取最新 Action。 |
| `services/test_policy_service.py` | 本地 policy 加载、运行时输入更新和 action 下发。 |
| `utils/test_core_utils.py` | import/path/timestamp 等小型通用工具。 |
| `utils/test_device_identity.py` | 为同一台机器生成稳定且不暴露原始信息的后缀。 |
| `visualization/test_visualization_server.py` | 内置 Debug UI 生命周期、空闲行为和端口回退。 |

## 实机性能测试

实机性能测试由操作人员按需运行，不会被 pytest 自动采集。在仓库根目录阅读 [`benchmarks/live_device/README.zh-CN.md`](benchmarks/live_device/README.zh-CN.md)，然后执行：

```bash
python tests/benchmarks/live_device/live_benchmark.py \
  --config robots/lerobot_so101/rynnrcp_robot_so101/config/so101_follower_server.yaml
```

采集完成后，终端会打印结果目录。使用 `summary.json` 检查时延、频率和资源数据；设备预检中止时查看 `failure.json`。

## SO101 真机验证

SO101 真机验证需要机械臂、相机、标定文件和网络凭据，因此 pytest 不会自动执行。性能数据使用上面的实机性能测试采集；功能验收按 [`../robots/lerobot_so101/README.zh-CN.md`](../robots/lerobot_so101/README.zh-CN.md) 启动从臂和主臂 Server，并用 Teleop、MCP 或 RynnBot App 验证实际工作流。
