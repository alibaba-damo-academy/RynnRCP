# 测试套件说明

[English](README.md)

这个目录保存 RynnRCP Python 标准实现的测试套件。每个测试文件应尽量只覆盖一个
明确的归属范围；如果一个新测试跨多个子系统，优先写成 runtime 或 service 集成测试，
不要在多个低层单元测试里重复断言。

## 命令

运行框架实现测试：

```bash
venv/bin/python -m pytest tests -q
```

IPC 和进程相关测试会创建 POSIX shared memory 和 semaphore，需要在不受限的本地环境
运行。

## 核心测试归属

| 文件 | 覆盖范围 |
| --- | --- |
| `adapters/test_protocol_adapters.py` | 当前协议 input/image/action adapter 行为。 |
| `config/test_core_config.py` | 源配置加载器和校验覆盖。 |
| `config/test_runtime_config.py` | Runtime 直接从 server、core、integration 配置初始化。 |
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
| `services/test_policy_service.py` | 本地 policy 加载、运行时输入更新和 action 下发。 |
| `utils/test_core_utils.py` | import/path/timestamp 等小型通用工具。 |

## SO101 真机验证

SO101 真机验证不放在框架测试目录内。需要真实机械臂、相机、标定文件和网络环境时，
按 [`../robots/so101/README.zh-CN.md`](../robots/so101/README.zh-CN.md) 启动 follower /
leader Server，并用 Teleop、MCP 或 RynnBot App 验证实际工作流。
