# Test Suite Map

[简体中文](README.zh-CN.md)

This directory holds the test suite for the RynnRCP Python implementation. Keep
each test file focused on one ownership area; if a new test spans multiple
subsystems, prefer a runtime or service integration test instead of duplicating
lower-level unit assertions.

## Commands

Run core tests:

```bash
venv/bin/python -m pytest tests -q
```

IPC and process tests create POSIX shared memory and semaphores, so they must run
outside restricted sandboxes.

## Core Test Ownership

| File | Scope |
| --- | --- |
| `adapters/test_protocol_adapters.py` | Current protocol input/image/action adapter behavior. |
| `config/test_core_config.py` | Source config loader and validator coverage. |
| `config/test_lekiwi_configure_web.py` | LeKiwi web configuration persistence and machine-specific IDs. |
| `config/test_runtime_config.py` | Runtime initialization directly from server, core, and integration configs. |
| `config/test_so101_configure_web.py` | SO101 web configuration persistence and machine-specific IDs. |
| `connectors/test_connector.py` | Connector factory and module/port connector behavior. |
| `connectors/test_ros2_connector_real.py` | Optional ROS2 connector smoke coverage. |
| `diagnostics/test_logging.py` | Logging setup and formatting. |
| `interface/test_interface_mock_server.py` | Protocol-level mock server over gRPC. |
| `interface/test_interface_server.py` | RynnRCP server and Interface dispatcher behavior. |
| `ipc/test_channel.py` | Channel manager and publisher/subscriber behavior. |
| `ipc/test_ring_buffer.py` | Shared-memory ring buffer correctness. |
| `ipc/test_transport.py` | Memory/SHM transport behavior and notifier semantics. |
| `kit/test_sensor_base.py` | Shared `rynnkit` sensor/camera base contracts. |
| `kit/test_sensor_camera.py` | USB camera backend, encoding, and lifecycle behavior. |
| `native/test_event.py` | Native/manual event wrapper behavior. |
| `native/test_native_config.py` | Native feature configuration toggles. |
| `native/test_notifier.py` | Cross-platform notifier abstraction. |
| `native/test_shm.py` | Native shared-memory wrapper behavior. |
| `process/test_process.py` | Process node, launcher, registry, and multiprocessing behavior. |
| `packaging/test_install_boundaries.py` | Package dependency boundaries and entry-point declarations. |
| `robot/test_base_controller.py` | Public robot controller interface contract. |
| `runtime/test_pipeline.py` | Scheduler/channel pipeline integration. |
| `runtime/test_scheduler.py` | Scheduler component timing and lifecycle. |
| `services/test_action_service_snapshot.py` | On-demand Action snapshots used by the read-only Debug UI. |
| `services/test_policy_service.py` | Local policy loading, runtime input updates, and action dispatch. |
| `utils/test_core_utils.py` | Small shared import/path/timestamp helpers. |
| `utils/test_device_identity.py` | Stable privacy-preserving suffixes derived for one host. |
| `visualization/test_visualization_server.py` | Embedded Debug UI lifecycle, idle behavior, and port fallback. |

## Live-device benchmark

The real-device benchmark is an operator-run integration test and is not collected automatically by pytest. From the repository root, follow [`benchmarks/live_device/README.zh-CN.md`](benchmarks/live_device/README.zh-CN.md) and run:

```bash
python tests/benchmarks/live_device/live_benchmark.py \
  --config robots/lerobot_so101/rynnrcp_robot_so101/config/so101_follower_server.yaml
```

The command prints the result directory when collection completes. Review `summary.json` for latency, frequency, and resource results; use `failure.json` when device validation stops early.

## SO101 hardware validation

SO101 hardware validation needs real arms, cameras, calibration files, and network credentials, so pytest does not run it automatically. Use the live-device benchmark above for repeatable measurements. Follow [`../robots/lerobot_so101/README.md`](../robots/lerobot_so101/README.md) to start follower and leader Servers and validate Teleop, MCP, or RynnBot workflows.
