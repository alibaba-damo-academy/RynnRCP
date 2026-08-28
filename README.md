# RynnRCP

[简体中文](README.zh-CN.md)

RynnRCP is a Robotics Capability Protocol (RCP) for robot capability and
experience loops in embodied-agent systems. This repository contains the
standard Python implementation of that protocol.

RCP standardizes how robots expose Manifest, Observation, Action, Policy,
Health, Resource, and Data Collection objects. Apps, agents, MCP tools, and
cloud systems work through those protocol objects instead of per-robot SDK,
serial port, camera ID, or internal channel details.

In short:

> MCP defines how agents call tools. RCP defines how robot execution becomes
> data, how local policies become reusable skills, and how those skills can be
> called safely by agents.

ROS, robot SDKs, and low-level controllers stay responsible for hard real-time
control, interpolation, force control, emergency handling, and safety limits.

## What This Repository Provides

This repository is a concrete implementation of the protocol:

- a Python runtime that loads robot configs and runs standard robot services;
- standalone apps for Teleop, MCP, RynnBot cloud access, and protocol debugging,
  with shared recording, encoding, Resource transfer, and export helpers;
- robot integration packages, including SO101, Aero Hand, Atom01, Booster T1,
  Noetix Bumi, and Isaac Sim robots;
- documentation for App integration, robot integration, policy service, protocol
  objects, and config writing;
- tests for the implementation boundaries.

## Who It Is For

- Robot integrators expose real robots, cameras, ROS2/LCM systems, or Python
  SDKs as RCP Servers.
- Agent or application developers call robots through a unified Interface
  instead of per-robot SDK details.
- Data and training users collect episodes, replay runs, export datasets, and
  keep intervention records.
- Cloud integrators connect cloud workflows to an RCP Server through the
  RynnBot App.

## Start Here

RynnRCP Runtime, official apps, and robot packages are maintained in this
repository. When using a specific robot package, start from that package README
and use the setup script it provides.

| Goal | Start with |
| --- | --- |
| Use SO101 | [robots/lerobot_so101/README.md](robots/lerobot_so101/README.md) |
| Use Aero Hand | [robots/tetheria_aerohand/README.md](robots/tetheria_aerohand/README.md) |
| Use Atom01 | [robots/roboparty_atom01/README.md](robots/roboparty_atom01/README.md) |
| Use Booster T1 | [robots/booster_t1/README.zh-CN.md](robots/booster_t1/README.zh-CN.md) |
| Use Noetix Bumi | [robots/noetix_bumi/README.zh-CN.md](robots/noetix_bumi/README.zh-CN.md) |
| Use Franka Research 3 | [robots/franka_fr3/README.md](robots/franka_fr3/README.md) |
| Use an Isaac Sim robot | [robots/sim_robot/README.zh-CN.md](robots/sim_robot/README.zh-CN.md) |
| Use Teleop / MCP / RynnBot | [apps/README.zh-CN.md](apps/README.zh-CN.md), then the app README |
| Inspect one Server protocol | [apps/protocol_debug/README.zh-CN.md](apps/protocol_debug/README.zh-CN.md) |
| Build a new App | [docs/RCP App 接入 Server 指南.html](docs/RCP%20App%20接入%20Server%20指南.html) |
| Add a new robot package | [docs/RCP 机器人构型接入指南.html](docs/RCP%20机器人构型接入指南.html) |
| Add local policies | [docs/RCP 策略服务接入指南.html](docs/RCP%20策略服务接入指南.html) |
| Check protocol objects and methods | [docs/RCP使用场景及协议.md](docs/RCP使用场景及协议.md) |
| Check config fields | [docs/RCP配置文件说明.md](docs/RCP配置文件说明.md) |
| Trace logs and troubleshoot a run | [docs/RCP日志与排障.md](docs/RCP日志与排障.md) |
| Benchmark live-device latency and resource load | [tests/benchmarks/live_device/README.zh-CN.md](tests/benchmarks/live_device/README.zh-CN.md) |
| Choose a document | [docs/README.zh-CN.md](docs/README.zh-CN.md) |

When adding a robot, first verify the robot's native SDK, ROS2/LCM path, serial
tool, or camera tool with minimal hardware behavior. Then wrap those working
abilities as controller/source definitions and write the RCP config.

## Server And App

RynnRCP keeps the runtime boundary simple:

- Server: runs near the robot capability, loads robot config, connects
  controllers, cameras, ROS2/LCM, or Python SDKs, and exposes protocol tools.
- App: runs on the user or cloud side, calls a Server through RCP Interface, and
  does not touch robot SDKs, serial ports, camera IDs, or internal channels.

```text
App / Agent / MCP / Cloud
        |
        v
RCP Interface
        |
        v
RCP Server
        |
        v
controller / camera / ROS2 / LCM / SDK
```

## Install Runtime And Official Apps

Run the commands below from the repository root.

Create and activate a Python environment:

```bash
python -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
```

Install the RynnRCP implementation and official apps:

```bash
python -m pip install -e . -e apps/common -e apps/protocol_debug -e apps/mcp -e apps/rynnbot -e apps/teleop
```

After installation, the current Python environment has these commands:

```bash
rynnrcp-server
rynnrcp-teleop-app
rynnrcp-mcp-app
rynnrcp-rynnbot-app
rynnrcp-protocol-debug
```

Use `-h` to inspect command arguments.

### Inspect Live Server State

`rynnrcp-server` starts a built-in read-only status page without opening a browser. After the Server is ready, open the `Debug UI` address printed in its terminal:

```text
Debug UI:   http://127.0.0.1:8092/
```

The page displays Observations/state, Actions, camera images, and live charts; it does not provide robot controls. The default port is `8092`. If that port is occupied, the Server logs a warning and selects the next available port, so use the address printed by the current process.

When no page is active, the Server does not perform additional state or image reads. Closing or hiding the page stops polling, and Action snapshot capture shuts down and clears its cache after about three seconds.

For a concrete robot package, prefer that package's setup script. SO101 uses
[`robots/lerobot_so101/setup_so101.sh`](robots/lerobot_so101/setup_so101.sh), and Aero Hand uses
[`robots/tetheria_aerohand/setup_aero_hand.sh`](robots/tetheria_aerohand/setup_aero_hand.sh).
Atom01 uses [`robots/roboparty_atom01/setup_atom01.sh`](robots/roboparty_atom01/setup_atom01.sh).
Other robot setup scripts are linked from [robots/README.md](robots/README.md).
These scripts install Runtime, official apps, and the robot package together.

## Included Packages

- `rynnrcp`: protocol runtime implementation.
- `rynnkit`: shared implementation helpers installed with `rynnrcp`.
- Python package `rynnrcp-app-common`: shared recording, encoding, Resource
  transfer, and export helpers; no standalone command.
- Python package `rynnrcp-app-mcp`: MCP-facing app that provides
  `rynnrcp-mcp-app`.
- Python package `rynnrcp-app-teleop`: teleoperation Web app that provides
  `rynnrcp-teleop-app`.
- Python package `rynnrcp-app-rynnbot`: RynnBot cloud workflow app that provides
  `rynnrcp-rynnbot-app`.
- `rynnrcp-protocol-debug`: browser protocol debug app installed with `rynnrcp`.
- Python package `rynnrcp-robot-so101`: SO101 robot integration package that
  provides `rynnrcp-so101-configure`.
- Python package `rynnrcp-robot-aero-hand`: Aero Hand integration package that
  provides `rynnrcp-aero-hand-configure`.
- Python package `rynnrcp-robot-atom01`: Atom01 humanoid integration package
  that provides `rynnrcp-atom01-configure` and `rynnrcp-atom01-test-policy`.
- Robot packages also include Booster T1, Noetix Bumi, and Isaac Sim
  integrations; see their package READMEs for hardware prerequisites and commands.

## Repository Layout

```text
pyproject.toml          RynnRCP implementation package metadata
README.zh-CN.md         Chinese README

rynnrcp/                protocol runtime implementation
  adapters/             payload adapters
  config/               config loading and expansion
  connectors/           protocol and module connectors
  interface/            gRPC transport and protocol client
  ipc/                  channels and shared-memory transport
  native/               native helpers
  process/              process lifecycle helpers
  protocol/             protocol method definitions
  robot/                robot controller abstractions
  runtime/              runtime orchestration
  services/             built-in services
  utils/                shared utilities

rynnkit/                helpers installed with RynnRCP
  cameras/              camera interfaces and USB camera support

apps/                   official standalone app packages
  common/               shared recording and export helpers
  mcp/                  MCP app
  protocol_debug/       browser protocol debug app
  rynnbot/              RynnBot cloud app
  teleop/               teleoperation Web app

robots/                 robot integration packages
  lerobot_so101/        SO101 integration
  lerobot_lekiwi/       LeKiwi integration
  tetheria_aerohand/    Aero Hand integration
  roboparty_atom01/     Atom01 integration
  booster_t1/           Booster T1 high-level and low-level integration
  noetix_bumi/          Noetix Bumi high-level and low-level integration
  sim_robot/            Isaac Sim integration

benchmarks/             reproducible live-device performance benchmarks
docs/                   long-lived implementation documentation
tests/                  implementation tests
```

## Testing

Run the implementation test suite:

```bash
python -m pytest tests -q
```

Hardware validation needs real devices and local machine configuration. Follow
the corresponding robot README for its runtime validation flow.

## License

RynnRCP is licensed under the Apache License 2.0 unless otherwise noted in a
file header or package README.

The Atom01 robot package includes GPL-3.0 C++ control binding sources under
`robots/roboparty_atom01/rynnrcp_robot_atom01/atom_control/`. Those files remain under
GPL-3.0 as marked by their SPDX headers.

Third-party code and asset notices are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Documentation

- [RCP protocol](docs/RCP使用场景及协议.md): external protocol contract,
  standard objects, tools, and data/resource flow.
- [Robot integration guide](docs/RCP%20机器人构型接入指南.html): how to
  add a robot package after validating hardware abilities.
- [Configuration guide](docs/RCP配置文件说明.md): config field reference for
  server config, robot integration, source, and codec.
- [Policy service guide](docs/RCP%20策略服务接入指南.html): how to add
  local policies and expose policy tools.
- [App integration guide](docs/RCP%20App%20接入%20Server%20指南.html):
  how apps call an RCP Server through Interface and Resource tools.
- [Apps README](apps/README.zh-CN.md): official app entry point.
- [Robots README](robots/README.md): robot package entry point.
- [SO101 README](robots/lerobot_so101/README.md): SO101 installation, configuration,
  runtime commands, and hardware notes.
- [Aero Hand README](robots/tetheria_aerohand/README.md): Aero Hand single/dual setup,
  camera-gesture Teleop collection, runtime commands, and RynnBot notes.
- [Atom01 README](robots/roboparty_atom01/README.md): Atom01 setup, zero calibration,
  runtime commands, and RynnBot notes.
- [Booster T1 README](robots/booster_t1/README.zh-CN.md): high-level control,
  low-level `LowCmd`, and local walk policy setup.
- [Protocol Debug README](apps/protocol_debug/README.zh-CN.md): browser UI for
  direct protocol inspection and raw requests.
- [Test guide](tests/README.md): test suite ownership and recommended commands.
