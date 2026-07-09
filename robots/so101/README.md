# SO101 Robot Package

[简体中文](README.zh-CN.md)

This package is the SO101 robot integration for the RynnRCP Python
implementation. It maps the lightweight SO101 driver into the RCP device model:
robot state, camera observations, action execution, health/status, recording,
teleoperation, MCP access, and RynnBot cloud workflows.

The package does not replace the robot-side controller or hardware safety
layer. Calibration, low-level execution, interpolation, limits, and emergency
handling remain the responsibility of the SO101 driver and the physical robot
setup.

## What It Provides

- SO101 controller wrapper for follower/leader arms.
- A long-running RynnRCP server config plus standalone MCP, Teleop,
  and RynnBot app configs.
- A browser-based configuration helper for serial ports, cameras, calibration,
  and Server/App settings.
- Console commands for starting the SO101 server and apps.

## Installation

The setup script uses Python 3.10. On Windows, run it from Git Bash or WSL.
From the SO101 package directory:

```bash
cd robots/so101
./setup_so101.sh
source venv/bin/activate
```

The script creates `robots/so101/venv`, installs RynnRCP as a local library,
installs the official apps, and installs the SO101 robot package in editable
mode. Most users should use this script instead of installing Runtime first and
then installing SO101 manually.

## Configuration

SO101 configs live under:

```text
robots/so101/rynnrcp_robot_so101/config/
```

Important files:

| File | Purpose |
| --- | --- |
| `robot_integration.yaml` | Integration definition maintained by the robot package. |
| `so101_follower_server.yaml` | Follower SO101 RynnRCP Server config with `manifest.robot_id`, arm serial port, and cameras. |
| `so101_leader_server.yaml` | Leader SO101 RynnRCP Server config with `manifest.robot_id` and arm serial port. |
| `so101_rynnbot_app.yaml` | RynnBot App cloud device credentials. |

Use the configuration helper to edit common hardware and Server/App settings:

```bash
rynnrcp-so101-configure
```

The helper supports serial port scanning, USB camera scan/preview, SO101
calibration, Server ID, RynnBot credentials, and config validation. By default it
opens `http://127.0.0.1:28401`; if the port is already in use, it prints the
actual selected address.

## Runtime Commands

Run these commands from the SO101 package directory after activating `venv`:

```bash
cd robots/so101
source venv/bin/activate
```

Choose the program by what you want to do:

| Goal | Start |
| --- | --- |
| Check follower state, cameras, actions, or serve MCP/RynnBot | Follower Server |
| Teleoperate the follower | Follower Server + Leader Server + Teleop App |
| Debug protocol tools through MCP | Follower Server + MCP App |
| Connect to RynnBot cloud | Follower Server + RynnBot App |
| Edit serial ports, cameras, calibration, or RynnBot credentials | Configuration helper |

Start the follower SO101 Server:

```bash
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

Start the leader SO101 Server:

```bash
rynnrcp-server --config rynnrcp_robot_so101/config/so101_leader_server.yaml
```

Start the Teleop App Web UI:

```bash
rynnrcp-teleop-app
```

Start the MCP App:

```bash
rynnrcp-mcp-app --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

Start the RynnBot App:

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_so101/config/so101_rynnbot_app.yaml --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

Open the configuration helper:

```bash
rynnrcp-so101-configure
```

The Teleop App starts the local Web UI. Open:

```text
http://<teleop_web_host>:28402
```

For a local two-terminal test, use:

```text
http://127.0.0.1:28402
```

For a custom SO101 server config, use:

```bash
rynnrcp-server --config <path-to-so101-server.yaml>
```

## Follower Response Note

The SO101 follower keeps the upstream LeRobot Feetech motor configuration. The
upstream implementation sets follower motor `P_Coefficient` to `16` to reduce
shakiness; the motor default is `32`. This package keeps that behavior instead
of tuning PID by default.

Direct motor tests show millisecond-level send/read time, but `115-140ms` state
lag and only `0.65-0.84` target amplitude tracking. The bottleneck is the
follower motor position-loop response, not gRPC, RCP, Python, or serial calls.

Use the direct response test to bypass Teleop/RCP and measure the hardware:

```bash
python -m lerobot_so101.motor_response_test \
  --port /dev/cu.usbmodem5AE70441561 \
  --joint shoulder_lift \
  --amplitude 0.20 \
  --frequency 0.5 \
  --duration 8 \
  --control-hz 60
```

The script writes CSV/SVG output and prints a summary. Check:

- `lag_ms`: state lag relative to the target.
- `amplitude_ratio`: measured motion amplitude divided by target amplitude.
- `send_p95_ms` and `read_p95_ms`: millisecond-level values mean the Python
  send/read calls are not the main bottleneck.

During Teleop, enable the controller action/state trace with environment
variables:

```bash
export RYNNRCP_SO101_TRACE_POSITIONS=1
export RYNNRCP_SO101_TRACE_DIR=/tmp/so101_trace

rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

When the follower Server stops, it writes:

- `so101_trace_*.csv`: position samples with `action`, `sent`, and `state`.
- `so101_trace_*.timing.csv`: `read_state`, `send_action`, and `worker_tick`
  timings.
- `so101_trace_*.svg`: plot output. Red is target `action`, green is actual
  `sent`, and blue is measured `state`.

For temporary control-loop debugging, these environment variables are also
available:

```bash
export RYNNRCP_SO101_CONTROL_LOOP_HZ=100
export RYNNRCP_SO101_STATE_READ_HZ=60
export RYNNRCP_SO101_MAX_JOINT_VELOCITY_RAD_S=30
export RYNNRCP_SO101_MAX_GRIPPER_VELOCITY_PER_S=30
```

## Controller Conventions

- The first five arm joints use radians internally.
- The gripper uses a normalized ratio in `[0, 1]`.
- Use small hold-current or small joint-space actions for first hardware checks.
- Confirm robot state and camera streams are fresh before sending motion
  commands.

## Tests

Framework-level tests are documented in [../../tests/README.md](../../tests/README.md).
SO101 hardware validation is performed by running the configuration helper,
starting the follower/leader Servers, and checking Teleop, MCP, or RynnBot
workflows on the real device.
