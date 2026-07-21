# SO101 Debugging Reference

Use this guide to diagnose serial, calibration, camera, Teleop, and motion-response issues on single-arm and dual-arm SO101 hardware. See the [README](README.md) for installation, configuration, and startup.

[简体中文](DEBUGGING.zh-CN.md)

## Identify the failing component

1. Open the `Debug UI` address printed by the Server terminal.
2. Check `health`, joint state, and whether every camera keeps updating.
3. Find the first `ERROR` or repeating `WARNING` in the terminal log.
4. For dual-arm hardware, `details.side` identifies the `left` or `right` side.

After configuration, start only the follower for diagnosis:

```bash
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml
# or
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

## Serial connection failures

Rescan serial devices in the configuration page and assign a different device to every arm. Dual-arm use requires four unique serial ports.

On macOS, inspect serial-port ownership with:

```bash
lsof /dev/cu.usbmodem*
```

On Linux, inspect devices and ownership with:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
fuser /dev/ttyACM0
```

Resolve the issue in this order:

1. Stop Servers, calibration tools, or serial terminals using the same port.
2. Reconnect the device and refresh the configuration page.
3. Bind the left/right and leader/follower roles again.
4. Save the configuration and restart the Server.

If left and right motion are reversed, swap the left/right serial bindings and repeat a small motion test.

## Calibration failures

During calibration, the configuration tool must have exclusive access to the serial port. Stop the Server connected to that arm before calibration.

Dual-arm use stores four independent calibrations:

- Left follower
- Right follower
- Left leader
- Right leader

For each arm:

1. Confirm that the page shows the correct port and role.
2. Connect and calibrate only that arm.
3. Save the result before moving to the next arm.
4. Run a small motion test after all arms are complete.

If one side moves in the wrong direction, select that side's port and calibrate it again. Save different robot IDs for the left and right arms.

## Camera open or frame-read failures

Rescan and preview cameras in the configuration page. Bind a different camera index to every position:

- Single: `front`, `wrist`
- Dual: `front`, `left_wrist`, `right_wrist`

If the log repeatedly shows:

```text
<device_id>: Failed to read a frame
reopening camera after 5 failed reads
```

`<device_id>` is the failing camera index. When the third camera fails while the first two continue streaming, shared USB Hub bandwidth or power is the usual cause.

Resolve it in this order:

1. Move at least one camera to an independent host USB port.
2. Use an externally powered Hub, and avoid placing three cameras plus four arms on one passive Hub.
3. Disconnect one camera temporarily and confirm that the other two stream reliably.
4. Rescan and save camera bindings, then restart the follower Server.

`encoding: jpg` controls the published image payload. USB transport compression depends on the capture backend, so verify USB bandwidth and power separately.

If wrist images are reversed, rebind `left_wrist` and `right_wrist`, save, and restart the follower Server.

## Only one side follows during Teleop

Confirm that the follower and leader use the same configuration:

```bash
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_leader_server.yaml
rynnrcp-teleop-app
```

In Debug UI, confirm:

- Follower and leader joint states both contain 12 values.
- Action also contains 12 values ordered as left 6 + right 6.
- `health` contains no per-side connection error.
- Leader serial bindings are unique and assigned to the correct side.

Use hold-current or small joint-space actions for the first hardware check, and confirm that robot state and camera streams keep updating.

## Slow motion response or reduced tracking amplitude

Use the motor response test to bypass Teleop and RCP and measure one arm directly:

```bash
python -m lerobot_so101.motor_response_test \
  --port /dev/cu.usbmodem-follower \
  --joint shoulder_lift \
  --amplitude 0.20 \
  --frequency 0.5 \
  --duration 8 \
  --control-hz 60
```

The script writes CSV/SVG output and prints a summary. Check:

- `lag_ms`: measured position lag relative to the target.
- `amplitude_ratio`: measured motion amplitude divided by target amplitude.
- `send_p95_ms` and `read_p95_ms`: send and read call latency.

Enable the controller action/state trace when deeper inspection is needed:

```bash
export RYNNRCP_SO101_TRACE_POSITIONS=1
export RYNNRCP_SO101_TRACE_DIR=/tmp/so101_trace
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

When the follower Server stops, it writes:

- `so101_trace_*.csv`: `action`, `sent`, and `state` samples.
- `so101_trace_*.timing.csv`: `read_state`, `send_action`, and `worker_tick` timing.
- `so101_trace_*.svg`: target, sent, and measured-state curves.

Control-loop diagnostic settings:

```bash
export RYNNRCP_SO101_CONTROL_LOOP_HZ=100
export RYNNRCP_SO101_STATE_READ_HZ=60
export RYNNRCP_SO101_MAX_JOINT_VELOCITY_RAD_S=30
export RYNNRCP_SO101_MAX_GRIPPER_VELOCITY_PER_S=30
```

Validate with small motions before changing limits on the full workspace.

## RynnBot connection failures

Choose credentials by identifying which robot body connects to RynnBot in this run. When the follower body connects and runs the App below, enter one `product_key`, `device_name`, and `device_secret` set under **Follower target**. Local Teleop, MCP, and Protocol Debug use the local Server configuration directly.

When a physical leader acts as the control source for a simulated follower through RynnBot, enter the **Leader controller** set. **Follower target** and **Leader controller** correspond to these two workflows.

Save, then run:

```bash
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
# or
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_bimanual_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

App ID is generated automatically, and HTTP URL normally stays at its default. Local leader-follower Teleop connects directly to the leader Server through its Robot ID.

## Controller conventions

- The first five joints of each arm use radians.
- The gripper uses a normalized `[0, 1]` ratio.
- Dual-arm state and action are always ordered as left 6 + right 6.
- Dual-arm health uses `details.side` to identify the failing side.

## Configuration and test reference

Configuration files live in `rynnrcp_robot_so101/config/`. For direct inspection, start with:

| File | Purpose |
| --- | --- |
| `robot_integration.yaml` | Single-arm 6-DoF integration |
| `robot_integration_bimanual.yaml` | Dual-arm 12-DoF integration |
| `so101_follower_server.yaml` | Single follower, serial port, and cameras |
| `so101_leader_server.yaml` | Single leader serial port |
| `so101_bimanual_follower_server.yaml` | Dual followers, left/right ports, and three cameras |
| `so101_bimanual_leader_server.yaml` | Dual leaders and left/right ports |

See [tests/README.md](../../tests/README.md) for framework-level test entry points.
