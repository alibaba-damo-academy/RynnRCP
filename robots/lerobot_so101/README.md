# LeRobot SO101

RynnRCP hardware integration for single-arm and dual-arm SO101 systems, including USB cameras, leader-follower teleoperation, motion tests, data collection, MCP, and RynnBot.

[简体中文](README.zh-CN.md)

## Choose a configuration

| Configuration | Arms | Cameras | State and action |
| --- | --- | --- | --- |
| `single` | One follower + one leader | `front` + `wrist` | 6 values |
| `dual` | Left/right followers and leaders | `front` + `left_wrist` + `right_wrist` | 12 values, left 6 + right 6 |

The dual configuration appears as one robot in RynnRCP. The left leader controls the left follower, and the right leader controls the right follower.

## Quick start

Install Python 3.10 and connect the arms and cameras you plan to use. From the repository root:

```bash
cd robots/lerobot_so101
bash setup_so101.sh
source venv/bin/activate
rynnrcp-so101-configure
```

On Windows, run setup in Git Bash. Activate with `source venv/Scripts/activate` in Git Bash or `.\venv\Scripts\Activate.ps1` in PowerShell. In every later terminal, enter `robots/lerobot_so101` and activate the virtual environment first.

The configuration page opens automatically. Complete these steps in order:

1. Select the single-arm or dual-arm configuration.
2. Bind each camera.
3. Assign a different serial port to every arm.
4. Calibrate every arm shown on the page.
5. Verify joint directions with a small motion test.
6. Save the configuration.

Dual-arm use requires four serial ports and four independent calibrations: left follower, right follower, left leader, and right leader.

## Decide which RynnBot credentials to enter

Use one rule: **identify which robot body connects to RynnBot in this run, and enter that endpoint's credential set.** Use **Follower target** when the follower body connects to the cloud, and **Leader controller** when a physical leader controls a simulated follower.

| Workflow | Product Key / Device Name / Device Secret sets |
| --- | --- |
| Motion tests in the configuration page | 0 |
| Local leader-follower Teleop | 0 |
| MCP / Protocol Debug | 0 |
| Follower robot body connects to RynnBot | 1: follower target only |
| Physical leader controls a simulated follower through RynnBot | 1: leader controller |

The **leader controller credentials** register the leader Server as a RynnBot controller. App IDs are generated automatically, and HTTP URLs normally stay at their defaults.

## Prepare each terminal

Before running the commands below, enter the SO101 directory and activate the environment in every terminal:

```bash
cd robots/lerobot_so101
source venv/bin/activate
```

Use `source venv/Scripts/activate` in Windows Git Bash or `.\venv\Scripts\Activate.ps1` in PowerShell.

Each Server prints a `Debug UI` address after startup. Open that address to inspect state, Actions, camera images, or live charts. If the default port is occupied, use the fallback address printed by that Server terminal.

## Local leader-follower Teleop and collection

### Single-arm local Teleop

Use three terminals. Start both Servers first, wait for their ready messages, then start the Teleop App.

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml

# Terminal 2
rynnrcp-server --config rynnrcp_robot_so101/config/so101_leader_server.yaml

# Terminal 3
rynnrcp-teleop-app
```

### Dual-arm local Teleop

Use three terminals:

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml

# Terminal 2
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_leader_server.yaml

# Terminal 3
rynnrcp-teleop-app
```

Open the Web UI address printed by the Teleop terminal, select the matching leader and follower, and start teleoperation. Start and stop data collection from the same page. Press `Ctrl+C` in a terminal to stop its process.

## Connect the follower body to RynnBot

Select the matching configuration in the configuration page and enter the **Follower target** credentials. Each configuration uses two terminals: start the follower Server, wait for its ready message, then start the RynnBot App.

### Single-arm follower on RynnBot

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml

# Terminal 2
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

### Dual-arm follower on RynnBot

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml

# Terminal 2
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_bimanual_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

## Use the leader as a RynnBot controller for simulation

Select the matching configuration in the configuration page and enter the **Leader controller** credentials. The simulation environment starts the simulated follower. On the computer connected to the physical leader, start the leader Server and then its controller RynnBot App.

### Single-arm leader controls a simulated follower

```bash
# Terminal 1: start the 6-value single-arm leader Server
rynnrcp-server --config rynnrcp_robot_so101/config/so101_leader_server.yaml

# Terminal 2: register the leader as a RynnBot controller
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_master_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_leader_server.yaml
```

Select the 6-value SO101 interface in the simulation task. Leader motion is read from `observation.robot.joint_state` and sent as cloud `action`.

### Dual-arm leader controls a simulated follower

```bash
# Terminal 1: start the 12-value dual-arm leader Server
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_leader_server.yaml

# Terminal 2: register the dual-arm leader as a RynnBot controller
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_bimanual_master_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_bimanual_leader_server.yaml
```

Select the 12-value dual-arm interface in the simulation task. Action order is left six values followed by right six values.

## Protocol Debug

Protocol Debug starts or connects to the follower Server selected by its configuration and opens the browser debugger.

### Single-arm Protocol Debug

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

### Dual-arm Protocol Debug

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

## MCP

The MCP App connects to an active follower Server, so each configuration uses two terminals.

### Single-arm MCP

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml

# Terminal 2
rynnrcp-mcp-app --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

### Dual-arm MCP

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml

# Terminal 2
rynnrcp-mcp-app --server-config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml
```

## Verify the setup

Use the follower Server `Debug UI` to confirm:

- Single-arm state and action contain 6 values; dual-arm state and action contain 12.
- Single-arm cameras show `front` and `wrist`; dual-arm cameras show `front`, `left_wrist`, and `right_wrist`.
- In dual-arm Teleop, each leader drives the matching follower.

## Dual-arm data contract

```json
{
  "n_dof": 12,
  "task_keys": [
    "observation.state",
    "observation.images.front",
    "observation.images.left_wrist",
    "observation.images.right_wrist",
    "action"
  ]
}
```

Both `observation.state` and `action` place the left six values before the right six values. The first five joints of each arm use radians; each gripper uses a normalized `[0, 1]` ratio.

## Configuration files

Configuration files live in `rynnrcp_robot_so101/config/`:

| Configuration | Follower Server | Leader Server | Follower RynnBot App | Leader-controller RynnBot App |
| --- | --- | --- | --- | --- |
| Single | `so101_follower_server.yaml` | `so101_leader_server.yaml` | `so101_rynnbot_app.yaml` | `so101_master_rynnbot_app.yaml` |
| Dual | `so101_bimanual_follower_server.yaml` | `so101_bimanual_leader_server.yaml` | `so101_bimanual_rynnbot_app.yaml` | `so101_bimanual_master_rynnbot_app.yaml` |

Run `rynnrcp-so101-configure` again after changing serial ports, cameras, calibration, or credentials.

See the [debugging reference](DEBUGGING.md) for serial ownership, calibration failures, camera read errors, and motion tracking.
