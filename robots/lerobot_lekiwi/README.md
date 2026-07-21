# LeKiwi

RynnRCP integration for a LeKiwi mobile manipulator. It controls a six-axis arm and three-wheel base through one 9D state and collects front and wrist camera images.

[中文](README.zh-CN.md)

## Install and configure

Use Python 3.10 and run the following commands in Bash:

```bash
cd robots/lerobot_lekiwi
bash setup_lekiwi.sh
```

The script prints the activation command for the current platform. Use `source venv/bin/activate` on macOS/Linux or `source venv/Scripts/activate` in Windows Git Bash, then run `rynnrcp-lekiwi-configure-web`.

The setup script installs Protocol Debug, MCP, RynnBot, and Teleop.

Use the web page to configure Robot IDs, both cameras, the follower port and calibration, and the leader port and calibration. When saved, the follower Robot ID, leader Robot ID, and RynnBot App ID receive the same stable eight-character suffix derived from this machine. The suffix remains stable on repeated runs on the same machine. The follower uses motor IDs 1–9; the leader uses IDs 1–6. During calibration, move every listed joint through its full range before saving.

## Decide which cloud fields to fill

| Workflow | Product Key / Device Name / Device Secret |
| --- | --- |
| Local leader-follower Teleop and collection | Keep the Robot IDs and continue device setup |
| MCP / Protocol Debug | Use the local Robot IDs |
| Connect the LeKiwi follower to RynnBot | Enter one follower-target credential set |

Follower and leader Robot IDs identify the two local RCP Servers; keep both values and make them different. App ID identifies the follower RynnBot App and is generated automatically. The leader connects through the local Teleop App, so the page provides the Product Key, Device Name, and Device Secret for the follower.

HTTP URL normally stays at its default. Image upload codec configures RynnBot cloud image uploads; local Teleop uses the camera output directly.

Raise all three wheels for the first base test and be ready to disconnect power. The configuration page should detect every motor and continuously display both `640 × 360` camera streams.

## Local leader-follower Teleop and collection

Run the following commands in terminals where the LeKiwi environment is active. In each new terminal, enter `robots/lerobot_lekiwi` and run the activation command for the current platform first.

Use three terminals. Terminal 1 runs on the LeKiwi board and may use SSH or a headless session:

```bash
# Terminal 1: LeKiwi board
rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
```

Start the leader Server in terminal 2 on the desktop laptop connected to the leader arm:

```bash
# Terminal 2: laptop connected to the leader arm
rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_leader_server.yaml
```

After both Servers print their ready messages, start the Teleop App in terminal 3 on the laptop:

```bash
# Terminal 3: laptop connected to the leader arm
rynnrcp-teleop-app
```

Each Server prints its own `Debug UI` address. Open that address to inspect joint state, Action, camera images, and live charts. If the default port `8092` is occupied, use the fallback address printed by that Server. The page polls robot data while open; close it after inspection.

In the Teleop page, select `lekiwi_leader` as the control device, `lekiwi` as the controlled device, and map:

```text
observation.robot.joint_state -> action.robot.joint_position
```

The leader Server uses `pynput` for global keyboard events. Use a Windows or macOS desktop session, or Linux X11/Xorg. On macOS, grant Input Monitoring permission to the terminal. Run the follower Server in Linux SSH, headless, or Wayland sessions that intercept global key capture.

| Keys | Control |
| --- | --- |
| `W/S` | Forward/backward |
| `A/D` | Left/right |
| `Q/E` | Turn left/right |
| `[/]` | Decrease/increase speed level |
| `Space` | Output zero velocity |

## Connect the follower body to RynnBot

Enter one follower-target credential set in the configuration page. Use two terminals: start the follower Server first, then start the RynnBot App.

```bash
# Terminal 1: LeKiwi board
rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml

# Terminal 2: LeKiwi board
rynnrcp-rynnbot-app --config rynnrcp_robot_lekiwi/config/lekiwi_rynnbot_app.yaml --server-config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
```

The RynnBot App publishes the follower Server's 9D state, two camera streams, and Action.

## Protocol Debug

Protocol Debug starts or connects to the follower Server selected by the LeKiwi configuration and opens the browser debugger:

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
```

## MCP

The MCP App connects to an active follower Server, so use two terminals on the same machine:

```bash
# Terminal 1: LeKiwi board
rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml

# Terminal 2: LeKiwi board
rynnrcp-mcp-app --server-config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
```

## Data

| Object | Rate | Content |
| --- | --- | --- |
| `observation.robot.joint_state` | 60 Hz | Actual follower 9D state |
| `action.robot.joint_position` | 60 Hz | Leader-arm and keyboard 9D command |
| `observation.front.image` | 30 Hz | `640 × 360` JPEG |
| `observation.wrist.image` | 30 Hz | `640 × 360` JPEG |

The 9D order is:

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper,
base_x_velocity, base_y_velocity, base_yaw_velocity
```

The first five values use radians, the gripper uses `0–1`, and the last three values use m/s, m/s, and rad/s. The base stops automatically after 0.5 seconds without a fresh command.

## Verify

- Moving the leader arm changes the first six action and observation values.
- Holding a base key changes the last three values; releasing it returns them to zero.
- Both camera streams update continuously at `640 × 360`.
- For motor, calibration, or camera errors, reopen the web configuration page and check the affected device.

## Configuration files

| Purpose | Configuration file |
| --- | --- |
| Follower Server | `lekiwi_server.yaml` |
| Leader Server | `lekiwi_leader_server.yaml` |
| Follower RynnBot App | `lekiwi_rynnbot_app.yaml` |
