# Aero Hand

RynnRCP integration for Aero Hand single-hand and dual-hand configurations, including local camera-gesture Teleop and collection with a physical hand, plus camera-gesture control of a simulated target through RynnBot.

[简体中文](README.zh-CN.md)

## Decide which endpoint needs cloud credentials

Use one rule: **select the credential group that matches the current workflow.** The physical-target group connects the Aero Hand body to RynnBot; the vision-controller group controls a simulated target.

| Workflow | Product Key / Device Name / Device Secret sets |
| --- | --- |
| **Start gesture teleoperation** in the configuration page | 0 |
| Local Teleop and local collection | 0 |
| Connect the physical Aero Hand body to RynnBot | 1: physical target |
| Vision controller controls a simulated target through RynnBot | 1: vision controller |

The **vision controller** is an RCP Server that reads the selected camera, recognizes hand gestures, and publishes joint commands. Select the camera under **Gesture camera and collection image** in the configuration page.

## Requirements

- Python 3
- Accessible Aero Hand serial devices
- An accessible camera for vision-controller workflows

## Install

Enter the Aero Hand directory, then install and activate the environment:

```bash
cd robots/tetheria_aerohand
bash setup_aero_hand.sh
source venv/bin/activate
rynnrcp-aero-hand-configure
```

The setup script installs RynnBot, MCP, Protocol Debug, and Teleop.

In every later terminal, enter the Aero Hand directory and activate the environment first:

```bash
cd robots/tetheria_aerohand
source venv/bin/activate
```

## Start the target hand

Start one physical-target Server configuration. Single hand:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

Dual hand:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

Each Server prints a `Debug UI` address after startup. Open that address to inspect state, Actions, camera images, or live charts. If the port is occupied, use the fallback address printed by that Server terminal.

## Protocol Debug

Protocol Debug starts or connects to the physical-target Server selected by its configuration.

### Single-hand Protocol Debug

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

### Dual-hand Protocol Debug

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

## MCP

The MCP App connects to an active physical-target Server, so each configuration uses two terminals.

### Single-hand MCP

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml

# Terminal 2
rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

### Dual-hand MCP

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml

# Terminal 2
rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

## Camera-gesture teleoperation

Run `rynnrcp-aero-hand-configure`, select the single- or dual-hand configuration, then scan, preview, and save a camera. Single-hand mode automatically follows either visible hand. Dual-hand mode detects both hands and always publishes `left 7 + right 7` joint values.

### Local single-hand Teleop and collection

Open three terminals. In each terminal, enter `robots/tetheria_aerohand` and activate the same virtual environment.

Terminal 1, start the single-hand vision controller:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
```

Terminal 2, start the single-hand target:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

Terminal 3, start Teleop:

```bash
rynnrcp-teleop-app
```

Bind `observation.robot.joint_state` to `action.robot.joint_position` in Teleop. Start teleoperation first, then start and stop collection from the same page.

### Local dual-hand Teleop and collection

Use the following command in terminal 1:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
```

Use the following command in terminal 2:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

Use the following command in terminal 3:

```bash
rynnrcp-teleop-app
```

Bind the same observation and action. The vision controller publishes its first complete state after both hands have been detected. If one hand briefly leaves the frame, its latest valid pose is retained.

With one physical camera, the vision controller and target share the same-machine frame. Either Server may therefore start first. The target still publishes that frame as `observation.front.image`.

### Connect to RynnBot

Select one configuration for the current workflow:

1. **Physical target**: connect the Aero Hand body to RynnBot to upload state and receive cloud actions.
2. **Vision controller**: connect the camera-gesture service to RynnBot to control a simulated target.

Each set contains Product Key, Device Name, and Device Secret. Keep HTTP URL at its default unless your deployment supplies another endpoint. App ID is generated automatically. Local Teleop and local collection use the local Server configuration directly.

The configuration page writes credentials to the matching YAML on the current machine. Keep `YOUR_*` placeholders in files you publish or share.

To connect the physical target, select the single- or dual-hand configuration and start its Server and RynnBot App in two terminals.

#### Single-hand physical target on RynnBot

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml

# Terminal 2
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

#### Dual-hand physical target on RynnBot

```bash
# Terminal 1
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml

# Terminal 2
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

#### Single-hand vision controller for a simulated target

Use two terminals. Start the controller Server first and wait until its terminal reports that the Server is ready and prints the `Debug UI` address:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
```

Then start its RynnBot App:

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
```

#### Dual-hand vision controller for a simulated target

Use two terminals. Start the dual-hand controller Server first:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
```

After it is ready, start its RynnBot App:

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
```

If the App reports `robot_id ... was not found`, its Server is not ready or a different configuration is running. Keep the matching Server terminal running, wait for its ready message, then start the App again.

To verify gesture control first, click **Start gesture teleoperation** under **Hardware debugging**. The page renders the live camera image with detected hand landmarks. Stop teleoperation before running homing or debug poses.

## Capabilities

- Single-hand and dual-hand joint state
- Joint target control
- MCP, RynnBot, Protocol Debug, and Teleop integration
- Camera-gesture controller for local collection and simulated-target control through RynnBot

Configure the serial ports before starting the Server. See the
[debugging reference](DEBUGGING.md) for device mapping and dual-hand
details.

## Configuration files

| Configuration | Physical-target Server | Vision-controller Server | Physical-target RynnBot App | Vision-controller RynnBot App |
| --- | --- | --- | --- | --- |
| Single hand | `aero_hand_single_server.yaml` | `aero_hand_single_hand_master_server.yaml` | `aero_hand_single_rynnbot_app.yaml` | `aero_hand_single_hand_master_rynnbot_app.yaml` |
| Dual hand | `aero_hand_dual_server.yaml` | `aero_hand_dual_hand_master_server.yaml` | `aero_hand_dual_rynnbot_app.yaml` | `aero_hand_dual_hand_master_rynnbot_app.yaml` |
