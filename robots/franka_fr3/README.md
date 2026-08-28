# Franka FR3

RynnRCP hardware integration for a Franka Research 3 with the original Franka Hand. Control data uses one eight-value joint-position vector ordered as `[7 arm joints, gripper]`.

[简体中文](README.zh-CN.md)

## Quick start

Prepare Ubuntu with Python 3.10–3.12, unlock the robot in Franka Desk, and activate FCI. From the repository root, run:

```bash
cd robots/franka_fr3
bash setup_franka_fr3.sh
source venv/bin/activate
rynnrcp-franka-fr3-configure
```

The setup script installs native build dependencies, downloads and builds official `libfranka 0.13.3` and `Ruckig 0.15.3`, installs the RealSense Python driver, and installs RynnRCP with the Franka FR3 package. Add `--skip-apt` when system dependencies are ready. Use `--libfranka-version` when the Robot System Version requires another compatible release.

The configuration page opens automatically. Complete these steps:

1. Enter the robot IP.
2. Save the configuration and connect.
3. Confirm that seven arm joints and one gripper state are displayed.
4. Clear the workspace and confirm the emergency stop.
5. Click **Go Home** and confirm that the robot safely reaches the configured Home.
6. Drag one joint slider for a single-joint test.
7. At a site-approved safe pose, save the current state as Home.
8. The page automatically opens a low-bandwidth live preview for every connected RealSense. Under each preview, bind the device to `cam_arm`, `cam_main`, or `cam_side`, then save the configuration.

Joint sliders use radians and the gripper slider uses `[0, 1]`. The page continuously sends the latest joint target and updates state at a target rate of 60 Hz. It coalesces drag events and keeps at most one HTTP request in flight. One native loop owns all arm state and control traffic: its 1 kHz libfranka callback caches measured state and uses Ruckig to generate commands within the configured velocity, acceleration, and jerk limits. A separate background worker refreshes the gripper cache, so the eight-value `joint_state` combines cached arm and gripper data without allowing gripper network reads to reduce the 60 Hz observation rate. Action requests only update control targets.

The default joint limits are `0.25 rad/s` velocity, `0.5 rad/s²` acceleration, and `2.5 rad/s³` jerk. Change the corresponding Server YAML values when different limits are required.

## Home position

The default Home is the initial pose commonly used by libfranka examples:

```text
[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```

All seven values are radians. Home moves the arm while the gripper remains independently controlled. Treat this as the initial configuration value. On first use, verify workspace, payload, and mounting direction, then save an approved current pose as the machine's Home.

The configuration is stored in:

```text
rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

The three cameras are disabled by default. The configuration page previews every connected device at 640 × 480 and 15 FPS so the views can be identified visually; each role can be bound to only one device. After saving, the Server enables the bound cameras and publishes 1280 × 720 JPEG images at 30 FPS. Before starting the Server, press `Ctrl+C` in the configuration-page terminal so its previews release the RealSense devices.

## Start the Server

Enter the package directory and activate its environment in each terminal:

```bash
cd robots/franka_fr3
source venv/bin/activate
```

Start the Server:

```bash
rynnrcp-server --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

Once connected, the native control loop holds the measured startup pose until it receives a target. Open the `Debug UI` address printed by the Server to inspect state and send actions. Keep the emergency stop reachable, confirm the workspace is clear, and press `Ctrl+C` in the Server terminal to stop it.

## Start an App

Keep the Server running. In a new terminal, enter `robots/franka_fr3`, activate `venv`, and start the app for the task.

Inspect protocol state and send actions manually:

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

Open the teleoperation and data-collection UI:

```bash
rynnrcp-teleop-app
```

For a Meta Quest 3 right-controller leader, start its Server in another terminal:

```bash
rynnrcp-server \
  --config rynnrcp_robot_meta_quest3/config/meta_quest3_franka_fr3_right_server.yaml
```

In Teleop, map the `meta_quest3_franka_fr3_right` `joint_state` to the `franka_fr3` `joint_position` action. This vector always contains `[7 arm joints, gripper]` as eight values. After A calibration, the first joint target is the configured Home. Hold grip to operate after the FR3 reaches Home.

Start the MCP bridge:

```bash
rynnrcp-mcp-app --server-config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

Before connecting RynnBot, fill in the device credentials in `franka_fr3_rynnbot_app.yaml`, then run:

```bash
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_franka_fr3/config/franka_fr3_rynnbot_app.yaml \
  --server-config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
```

## Interface

- `observation.robot.joint_state`: publish eight cached values at 60 Hz, ordered as seven arm joints followed by normalized gripper opening.
- `observation.cam_arm.image`: RealSense color image for the arm view.
- `observation.cam_main.image`: RealSense color image for the main view.
- `observation.cam_side.image`: RealSense color image for the side view.
- `action.robot.joint_position`: accept `[7 joint targets in radians, gripper 0–1]` as eight values at 60 Hz.
- `action.robot.home`: use the configured `home_joint_positions`.

## Communication checks

The configuration page reads arm and gripper state when it connects. For communication diagnostics, run:

```bash
rynnrcp-franka-fr3-check --robot-ip 192.168.0.110
.deps/libfranka-build/examples/communication_test 192.168.0.110
```

Use a Franka-supported real-time kernel for motion control and enable real-time enforcement on the configuration page.

When the target stream is silent longer than `target_timeout_s`, the control loop cancels the unfinished target and uses Ruckig to decelerate smoothly into a joint-position hold. The loop remains ready for the next target. The `franka.target_timeout` health warning reports the interrupted target stream.
