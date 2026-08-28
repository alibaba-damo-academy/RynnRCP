# Astribot S1

[简体中文](README.zh-CN.md)

The Astribot S1 integration for RynnRCP provides observations and controls for
the 22-value upper body, two grippers, mobile chassis, and three cameras. It
also includes a browser-based configuration and supervised hardware-test page.

> **Hardware safety**
>
> Clear the motion area and keep the emergency stop within reach before
> connecting, taking control, or sending an Action. Taking control immediately
> stops motion commanded by the previous controller. Run one robot-control
> process at a time.

## Prerequisites

- Work on the Astribot S1 development computer.
- Ubuntu 22.04 ARM64, ROS 2 Humble, and Python 3.10.
- The Astribot SDK installed at `/home/astribot/astribot_sdk_aarch64`.
- Wired access from the development computer to the robot at `192.168.0.10`.
- Start from the RynnRCP repository root.

Set the standard SDK directory once in each new terminal:

```bash
export ASTRIBOT_SDK_ROOT=/home/astribot/astribot_sdk_aarch64
```

## Log In to the Development Computer

Connect your computer to the Astribot development computer's wired network:

```bash
ssh astribot@<development-computer-ip>
cd <RynnRCP-repository>
```

Use the actual IP address and repository location. Run all remaining commands
on the development computer.

## Power On and Start the Robot Driver

Complete this sequence on the robot control page before connecting the SDK,
opening Configure, or starting RynnRCP Server:

1. Clear the motion area and reset the emergency stop.
2. Open the [robot control page](http://192.168.0.10:5141).
3. Click **上电** and wait for the page to report that power is on.
4. Click **启动机器人驱动** and wait for the driver to run without component faults.
5. Press and hold **进入初始姿势** until the initialization motion finishes and the robot is stationary.
6. Clear active alarms before connecting the SDK or RynnRCP.

Verify readiness with the read-only SDK example:

```bash
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
python "$ASTRIBOT_SDK_ROOT/examples/100-get_robot_properties.py"
```

The output should report a live interface and list the chassis, torso, both
arms, both grippers, and head.

## Install

Run from the repository root:

```bash
cd robots/astribot_s1
bash setup_astribot_s1.sh
source venv/bin/activate
```

The setup script sources the SDK environment, creates an isolated virtual
environment with access to system ROS 2 packages, and installs RynnRCP and the
Astribot S1 integration.

`Astribot SDK import OK`, `RynnBot App import OK`, `RynnRCP controller import OK`, and
`Astribot S1 setup completed.` confirm a successful installation.

## Configure and Test the Robot

After power-on, driver startup, and the initial-pose motion, run from
`robots/astribot_s1`:

```bash
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
source venv/bin/activate
rynnrcp-astribot-s1-configure
```

Open the Configure address printed in the terminal:

1. Save the Astribot SDK directory.
2. To connect to the cloud, enter and save the Product Key, Device Name, and
   Device Secret.
3. Click **连接并读取状态（不抢占）** and verify that all 22 joints, the chassis,
   and both grippers update.
4. Confirm that the workspace is safe and select the on-page safety checkbox.
5. Click **解锁运动控制** if the page already holds control, or
   **接管并解锁控制** if another client controls the robot.
6. Begin with one gripper or a small single-joint target. While a slider is
   dragged, the background loop updates the robot target smoothly at 100 Hz.

A normal connection automatically continues in read-only mode, so the launch
terminal does not require input. Motion inputs lock immediately if control
rights are lost.

The SDK path is saved in the local Server configuration. Review configuration
changes before committing so the values match the target deployment.

## Start Server

Run from `robots/astribot_s1`:

```bash
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
source venv/bin/activate
rynnrcp-server --config rynnrcp_robot_astribot_s1/config/astribot_s1_server.yaml
```

Keep the first launch in the foreground. After the terminal prints the ready
message and `Debug UI` address, confirm that
`observation.robot.joint_state` updates before sending a small Action.

Stop Server with `Ctrl+C`. Wait for the robot to become stable before stopping
the driver or disconnecting robot power.

## Connect RynnBot

Enter and save the RynnBot cloud credentials on the Configure page. Keep Server
running, then use another terminal on the development computer:

```bash
cd <RynnRCP-repository>/robots/astribot_s1
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
source venv/bin/activate
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_astribot_s1/config/astribot_s1_rynnbot_app.yaml \
  --server-config rynnrcp_robot_astribot_s1/config/astribot_s1_server.yaml
```

The terminal reports when the device is connected and ready for Astribot S1
observations and Actions through RynnBot.

## Open Protocol Debug

Keep Server running and open another terminal on the development computer:

```bash
cd <RynnRCP-repository>/robots/astribot_s1
set +u
source "$ASTRIBOT_SDK_ROOT/env.sh"
set -u
source venv/bin/activate
rynnrcp-protocol-debug \
  --config rynnrcp_robot_astribot_s1/config/astribot_s1_server.yaml
```

Open the printed Protocol Debug address. Check Observations and `get_health`
before testing one small Action at a time.

## Protocol Mapping

The 22-value `joint_state`, `joint_position`, and `joint_velocity` order is:

1. torso: 4
2. left arm: 7
3. left gripper: 1
4. right arm: 7
5. right gripper: 1
6. head: 2

Torso, arm, and head positions use radians. Gripper slots 11 and 19 are
normalized to `0–1`; the adapter converts them to and from the SDK's `0–100`
range. Dedicated gripper observations retain the raw SDK values for diagnosis.

Available Actions:

- `joint_position`: 22-value upper-body position control; slots 11 and 19 use `0–1`.
- `joint_velocity`: 22-value upper-body velocity control.
- `base_velocity`: chassis x, y, and yaw velocity in the local frame.
- `left_gripper` / `right_gripper`: normalized `0–1` position with optional maximum `force` in N.
- `home`: SDK collision-checked homing.
- `stop` / `restart`: stop current motion or restore control.

`chassis_state` reports `[x, y, yaw]` position and velocity. The three camera
observations are `head_camera.image`, `left_wrist_camera.image`, and
`right_wrist_camera.image`.

## Control Rights

Configure owns the complete control-rights flow: normal connections stay
read-only, and taking control requires the on-page safety confirmation.

`high_control_rights` applies only to RynnRCP Server startup:

- `false`: the SDK asks whether to take control while Server starts.
- `true`: Server immediately takes control from the current client.

Use `high_control_rights: true` only for a supervised deployment that
intentionally takes control.

## Troubleshooting

For `not alive` or `No simulation or real robot is started`:

1. Confirm **上电**, **启动机器人驱动**, and **进入初始姿势** on the control page.
2. Reset the emergency stop and clear active faults.
3. Keep only one Astribot SDK or RynnRCP control process running.
4. Run `ping 192.168.0.10` to verify the wired connection.
5. Run `examples/100-get_robot_properties.py` and
   `examples/101-get_joint_states.py` in the same SDK environment.
6. Check `sdk_root`, `ROS_DOMAIN_ID=25`, and the development computer's
   `192.168.0.x` address.
