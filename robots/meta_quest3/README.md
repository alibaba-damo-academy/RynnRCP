# Meta Quest 3 Leader

This package converts Meta Quest 3 controller motion into joint-position targets for a configured robot arm. YAML and URDF files select the robot; the controller, IK, and Web simulator remain shared.

- Single arm with gripper: `[arm joints, gripper]`
- Single arm without gripper: `[arm joints]`
- Dual arm: `[left joints, optional left gripper, right joints, optional right gripper]`
- Gripper-enabled configurations use `0` closed and `1` open
- Control rate: 60 Hz

[中文](README.zh-CN.md)

## Available configurations

### Web simulator

Pass one of these files directly to `rynnrcp-meta-quest3-sim --config`:

| Target robot | Configuration | Mode | Control values |
| --- | --- | --- | ---: |
| Franka FR3 | `franka_fr3_right.yaml` | Right arm | 8 |
| UR5e | `ur5e_right.yaml` | Right arm | 6 |
| Piper | `piper_right.yaml` | Right arm | 6 |
| RealMan RM75 | `rm75_right.yaml` | Right arm | 7 |
| SO101 Follower | `so101_follower_right.yaml` | Right arm | 6 |
| Rizon 4s | `rizon4s_right.yaml` | Right arm | 7 |
| ECO65 | `eco65_right.yaml` | Right arm | 6 |
| Damiao 6DoF | `dm6dof_right.yaml` | Right arm | 6 |
| OpenArm | `openarm_right.yaml` | Right arm | 7 |
| Dual Franka FR3 | `franka_fr3_dual.yaml` | Dual arm | 16 |

All simulator configurations are under:

```text
rynnrcp_robot_meta_quest3/config/sim/
```

Every listed configuration loads packaged low-detail STL meshes. Like the FR3
assets, these use compact collision-style envelopes that preserve each link's
overall shape and connection points. They are intended for Web debugging, not
high-fidelity presentation or collision checking. The page attaches each mesh to
its URDF `visual` link and falls back to `collision` when no supported visual STL
exists. It only renders the rod skeleton when model assets are missing.

Each simulator's `home_joint_positions` defines its Cartesian debugging reference
pose. Simulator configurations favor a workspace-centered, non-singular pose so
the XYZ and rotation sliders can move in both directions. Hardware startup poses
come from the corresponding target-robot configuration.

### RynnRCP leader servers

| Configuration | Server YAML | Output |
| --- | --- | --- |
| Quest right controller → one FR3 | `meta_quest3_franka_fr3_right_server.yaml` | 7 joints + 1 gripper |
| Quest dual controllers → two FR3 arms | `meta_quest3_franka_fr3_dual_server.yaml` | left 7+1 + right 7+1 |

Server YAML files are under `rynnrcp_robot_meta_quest3/config/` and run with `rynnrcp-meta-quest3-server --config <configuration> --source-ip <Quest IP>`. The current single-FR3 hardware setup uses `meta_quest3_franka_fr3_right_server.yaml`.

## Quick start: local Web simulator

The simulator uses Python 3.10+ and a browser. It runs on macOS, Linux, and Windows through WSL. The Bash commands below work in these environments.

From the repository root:

```bash
cd robots/meta_quest3
bash setup_meta_quest3.sh
source venv/bin/activate

rynnrcp-meta-quest3-sim \
  --config rynnrcp_robot_meta_quest3/config/sim/franka_fr3_right.yaml
```

Open `http://127.0.0.1:8765`. The page loads the URDF, Home pose, joint limits, workspace, and motion limits from the selected YAML.

Add `--quest` to consume Quest UDP data:

```bash
rynnrcp-meta-quest3-sim \
  --config rynnrcp_robot_meta_quest3/config/sim/rm75_right.yaml \
  --quest
```

Run a dual-arm configuration the same way:

```bash
rynnrcp-meta-quest3-sim \
  --config rynnrcp_robot_meta_quest3/config/sim/franka_fr3_dual.yaml \
  --quest
```

## Use Quest controllers

The Franka right-arm and dual-arm configurations listen on UDP `0.0.0.0:8888`. Pass the Quest 3 address at startup with `--source-ip`, for example `172.16.1.29`; update the command argument when the address changes.

The sender transmits one JSON object per UDP packet. Single-arm mode reads
`rightController`; dual-arm mode reads both controller objects:

```json
{
  "leftController": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "timestamp": 1.0,
    "input": {
      "primaryButton": false,
      "gripPressed": false,
      "trigger": 0.0
    }
  },
  "rightController": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "timestamp": 1.0,
    "input": {
      "primaryButton": false,
      "gripPressed": false,
      "trigger": 0.0
    }
  }
}
```

`position` and `rotation` contain the sender's absolute controller pose.
`rotation` uses an xyzw quaternion. `gripPressed` is the motion clutch,
`trigger` is normalized to `[0, 1]`, and `primaryButton` performs calibration.
`timestamp` is optional; when provided, keep it increasing for each controller.

### Single arm

1. Hold the right controller at the neutral teleoperation pose.
2. Press A once.
3. Hold the right Grip to publish joint targets and move; release it to stop publishing and hold position.
4. Use trigger to control the gripper.
5. Press the calibration button again to return smoothly to the configured Home and recalibrate.

### Dual arm

1. Hold both controllers at their neutral poses.
2. Press X+A together.
3. Each Grip controls its corresponding arm. The merged vector is published while at least one Grip is held; an inactive arm keeps its last target.
4. Each trigger controls its corresponding gripper.

The dual-arm output is one vector. Each configured gripper follows its arm
joints:

```text
[
  left joint positions,
  optional left gripper,
  right joint positions,
  optional right gripper
]
```

## Coordinates and calibration

Controller motion is converted to `rcp_cartesian_v1`:

```text
+X: forward
+Y: left
+Z: up
position: metres
orientation: xyzw quaternion
```

Calibration records the current controller pose. Later controller motion is measured relative to this pose, added to the robot's configured Home end-effector pose, and converted to absolute joint targets through URDF IK.

When connecting a new Quest sender, move the controller forward, left, and up and confirm that X, Y, and Z respectively increase. These fields control the mapping:

- `coordinate_basis`: translation mapping
- `rotation_basis`: rotation mapping
- `rotation_component_signs`: rotation direction about standard X, Y, and Z
- `translation_scale`: controller translation scale

## Control a real FR3

On the Ubuntu host connected to the FR3:

```bash
cd robots/franka_fr3
bash setup_franka_fr3.sh
source venv/bin/activate
```

The setup script installs Franka, Meta Quest 3, the RynnRCP Server, and Teleop into `robots/franka_fr3/venv`.

Start three processes:

```bash
# Terminal 1: real FR3
rynnrcp-server \
  --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml

# Terminal 2: Quest right-controller joint targets
rynnrcp-meta-quest3-server \
  --config rynnrcp_robot_meta_quest3/config/meta_quest3_franka_fr3_right_server.yaml \
  --source-ip 172.16.1.29

# Terminal 3: teleoperation and recording
rynnrcp-teleop-app
```

In Teleop, select:

- Leader: `meta_quest3_franka_fr3_right`
- Follower: `franka_fr3`
- Mapping: `observation.robot.joint_state` → `action.robot.joint_position`

Clear the workspace and keep the emergency stop ready. Press A to calibrate. Joint targets are published only while the right Grip is held. Releasing Grip stops publication, and the FR3 smoothly stops and holds after its target-stream timeout. Pressing Grip again anchors a new clutch reference at the current controller pose, so motion made while released is not applied.

The FR3 vector is:

```text
[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, gripper]
```

`meta_quest3_franka_fr3_right_server.yaml` stores Home, workspace, and motion limits. Its current workspace relative to Home is X `[-0.18, 0.40] m`, Y `[-0.50, 0.50] m`, and Z `[-0.50, 0.00] m`.

## Add a robot configuration

### Web simulation

1. Add the URDF under `rynnrcp_robot_meta_quest3/models/`.
2. Put the STL files referenced by that URDF under
   `rynnrcp_robot_meta_quest3/models/assets/<target_model>/`. Filename matching is
   case-insensitive; the Web simulator prefers `visual` and falls back to
   `collision`.
3. Copy a `config/sim/*_right.yaml` file.
4. Set `target_urdf`, `target_dof`, `base_link`, `tip_link`, and `home_joint_positions`.
5. Verify it with `rynnrcp-meta-quest3-sim --config <config>`.

A single-arm control vector has `target_dof + int(has_gripper)` values.

For dual arms, configure separate chains under `components.robot.arms.left` and `right`. The arms may use different URDFs and joint counts. The control size is:

```text
left target_dof + int(left has_gripper)
+ right target_dof + int(right has_gripper)
```

### RynnRCP leader service

Copy `meta_quest3_franka_fr3_right_server.yaml` for one arm or `meta_quest3_franka_fr3_dual_server.yaml` for two arms. Set:

- `manifest.robot_id` and `manifest.robot_name`
- `target_model` and `target_urdf`
- `target_dof` and `control_dof`
- `base_link` and `tip_link`
- `home_joint_positions`
- `workspace_delta_limits_m`
- coordinate mapping and motion limits

Single-arm configurations share `MetaQuest3UrdfJointController`; dual-arm configurations share `MetaQuest3DualUrdfJointController`. Both Integration files read `control_dof` and `joint_order` from the selected Server configuration. To add a hardware robot, create a Server YAML with the correct joint count and optional gripper for each arm; the shared Integration and controller code then publish that configured vector.

Grip gating is implemented in these shared controllers. When no Grip is held, the leader observation waits for fresh controller data instead of producing a null value or a new channel sample, so Teleop has no action to forward. New configurations reuse the same controllers and inherit this clutch behavior through configuration alone.

## Main configuration fields

| Field | Purpose |
| --- | --- |
| `target_urdf` | Target robot URDF |
| `target_dof` | Active base-to-tip joint count |
| `control_dof` | Published vector size, including each enabled gripper |
| `joint_order` | Published joint and gripper order |
| `has_gripper` | Whether the control vector and Web UI include a gripper |
| `home_includes_gripper` | Treat the last Home value as the normalized gripper Home |
| `native_gripper_joint` | Animate the named URDF joint instead of adding a generic gripper model |
| `joint_names` | Optional protocol-facing names for the active arm joints |
| `control_dof` | External vector size derived from joints and `has_gripper` |
| `base_link` / `tip_link` | Start and end of the IK chain |
| `home_joint_positions` | Arm Home joints, followed by the gripper Home value when `has_gripper` is enabled |
| `workspace_delta_limits_m` | XYZ range relative to the Home end-effector pose |
| `max_joint_velocity_rad_s` | Maximum joint-target velocity |
| `max_joint_acceleration_rad_s2` | Maximum joint-target acceleration |
| `max_target_translation_m_s` | Cartesian translation tracking speed |
| `max_target_rotation_rad_s` | Cartesian rotation tracking speed |

## Status and troubleshooting

- `meta_quest3.waiting`: check the Quest destination, UDP port, and the `--source-ip` startup argument.
- `meta_quest3.calibration_required`: press A for single-arm mode or X+A for dual-arm mode.
- `meta_quest3.stale`: check the wireless network and Quest sender.
- `meta_quest3.ik_not_converged`: return the controller toward the last reachable pose and reduce translation or rotation.
- `meta_quest3.left_ik_not_converged` / `right_ik_not_converged`: the named side did not converge.
- `controller_state.calibration_count`: number of successful calibrations.
- `controller_state.packet_age_s`: age of the latest UDP packet.
