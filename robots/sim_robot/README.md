# Sim Robot

RynnRCP Isaac Sim integration for SO101 single-arm/bimanual, RM75, Franka R3,
Aero Hand single/dual, and other simulated robot profiles through ZeroMQ.

[简体中文](README.zh-CN.md)

## Requirements

- Isaac Sim environment
- Running simulation-side ZeroMQ service

## Install

Enter the Sim Robot directory, then install and activate the environment:

```bash
cd robots/sim_robot
bash setup_sim.sh
source venv/bin/activate
```

The setup script installs RynnBot, MCP, Protocol Debug, and Teleop.

## Run

Start one simulation profile:

```bash
bash start_rcp.sh --config so101
bash start_rcp.sh --config lerobot_so101_dual_sim_v1
bash start_rcp.sh --config lerobot_so101_dual_sim_v2
bash start_rcp.sh --config aero_hand_dual_sim_v1
bash start_rcp.sh --config aero_hand_dual_sim_v2
bash start_rcp.sh --config rm75_rmg24_sim_v1
bash start_rcp.sh --config rm75_rmg24_sim_v2
bash start_rcp.sh --config franka_r3_sim_v1
bash start_rcp.sh --config franka_r3_sim_v2
```

Only these 9 server configs are kept: 8 precise robot_type_id configs plus the original `so101` profile.

After the Server is running, start any required app in another terminal:

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_sim/config/sim_server_so101.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_sim/config/sim_server_so101.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_sim/config/sim_rynnbot_app.yaml --server-config rynnrcp_robot_sim/config/sim_server_so101.yaml
rynnrcp-teleop-app
```

Each Server prints a `Debug UI` address after startup. Open that address in a
browser to inspect state, Actions, camera images, or live charts. If the port is
occupied, use the fallback address printed by that Server terminal.

## Capabilities

- `aero_hand_dual_sim_v1`: merged 14DoF compact vector + front camera; action/state names are left_* + right_* tendon/actuator
- `aero_hand_dual_sim_v2`: merged 14DoF compact vector + front/top/left/right cameras; no left/right component split
- `rm75_rmg24_sim_v1` / `rm75_rmg24_sim_v2`: 8DoF + 5 cameras
- `franka_r3_sim_v1` / `franka_r3_sim_v2`: 9DoF + 5 cameras
- `lerobot_so101_dual_sim_v1` / `lerobot_so101_dual_sim_v2`: merged 12DoF vector (left 6 + right 6); RCP/cloud sees one `robot` component, while the controller internally connects to `left_robot` / `right_robot`
- `so101`: original SO101 single-arm profile, 6DoF + front/top/left/right/wrist cameras
- Simulated joint state and control
- Multi-camera observations
- Multiple robot profile configs

See the [debugging reference](DEBUGGING.zh-CN.md) for ZeroMQ data flow, cloud
config, and adding new profiles.
