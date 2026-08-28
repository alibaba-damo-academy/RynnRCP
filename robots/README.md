# Robots

This directory contains robot integration packages. Each subdirectory exposes a
specific robot or simulator through RynnRCP Server configs, controllers, setup
scripts, and optional policies.

[简体中文](README.zh-CN.md)

## Packages

| Directory | Description | Entry |
| --- | --- | --- |
| `lerobot_so101/` | SO101 leader/follower arms | [`lerobot_so101/README.md`](lerobot_so101/README.md) |
| `tetheria_aerohand/` | Aero Hand single/dual profiles and camera-gesture master | [`tetheria_aerohand/README.md`](tetheria_aerohand/README.md) |
| `roboparty_atom01/` | Atom01 humanoid robot | [`roboparty_atom01/README.md`](roboparty_atom01/README.md) |
| `booster_t1/` | Booster T1 high-level, low-level, and local walking policy | [`booster_t1/README.md`](booster_t1/README.md) |
| `noetix_bumi/` | Noetix Bumi high-level, low-level, and local walking policy | [`noetix_bumi/README.md`](noetix_bumi/README.md) |
| `sim_robot/` | Isaac Sim robot profiles | [`sim_robot/README.md`](sim_robot/README.md) |
| `lerobot_lekiwi/` | LeKiwi mobile manipulator | [`lerobot_lekiwi/README.md`](lerobot_lekiwi/README.md) |
| `franka_fr3/` | Franka Research 3 through official libfranka | [`franka_fr3/README.md`](franka_fr3/README.md) |
| `meta_quest3/` | Meta Quest 3 left, right, and dual-arm leaders | [`meta_quest3/README.md`](meta_quest3/README.md) |
| `astribot_s1/` | Astribot S1 through the official ROS 2 Python SDK | [`astribot_s1/README.md`](astribot_s1/README.md) |

## Installation Convention

Run each robot's setup script from its own directory:

```bash
cd robots/<robot>
bash setup_<robot>.sh
source venv/bin/activate
```

Use the exact setup filename documented by that robot; for example, Noetix
Bumi uses `setup_bumi.sh` and Sim Robot uses `setup_sim.sh`.

Every `rynnrcp-server` prints a `Debug UI` address after startup. Open that
address in a browser to inspect joint state, Actions, camera images, and live
charts. If the default port is occupied, use the fallback address printed by
that Server terminal.

## What Each Package Contains

- `controller`: maps the robot SDK to RCP observations and actions.
- `config`: Server and robot integration configs.
- `policies` optional: local policy assets loaded by `policy_service`.
- `setup_*.sh`: robot-local installation script.
- `README.md` and `README.zh-CN.md`: first-run install and usage.
- `DEBUGGING*.md`: detailed debugging notes when needed.

For a new robot, first verify the native SDK, serial device, ROS2/LCM bridge, or
camera path outside RynnRCP. Then map the working minimum into a controller and
Server config.
