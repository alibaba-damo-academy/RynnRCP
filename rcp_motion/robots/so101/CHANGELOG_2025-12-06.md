# RynnRCP Sync Update - 2025-12-06

## Summary

Synced changes from RynnMotion to RynnRCP.

## Core Module Changes

### rcp_motion/core/robotinterface_base.py
- Added `self.connected = False` attribute in `__init__`
- Added `is_connected()` method to check connection status
- Added `get_robot_state_without_update()` method

### rcp_motion/core/mjrobot_interface.py
- Changed `self.is_connected` to `self.connected` (standardized attribute name)
- Changed `mj.viewer.launch_passive` to `mujoco.viewer.launch_passive`

---

## SO101 Changes

Synced changes from RynnLeRobot to RynnRCP/rcp_motion/robots/so101/.

### Files NOT Synced (RynnRCP-specific paths)
These files have different MJCF paths and `get_models_root()` implementations:
- `controller/so101_inference.py` - RynnRCP uses `models/lerobot/so101/`
- `controller/so101_motion.py` - RynnRCP uses `models/lerobot/so101/`
- `interface/so101_interface.py` - RynnRCP-specific config paths
- `scripts/follower_jointsine.py` - RynnRCP-specific paths

## Changes

### Gripper Normalization
- Changed gripper output from `[0, 100]` to `[0, 1]` range
- `0` = closed, `1` = fully open
- Added `MotorNormMode.RANGE_0_1` enum in `motors_bus.py`

### New Scripts
| New File | Replaces | Command |
|----------|----------|---------|
| `scripts/calibrate.py` | `follower_calibrate.py` | `so101-calibrate` |
| `scripts/show_joint_angles.py` | `follower_show_encoder.py` | `so101-show-joint-angles` |

### Files Modified
- `hardware/motors/motors_bus.py` - Added `RANGE_0_1` normalization mode
- `hardware/robots/so101_follower/so101_follower.py` - Gripper uses `RANGE_0_1`
- `hardware/teleoperators/so101_leader/so101_leader.py` - Gripper uses `RANGE_0_1`
- `scripts/follower_jointsine.py` - Gripper uses `RANGE_0_1`
- `scripts/lang.py` - Added translations for new scripts

### Files Deleted
- `scripts/follower_calibrate.py`
- `scripts/follower_show_encoder.py`

### Entry Points Updated (pyproject.toml)
```
so101-calibrate -> rcp_motion.robots.so101.scripts.calibrate:main
so101-show-joint-angles -> rcp_motion.robots.so101.scripts.show_joint_angles:main
```

## Testing

### 1. Reinstall Package
```bash
cd ~/Documents/Damowork/RynnRCP
pip install -e ".[so101]"
```

### 2. Test Calibration
```bash
# Run calibration (English)
so101-calibrate

# Run calibration (Chinese)
RYNNRCP_LANG=zh so101-calibrate
```

### 3. Test Joint Angles Monitor
```bash
# Show joint angles (English)
so101-show-joint-angles

# Show joint angles (Chinese)
RYNNRCP_LANG=zh so101-show-joint-angles
```

### 4. Test Joint Sine Wave
```bash
so101-jointsine
```

### 5. Verify Gripper Range
When running `so101-show-joint-angles`:
- Gripper closed: value should be ~0.0
- Gripper fully open: value should be ~1.0

## Environment Variable

Set language preference:
```bash
export RYNNRCP_LANG=en  # English (default)
export RYNNRCP_LANG=zh  # Chinese
```

## Notes

- No second leader support in RynnRCP (unlike RynnLeRobot)
- Config file auto-detected via `get_config_path()` from installed package location
- Calibration files are independent of normalization mode (store raw encoder values)
