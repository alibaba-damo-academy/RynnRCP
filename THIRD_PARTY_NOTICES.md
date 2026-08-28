# Third-Party Notices

RynnRCP is licensed under the Apache License 2.0 unless otherwise noted in a
file header, package README, or this notice file.

This file summarizes third-party code and redistributable assets that are
checked into this repository. Individual source file headers remain
authoritative.

## TetherIA Aero Hand SDK

- Path: `robots/tetheria_aerohand/rynnrcp_robot_aero_hand/aero_open_sdk/`
- Copyright: 2025 TetherIA, Inc.
- License: Apache License 2.0
- Notes: Source files include Apache-2.0 license headers.

## HuggingFace LeRobot SO101 Helpers

- Path: `robots/lerobot_so101/lerobot_so101/`
- Copyright: 2024 The HuggingFace Inc. team.
- License: Apache License 2.0
- Notes: Source files include Apache-2.0 license headers.

## RynnMotion Robot Mesh Assets

- Path: `robots/meta_quest3/rynnrcp_robot_meta_quest3/models/assets/`
- Source: RynnMotion robot models for UR5e, Piper, RM75, SO101, Rizon 4s,
  ECO65, DM6DoF, and OpenArm.
- Source revision: `87097dce2aae3caeba9a29f3e0fe97d41940de03`
- License: Apache License 2.0, as declared by the RynnMotion repository.
- Notes: The browser assets are low-detail, collision-style derivatives of the
  listed RynnMotion meshes. UR5e OBJ files were converted to binary STL; the
  other source meshes were converted to compact binary STL envelopes. These
  changes reduce package size and are not intended for precision collision
  checking.

## Franka Description FR3 Assets

- Paths:
  - `robots/meta_quest3/rynnrcp_robot_meta_quest3/models/fr3.urdf`
  - `robots/meta_quest3/rynnrcp_robot_meta_quest3/models/assets/franka_fr3/`
- Source: Franka Robotics `franka_description`.
- License: Apache License 2.0.
- Notes: The included STL files are low-detail collision meshes used only for
  browser visualization.

## Three.js Browser Runtime

- Paths:
  - `robots/meta_quest3/rynnrcp_robot_meta_quest3/web/three.min.js`
  - `robots/meta_quest3/rynnrcp_robot_meta_quest3/web/STLLoader.classic.js`
- Copyright: 2010-2025 three.js authors.
- License: MIT.
- License text:
  `robots/meta_quest3/rynnrcp_robot_meta_quest3/web/THREE_LICENSE.txt`.

## HiPNUC IMU Decoder

- Paths:
  - `robots/roboparty_atom01/rynnrcp_robot_atom01/atom_control/core/include/imu/hipnuc_dec.h`
  - `robots/roboparty_atom01/rynnrcp_robot_atom01/atom_control/core/src/imu/hipnuc_dec.c`
- Copyright: 2006-2024 HiPNUC
- License: Apache-2.0
- Notes: Source files include SPDX license identifiers.

## Atom01 Control Binding Sources

- Path: `robots/roboparty_atom01/rynnrcp_robot_atom01/atom_control/`
- Copyright: as marked in source file headers, including Luo1imasi and
  wentywenty.
- License: GPL-3.0 for files marked with `SPDX-License-Identifier: GPL-3.0`.
- Notes: These files are not relicensed by the repository-level Apache-2.0
  license. Their SPDX headers remain authoritative.

## Atom01 Policy Assets

- Paths:
  - `robots/roboparty_atom01/rynnrcp_robot_atom01/policies/`
- Contents: ONNX policy models and motion data.
- Notes: These assets are distributed as part of the Atom01 robot package.
  Confirm redistribution rights before publishing a public release that includes
  these files.
