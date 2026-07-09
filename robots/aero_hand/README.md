# Aero Hand Robot Package

Chinese documentation: [README.zh-CN.md](README.zh-CN.md)

This package exposes two cloud robot profiles:

- `aero_hand_single_server.yaml`: one 7-DoF hand, no left/right semantics.
- `aero_hand_dual_server.yaml`: one 14-DoF dual-hand robot, ordered left 7 DoF then right 7 DoF.

Install with:

```bash
cd robots/aero_hand
./setup_aero_hand.sh
source venv/bin/activate
```

Configure with:

```bash
rynnrcp-aero-hand-configure
```

The setup script installs the Runtime, official apps, and the Aero Hand package
in editable mode. Use `./setup_aero_hand.sh -h` to see options such as
`--pip-index-url`, `--venv`, and `--recreate`.

Run from this directory after configuring ports:

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml

rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```
