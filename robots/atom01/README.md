# Atom01 Robot Package

Chinese documentation: [README.zh-CN.md](README.zh-CN.md)

This package integrates the Atom01 humanoid robot with RynnRCP. It includes the
Atom01 C++ control binding, RCP controller, Server/RynnBot configs,
zero-position calibration UI, and sample policies.

## License

This package is distributed with the repository, whose root license is Apache
License 2.0. The C++ control binding sources under
`rynnrcp_robot_atom01/atom_control/` include GPL-3.0 files; those files remain
under GPL-3.0 as marked by their SPDX headers.

Install on the Atom01 robot machine:

```bash
cd robots/atom01
./setup_atom01.sh
source venv/bin/activate
```

Configure and calibrate:

```bash
rynnrcp-atom01-configure
```

The configuration UI opens at:

```text
http://127.0.0.1:28421
```

Run the Server after configuration:

```bash
rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

Atom01 enables `policy_service` and scans the package `policies/` directory.
`list_policies` exposes `stand`, `walk`, and `sim2simdance`. `stand` and
`walk` accept `cmd_vel` as a `[3]` runtime input and reset it to the default
after 500 ms without updates.

Dry-run a policy before sending targets to hardware:

```bash
rynnrcp-atom01-test-policy --policy stand
```

Use `--apply --i-understand-risk` only after the robot is safely supported and
the zero position and output range have been checked.

Connect RynnBot:

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_atom01/config/atom01_rynnbot_app.yaml --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
```

Protocol-level debugging:

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

Before sending real actions, check `get_health` and clear warnings such as
`atom01.zero_calibration_unconfirmed` or `atom01.motors_not_enabled`.
