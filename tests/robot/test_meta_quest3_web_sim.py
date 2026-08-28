from __future__ import annotations

import json
import struct
import threading
from http import HTTPStatus
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.request import urlopen

import pytest

from rynnrcp_robot_meta_quest3.web_sim import (
    DualArmSimulation,
    QuestSimulationBridge,
    UrdfJointSimulation,
    _SimulatorHandler,
    _visual_transform,
    load_simulation_config,
)
from rynnrcp_robot_meta_quest3.kinematics import (
    UrdfKinematicChain,
    rotation_from_rpy,
    rotation_multiply,
    rpy_from_rotation,
)


MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / "robots"
    / "meta_quest3"
    / "rynnrcp_robot_meta_quest3"
    / "models"
    / "fr3.urdf"
)
SIM_CONFIG_DIR = (
    Path(__file__).resolve().parents[2]
    / "robots"
    / "meta_quest3"
    / "rynnrcp_robot_meta_quest3"
    / "config"
    / "sim"
)
MODEL_ASSET_DIR = SIM_CONFIG_DIR.parents[1] / "models" / "assets"
FR3_HOME_JOINT_POSITIONS = (
    0.0,
    -0.785,
    0.0,
    -2.356,
    0.0,
    1.571,
    0.785,
)


def _fr3_simulation() -> UrdfJointSimulation:
    simulation, _ = load_simulation_config(
        SIM_CONFIG_DIR / "franka_fr3_right.yaml"
    )
    assert isinstance(simulation, UrdfJointSimulation)
    return simulation


def _dual_fr3_simulation() -> DualArmSimulation:
    simulation, _ = load_simulation_config(
        SIM_CONFIG_DIR / "franka_fr3_dual.yaml"
    )
    assert isinstance(simulation, DualArmSimulation)
    return simulation


def test_fr3_urdf_exposes_the_expected_arm_chain() -> None:
    chain = UrdfKinematicChain(MODEL_PATH)
    home = chain.forward(FR3_HOME_JOINT_POSITIONS)
    transforms = dict(home.link_transforms)

    assert chain.joint_names == tuple(f"fr3_joint{index}" for index in range(1, 8))
    assert len(home.link_points) >= 8
    assert transforms["fr3_hand"] == transforms["fr3_link8"]
    assert all(low < high for low, high in zip(chain.lower_limits, chain.upper_limits))


def test_damped_least_squares_ik_recovers_a_nearby_pose() -> None:
    chain = UrdfKinematicChain(MODEL_PATH)
    target_joints = list(FR3_HOME_JOINT_POSITIONS)
    target_joints[0] += 0.05
    target_joints[2] -= 0.04
    target = chain.forward(target_joints)

    result = chain.solve_ik(
        target.position,
        target.rotation,
        FR3_HOME_JOINT_POSITIONS,
    )

    assert result.converged
    assert result.position_error_m <= 0.001
    assert result.rotation_error_rad <= 0.01
    assert all(
        low <= value <= high
        for value, low, high in zip(
            result.joint_positions,
            chain.lower_limits,
            chain.upper_limits,
        )
    )


def test_web_state_endpoint_returns_renderable_fr3_state() -> None:
    simulation = _fr3_simulation()
    _SimulatorHandler.simulation = simulation
    server = ThreadingHTTPServer(("127.0.0.1", 0), _SimulatorHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urlopen(
            f"http://127.0.0.1:{server.server_address[1]}/api/state",
            timeout=2,
        ) as response:
            state = json.load(response)
        with urlopen(
            f"http://127.0.0.1:{server.server_address[1]}/",
            timeout=2,
        ) as response:
            page = response.read().decode()
        with urlopen(
            f"http://127.0.0.1:{server.server_address[1]}"
            "/static/sim_shared.js",
            timeout=2,
        ) as response:
            shared_script = response.read().decode()
        with urlopen(
            f"http://127.0.0.1:{server.server_address[1]}"
            "/assets/models/franka_fr3/link0.stl",
            timeout=2,
        ) as response:
            mesh = response.read()
        with urlopen(
            f"http://127.0.0.1:{server.server_address[1]}"
            "/assets/models/piper/base_link.stl",
            timeout=2,
        ) as response:
            generic_mesh = response.read()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)

    assert state["joint_names"] == [f"fr3_joint{index}" for index in range(1, 8)]
    assert len(state["joint_positions"]) == 7
    assert state["gripper_position"] == pytest.approx(1.0)
    assert state["gripper_mount_link"] == "fr3_hand"
    assert state["gripper_mount_link"] in state["link_transforms"]
    assert len(state["link_points"]) >= 8
    assert {
        "fr3_link0",
        "fr3_link1",
        "fr3_link2",
        "fr3_link3",
        "fr3_link4",
        "fr3_link5",
        "fr3_link6",
        "fr3_link7",
        "fr3_hand",
    }.issubset(state["link_transforms"])
    assert state["ik"]["converged"] is True
    assert "/static/STLLoader.classic.js" in page
    assert "/static/sim_shared.js" in page
    assert "attachOrbitControls" in shared_script
    assert all(
        visual["url"].startswith("/assets/models/franka_fr3/")
        for visual in state["visuals"]
    )
    assert "state.control_dof" in page
    assert len(mesh) > 1_000
    assert len(generic_mesh) > 1_000


@pytest.mark.parametrize(
    "disconnect_error",
    [BrokenPipeError, ConnectionAbortedError, ConnectionResetError],
)
def test_http_response_ignores_client_disconnect(disconnect_error) -> None:
    class DisconnectedWriter:
        def write(self, _body: bytes) -> None:
            raise disconnect_error

    handler = object.__new__(_SimulatorHandler)
    handler.send_response = lambda *_args: None
    handler.send_header = lambda *_args: None
    handler.end_headers = lambda: None
    handler.wfile = DisconnectedWriter()

    handler._send(HTTPStatus.OK, b"{}", "application/json")


def test_dual_web_state_exposes_two_eight_value_arm_blocks() -> None:
    simulation = _dual_fr3_simulation()
    displaced = [
        *FR3_HOME_JOINT_POSITIONS,
        0.3,
        *FR3_HOME_JOINT_POSITIONS,
        0.7,
    ]
    displaced[0] += 0.1
    displaced[8] -= 0.1
    simulation.set_joint_positions(displaced)

    _SimulatorHandler.simulation = simulation
    server = ThreadingHTTPServer(("127.0.0.1", 0), _SimulatorHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urlopen(
            f"http://127.0.0.1:{server.server_address[1]}/api/state",
            timeout=2,
        ) as response:
            state = json.load(response)
        with urlopen(
            f"http://127.0.0.1:{server.server_address[1]}/",
            timeout=2,
        ) as response:
            page = response.read().decode()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)

    assert state["mode"] == "dual"
    assert state["joint_order"] == (
        "left_joints_left_gripper_right_joints_right_gripper"
    )
    assert state["joint_positions"] == displaced
    assert len(state["joint_names"]) == 16
    assert state["left"]["joint_positions"] == displaced[:7]
    assert state["left"]["gripper_position"] == pytest.approx(displaced[7])
    assert state["right"]["joint_positions"] == displaced[8:15]
    assert state["right"]["gripper_position"] == pytest.approx(displaced[15])
    assert "state.control_dof" in page
    assert "双臂平滑回到 Home" in page
    assert "当前调试机械臂" not in page
    assert "左臂手动调试" in page
    assert "右臂手动调试" in page


def test_dual_web_state_omits_disabled_gripper_dimensions() -> None:
    left, _ = load_simulation_config(
        SIM_CONFIG_DIR / "franka_fr3_right.yaml"
    )
    right, _ = load_simulation_config(SIM_CONFIG_DIR / "ur5e_right.yaml")
    assert isinstance(left, UrdfJointSimulation)
    assert isinstance(right, UrdfJointSimulation)
    simulation = DualArmSimulation(left, right)
    positions = [
        *left.home_joint_positions,
        0.25,
        *right.home_joint_positions,
    ]

    simulation.set_joint_positions(positions)
    state = simulation.snapshot()

    assert state["control_dof"] == 14
    assert state["joint_order"] == (
        "left_joints_left_gripper_right_joints"
    )
    assert state["joint_positions"] == pytest.approx(positions)
    assert state["left"]["gripper_position"] == pytest.approx(0.25)
    assert state["right"]["has_gripper"] is False
    assert "gripper_position" not in state["right"]
    with pytest.raises(ValueError, match="right arm has no gripper"):
        simulation.set_gripper("right", 0.5)


def test_all_packaged_simulator_configs_load_their_urdf_chain() -> None:
    expected_shapes = {
        "dm6dof_right.yaml": ("single", 6),
        "eco65_right.yaml": ("single", 6),
        "franka_fr3_dual.yaml": ("dual", 16),
        "franka_fr3_right.yaml": ("single", 8),
        "openarm_right.yaml": ("single", 7),
        "piper_right.yaml": ("single", 6),
        "rizon4s_right.yaml": ("single", 7),
        "rm75_right.yaml": ("single", 7),
        "so101_follower_right.yaml": ("single", 6),
        "ur5e_right.yaml": ("single", 6),
    }

    assert {path.name for path in SIM_CONFIG_DIR.glob("*.yaml")} == set(
        expected_shapes
    )
    for name, expected in expected_shapes.items():
        simulation, robot_config = load_simulation_config(
            SIM_CONFIG_DIR / name
        )
        snapshot = simulation.snapshot()
        assert (snapshot["mode"], snapshot["control_dof"]) == expected
        if snapshot["mode"] == "single":
            assert robot_config["controller_side"] == "right"
        arms = (
            [snapshot["left"], snapshot["right"]]
            if snapshot["mode"] == "dual"
            else [snapshot]
        )
        for arm in arms:
            assert arm["visuals"]
            if name in {
                "dm6dof_right.yaml",
                "eco65_right.yaml",
                "openarm_right.yaml",
                "piper_right.yaml",
                "rizon4s_right.yaml",
                "rm75_right.yaml",
                "ur5e_right.yaml",
            }:
                assert arm["has_gripper"] is False
                assert "gripper_position" not in arm
                assert arm["control_dof"] == len(arm["joint_positions"])
            if name == "so101_follower_right.yaml":
                assert arm["joint_positions"] == pytest.approx(
                    [-0.0, -1.62, 1.62, 0.80, 0.31]
                )
                assert robot_config["home_joint_positions"] == pytest.approx(
                    [-0.0, -1.62, 1.62, 0.80, 0.31, 0.0]
                )
                assert arm["has_gripper"] is True
                assert arm["gripper_position"] == pytest.approx(0.0)
            home = arm["home_pose"]
            assert home["position"] == pytest.approx(
                arm["target_pose"]["position"]
            )
            assert home["orientation_rpy"] == pytest.approx(
                arm["target_pose"]["orientation_rpy"]
            )
            assert all(
                low < high
                for low, high in zip(
                    arm["joint_lower_limits"],
                    arm["joint_upper_limits"],
                )
            )
            assert all(
                visual["link_name"] in arm["link_transforms"]
                for visual in arm["visuals"]
            )
            assert all(
                len(visual["origin_transform"]) == 4
                and all(len(row) == 4 for row in visual["origin_transform"])
                for visual in arm["visuals"]
            )


def test_single_simulator_config_rejects_left_controller(
    tmp_path: Path,
) -> None:
    source = (
        SIM_CONFIG_DIR / "franka_fr3_right.yaml"
    ).read_text(encoding="utf-8")
    config_path = tmp_path / "franka_left.yaml"
    config_path.write_text(
        source.replace("controller_side: right", "controller_side: left"),
        encoding="utf-8",
    )

    with pytest.raises(
        ValueError,
        match="single-arm simulator config requires controller_side: right",
    ):
        load_simulation_config(config_path)


def test_simulator_config_requires_explicit_has_gripper(
    tmp_path: Path,
) -> None:
    source = (
        SIM_CONFIG_DIR / "franka_fr3_right.yaml"
    ).read_text(encoding="utf-8")
    config_path = tmp_path / "missing_has_gripper.yaml"
    config_path.write_text(
        source.replace("    has_gripper: true\n", ""),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="requires has_gripper"):
        load_simulation_config(config_path)


def test_left_bridge_requires_the_dual_controller() -> None:
    with pytest.raises(
        ValueError,
        match="left bridge requires a shared dual-arm controller",
    ):
        QuestSimulationBridge(
            _fr3_simulation(),
            bind_host="127.0.0.1",
            udp_port=0,
            source_ip="",
            translation_scale=1.0,
            controller_side="left",
        )


def test_absolute_ee_pose_target_uses_the_home_base_frame() -> None:
    simulation = _fr3_simulation()
    home = simulation.snapshot()["home_pose"]
    position = list(home["position"])
    position[0] += 0.05
    orientation_rpy = list(home["orientation_rpy"])
    orientation_rpy[2] += 0.1

    simulation.set_target_ee_pose(position, orientation_rpy)
    target = simulation.snapshot()["target_pose"]

    assert target["position"] == pytest.approx(position)
    assert target["orientation_rpy"] == pytest.approx(orientation_rpy)


def test_gripper_position_is_independent_from_arm_joints() -> None:
    simulation = _fr3_simulation()
    joints = simulation.snapshot()["joint_positions"]

    simulation.set_gripper(0.0)
    closed = simulation.snapshot()
    simulation.set_gripper(1.0)
    opened = simulation.snapshot()

    assert closed["gripper_position"] == pytest.approx(0.0)
    assert opened["gripper_position"] == pytest.approx(1.0)
    assert closed["joint_positions"] == joints
    assert opened["joint_positions"] == joints


def test_so101_gripper_moves_its_native_urdf_joint() -> None:
    simulation, _ = load_simulation_config(
        SIM_CONFIG_DIR / "so101_follower_right.yaml"
    )
    closed = simulation.snapshot()
    simulation.set_gripper(1.0)
    opened = simulation.snapshot()

    assert closed["joint_names"] == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ]
    assert closed["native_gripper_visual"] is True
    assert (
        closed["link_transforms"]["moving_jaw_so101_v1_link"]
        != opened["link_transforms"]["moving_jaw_so101_v1_link"]
    )


def test_underactuated_so101_uses_shoulder_pan_for_y_motion() -> None:
    simulation, _ = load_simulation_config(
        SIM_CONFIG_DIR / "so101_follower_right.yaml"
    )
    home = simulation.chain.forward(simulation.home_joint_positions)
    shoulder_pan_home = simulation.home_joint_positions[0]

    for direction in (-1.0, 1.0):
        target = list(home.position)
        target[1] += direction * 0.05
        result = simulation.chain.solve_ik(
            target,
            home.rotation,
            simulation.home_joint_positions,
        )
        assert result.converged
        assert result.position_error_m < 0.002
        assert abs(result.joint_positions[0] - shoulder_pan_home) > 0.2


def test_underactuated_so101_tracks_reachable_rotation_without_position_drift() -> None:
    simulation, _ = load_simulation_config(
        SIM_CONFIG_DIR / "so101_follower_right.yaml"
    )
    home = simulation.chain.forward(simulation.home_joint_positions)

    for axis in range(3):
        delta_rpy = [0.0, 0.0, 0.0]
        delta_rpy[axis] = 0.2
        result = simulation.chain.solve_ik(
            home.position,
            rotation_multiply(
                rotation_from_rpy(delta_rpy),
                home.rotation,
            ),
            simulation.home_joint_positions,
        )
        assert result.converged
        assert result.position_error_m < 0.002
        assert result.rotation_error_rad < 0.18


def test_rpy_round_trip_preserves_packaged_home_rotations() -> None:
    for config_path in sorted(SIM_CONFIG_DIR.glob("*_right.yaml")):
        simulation, _ = load_simulation_config(config_path)
        assert isinstance(simulation, UrdfJointSimulation)
        rotation = simulation.chain.forward(
            simulation.home_joint_positions
        ).rotation
        recovered = rotation_from_rpy(rpy_from_rotation(rotation))
        for row, expected_row in zip(recovered, rotation, strict=True):
            assert row == pytest.approx(expected_row)


def test_simulators_solve_a_useful_cartesian_range_from_home() -> None:
    minimum_translation_by_config = {
        "dm6dof_right.yaml": 0.15,
        "eco65_right.yaml": 0.15,
        "franka_fr3_right.yaml": 0.15,
        "openarm_right.yaml": 0.15,
        "piper_right.yaml": 0.15,
        "rizon4s_right.yaml": 0.15,
        "rm75_right.yaml": 0.15,
        "so101_follower_right.yaml": 0.10,
        "ur5e_right.yaml": 0.15,
    }

    for config_path in sorted(SIM_CONFIG_DIR.glob("*_right.yaml")):
        simulation, _ = load_simulation_config(config_path)
        assert isinstance(simulation, UrdfJointSimulation)
        if config_path.name.startswith("so101_"):
            # SO101 has five Cartesian arm joints plus one gripper value.
            continue
        home = simulation.chain.forward(simulation.home_joint_positions)
        distance = minimum_translation_by_config[config_path.name]
        for axis in range(3):
            results = []
            for direction in (-1.0, 1.0):
                target = list(home.position)
                target[axis] += direction * distance
                result = simulation.chain.solve_ik(
                    target,
                    home.rotation,
                    simulation.home_joint_positions,
                )
                results.append(result)
            for direction, result in zip((-1.0, 1.0), results, strict=True):
                assert result.converged, (
                    f"{config_path.name} failed axis {axis} "
                    f"direction {direction}: {result}"
                )
                assert result.position_error_m <= 0.002
                assert result.rotation_error_rad <= 0.02


def test_six_axis_simulators_solve_thirty_degree_rotation_from_home() -> None:
    for config_path in sorted(SIM_CONFIG_DIR.glob("*_right.yaml")):
        simulation, _ = load_simulation_config(config_path)
        assert isinstance(simulation, UrdfJointSimulation)
        if config_path.name == "so101_follower_right.yaml":
            # Its sixth motor moves the jaw; the Cartesian arm remains 5DoF.
            continue
        if len(simulation.chain.active_joints) < 6:
            # A five-axis arm cannot generally satisfy an arbitrary 6D pose.
            continue
        home = simulation.chain.forward(simulation.home_joint_positions)
        for axis in range(3):
            results = []
            for direction in (-1.0, 1.0):
                delta_rpy = [0.0, 0.0, 0.0]
                delta_rpy[axis] = direction * 0.3
                result = simulation.chain.solve_ik(
                    home.position,
                    rotation_multiply(
                        rotation_from_rpy(delta_rpy),
                        home.rotation,
                    ),
                    simulation.home_joint_positions,
                )
                results.append(result)
            for direction, result in zip((-1.0, 1.0), results, strict=True):
                assert result.converged, (
                    f"{config_path.name} failed rotation axis {axis} "
                    f"direction {direction}: {result}"
                )
                assert result.position_error_m <= 0.002
                assert result.rotation_error_rad <= 0.02


def test_urdf_visual_transform_uses_fixed_axis_rpy_order() -> None:
    transform = _visual_transform(
        (1.0, 2.0, 3.0),
        (0.2, -0.3, 0.4),
        (2.0, 3.0, 4.0),
    )

    expected = [
        [1.7598463526, -1.3071963944, -0.7576037324, 1.0],
        [0.7440511039, 2.6395140999, -1.1830944094, 2.0],
        [0.5910404133, 0.5693881830, 3.7451734543, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for row, expected_row in zip(transform, expected, strict=True):
        assert row == pytest.approx(expected_row)


def test_packaged_web_meshes_are_valid_compact_binary_stl() -> None:
    model_sizes: dict[str, int] = {}
    for path in MODEL_ASSET_DIR.glob("*/*"):
        data = path.read_bytes()
        assert len(data) >= 84
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        assert triangle_count > 0
        assert len(data) == 84 + 50 * triangle_count
        model_sizes[path.parent.name] = (
            model_sizes.get(path.parent.name, 0) + len(data)
        )

    assert model_sizes
    assert max(model_sizes.values()) < 1024 * 1024
    assert sum(model_sizes.values()) < 3 * 1024 * 1024


def test_cartesian_target_is_smoothed_with_joint_speed_limits() -> None:
    simulation = _fr3_simulation()
    previous = simulation.snapshot()["joint_positions"]
    simulation.set_target_delta((0.05, 0.0, 0.0), (0.0, 0.0, 0.0))

    assert simulation.snapshot()["joint_positions"] == previous
    for _ in range(90):
        simulation.advance(1.0 / 60.0)
        current = simulation.snapshot()["joint_positions"]
        assert max(
            abs(left - right) for left, right in zip(current, previous)
        ) <= 0.25 / 60.0 + 1e-9
        previous = current

    assert simulation.snapshot()["joint_positions"] != list(
        FR3_HOME_JOINT_POSITIONS
    )


def test_cartesian_target_is_clamped_to_the_measured_workspace() -> None:
    simulation = _fr3_simulation()
    home = simulation.snapshot()["home_pose"]["position"]

    simulation.set_target_delta(
        (1.0, -1.0, 1.0),
        (0.0, 0.0, 0.0),
    )
    target = simulation.snapshot()["target_pose"]["position"]

    assert target == [
        home[0] + 0.40,
        home[1] - 0.50,
        home[2],
    ]

    simulation.set_target_delta(
        (-1.0, 1.0, -1.0),
        (0.0, 0.0, 0.0),
    )
    target = simulation.snapshot()["target_pose"]["position"]

    assert target == [
        home[0] - 0.18,
        home[1] + 0.50,
        home[2] - 0.50,
    ]


def test_home_uses_the_same_smooth_joint_motion_profile() -> None:
    simulation = _fr3_simulation()
    displaced = list(FR3_HOME_JOINT_POSITIONS)
    displaced[0] += 0.2
    displaced[1] += 0.1
    simulation.set_joint_positions(displaced)
    previous = simulation.snapshot()["joint_positions"]

    simulation.home()

    assert simulation.snapshot()["joint_positions"] == previous
    for _ in range(240):
        simulation.advance(1.0 / 60.0)
        current = simulation.snapshot()["joint_positions"]
        assert max(
            abs(left - right) for left, right in zip(current, previous)
        ) <= 0.25 / 60.0 + 1e-9
        previous = current

    assert simulation.snapshot()["joint_positions"] == list(
        simulation.home_joint_positions
    )


def test_quest_bridge_keeps_waiting_before_the_controller_first_frame() -> None:
    class WaitingDualController:
        controller_mode = "dual"

        def get_controller_state(self) -> dict[str, object]:
            return {
                "calibrated": False,
                "calibration_count": 0,
                "grip_pressed": {"left": False, "right": False},
                "packet_count": 0,
                "packet_age_s": None,
            }

        def get_left_gripper_state(self) -> dict[str, float]:
            raise RuntimeError("Waiting for leftController UDP data")

    simulation = _dual_fr3_simulation()
    bridge = QuestSimulationBridge(
        simulation.left,
        bind_host="127.0.0.1",
        udp_port=0,
        source_ip="",
        translation_scale=1.0,
        controller_side="left",
        controller=WaitingDualController(),
        manage_controller_lifecycle=False,
    )

    bridge.update_once()

    snapshot = simulation.snapshot()
    assert snapshot["left"]["gripper_position"] == pytest.approx(1.0)
    assert snapshot["left"]["quest"] == {
        "enabled": True,
        "calibrated": False,
        "calibration_count": 0,
        "clutch_pressed": False,
        "packet_count": 0,
        "packet_age_s": None,
        "gripper_position": pytest.approx(1.0),
    }
