"""Tests for rynnrcp.config (loader + validator)."""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rynnrcp.config.loader import load_config
from rynnrcp.config.validator import ConfigValidator, ConfigValidationError


# ── Loader tests ─────────────────────────────────────────────────────
class TestLoader(unittest.TestCase):

    def _write_tmp(self, content: str, suffix: str) -> str:
        fd, path = tempfile.mkstemp(suffix=suffix)
        os.write(fd, content.encode())
        os.close(fd)
        self.addCleanup(os.unlink, path)
        return path

    def test_json_is_not_a_config_format(self):
        p = self._write_tmp(
            '{"config_type": "rynnrcp_app_config", "version": 1}',
            ".json",
        )
        with self.assertRaises(ValueError):
            load_config(p)

    def test_load_yaml(self):
        p = self._write_tmp(
            """
config_type: rynnrcp_core_config
version: 1
runtime:
  runner_mode: process
ipc:
  thread_transport: memory
  process_transport: shm
buffer:
  state_msg_size: 4096
  action_msg_size: 65536
  ref_msg_size: 16384
  image_padding_bytes: 1024
  shared_data_buffer_size: 104857600
  shared_data_slot_count: 100
  shared_image_data_buffer_size: 104857600
  shared_image_data_slot_count: 10
""",
            ".yaml",
        )
        cfg = load_config(p)
        self.assertEqual(cfg["config_type"], "rynnrcp_core_config")
        self.assertEqual(cfg["runtime"]["runner_mode"], "process")

    def test_load_yml(self):
        p = self._write_tmp(
            """
config_type: rynnrcp_app_config
version: 1
app:
  name: demo
defaults: {}
""",
            ".yml",
        )
        self.assertEqual(load_config(p)["app"]["name"], "demo")

    def test_missing_file(self):
        with self.assertRaises(FileNotFoundError):
            load_config("/nonexistent/file.yaml")

    def test_non_dict_raises(self):
        p = self._write_tmp('"just a string"', ".yaml")
        with self.assertRaises(ValueError):
            load_config(p)

    def test_unsupported_extension_raises(self):
        p = self._write_tmp('{"a": 1}', ".txt")
        with self.assertRaises(ValueError):
            load_config(p)


class TestValidator(unittest.TestCase):

    def test_valid_server_config(self):
        ConfigValidator.validate_source({
            "config_type": "rynnrcp_server_config",
            "version": 1,
            "manifest": {
                "robot_id": "so101_follower",
                "robot_name": "SO101 Follower",
                "capabilities": _capabilities(),
            },
            "integration": {"config": "package://demo/config/robot_integration.yaml"},
            "components": {"robot": {"enabled": True, "port": "/dev/ttyUSB0"}},
        })

    def test_server_config_rejects_old_plugin_field(self):
        with self.assertRaises(ConfigValidationError):
            ConfigValidator.validate_source({
                "config_type": "rynnrcp_server_config",
                "version": 1,
                "manifest": {
                    "robot_id": "so101_follower",
                    "robot_name": "SO101 Follower",
                    "capabilities": _capabilities(),
                },
                "integration": {"config": "package://demo/config/robot_integration.yaml"},
                "components": {"robot": {"enabled": True}},
                "plugins": [],
            })

    def test_robot_integration_accepts_ros2_standard_joint_state(self):
        cfg = _robot_integration_config()
        cfg["components"][0]["observations"] = [
            {
                "name": "joint_state",
                "type": "joint_state",
                "description": "ROS2 joint state",
                "source": {
                    "connector": "ros2",
                    "topic": "/joint_states",
                    "msg_type": "sensor_msgs.msg.JointState",
                    "payload_mode": "ros2_standard",
                    "qos": {"depth": 1},
                },
                "codec": {"adapter": "Ros2StandardInputAdapter"},
            }
        ]

        ConfigValidator.validate_source(cfg)

    def test_robot_integration_rejects_ros2_standard_type_mismatch(self):
        cfg = _robot_integration_config()
        cfg["components"][0]["observations"] = [
            {
                "name": "image",
                "type": "image",
                "description": "Invalid ROS2 image",
                "source": {
                    "connector": "ros2",
                    "topic": "/joint_states",
                    "msg_type": "sensor_msgs.msg.JointState",
                    "payload_mode": "ros2_standard",
                    "qos": {"depth": 1},
                },
                "codec": {"adapter": "ProtocolImageInputAdapter"},
            }
        ]

        with self.assertRaises(ConfigValidationError):
            ConfigValidator.validate_source(cfg)

    def test_core_config_rejects_unknown_transport(self):
        cfg = {
            "config_type": "rynnrcp_core_config",
            "version": 1,
            "runtime": {"runner_mode": "process"},
            "ipc": {"thread_transport": "l1", "process_transport": "shm"},
            "buffer": {
                "state_msg_size": 4096,
                "action_msg_size": 65536,
                "ref_msg_size": 16384,
                "image_padding_bytes": 1024,
                "shared_data_buffer_size": 104857600,
                "shared_data_slot_count": 100,
                "shared_image_data_buffer_size": 104857600,
                "shared_image_data_slot_count": 10,
            },
        }
        with self.assertRaises(ConfigValidationError):
            ConfigValidator.validate_source(cfg)

    def test_core_config_ignores_action_poll_hz(self):
        cfg = {
            "config_type": "rynnrcp_core_config",
            "version": 1,
            "runtime": {"runner_mode": "process"},
            "ipc": {"thread_transport": "memory", "process_transport": "shm"},
            "runner": {"action_poll_hz": 60},
            "buffer": {
                "state_msg_size": 4096,
                "action_msg_size": 65536,
                "ref_msg_size": 16384,
                "image_padding_bytes": 1024,
                "shared_data_buffer_size": 104857600,
                "shared_data_slot_count": 100,
                "shared_image_data_buffer_size": 104857600,
                "shared_image_data_slot_count": 10,
            },
        }
        ConfigValidator.validate_source(cfg)

    def test_valid_robot_integration(self):
        ConfigValidator.validate_source(_robot_integration_config())

    def test_robot_integration_accepts_custom_action(self):
        cfg = _robot_integration_config()
        cfg["components"][0]["actions"].append(
            {
                "name": "pick",
                "type": "custom",
                "description": "Custom pick action",
                "frame_rate": 1,
                "input_schema": {
                    "type": "object",
                    "fields": {
                        "target": {"type": "string"},
                        "speed": {"type": "float", "optional": True},
                    },
                },
                "source": {
                    "connector": "module",
                    "module_name": "demo.Controller",
                    "init": {"port": "${components.robot.port}"},
                    "start": "connect",
                    "stop": "shutdown",
                    "method_name": "pick",
                },
                "codec": {"adapter": "ProtocolActionOutputAdapter"},
            }
        )

        ConfigValidator.validate_source(cfg)

    def test_robot_integration_ignores_action_poll_hz(self):
        cfg = _robot_integration_config()
        cfg["components"][0]["actions"][0]["action_poll_hz"] = 60

        ConfigValidator.validate_source(cfg)

    def test_robot_integration_accepts_custom_action_without_input_schema(self):
        cfg = _robot_integration_config()
        cfg["components"][0]["actions"].append(
            {
                "name": "unknown",
                "type": "custom",
                "description": "Arbitrary custom action",
                "frame_rate": 1,
                "source": {
                    "connector": "module",
                    "module_name": "demo.Controller",
                    "init": {"port": "${components.robot.port}"},
                    "start": "connect",
                    "stop": "shutdown",
                    "method_name": "unknown",
                },
                "codec": {"adapter": "ProtocolActionOutputAdapter"},
            }
        )

        ConfigValidator.validate_source(cfg)

    def test_robot_integration_rejects_unknown_protocol(self):
        cfg = _robot_integration_config()
        cfg["components"][0]["observations"][0]["source"]["connector"] = "grpc"
        with self.assertRaises(ConfigValidationError):
            ConfigValidator.validate_source(cfg)

    def test_robot_integration_rejects_port_output(self):
        cfg = _robot_integration_config()
        cfg["components"][1]["actions"] = [
            {
                "name": "capture",
                "type": "custom",
                "source": {
                    "connector": "port",
                    "port_type": "demo.Camera",
                    "init": {},
                    "method_name": "capture",
                },
                "codec": {"adapter": "ProtocolActionOutputAdapter"},
            }
        ]
        with self.assertRaises(ConfigValidationError):
            ConfigValidator.validate_source(cfg)


def _capabilities() -> dict[str, bool]:
    return {
        "observations": True,
        "actions": True,
        "health": True,
        "resources": True,
        "data_collection": True,
        "policy_service": False,
    }


def _robot_integration_config():
    return {
        "config_type": "rynnrcp_robot_integration",
        "version": 1,
        "manifest": {
            "robot_id": "${manifest.robot_id}",
            "robot_name": "${manifest.robot_name}",
            "embodiment_type": "single_arm",
            "capabilities": {
                "observations": "${manifest.capabilities.observations}",
                "actions": "${manifest.capabilities.actions}",
                "health": "${manifest.capabilities.health}",
                "resources": "${manifest.capabilities.resources}",
                "data_collection": "${manifest.capabilities.data_collection}",
                "policy_service": "${manifest.capabilities.policy_service}",
            },
            "model_refs": {"urdf": "", "mjcf": "", "calibration": ""},
            "metadata": {},
        },
        "components": [
            {
                "name": "robot",
                "type": "arm",
                "parent_component": None,
                "dof": 6,
                "frame": "base",
                "description": "Demo arm",
                "enabled": "${components.robot.enabled}",
                "observations": [
                    {
                        "name": "joint_state",
                        "type": "joint_state",
                        "description": "Demo joint state",
                        "frame_rate": 60,
                        "source": {
                            "connector": "module",
                            "module_name": "demo.Controller",
                            "init": {"port": "${components.robot.port}"},
                            "start": "connect",
                            "stop": "shutdown",
                            "method_name": "get_state",
                        },
                        "codec": {"adapter": "ProtocolInputAdapter"},
                    }
                ],
                "actions": [
                    {
                        "name": "joint_position",
                        "type": "joint_position",
                        "description": "Demo joint action",
                        "frame_rate": 60,
                        "source": {
                            "connector": "module",
                            "module_name": "demo.Controller",
                            "init": {"port": "${components.robot.port}"},
                            "start": "connect",
                            "stop": "shutdown",
                            "method_name": "set_action",
                        },
                        "codec": {"adapter": "ProtocolActionOutputAdapter"},
                    }
                ],
            },
            {
                "name": "camera",
                "type": "camera",
                "parent_component": "robot",
                "frame": "camera",
                "description": "Demo camera",
                "enabled": "${components.camera.enabled}",
                "observations": [
                    {
                        "name": "image",
                        "type": "image",
                        "description": "Demo image",
                        "frame_rate": 30,
                        "source": {
                            "connector": "port",
                            "port_type": "cameras.usb_camera.USBCamera",
                            "init": {"device_id": "${components.camera.device_id}"},
                            "method_name": "read",
                        },
                        "codec": {"adapter": "ProtocolImageInputAdapter"},
                    }
                ],
            },
        ],
    }


# ── Run ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=== rynnrcp.config tests ===")
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    suite.addTests(loader.loadTestsFromTestCase(TestLoader))
    suite.addTests(loader.loadTestsFromTestCase(TestValidator))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
