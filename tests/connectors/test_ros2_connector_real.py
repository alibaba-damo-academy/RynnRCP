"""Optional smoke test for the real ROS2 connector.

This test is skipped unless a working ROS2 Python environment is available.
On macOS, ``rclpy`` is normally provided by a ROS2 installation rather than
PyPI, so the standard local test path uses ``MockROS2Connector`` in
``test_connector_runtime.py``.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

rclpy = pytest.importorskip("rclpy")

from rynnrcp.connectors.ros2_connector import ROS2Connector


def test_real_ros2_connector_can_initialize_and_stop():
    connector = ROS2Connector(node_name="rynnrcp_ros2_smoke_test")
    try:
        assert connector.context.ok()
    finally:
        connector.stop()
