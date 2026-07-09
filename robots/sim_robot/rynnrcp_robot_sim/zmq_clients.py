"""ZeroMQ clients for communicating with Isaac Sim JointManager and FrameManager.

The simulation (Isaac Sim / IsaacLab) runs ZMQ ROUTER servers:
  - JointManager on tcp://<host>:<base_port+1>  (default 8081)
  - FrameManager on tcp://<host>:<base_port+3>  (default 8083)

These lightweight clients use ZMQ DEALER sockets with pickle-serialized
request/response messages.
"""

from __future__ import annotations

import logging
import pickle
import time
from typing import Any, Dict, List, Optional

import numpy as np
import zmq

logger = logging.getLogger(__name__)


# ======================================================================
# JointClient — communicates with JointManager ZMQ server
# ======================================================================


class JointClient:
    """ZeroMQ client for the simulation JointManager server.

    Provides methods to get/set joint data for a named robot in the simulation.
    """

    def __init__(
        self,
        robot_name: str = "robot",
        server_host: str = "localhost",
        server_port: int = 8081,
        timeout_ms: int = 1000,
    ) -> None:
        self.robot_name = robot_name
        self.server_host = server_host
        self.server_port = server_port
        self.timeout_ms = timeout_ms

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.DEALER)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self._socket.connect(f"tcp://{server_host}:{server_port}")
        logger.info(
            "JointClient connected to tcp://%s:%s for robot '%s'",
            server_host,
            server_port,
            robot_name,
        )

    def get_joint_data(self) -> Optional[Dict[str, Any]]:
        """Fetch current joint state (positions, velocities, efforts, etc.)."""
        try:
            request = {"action": "get_joint_data", "robot_name": self.robot_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                return response.get("joint_data")
            logger.warning(
                "get_joint_data failed for '%s': %s",
                self.robot_name,
                response.get("message"),
            )
            return None
        except zmq.Again:
            logger.warning("Timeout getting joint data for '%s'", self.robot_name)
            return None
        except Exception as e:
            logger.error("Error getting joint data: %s", e)
            return None

    def get_joint_command(self) -> Optional[Dict[str, Any]]:
        """Fetch current joint control command (from teleop/keyboard)."""
        try:
            request = {"action": "get_joint_command", "robot_name": self.robot_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                return response.get("joint_command")
            logger.warning(
                "get_joint_command failed for '%s': %s",
                self.robot_name,
                response.get("message"),
            )
            return None
        except zmq.Again:
            logger.warning("Timeout getting joint command for '%s'", self.robot_name)
            return None
        except Exception as e:
            logger.error("Error getting joint command: %s", e)
            return None

    def get_joint_ee_pose(self) -> Optional[Dict[str, Any]]:
        """Fetch end-effector pose from the simulation."""
        try:
            request = {"action": "get_joint_ee_pose", "robot_name": self.robot_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                return response.get("joint_ee_pose")
            return None
        except zmq.Again:
            logger.warning("Timeout getting joint ee_pose for '%s'", self.robot_name)
            return None
        except Exception as e:
            logger.error("Error getting joint ee_pose: %s", e)
            return None

    def update_joint_data(self, joint_data: Dict[str, Any]) -> bool:
        """Send joint position commands to the simulation."""
        try:
            request = {
                "action": "update_joint_data",
                "robot_name": self.robot_name,
                "joint_data": joint_data,
            }
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            return response.get("status") == "success"
        except zmq.Again:
            logger.warning("Timeout updating joint data for '%s'", self.robot_name)
            return False
        except Exception as e:
            logger.error("Error updating joint data: %s", e)
            return False

    def get_robot_info(self) -> Optional[Dict[str, Any]]:
        """Fetch robot metadata (num_joints, joint_names, etc.)."""
        try:
            request = {"action": "get_robot_info", "robot_name": self.robot_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                return response.get("info")
            return None
        except zmq.Again:
            logger.warning("Timeout getting robot info for '%s'", self.robot_name)
            return None
        except Exception as e:
            logger.error("Error getting robot info: %s", e)
            return None

    def list_robots(self) -> List[str]:
        """List all registered robots in the simulation."""
        try:
            request = {"action": "list_robots"}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                return response.get("robots", [])
            return []
        except (zmq.Again, Exception):
            return []

    def close(self) -> None:
        """Release ZMQ resources."""
        if self._socket:
            self._socket.close()
        if self._context:
            self._context.term()


# ======================================================================
# FrameClient — communicates with FrameManager ZMQ server
# ======================================================================


class FrameClient:
    """ZeroMQ client for the simulation FrameManager server.

    Provides methods to get camera frames from the simulation.
    """

    def __init__(
        self,
        server_host: str = "localhost",
        server_port: int = 8083,
        timeout_ms: int = 1000,
    ) -> None:
        self.server_host = server_host
        self.server_port = server_port
        self.timeout_ms = timeout_ms

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.DEALER)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self._socket.connect(f"tcp://{server_host}:{server_port}")
        logger.info(
            "FrameClient connected to tcp://%s:%s",
            server_host,
            server_port,
        )

    def get_frame(self, camera_name: str) -> Optional[np.ndarray]:
        """Fetch a single camera frame by name (e.g. 'camera_front')."""
        try:
            request = {"action": "get_frame", "camera_name": camera_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                return response.get("frame")
            logger.warning(
                "get_frame failed for '%s': %s",
                camera_name,
                response.get("message"),
            )
            return None
        except zmq.Again:
            logger.warning("Timeout getting frame for '%s'", camera_name)
            return None
        except Exception as e:
            logger.error("Error getting frame: %s", e)
            return None

    def get_frames(self) -> Dict[str, np.ndarray]:
        """Fetch all camera frames."""
        try:
            request = {"action": "get_frames"}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                return response.get("frames", {})
            return {}
        except zmq.Again:
            logger.warning("Timeout getting all frames")
            return {}
        except Exception as e:
            logger.error("Error getting frames: %s", e)
            return {}

    def get_camera_info(self, camera_name: str) -> Optional[Dict[str, Any]]:
        """Fetch camera metadata (resolution, shape)."""
        try:
            request = {"action": "get_camera_info", "camera_name": camera_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                return response.get("info")
            return None
        except zmq.Again:
            logger.warning("Timeout getting camera info for '%s'", camera_name)
            return None
        except Exception as e:
            logger.error("Error getting camera info: %s", e)
            return None

    def close(self) -> None:
        """Release ZMQ resources."""
        if self._socket:
            self._socket.close()
        if self._context:
            self._context.term()
