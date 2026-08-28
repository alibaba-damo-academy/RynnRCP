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
from typing import Any, Dict, List, Optional

import numpy as np
import zmq

from rynnrcp.utils.log_gate import LogGate

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
        self._log_gates: Dict[str, LogGate] = {}
        logger.info(
            "[SimZMQ][JOINT_CONNECTED] endpoint=tcp://%s:%s robot=%s",
            server_host,
            server_port,
            robot_name,
        )

    def _gate(self, operation: str) -> LogGate:
        return self._log_gates.setdefault(
            operation,
            LogGate(
                logger,
                f"SimZMQ/JointClient/{self.robot_name}/{operation}",
                interval_s=5.0,
                level=logging.WARNING,
            ),
        )

    def _recover(self, operation: str) -> None:
        gate = self._log_gates.pop(operation, None)
        if gate is not None:
            gate.success()

    def get_joint_data(self) -> Optional[Dict[str, Any]]:
        """Fetch current joint state (positions, velocities, efforts, etc.)."""
        try:
            request = {"action": "get_joint_data", "robot_name": self.robot_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                self._recover("get_joint_data")
                return response.get("joint_data")
            self._gate("get_joint_data").failure(
                "robot=%s endpoint=tcp://%s:%s status=%s message=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return None
        except zmq.Again:
            self._gate("get_joint_data").failure(
                "robot=%s endpoint=tcp://%s:%s request timed out",
                self.robot_name,
                self.server_host,
                self.server_port,
            )
            return None
        except Exception as e:
            self._gate("get_joint_data").failure(
                "robot=%s endpoint=tcp://%s:%s error=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                e,
                exc_info=True,
            )
            return None

    def get_joint_command(self) -> Optional[Dict[str, Any]]:
        """Fetch current joint control command (from teleop/keyboard)."""
        try:
            request = {"action": "get_joint_command", "robot_name": self.robot_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                self._recover("get_joint_command")
                return response.get("joint_command")
            self._gate("get_joint_command").failure(
                "robot=%s endpoint=tcp://%s:%s status=%s message=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return None
        except zmq.Again:
            self._gate("get_joint_command").failure(
                "robot=%s endpoint=tcp://%s:%s request timed out",
                self.robot_name,
                self.server_host,
                self.server_port,
            )
            return None
        except Exception as e:
            self._gate("get_joint_command").failure(
                "robot=%s endpoint=tcp://%s:%s error=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                e,
                exc_info=True,
            )
            return None

    def get_joint_ee_pose(self) -> Optional[Dict[str, Any]]:
        """Fetch end-effector pose from the simulation."""
        try:
            request = {"action": "get_joint_ee_pose", "robot_name": self.robot_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                self._recover("get_joint_ee_pose")
                return response.get("joint_ee_pose")
            self._gate("get_joint_ee_pose").failure(
                "robot=%s endpoint=tcp://%s:%s status=%s message=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return None
        except zmq.Again:
            self._gate("get_joint_ee_pose").failure(
                "robot=%s endpoint=tcp://%s:%s request timed out",
                self.robot_name,
                self.server_host,
                self.server_port,
            )
            return None
        except Exception as e:
            self._gate("get_joint_ee_pose").failure(
                "robot=%s endpoint=tcp://%s:%s error=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                e,
                exc_info=True,
            )
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
            if response.get("status") == "success":
                self._recover("update_joint_data")
                return True
            self._gate("update_joint_data").failure(
                "robot=%s endpoint=tcp://%s:%s status=%s message=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return False
        except zmq.Again:
            self._gate("update_joint_data").failure(
                "robot=%s endpoint=tcp://%s:%s request timed out",
                self.robot_name,
                self.server_host,
                self.server_port,
            )
            return False
        except Exception as e:
            self._gate("update_joint_data").failure(
                "robot=%s endpoint=tcp://%s:%s error=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                e,
                exc_info=True,
            )
            return False

    def get_robot_info(self) -> Optional[Dict[str, Any]]:
        """Fetch robot metadata (num_joints, joint_names, etc.)."""
        try:
            request = {"action": "get_robot_info", "robot_name": self.robot_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                self._recover("get_robot_info")
                return response.get("info")
            self._gate("get_robot_info").failure(
                "robot=%s endpoint=tcp://%s:%s status=%s message=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return None
        except zmq.Again:
            self._gate("get_robot_info").failure(
                "robot=%s endpoint=tcp://%s:%s request timed out",
                self.robot_name,
                self.server_host,
                self.server_port,
            )
            return None
        except Exception as e:
            self._gate("get_robot_info").failure(
                "robot=%s endpoint=tcp://%s:%s error=%s",
                self.robot_name,
                self.server_host,
                self.server_port,
                e,
                exc_info=True,
            )
            return None

    def list_robots(self) -> List[str]:
        """List all registered robots in the simulation."""
        try:
            request = {"action": "list_robots"}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                self._recover("list_robots")
                return response.get("robots", [])
            self._gate("list_robots").failure(
                "endpoint=tcp://%s:%s status=%s message=%s",
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return []
        except zmq.Again:
            self._gate("list_robots").failure(
                "endpoint=tcp://%s:%s request timed out",
                self.server_host,
                self.server_port,
            )
            return []
        except Exception as exc:
            self._gate("list_robots").failure(
                "endpoint=tcp://%s:%s error=%s",
                self.server_host,
                self.server_port,
                exc,
                exc_info=True,
            )
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
        self._log_gates: Dict[str, LogGate] = {}
        logger.info(
            "[SimZMQ][FRAME_CONNECTED] endpoint=tcp://%s:%s",
            server_host,
            server_port,
        )

    def _gate(self, operation: str, target: str | None = None) -> LogGate:
        gate_key = f"{operation}:{target}" if target else operation
        source = f"SimZMQ/FrameClient/{operation}"
        if target:
            source = f"{source}/{target}"
        return self._log_gates.setdefault(
            gate_key,
            LogGate(
                logger,
                source,
                interval_s=5.0,
                level=logging.WARNING,
            ),
        )

    def _recover(self, operation: str, target: str | None = None) -> None:
        gate_key = f"{operation}:{target}" if target else operation
        gate = self._log_gates.pop(gate_key, None)
        if gate is not None:
            gate.success()

    def get_frame(self, camera_name: str) -> Optional[np.ndarray]:
        """Fetch a single camera frame by name (e.g. 'camera_front')."""
        try:
            request = {"action": "get_frame", "camera_name": camera_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                self._recover("get_frame", camera_name)
                return response.get("frame")
            self._gate("get_frame", camera_name).failure(
                "camera=%s endpoint=tcp://%s:%s status=%s message=%s",
                camera_name,
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return None
        except zmq.Again:
            self._gate("get_frame", camera_name).failure(
                "camera=%s endpoint=tcp://%s:%s request timed out",
                camera_name,
                self.server_host,
                self.server_port,
            )
            return None
        except Exception as e:
            self._gate("get_frame", camera_name).failure(
                "camera=%s endpoint=tcp://%s:%s error=%s",
                camera_name,
                self.server_host,
                self.server_port,
                e,
                exc_info=True,
            )
            return None

    def get_frames(self) -> Dict[str, np.ndarray]:
        """Fetch all camera frames."""
        try:
            request = {"action": "get_frames"}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                self._recover("get_frames")
                return response.get("frames", {})
            self._gate("get_frames").failure(
                "endpoint=tcp://%s:%s status=%s message=%s",
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return {}
        except zmq.Again:
            self._gate("get_frames").failure(
                "endpoint=tcp://%s:%s request timed out",
                self.server_host,
                self.server_port,
            )
            return {}
        except Exception as e:
            self._gate("get_frames").failure(
                "endpoint=tcp://%s:%s error=%s",
                self.server_host,
                self.server_port,
                e,
                exc_info=True,
            )
            return {}

    def get_camera_info(self, camera_name: str) -> Optional[Dict[str, Any]]:
        """Fetch camera metadata (resolution, shape)."""
        try:
            request = {"action": "get_camera_info", "camera_name": camera_name}
            self._socket.send(pickle.dumps(request))
            response = pickle.loads(self._socket.recv())
            if response.get("status") == "success":
                self._recover("get_camera_info", camera_name)
                return response.get("info")
            self._gate("get_camera_info", camera_name).failure(
                "camera=%s endpoint=tcp://%s:%s status=%s message=%s",
                camera_name,
                self.server_host,
                self.server_port,
                response.get("status"),
                response.get("message"),
            )
            return None
        except zmq.Again:
            self._gate("get_camera_info", camera_name).failure(
                "camera=%s endpoint=tcp://%s:%s request timed out",
                camera_name,
                self.server_host,
                self.server_port,
            )
            return None
        except Exception as e:
            self._gate("get_camera_info", camera_name).failure(
                "camera=%s endpoint=tcp://%s:%s error=%s",
                camera_name,
                self.server_host,
                self.server_port,
                e,
                exc_info=True,
            )
            return None

    def close(self) -> None:
        """Release ZMQ resources."""
        if self._socket:
            self._socket.close()
        if self._context:
            self._context.term()
