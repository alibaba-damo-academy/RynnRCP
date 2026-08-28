"""Simulation robot controller for RynnRCP.

Connects to Isaac Sim's JointManager via ZeroMQ to read/write joint data.
Supports any robot type through the n_dof parameter.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any, Dict, List

from rynnrcp.robot.base_controller import BaseRobotController
from .zmq_clients import JointClient


def _resolve_port(port: int) -> int:
    """If port is 0, read from the PORT environment variable."""
    if int(port) != 0:
        return int(port)
    env_val = os.getenv("PORT")
    if not env_val:
        raise RuntimeError("server_port=0, but environment variable PORT is not set")
    return int(env_val)


class SimRobotController(BaseRobotController):
    """Controller for a simulated robot arm in Isaac Sim.

    Communicates with the Isaac Sim JointManager ZMQ server.
    Works with any robot type - DOF is configured via n_dof parameter.

    Constructor args (from robot_integration.yaml):
      - robot_id: robot name in the simulation (default "robot")
      - server_host: ZMQ server host (default "localhost")
      - server_port: base port of the simulation (default 8080)
      - n_dof: number of joints (6 for SO101, 8 for Franka R3, etc.)
    """

    def __init__(
        self,
        robot_id: str = "robot",
        server_host: str = "localhost",
        server_port: int = 8080,
        n_dof: int = 6,
        robot_family: str | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_id = robot_id
        self.server_host = server_host
        self.server_port = _resolve_port(server_port)
        self._joint_port = self.server_port + 1
        self._n_dof = int(n_dof)
        self._robot_family = robot_family

        self._joint_client: JointClient | None = None
        self._lock = threading.Lock()
        self._running = False
        self._cached_positions: List[float] = [0.0] * self._n_dof
        self._cached_command: List[float] = [0.0] * self._n_dof

    def _collapse_franka_state(self, positions: List[float]) -> List[float]:
        """Collapse physical Franka joints (7 arm + N fingers) to 8-DOF RCP state.

        Handles 9-DOF (7+2 prismatic) and 14-DOF (7+2 prismatic + coupled) alike.
        """
        if len(positions) <= 8:
            return positions[: self._n_dof]
        arm = positions[:7]
        gripper = (positions[7] + positions[8]) / 2.0
        return arm + [gripper]

    def _expand_franka_action(self, positions: List[float]) -> List[float]:
        """Expand 8-DOF RCP action (7 arm + 1 gripper) to 9 physical Franka joints."""
        if len(positions) != 8:
            return positions
        arm = positions[:7]
        gripper = positions[7]
        return arm + [gripper, gripper]

    def start(self) -> None:
        """Connect to the simulation JointManager ZMQ server."""
        with self._lock:
            if self._running:
                return
            self._joint_client = JointClient(
                robot_name=self.robot_id,
                server_host=self.server_host,
                server_port=self._joint_port,
            )
            self._running = True
            self.logger.info(
                "SimRobotController started: robot_id=%s, n_dof=%d, server=%s:%s",
                self.robot_id,
                self._n_dof,
                self.server_host,
                self._joint_port,
            )

    def shutdown(self) -> None:
        """Disconnect from the simulation."""
        with self._lock:
            self._running = False
            if self._joint_client is not None:
                self._joint_client.close()
                self._joint_client = None
            self.logger.info("SimRobotController shutdown: robot_id=%s", self.robot_id)

    def get_joint_positions(self) -> Dict[str, List[float]]:
        """Read current joint positions from the simulation."""
        with self._lock:
            if not self._running or self._joint_client is None:
                return {"joint_positions": list(self._cached_positions)}

            joint_data = self._joint_client.get_joint_data()
            if joint_data and "positions" in joint_data:
                positions = joint_data["positions"]
                if self._robot_family == "franka_r3":
                    positions = self._collapse_franka_state(positions)
                else:
                    positions = positions[: self._n_dof]
                if len(positions) == self._n_dof:
                    self._cached_positions = list(positions)
            return {"joint_positions": list(self._cached_positions)}

    def set_joint_positions(self, value: Dict[str, Any]) -> None:
        """Send target joint positions to the simulation."""
        if not isinstance(value, dict) or "joint_positions" not in value:
            raise ValueError("set_joint_positions requires {'joint_positions': [...]}")

        positions = [float(v) for v in value["joint_positions"]]
        if len(positions) != self._n_dof:
            # action_bridge may send raw sim joints for Franka R3 (9 or 14 DOF);
            # accept and collapse to 8-DOF before expanding back.
            if self._robot_family == "franka_r3" and len(positions) > 8:
                positions = self._collapse_franka_state(positions)
            else:
                raise ValueError(f"Expected {self._n_dof} joint values, got {len(positions)}")

        with self._lock:
            if not self._running or self._joint_client is None:
                self.logger.warning("SimRobotController not started, cannot set positions")
                return
            if self._robot_family == "franka_r3":
                positions = self._expand_franka_action(positions)
            self._joint_client.update_joint_data({"positions": positions})

    def get_joint_command(self) -> Dict[str, List[float]]:
        """Read the latest joint control command from simulation."""
        with self._lock:
            if not self._running or self._joint_client is None:
                return {"joint_positions": list(self._cached_command)}

            joint_command = self._joint_client.get_joint_command()
            if joint_command and "positions" in joint_command:
                positions = joint_command["positions"]
                if self._robot_family == "franka_r3":
                    positions = self._collapse_franka_state(positions)
                else:
                    positions = positions[: self._n_dof]
                if len(positions) == self._n_dof:
                    self._cached_command = list(positions)
            return {"joint_positions": list(self._cached_command)}

    def get_health(self) -> Dict[str, Any]:
        """Return health status."""
        warnings: List[Dict[str, Any]] = []
        if not self._running:
            warnings.append(
                {
                    "code": "sim_robot.not_connected",
                    "message": "SimRobotController is not connected to simulation",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {
                        "robot_id": self.robot_id,
                        "n_dof": self._n_dof,
                        "server_host": self.server_host,
                        "server_port": self._joint_port,
                    },
                }
            )
        return {"errors": [], "warnings": warnings}


class SimBimanualRobotController(BaseRobotController):
    """Controller for a bimanual simulated robot arm pair in Isaac Sim.

    The simulation registers the two arms as independent JointManager robots
    (``left_robot`` / ``right_robot``, 6 DoF each). RynnRCP / cloud platforms,
    however, must see a SINGLE merged component: observation.state and action
    are one 12-dim vector (first 6 = left arm, last 6 = right arm), matching
    the dataset recording format used by ``run_teleop_lerobot_bimanual.py``.

    This controller hides the two ZMQ connections behind one 12-dim
    get/set interface, so state and action are never split at the RCP layer.

    Constructor args (from robot_integration.yaml):
      - left_robot_id: JointManager robot_name for the left arm (default "left_robot")
      - right_robot_id: JointManager robot_name for the right arm (default "right_robot")
      - server_host: ZMQ server host (default "localhost")
      - server_port: base port of the simulation (default 8080)
      - n_dof_per_arm: joints per arm (default 6 for SO101)
    """

    def __init__(
        self,
        left_robot_id: str = "left_robot",
        right_robot_id: str = "right_robot",
        server_host: str = "localhost",
        server_port: int = 8080,
        n_dof_per_arm: int = 6,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.left_robot_id = left_robot_id
        self.right_robot_id = right_robot_id
        self.server_host = server_host
        self.server_port = _resolve_port(server_port)
        self._joint_port = self.server_port + 1
        self._n_dof_per_arm = int(n_dof_per_arm)
        self._n_dof = self._n_dof_per_arm * 2

        self._left_client: JointClient | None = None
        self._right_client: JointClient | None = None
        self._lock = threading.Lock()
        self._running = False
        self._cached_positions: List[float] = [0.0] * self._n_dof
        self._cached_command: List[float] = [0.0] * self._n_dof

    def start(self) -> None:
        """Connect to both arms' JointManager ZMQ endpoints."""
        with self._lock:
            if self._running:
                return
            self._left_client = JointClient(
                robot_name=self.left_robot_id,
                server_host=self.server_host,
                server_port=self._joint_port,
            )
            self._right_client = JointClient(
                robot_name=self.right_robot_id,
                server_host=self.server_host,
                server_port=self._joint_port,
            )
            self._running = True
            self.logger.info(
                "SimBimanualRobotController started: left=%s, right=%s, n_dof=%d, server=%s:%s",
                self.left_robot_id,
                self.right_robot_id,
                self._n_dof,
                self.server_host,
                self._joint_port,
            )

    def shutdown(self) -> None:
        """Disconnect from the simulation."""
        with self._lock:
            self._running = False
            if self._left_client is not None:
                self._left_client.close()
                self._left_client = None
            if self._right_client is not None:
                self._right_client.close()
                self._right_client = None
            self.logger.info(
                "SimBimanualRobotController shutdown: left=%s, right=%s",
                self.left_robot_id,
                self.right_robot_id,
            )

    def get_joint_positions(self) -> Dict[str, List[float]]:
        """Read current joint positions from both arms and merge into one 12-dim vector."""
        n = self._n_dof_per_arm
        with self._lock:
            if not self._running or self._left_client is None or self._right_client is None:
                return {"joint_positions": list(self._cached_positions)}

            left_data = self._left_client.get_joint_data()
            right_data = self._right_client.get_joint_data()
            left_pos = (
                left_data["positions"][:n]
                if left_data and "positions" in left_data
                else self._cached_positions[:n]
            )
            right_pos = (
                right_data["positions"][:n]
                if right_data and "positions" in right_data
                else self._cached_positions[n:]
            )
            self._cached_positions = list(left_pos) + list(right_pos)
            return {"joint_positions": list(self._cached_positions)}

    def set_joint_positions(self, value: Dict[str, Any]) -> None:
        """Send target joint positions (merged 12-dim vector) to both arms."""
        if not isinstance(value, dict) or "joint_positions" not in value:
            raise ValueError("set_joint_positions requires {'joint_positions': [...]}")

        positions = [float(v) for v in value["joint_positions"]]
        if len(positions) != self._n_dof:
            raise ValueError(f"Expected {self._n_dof} joint values, got {len(positions)}")

        n = self._n_dof_per_arm
        with self._lock:
            if not self._running or self._left_client is None or self._right_client is None:
                self.logger.warning("SimBimanualRobotController not started, cannot set positions")
                return
            self._left_client.update_joint_data({"positions": positions[:n]})
            self._right_client.update_joint_data({"positions": positions[n:]})

    def get_joint_command(self) -> Dict[str, List[float]]:
        """Read the latest merged 12-dim joint command from both arms."""
        n = self._n_dof_per_arm
        with self._lock:
            if not self._running or self._left_client is None or self._right_client is None:
                return {"joint_positions": list(self._cached_command)}

            left_cmd = self._left_client.get_joint_command()
            right_cmd = self._right_client.get_joint_command()
            left_pos = self._cached_command[:n]
            right_pos = self._cached_command[n:]
            # 全零命令也是合法命令，不再用 any(p != 0.0) 过滤
            if left_cmd and "positions" in left_cmd and len(left_cmd["positions"]) >= n:
                left_pos = left_cmd["positions"][:n]
            if right_cmd and "positions" in right_cmd and len(right_cmd["positions"]) >= n:
                right_pos = right_cmd["positions"][:n]
            self._cached_command = list(left_pos) + list(right_pos)
            return {"joint_positions": list(self._cached_command)}

    def get_health(self) -> Dict[str, Any]:
        """Return health status."""
        warnings: List[Dict[str, Any]] = []
        if not self._running:
            warnings.append(
                {
                    "code": "sim_robot.not_connected",
                    "message": "SimBimanualRobotController is not connected to simulation",
                    "source": "robot",
                    "timestamp": time.time(),
                    "details": {
                        "left_robot_id": self.left_robot_id,
                        "right_robot_id": self.right_robot_id,
                        "n_dof": self._n_dof,
                        "server_host": self.server_host,
                        "server_port": self._joint_port,
                    },
                }
            )
        return {"errors": [], "warnings": warnings}
