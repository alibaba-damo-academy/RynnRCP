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
      - n_dof: number of joints (6 for SO101, 9 for RM75, etc.)
    """

    def __init__(
        self,
        robot_id: str = "robot",
        server_host: str = "localhost",
        server_port: int = 8080,
        n_dof: int = 6,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(logger=logger)
        self.robot_id = robot_id
        self.server_host = server_host
        self.server_port = _resolve_port(server_port)
        self._joint_port = self.server_port + 1
        self._n_dof = int(n_dof)

        self._joint_client: JointClient | None = None
        self._lock = threading.Lock()
        self._running = False
        self._cached_positions: List[float] = [0.0] * self._n_dof
        self._cached_command: List[float] = [0.0] * self._n_dof

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
                positions = joint_data["positions"][:self._n_dof]
                self._cached_positions = list(positions)
            return {"joint_positions": list(self._cached_positions)}

    def set_joint_positions(self, value: Dict[str, Any]) -> None:
        """Send target joint positions to the simulation."""
        if not isinstance(value, dict) or "joint_positions" not in value:
            raise ValueError("set_joint_positions requires {'joint_positions': [...]}")

        positions = [float(v) for v in value["joint_positions"]]
        if len(positions) != self._n_dof:
            raise ValueError(f"Expected {self._n_dof} joint values, got {len(positions)}")

        with self._lock:
            if not self._running or self._joint_client is None:
                self.logger.warning("SimRobotController not started, cannot set positions")
                return
            self._joint_client.update_joint_data({"positions": positions})

    def get_joint_command(self) -> Dict[str, List[float]]:
        """Read the latest joint control command from simulation."""
        with self._lock:
            if not self._running or self._joint_client is None:
                return {"joint_positions": list(self._cached_command)}

            joint_command = self._joint_client.get_joint_command()
            if joint_command and "positions" in joint_command:
                positions = joint_command["positions"][:self._n_dof]
                if any(p != 0.0 for p in positions):
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
