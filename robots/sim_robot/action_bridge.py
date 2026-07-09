#!/usr/bin/env python3
"""Action bridge: polls joint command from simulation and dispatches via RCP.

This script bridges the gap between simulation IK computation and RCP action logging:
1. Polls get_joint_command() from JointManager via ZMQ (sim-computed joint targets)
2. Calls run_action_chunk() on the RCP Server via gRPC Interface
3. RCP action_service records the action for data collection AND executes it on sim

Usage:
    source .venv/bin/activate  # or activate your project virtual environment
    python action_bridge.py [--host localhost] [--port 8080] [--robot-id so101_sim] [--hz 30]

Requires:
    - Simulation running with --rcp_command (computes but doesn't execute)
    - RynnRCP Server running (connected to sim via ZMQ)
"""

from __future__ import annotations

import argparse
import logging
import pickle
import signal
import sys
import time
from typing import Any, Dict, List, Optional

import zmq

from rynnrcp.interface.protocol_client import connect_to_server, RcpProtocolClient

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(message)s")
logger = logging.getLogger(__name__)

_running = True


def _signal_handler(sig, frame):
    global _running
    _running = False


class JointCommandPoller:
    """Polls get_joint_command from JointManager via ZMQ."""

    def __init__(self, robot_name: str, server_host: str, server_port: int, timeout_ms: int = 500):
        self.robot_name = robot_name
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.DEALER)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._sock.connect(f"tcp://{server_host}:{server_port}")

    def get_joint_command(self) -> Optional[List[float]]:
        try:
            request = {"action": "get_joint_command", "robot_name": self.robot_name}
            self._sock.send(pickle.dumps(request))
            response = pickle.loads(self._sock.recv())
            if response.get("status") == "success":
                cmd = response.get("joint_command")
                if cmd and "positions" in cmd:
                    return cmd["positions"]
            return None
        except zmq.Again:
            return None
        except Exception as e:
            logger.warning("ZMQ error: %s", e)
            return None

    def close(self):
        self._sock.close()
        self._ctx.term()


def main():
    parser = argparse.ArgumentParser(description="Action bridge: ZMQ → RCP run_action_chunk")
    parser.add_argument("--host", default="localhost", help="Simulation ZMQ host")
    parser.add_argument("--port", type=int, default=8080, help="Simulation base port (JointManager = port+1)")
    parser.add_argument("--robot-id", default="so101_sim", help="RCP Server robot_id")
    parser.add_argument("--hz", type=float, default=30.0, help="Polling frequency")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    joint_port = args.port + 1
    interval = 1.0 / args.hz

    logger.info("Connecting to JointManager tcp://%s:%s", args.host, joint_port)
    poller = JointCommandPoller(
        robot_name="robot",
        server_host=args.host,
        server_port=joint_port,
    )

    logger.info("Connecting to RCP Server robot_id=%s", args.robot_id)
    rcp_client = connect_to_server(robot_id=args.robot_id)
    logger.info("RCP connected. Starting action bridge at %.0f Hz", args.hz)

    last_positions: Optional[List[float]] = None
    action_count = 0

    while _running:
        loop_start = time.monotonic()

        positions = poller.get_joint_command()
        if positions and any(p != 0.0 for p in positions):
            # Only send if command changed
            if positions != last_positions:
                try:
                    resp = rcp_client.run_action_chunk(
                        name="action.robot.joint_position",
                        frames=[{"joint_positions": positions}],
                        frame_rate=args.hz,
                    )
                    action_count += 1
                    if action_count % 100 == 1:
                        logger.info(
                            "Action #%d sent: [%s]",
                            action_count,
                            ", ".join(f"{p:.3f}" for p in positions[:6]),
                        )
                except Exception as e:
                    logger.error("run_action_chunk failed: %s", e)
                last_positions = list(positions)

        elapsed = time.monotonic() - loop_start
        sleep_time = max(0.0, interval - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)

    logger.info("Shutting down. Total actions sent: %d", action_count)
    poller.close()
    rcp_client.close()


if __name__ == "__main__":
    main()
