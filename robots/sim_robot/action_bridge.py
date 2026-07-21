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
from typing import Any, Dict, List, Optional, Tuple

import zmq

from rynnrcp.interface.protocol_client import connect_to_server, RcpProtocolClient

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(message)s")
logger = logging.getLogger(__name__)

_running = True


def _signal_handler(sig, frame):
    global _running
    _running = False


class JointCommandPoller:
    """Polls get_joint_command from JointManager via ZMQ for a single simulation robot_name."""

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


class MergedJointCommandPoller:
    """Polls multiple simulation robot_names and concatenates positions in order.

    Used for bimanual/multi-arm robots where the simulation registers each arm
    as an independent JointManager robot (e.g. left_robot, right_robot), but
    RCP must see ONE merged action (e.g. 12-dim = left 6 + right 6).
    """

    def __init__(self, robot_names: List[str], server_host: str, server_port: int, timeout_ms: int = 500):
        self.robot_names = list(robot_names)
        self._pollers = [
            JointCommandPoller(name, server_host, server_port, timeout_ms)
            for name in self.robot_names
        ]

    def get_joint_command(self) -> Optional[List[float]]:
        merged: List[float] = []
        for poller in self._pollers:
            positions = poller.get_joint_command()
            if positions is None:
                return None
            merged.extend(positions)
        return merged

    def close(self):
        for poller in self._pollers:
            poller.close()


def _parse_mapping(value: str) -> Tuple[str, str]:
    if ":" not in value:
        raise argparse.ArgumentTypeError(
            "mapping must be '<sim_robot_name>[+<sim_robot_name>...]:<rcp_action_name>'"
        )
    sim_robot_name, action_name = value.split(":", 1)
    sim_robot_name = sim_robot_name.strip()
    action_name = action_name.strip()
    if not sim_robot_name or not action_name:
        raise argparse.ArgumentTypeError(
            "mapping must contain both simulation robot name(s) and RCP action name"
        )
    return sim_robot_name, action_name


def _dispatch_action_frame(
    rcp_client: RcpProtocolClient,
    action_name: str,
    positions: Optional[List[float]],
    frame_rate: float,
) -> bool:
    """Publish one action sample; unchanged and all-zero samples are valid."""
    if positions is None or len(positions) == 0:
        return False
    rcp_client.run_action_chunk(
        name=action_name,
        frames=[{"joint_positions": positions}],
        frame_rate=frame_rate,
    )
    return True



def main():
    parser = argparse.ArgumentParser(description="Action bridge: ZMQ → RCP run_action_chunk")
    parser.add_argument("--host", default="localhost", help="Simulation ZMQ host")
    parser.add_argument("--port", type=int, default=8080, help="Simulation base port (JointManager = port+1)")
    parser.add_argument("--robot-id", default="so101_sim", help="RCP Server robot_id")
    parser.add_argument("--hz", type=float, default=30.0, help="Polling frequency")
    parser.add_argument(
        "--mapping",
        action="append",
        type=_parse_mapping,
        default=[],
        help=(
            "Bridge mapping '<sim_robot_name>:<rcp_action_name>'. "
            "Can be repeated for multi-arm robots. Default: robot:action.robot.joint_position"
        ),
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    mappings: List[Tuple[str, str]] = args.mapping or [
        ("robot", "action.robot.joint_position")
    ]
    joint_port = args.port + 1
    interval = 1.0 / args.hz

    logger.info("Connecting to JointManager tcp://%s:%s", args.host, joint_port)
    pollers: Dict[str, Any] = {}
    for sim_robot_key, _ in mappings:
        robot_names = [n.strip() for n in sim_robot_key.split("+") if n.strip()]
        if len(robot_names) > 1:
            pollers[sim_robot_key] = MergedJointCommandPoller(
                robot_names=robot_names,
                server_host=args.host,
                server_port=joint_port,
            )
        else:
            pollers[sim_robot_key] = JointCommandPoller(
                robot_name=robot_names[0],
                server_host=args.host,
                server_port=joint_port,
            )

    logger.info("Connecting to RCP Server robot_id=%s", args.robot_id)
    rcp_client = None
    max_retries = 10
    retry_delay = 2.0
    for attempt in range(1, max_retries + 1):
        try:
            rcp_client = connect_to_server(robot_id=args.robot_id)
            break
        except Exception as e:
            logger.warning("RCP connect attempt %d/%d failed: %s", attempt, max_retries, e)
            if attempt < max_retries:
                time.sleep(retry_delay)
            else:
                raise
    logger.info("RCP connected. Starting action bridge at %.0f Hz", args.hz)
    for sim_robot_name, action_name in mappings:
        logger.info("Bridge mapping: %s -> %s", sim_robot_name, action_name)

    # 等待仿真侧准备好第一条 joint command，避免过早 poll 触发 joint_data_manager 报错
    logger.info("Waiting for first joint command from simulation...")
    ready = False
    deadline = time.monotonic() + 10.0
    while not ready and time.monotonic() < deadline and _running:
        for sim_robot_name, _ in mappings:
            try:
                positions = pollers[sim_robot_name].get_joint_command()
                if positions:
                    ready = True
                    break
            except Exception:
                pass
        if not ready:
            time.sleep(0.1)
    if ready:
        logger.info("First joint command ready, starting main loop")
    else:
        logger.warning("Timed out waiting for first joint command, starting main loop anyway")

    action_count = 0

    while _running:
        loop_start = time.monotonic()

        for sim_robot_name, action_name in mappings:
            try:
                positions = pollers[sim_robot_name].get_joint_command()
            except Exception as e:
                # 仿真侧可能还没准备好（right_arm_joints_target 为 None），跳过本帧
                logger.warning("Failed to get joint command for %s: %s", sim_robot_name, e)
                continue
            try:
                if not _dispatch_action_frame(
                    rcp_client, action_name, positions, args.hz
                ):
                    continue
                action_count += 1
                if action_count % 100 == 1:
                    logger.info(
                        "Action #%d sent for %s: [%s]",
                        action_count,
                        sim_robot_name,
                        ", ".join(f"{p:.3f}" for p in positions[:6]),
                    )
            except Exception as e:
                logger.error("run_action_chunk failed for %s: %s", sim_robot_name, e)

        elapsed = time.monotonic() - loop_start
        sleep_time = max(0.0, interval - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)

    logger.info("Shutting down. Total actions sent: %d", action_count)
    for poller in pollers.values():
        poller.close()
    rcp_client.close()


if __name__ == "__main__":
    main()
