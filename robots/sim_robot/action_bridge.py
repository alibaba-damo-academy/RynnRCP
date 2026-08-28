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
import time
from typing import Any, Dict, List, NamedTuple, Optional, Tuple

import zmq

from rynnrcp.interface.protocol_client import connect_to_server, RcpProtocolClient
from rynnrcp.utils.log_gate import LogGate
from rynnrcp.utils.logging import (
    configure_logging,
    log_file_enabled,
    resolve_log_run_id,
    set_log_context,
)
from rynnrcp.utils.user_paths import logs_dir, robot_root

LOGGER_NAME = "rynnrcp.action_bridge"
logger = logging.getLogger(LOGGER_NAME)

_running = True


class ActionReceiveEvent(NamedTuple):
    total: int
    dimension: int
    previous_dimension: Optional[int]
    all_zero: bool
    consecutive_zero: int
    zero_duration: float
    should_warn_zero: bool
    fps: Optional[float]
    window_count: int
    window_zero_count: int
    window_changed_count: int
    window_max_delta: float
    joint_ranges: Tuple[float, ...]


def _signal_handler(sig, frame):
    global _running
    _running = False


def _configure_action_bridge_logging(robot_id: str) -> str:
    root = robot_root(robot_id)
    log_path = logs_dir(root) / "action_bridge.log"
    set_log_context(
        robot_id=robot_id,
        run_id=resolve_log_run_id(),
        process="action_bridge",
    )
    configure_logging(
        level=logging.INFO,
        sinks=["stderr", "file"],
        file_path=str(log_path),
        logger_name=LOGGER_NAME,
    )
    if log_file_enabled():
        logger.info(
            "[ActionBridge][LOG_FILE_TARGET] path=%s",
            log_path,
        )
    else:
        logger.info(
            "[ActionBridge][LOG_FILE_DISABLED] set RYNNRCP_LOG_TO_FILE=1 to write %s",
            log_path,
        )
    return str(log_path)


class JointCommandPoller:
    """Polls get_joint_command from JointManager via ZMQ for a single simulation robot_name."""

    def __init__(
        self, robot_name: str, server_host: str, server_port: int, timeout_ms: int = 500
    ):
        self.robot_name = robot_name
        self._endpoint = f"tcp://{server_host}:{server_port}"
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.DEALER)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._sock.connect(self._endpoint)
        self._availability_log: LogGate | None = None

    def _recover_availability_log(self) -> None:
        gate = self._availability_log
        if gate is not None:
            self._availability_log = None
            gate.success()

    def _get_availability_log(self) -> LogGate:
        if self._availability_log is None:
            self._availability_log = LogGate(
                logger,
                f"ActionBridge/ZMQ_NO_ACTION/{self.robot_name}",
                interval_s=5.0,
                level=logging.WARNING,
            )
        return self._availability_log

    def get_joint_command(self) -> Optional[List[float]]:
        try:
            request = {"action": "get_joint_command", "robot_name": self.robot_name}
            self._sock.send(pickle.dumps(request))
            response = pickle.loads(self._sock.recv())
            if response.get("status") == "success":
                cmd = response.get("joint_command")
                if cmd and "positions" in cmd:
                    positions = cmd["positions"]
                    if positions is not None and len(positions) > 0:
                        self._recover_availability_log()
                        return positions
                    self._record_missing_action("joint_command.positions is empty")
                    return None
                self._record_missing_action("response has no joint_command.positions")
                return None
            self._record_missing_action(
                f"server status={response.get('status')} message={response.get('message')}"
            )
            return None
        except zmq.Again:
            self._record_missing_action("request timed out")
            return None
        except Exception as e:
            self._get_availability_log().failure(
                "source=%s endpoint=%s error=%s; verify the simulator endpoint and protocol",
                self.robot_name,
                self._endpoint,
                e,
                exc_info=True,
            )
            return None

    def _record_missing_action(self, reason: str) -> None:
        self._get_availability_log().failure(
            "source=%s endpoint=%s reason=%s; check the simulator's JointManager command publisher",
            self.robot_name,
            self._endpoint,
            reason,
        )

    def close(self):
        self._sock.close()
        self._ctx.term()


class MergedJointCommandPoller:
    """Polls multiple simulation robot_names and concatenates positions in order.

    Used for bimanual/multi-arm robots where the simulation registers each arm
    as an independent JointManager robot (e.g. left_robot, right_robot), but
    RCP must see ONE merged action (e.g. 12-dim = left 6 + right 6).
    """

    def __init__(
        self,
        robot_names: List[str],
        server_host: str,
        server_port: int,
        timeout_ms: int = 500,
    ):
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
    response = rcp_client.run_action_chunk(
        name=action_name,
        frames=[{"joint_positions": positions}],
        frame_rate=frame_rate,
    )
    if response is not None and not getattr(response, "ok", True):
        logger.error(
            "[ActionBridge][RCP_REJECT] target=%s dim=%d status=%s message=%s",
            action_name,
            len(positions),
            getattr(response, "status", "unknown"),
            getattr(response, "message", "unknown"),
        )
        return False
    return True


def _format_positions(positions: List[float], limit: int = 12) -> str:
    displayed = ", ".join(f"{float(value):.4f}" for value in positions[:limit])
    remaining = len(positions) - limit
    suffix = f", ... (+{remaining})" if remaining > 0 else ""
    return f"[{displayed}{suffix}]"


def _update_action_receive_stats(
    stats: Dict[str, Dict[str, Any]],
    sim_robot_name: str,
    positions: List[float],
    log_interval: float = 1.0,
) -> ActionReceiveEvent:
    """Update receive FPS stats for one valid action frame."""
    now = time.monotonic()
    numeric_positions = [float(value) for value in positions]
    dimension = len(numeric_positions)
    all_zero = all(value == 0.0 for value in numeric_positions)
    robot_stats = stats.setdefault(
        sim_robot_name,
        {
            "total": 0.0,
            "window_count": 0.0,
            "window_zero_count": 0.0,
            "window_changed_count": 0.0,
            "window_max_delta": 0.0,
            "window_start": now,
            "dimension": float(dimension),
            "consecutive_zero": 0.0,
            "zero_start": 0.0,
            "last_zero_warning": 0.0,
            "last_positions": [],
            "window_min": list(numeric_positions),
            "window_max": list(numeric_positions),
        },
    )
    previous_dimension = int(robot_stats["dimension"])
    robot_stats["dimension"] = float(dimension)
    robot_stats["total"] += 1.0
    robot_stats["window_count"] += 1.0
    previous_positions = list(robot_stats["last_positions"])
    if previous_positions:
        if len(previous_positions) == dimension:
            deltas = [
                abs(current - previous)
                for current, previous in zip(numeric_positions, previous_positions)
            ]
            frame_max_delta = max(deltas, default=0.0)
        else:
            frame_max_delta = 0.0
        if len(previous_positions) != dimension or frame_max_delta > 1e-9:
            robot_stats["window_changed_count"] += 1.0
        robot_stats["window_max_delta"] = max(
            float(robot_stats["window_max_delta"]),
            frame_max_delta,
        )
    robot_stats["last_positions"] = list(numeric_positions)

    window_min = list(robot_stats["window_min"])
    window_max = list(robot_stats["window_max"])
    if len(window_min) != dimension or len(window_max) != dimension:
        window_min = list(numeric_positions)
        window_max = list(numeric_positions)
    else:
        window_min = [
            min(previous, current)
            for previous, current in zip(window_min, numeric_positions)
        ]
        window_max = [
            max(previous, current)
            for previous, current in zip(window_max, numeric_positions)
        ]
    robot_stats["window_min"] = window_min
    robot_stats["window_max"] = window_max
    joint_ranges = tuple(
        maximum - minimum for minimum, maximum in zip(window_min, window_max)
    )

    if all_zero:
        robot_stats["window_zero_count"] += 1.0
        robot_stats["consecutive_zero"] += 1.0
        if robot_stats["zero_start"] == 0.0:
            robot_stats["zero_start"] = now
    else:
        robot_stats["consecutive_zero"] = 0.0
        robot_stats["zero_start"] = 0.0

    zero_duration = (
        now - robot_stats["zero_start"]
        if all_zero and robot_stats["zero_start"] > 0.0
        else 0.0
    )
    should_warn_zero = (
        zero_duration >= 3.0 and now - robot_stats["last_zero_warning"] >= 5.0
    )
    if should_warn_zero:
        robot_stats["last_zero_warning"] = now

    elapsed = now - robot_stats["window_start"]
    if elapsed < log_interval:
        return ActionReceiveEvent(
            total=int(robot_stats["total"]),
            dimension=dimension,
            previous_dimension=previous_dimension,
            all_zero=all_zero,
            consecutive_zero=int(robot_stats["consecutive_zero"]),
            zero_duration=zero_duration,
            should_warn_zero=should_warn_zero,
            fps=None,
            window_count=int(robot_stats["window_count"]),
            window_zero_count=int(robot_stats["window_zero_count"]),
            window_changed_count=int(robot_stats["window_changed_count"]),
            window_max_delta=float(robot_stats["window_max_delta"]),
            joint_ranges=joint_ranges,
        )

    fps = robot_stats["window_count"] / max(elapsed, 1e-9)
    window_count = int(robot_stats["window_count"])
    window_zero_count = int(robot_stats["window_zero_count"])
    window_changed_count = int(robot_stats["window_changed_count"])
    window_max_delta = float(robot_stats["window_max_delta"])
    robot_stats["window_count"] = 0.0
    robot_stats["window_zero_count"] = 0.0
    robot_stats["window_changed_count"] = 0.0
    robot_stats["window_max_delta"] = 0.0
    robot_stats["window_min"] = []
    robot_stats["window_max"] = []
    robot_stats["window_start"] = now
    return ActionReceiveEvent(
        total=int(robot_stats["total"]),
        dimension=dimension,
        previous_dimension=previous_dimension,
        all_zero=all_zero,
        consecutive_zero=int(robot_stats["consecutive_zero"]),
        zero_duration=zero_duration,
        should_warn_zero=should_warn_zero,
        fps=fps,
        window_count=window_count,
        window_zero_count=window_zero_count,
        window_changed_count=window_changed_count,
        window_max_delta=window_max_delta,
        joint_ranges=joint_ranges,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Action bridge: ZMQ → RCP run_action_chunk"
    )
    parser.add_argument("--host", default="localhost", help="Simulation ZMQ host")
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="Simulation base port (JointManager = port+1)",
    )
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
    _configure_action_bridge_logging(args.robot_id)

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
            logger.warning(
                "[ActionBridge][RCP_CONNECT_RETRY] attempt=%d/%d robot_id=%s error=%s",
                attempt,
                max_retries,
                args.robot_id,
                e,
            )
            if attempt < max_retries:
                time.sleep(retry_delay)
            else:
                logger.exception(
                    "[ActionBridge][RCP_CONNECT_FAILED] robot_id=%s attempts=%d; "
                    "verify that the RCP server is running and discoverable",
                    args.robot_id,
                    max_retries,
                )
                raise
    logger.info("RCP connected. Starting action bridge at %.0f Hz", args.hz)
    for sim_robot_name, action_name in mappings:
        logger.info("Bridge mapping: %s -> %s", sim_robot_name, action_name)

    # 等待仿真侧准备好第一条 joint command，避免过早 poll 触发 joint_data_manager 报错
    logger.info("Waiting for first joint command from simulation...")
    ready = False
    deadline = time.monotonic() + 10.0
    poll_error_logs: Dict[str, LogGate] = {}
    while not ready and time.monotonic() < deadline and _running:
        for sim_robot_name, _ in mappings:
            try:
                positions = pollers[sim_robot_name].get_joint_command()
                gate = poll_error_logs.pop(sim_robot_name, None)
                if gate is not None:
                    gate.success()
                if positions is not None and len(positions) > 0:
                    ready = True
                    break
            except Exception as exc:
                poll_error_logs.setdefault(
                    sim_robot_name,
                    LogGate(
                        logger,
                        f"ActionBridge/POLL_ERROR/{sim_robot_name}",
                        interval_s=5.0,
                        level=logging.ERROR,
                    ),
                ).failure(
                    "source=%s error=%s; inspect the poller implementation and simulator protocol",
                    sim_robot_name,
                    exc,
                    exc_info=True,
                )
        if not ready:
            time.sleep(0.1)
    if ready:
        logger.info("First joint command ready, starting main loop")
    else:
        logger.warning(
            "Timed out waiting for first joint command, starting main loop anyway"
        )

    action_count = 0
    receive_stats: Dict[str, Dict[str, Any]] = {}

    while _running:
        loop_start = time.monotonic()

        for sim_robot_name, action_name in mappings:
            try:
                positions = pollers[sim_robot_name].get_joint_command()
                gate = poll_error_logs.pop(sim_robot_name, None)
                if gate is not None:
                    gate.success()
            except Exception as e:
                poll_error_logs.setdefault(
                    sim_robot_name,
                    LogGate(
                        logger,
                        f"ActionBridge/POLL_ERROR/{sim_robot_name}",
                        interval_s=5.0,
                        level=logging.ERROR,
                    ),
                ).failure(
                    "source=%s error=%s; inspect the poller implementation and simulator protocol",
                    sim_robot_name,
                    e,
                    exc_info=True,
                )
                continue
            if positions is None or len(positions) == 0:
                continue

            receive_event = _update_action_receive_stats(
                receive_stats, sim_robot_name, positions
            )
            if receive_event.total == 1:
                logger.info(
                    "[ActionBridge][FIRST_ACTION] source=%s target=%s dim=%d "
                    "all_zero=%s",
                    sim_robot_name,
                    action_name,
                    receive_event.dimension,
                    receive_event.all_zero,
                )
                logger.debug(
                    "[ActionBridge][FIRST_ACTION_VALUES] source=%s values=%s",
                    sim_robot_name,
                    _format_positions(positions),
                )
            elif receive_event.previous_dimension != receive_event.dimension:
                logger.warning(
                    "[ActionBridge][DIM_CHANGED] source=%s target=%s dim=%d->%d; "
                    "align the simulator command with the RCP action schema",
                    sim_robot_name,
                    action_name,
                    receive_event.previous_dimension,
                    receive_event.dimension,
                )
                logger.debug(
                    "[ActionBridge][DIM_CHANGED_VALUES] source=%s values=%s",
                    sim_robot_name,
                    _format_positions(positions),
                )

            if receive_event.fps is not None:
                logger.info(
                    "[ActionBridge][RX] source=%s target=%s rate=%.2fHz total=%d "
                    "dim=%d zero_frames=%d/%d changed_frames=%d/%d "
                    "max_delta=%.6f max_joint_range=%.6f consecutive_zero=%d",
                    sim_robot_name,
                    action_name,
                    receive_event.fps,
                    receive_event.total,
                    receive_event.dimension,
                    receive_event.window_zero_count,
                    receive_event.window_count,
                    receive_event.window_changed_count,
                    receive_event.window_count,
                    receive_event.window_max_delta,
                    max(receive_event.joint_ranges, default=0.0),
                    receive_event.consecutive_zero,
                )
                logger.debug(
                    "[ActionBridge][RX_VALUES] source=%s values=%s joint_ranges=%s",
                    sim_robot_name,
                    _format_positions(positions),
                    _format_positions(list(receive_event.joint_ranges)),
                )
            if receive_event.should_warn_zero:
                logger.warning(
                    "[ActionBridge][ALL_ZERO] source=%s target=%s has returned "
                    "all-zero actions for %.1fs; verify the simulator's "
                    "get_joint_command output",
                    sim_robot_name,
                    action_name,
                    receive_event.zero_duration,
                )

            try:
                if not _dispatch_action_frame(
                    rcp_client, action_name, positions, args.hz
                ):
                    continue
                action_count += 1
                if action_count % 100 == 1:
                    logger.info(
                        "[ActionBridge][RCP_ACCEPT] count=%d source=%s target=%s dim=%d",
                        action_count,
                        sim_robot_name,
                        action_name,
                        len(positions),
                    )
            except Exception as e:
                logger.exception(
                    "[ActionBridge][RCP_ERROR] source=%s target=%s error=%s; "
                    "inspect the RCP server action-service logs for the same run_id",
                    sim_robot_name,
                    action_name,
                    e,
                )

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
