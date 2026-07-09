#!/usr/bin/env python3
"""Run Atom01 policies against the real controller and inspect outputs."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parent
ATOM_CONTROL_DIR = PACKAGE_DIR / "atom_control"
DEFAULT_CONFIG = ATOM_CONTROL_DIR / "config" / "robot.yaml"
POLICIES_DIR = PACKAGE_DIR / "policies"
HOME_POSITIONS = np.array([
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0, 0.0,
    0.18, 0.06, 0.0, 0.78, 0.0,
    0.18, -0.06, 0.0, 0.78, 0.0,
], dtype=np.float32)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.apply and not args.i_understand_risk:
        raise SystemExit("--apply requires --i-understand-risk")

    policy = _load_policy(args.policy)
    policy.reset({"cmd_vel": args.cmd_vel})

    ctrl = _load_controller()(
        str(args.config),
        initial_positions=HOME_POSITIONS.tolist() if args.apply else None,
        require_calibration=False,
    )
    if not args.apply:
        ctrl.set_damping()

    output_file = open(args.output, "w", encoding="utf-8") if args.output else None
    try:
        previous_action: np.ndarray | None = None
        for step in range(args.steps):
            started = time.monotonic()
            state = ctrl.get_state()
            obs = _obs_from_state(state, args.cmd_vel)
            chunk = policy.step(obs)
            action = _action_from_chunk(chunk)
            q = np.array(state["joint_q"], dtype=np.float32)
            metrics = _metrics(q, action, previous_action)
            previous_action = action

            line = {
                "step": step,
                "policy": args.policy,
                "cmd_vel": args.cmd_vel,
                "state": state.get("state"),
                "damping": state.get("damping"),
                **metrics,
            }
            print(_format_line(line))
            if output_file:
                output_file.write(json.dumps(line, ensure_ascii=False) + "\n")
                output_file.flush()

            if args.apply:
                if metrics["max_abs_delta_current"] > args.max_delta and not args.allow_large_delta:
                    raise RuntimeError(
                        f"policy output delta {metrics['max_abs_delta_current']:.3f} rad exceeds "
                        f"--max-delta {args.max_delta:.3f}; add --allow-large-delta only after checking the pose"
                    )
                if step == 0:
                    ok, msg = ctrl.clear_damping()
                    if not ok:
                        raise RuntimeError(msg)
                ok, msg = ctrl.set_targets(action.astype(float).tolist())
                if not ok:
                    raise RuntimeError(msg)

            sleep_s = max(0.0, (1.0 / args.rate) - (time.monotonic() - started))
            if sleep_s:
                time.sleep(sleep_s)
    finally:
        if output_file:
            output_file.close()
        ctrl.set_damping()
        ctrl.shutdown()
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Test Atom01 policy outputs with the real low-level controller.")
    parser.add_argument("--policy", choices=["stand", "walk", "sim2simdance"], default="stand")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--steps", type=int, default=50)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--cmd-vel", nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=("VX", "VY", "WZ"))
    parser.add_argument("--output", type=Path, help="Optional JSONL output path.")
    parser.add_argument("--apply", action="store_true", help="Send policy targets to the robot. Default only runs inference.")
    parser.add_argument("--i-understand-risk", action="store_true", help="Required with --apply.")
    parser.add_argument("--max-delta", type=float, default=0.6, help="Abort --apply if any joint target jumps more than this from current q.")
    parser.add_argument("--allow-large-delta", action="store_true", help="Do not abort on --max-delta.")
    return parser


def _load_controller() -> Any:
    for item in (ATOM_CONTROL_DIR / "python", ATOM_CONTROL_DIR / "build"):
        text = str(item)
        if text not in sys.path:
            sys.path.insert(0, text)
    path = ATOM_CONTROL_DIR / "python" / "controller.py"
    spec = importlib.util.spec_from_file_location("_atom01_low_rate_controller", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load controller: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RobotController


def _load_policy(policy_id: str) -> Any:
    policy_dir = POLICIES_DIR / policy_id
    path = policy_dir / "policy.py"
    spec = importlib.util.spec_from_file_location(f"_atom01_policy_{policy_id}", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load policy: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    policy = module.Policy()
    policy.load(policy_dir)
    return policy


def _obs_from_state(state: dict[str, Any], cmd_vel: list[float]) -> dict[str, Any]:
    return {
        "observation.robot.joint_state": {
            "joint_positions": state.get("joint_q", HOME_POSITIONS.tolist()),
            "joint_velocities": state.get("joint_vel", [0.0] * 23),
        },
        "observation.robot.imu": {
            "accel": state.get("imu_accel", [0.0, 0.0, 0.0]),
            "gyro": state.get("imu_ang_vel", [0.0, 0.0, 0.0]),
            "orientation_quat_wxyz": state.get("imu_quat", [1.0, 0.0, 0.0, 0.0]),
        },
        "cmd_vel": cmd_vel,
    }


def _action_from_chunk(chunk: dict[str, Any]) -> np.ndarray:
    frames = chunk.get("frames") or []
    if not frames:
        raise ValueError("policy returned no frames")
    value = frames[0]
    action = np.array(value.get("joint_positions"), dtype=np.float32).reshape(-1)
    if action.size != 23:
        raise ValueError(f"policy returned {action.size} joints, expected 23")
    return action


def _metrics(q: np.ndarray, action: np.ndarray, previous_action: np.ndarray | None) -> dict[str, float]:
    delta_current = action - q
    home_error = q - HOME_POSITIONS
    result = {
        "max_abs_q": float(np.max(np.abs(q))),
        "max_abs_home_error": float(np.max(np.abs(home_error))),
        "max_abs_action": float(np.max(np.abs(action))),
        "max_abs_delta_current": float(np.max(np.abs(delta_current))),
        "rms_delta_current": float(np.sqrt(np.mean(delta_current * delta_current))),
    }
    if previous_action is not None:
        delta_prev = action - previous_action
        result["max_abs_delta_previous_action"] = float(np.max(np.abs(delta_prev)))
    else:
        result["max_abs_delta_previous_action"] = 0.0
    return result


def _format_line(line: dict[str, Any]) -> str:
    return (
        f"[{line['step']:04d}] {line['policy']} state={line['state']} damping={line['damping']} "
        f"max|q-home|={line['max_abs_home_error']:.3f} "
        f"max|a-q|={line['max_abs_delta_current']:.3f} "
        f"rms|a-q|={line['rms_delta_current']:.3f} "
        f"max|da|={line['max_abs_delta_previous_action']:.3f}"
    )


if __name__ == "__main__":
    raise SystemExit(main())
