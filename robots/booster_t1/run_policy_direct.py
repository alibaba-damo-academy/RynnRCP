#!/usr/bin/env python3
"""Direct Booster T1 low-level policy runner.

Run on the motion board. This bypasses RCP/protocol_debug and talks to the
Booster SDK directly through BoosterT1LowController.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

from rynnrcp_robot_booster_t1.low_controller import LOW_CMD_DT, BoosterT1LowController
from rynnrcp_robot_booster_t1.policies.walk.policy import DAMPING, STIFFNESS, Policy


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--net", default="")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--high-prepare-time", type=float, default=5.0)
    parser.add_argument("--stand-time", type=float, default=3.0)
    parser.add_argument("--exit-mode", choices=["prepare", "damping"], default="prepare")
    parser.add_argument("--policy-dir", default=str(Path(__file__).parent / "rynnrcp_robot_booster_t1/policies/walk"))
    args = parser.parse_args()

    policy_dir = Path(args.policy_dir).resolve()

    policy = Policy()
    policy.load(policy_dir)
    policy.reset({"cmd_vel": [args.x, args.y, args.yaw]})
    controller = BoosterT1LowController(
        net=args.net,
        domain_id=args.domain_id,
        default_kp=STIFFNESS,
        default_kd=DAMPING,
    )

    stopping = {"value": False}

    def stop(_sig: int, _frame: object) -> None:
        stopping["value"] = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    started = False
    try:
        print(f"Starting low controller... domain_id={args.domain_id} net={args.net!r}", flush=True)
        controller.start()
        started = True
        _wait_for_state(controller)
        if args.high_prepare_time > 0.0:
            print(f"Preparing with high-level kPrepare for {args.high_prepare_time:.1f}s...", flush=True)
            status = controller.prepare_high(args.high_prepare_time)
            print(f"prepare_high status: {status}", flush=True)
            _print_pose_error(controller, policy.default_joint_pos.astype(float).tolist(), "after high prepare")
        handoff_pose = controller.get_joint_positions()["joint_positions"]
        print(f"Entering low custom mode with {1.0 / LOW_CMD_DT:.0f} Hz heartbeat...")
        status = controller.enter_low({"prepare_joint_positions": handoff_pose})
        print(f"enter_low status: {status}", flush=True)
        time.sleep(0.3)
        _print_pose_error(controller, handoff_pose, "custom handoff hold")

        print(f"Ramping to policy default pose for {args.stand_time:.1f}s...", flush=True)
        _ramp_to(controller, policy.default_joint_pos.astype(float).tolist(), args.stand_time, stopping)
        if stopping["value"]:
            return 0
        controller.set_joint_positions({"joint_positions": policy.default_joint_pos.astype(float).tolist()})
        _print_pose_error(controller, policy.default_joint_pos.astype(float).tolist(), "after ramp")
        print("Ramp complete.", flush=True)

        print(f"Running policy duration={args.duration:.1f}s cmd_vel={[args.x, args.y, args.yaw]}", flush=True)
        period = 1.0 / policy.frame_rate
        end_time = time.perf_counter() + args.duration
        while not stopping["value"] and time.perf_counter() < end_time:
            tick_started = time.perf_counter()
            obs = {
                "observation.robot.joint_state": controller.get_joint_positions(),
                "observation.robot.imu": controller.get_imu(),
                "cmd_vel": [args.x, args.y, args.yaw],
            }
            out = policy.step(obs)
            for frame in out["frames"]:
                controller.set_joint_positions({"joint_positions": frame["joint_positions"]})
            elapsed = time.perf_counter() - tick_started
            time.sleep(max(0.0, period - elapsed))
        print("Policy run complete.", flush=True)
    finally:
        print(f"Exiting to {args.exit_mode} and shutting down...", flush=True)
        if started:
            if args.exit_mode == "damping":
                controller.damping({})
                time.sleep(0.1)
                controller.shutdown(prepare=False)
            else:
                controller.shutdown()
    return 0


def _wait_for_state(controller: BoosterT1LowController, timeout_s: float = 10.0) -> None:
    deadline = time.perf_counter() + timeout_s
    last_error = None
    while time.perf_counter() < deadline:
        try:
            state = controller.get_joint_positions()
            if len(state["joint_positions"]) == 23:
                print(f"LowState received: {controller.get_motor_count()} motors mapped to 23 policy joints")
                return
        except Exception as exc:
            last_error = exc
            time.sleep(0.05)
    raise RuntimeError(
        "No 23-joint LowState received before timeout. "
        "Check that no other low-level process is owning the DDS reader, "
        "the Booster motion service is running, and try --net '' or --net <robot NIC IP>. "
        f"last_error={last_error!r}"
    )


def _ramp_to(
    controller: BoosterT1LowController,
    target: list[float],
    duration_s: float,
    stopping: dict[str, bool],
) -> None:
    start = controller.get_joint_positions()["joint_positions"]
    steps = max(1, int(duration_s * 50))
    for i in range(steps):
        if stopping["value"]:
            return
        alpha = (i + 1) / steps
        q = [(1.0 - alpha) * now + alpha * goal for now, goal in zip(start, target)]
        controller.set_prepare_positions(q)
        if i == 0 or (i + 1) % 50 == 0 or i + 1 == steps:
            current = controller.get_joint_positions()["joint_positions"]
            err = max(abs(a - b) for a, b in zip(target, current))
            print(f"Ramp {i + 1}/{steps} max_pose_error={err:.3f}", flush=True)
        time.sleep(0.02)


def _print_pose_error(controller: BoosterT1LowController, target: list[float], label: str) -> None:
    current = controller.get_joint_positions()["joint_positions"]
    err = max(abs(a - b) for a, b in zip(target, current))
    print(f"{label}: max_pose_error={err:.3f}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
