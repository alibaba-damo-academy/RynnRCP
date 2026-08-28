"""Read-only Franka arm and Franka Hand connectivity check."""

from __future__ import annotations

import argparse
import sys

from .controller import FrankaController


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read Franka arm and gripper state without enabling control or motion."
    )
    parser.add_argument(
        "--robot-ip", required=True, help="Franka FCI IP address or hostname."
    )
    parser.add_argument(
        "--without-gripper", action="store_true", help="Check only the seven-axis arm."
    )
    parser.add_argument(
        "--enforce-realtime",
        action="store_true",
        help="Enforce real-time scheduling even though this check does not start control.",
    )
    args = parser.parse_args(argv)
    controller = FrankaController(
        robot_ip=args.robot_ip,
        with_gripper=not args.without_gripper,
        realtime_enforce=args.enforce_realtime,
    )
    try:
        controller.start()
        joint_state = controller.get_joint_positions()
        gripper_state = None if args.without_gripper else controller.get_gripper_state()
    except Exception as exc:
        print(f"Franka read-only check failed: {exc}", file=sys.stderr)
        return 2
    finally:
        controller.shutdown()

    print(f"Franka connection OK: {args.robot_ip}")
    print(
        "Arm joint state [7 rad]: "
        + ", ".join(f"{item:.6f}" for item in joint_state["joint_positions"])
    )
    if gripper_state is not None:
        print(
            f"Franka Hand OK: width={gripper_state['width']:.4f} m, "
            f"max_width={gripper_state['max_width']:.4f} m, "
            f"grasped={gripper_state['is_grasped']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
