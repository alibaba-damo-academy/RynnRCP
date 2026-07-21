from __future__ import annotations

import argparse
import ctypes
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime as ort
import yaml


JOINT_NUM = 21


def main() -> None:
    parser = argparse.ArgumentParser(description="Bumi no-joystick ONNX policy test.")
    parser.add_argument("--sdk-root", default=os.environ.get("BUMI_SDK_ROOT", "."))
    parser.add_argument("--onnx", default=None)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--cmd-vel", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    parser.add_argument("--stand-seconds", type=float, default=2.0)
    parser.add_argument("--stand-hz", type=float, default=500.0)
    args = parser.parse_args()

    sdk_root = Path(args.sdk_root).expanduser().resolve()
    _load_sdk(sdk_root)

    from lowcontrol_py import LowController, MotorCmd

    cfg = (yaml.safe_load((sdk_root / "config" / "bumi_ac.yaml").read_text(encoding="utf-8")) or {})["run"]
    joint_names = list(cfg["joint_names"])
    default_pos = _array(cfg["default_joint_pos"])
    action_scale = _array(cfg["action_scale"])
    kp = _array(cfg["joint_stiffness"])
    kd = _array(cfg["joint_damping"])
    decimation = int(cfg["control"]["decimation"])
    obs_size = int(cfg["size"]["observations_size"])
    stack_size = int(cfg["size"]["stack_size"])
    action_size = int(cfg["size"]["actions_size"])
    clip_obs = float(cfg["normalization"]["clip_scales"]["clip_observations"])
    clip_actions = float(cfg["normalization"]["clip_scales"]["clip_actions"])
    axes = cfg["axis_mappings"]
    scalex = float(axes["scalex"])
    scaley = float(axes["scaley"])
    scalez = float(axes["scalez"])

    model = Path(args.onnx).expanduser() if args.onnx else sdk_root / "policy" / "policy.onnx"
    print(f"load onnx: {model}", flush=True)
    session = ort.InferenceSession(str(model), sess_options=ort.SessionOptions(), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    ctrl = LowController.instance()
    ctrl.init()

    history = np.zeros(obs_size * stack_size, dtype=np.float32)
    last_actions = np.zeros(action_size, dtype=np.float32)
    first_obs = True
    count = 0

    def joint_index(name: str) -> int:
        idx = int(ctrl.getJointsIndex(name))
        if idx < 0 or idx >= JOINT_NUM:
            raise RuntimeError(f"joint not found: {name}")
        return idx

    def read_joints() -> tuple[np.ndarray, np.ndarray]:
        states = ctrl.get_joint_state()
        q = np.zeros(JOINT_NUM, dtype=np.float32)
        dq = np.zeros(JOINT_NUM, dtype=np.float32)
        for i, name in enumerate(joint_names):
            s = states[joint_index(name)]
            q[i] = float(s.pos)
            dq[i] = float(s.vel)
        return q, dq

    def send_positions(pos: np.ndarray, use_policy_gains: bool = True) -> None:
        cmds = [MotorCmd() for _ in range(JOINT_NUM)]
        for i, name in enumerate(joint_names):
            hw = joint_index(name)
            cmds[hw].pos = float(pos[i])
            cmds[hw].vel = 0.0
            cmds[hw].kp = float(kp[i] if use_policy_gains else 10.0)
            cmds[hw].kd = float(kd[i] if use_policy_gains else 0.5)
            cmds[hw].tau = 0.0
            cmds[hw].motor_id = int(hw)
        ctrl.set_joint(cmds)

    def damping() -> None:
        cmds = [MotorCmd() for _ in range(JOINT_NUM)]
        for i, cmd in enumerate(cmds):
            cmd.pos = 0.0
            cmd.vel = 0.0
            cmd.kp = 0.0
            cmd.kd = 0.1
            cmd.tau = 0.0
            cmd.motor_id = i
        ctrl.set_joint(cmds)

    def make_obs(command: list[float]) -> tuple[np.ndarray, np.ndarray]:
        nonlocal first_obs, history, last_actions
        q, dq = read_joints()
        imu = ctrl.get_imu_data()
        quat = np.array(imu.ori, dtype=np.float32)
        gyro = np.array(imu.angular_vel, dtype=np.float32)
        zyx = _quat_to_zyx(quat)
        euler_xy = np.array([zyx[2], zyx[1]], dtype=np.float32)

        cmd = np.array([command[0] * scalex, command[1] * scaley, command[2] * scalez], dtype=np.float32)
        if cmd[0] < -0.5:
            cmd[0] = -0.5
        if 0.0 <= cmd[0] < 0.3:
            cmd[0] = 0.0
        if abs(cmd[2]) < 0.3:
            cmd[2] = 0.0
        cmd = np.array([np.clip(cmd[0], -1.5, 1.5), np.clip(cmd[1], -1.0, 1.0), np.clip(cmd[2], -1.0, 1.0)], dtype=np.float32)

        frame = np.concatenate([cmd, gyro, euler_xy, q - default_pos, dq, last_actions]).astype(np.float32)
        if frame.size != obs_size:
            raise RuntimeError(f"obs size {frame.size} != {obs_size}")
        if first_obs:
            for i in range(stack_size):
                history[i * obs_size:(i + 1) * obs_size] = frame
            first_obs = False
        else:
            history[:-obs_size] = history[obs_size:]
            history[-obs_size:] = frame
        return np.clip(history, -clip_obs, clip_obs), quat

    try:
        current, _ = read_joints()
        print(f"stand ramp: {args.stand_seconds:.2f}s, delta={float(np.max(np.abs(default_pos - current))):.3f}", flush=True)
        _ramp(current, default_pos, args.stand_seconds, args.stand_hz, lambda p: send_positions(p, use_policy_gains=False))

        deadline = time.monotonic() + max(0.0, args.seconds)
        print(f"policy start: seconds={args.seconds:.1f}, cmd_vel={args.cmd_vel}", flush=True)
        while time.monotonic() < deadline:
            started = time.monotonic()
            obs, quat = make_obs(args.cmd_vel)
            gravity_z = _projected_gravity_z(quat)
            if gravity_z >= -0.3:
                print(f"fall protection: projected_gravity_z={gravity_z:.3f}", flush=True)
                break

            if count % decimation == 0:
                count = 0
                out = session.run(None, {input_name: obs.reshape(1, -1)})[0].reshape(-1)[:action_size]
                last_actions[:] = np.clip(out, -clip_actions, clip_actions).astype(np.float32)

            target = last_actions * action_scale + default_pos
            send_positions(target, use_policy_gains=True)

            if count == 0:
                print(
                    f"policy tick action_abs={float(np.max(np.abs(last_actions))):.3f} "
                    f"target_abs={float(np.max(np.abs(target))):.3f} gravity_z={gravity_z:.3f}",
                    flush=True,
                )

            count += 1
            sleep_s = 0.002 - (time.monotonic() - started)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("interrupted", flush=True)
    finally:
        print("damping", flush=True)
        damping()


def _load_sdk(root: Path) -> None:
    os.environ["CYCLONEDDS_URI"] = "file://" + str(root / "config" / "dds.xml")
    for arch in ("aarch64", "x86_64"):
        lib_dir = root / "lib" / arch
        for lib in ("libcrypto.so.1.1", "libssl.so.1.1"):
            path = lib_dir / lib
            if path.exists():
                ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(root / "build"))


def _array(value: Any) -> np.ndarray:
    return np.array(value, dtype=np.float32).reshape(-1)


def _ramp(start: np.ndarray, target: np.ndarray, seconds: float, hz: float, send) -> None:
    steps = max(1, int(seconds * hz))
    period = 1.0 / max(1.0, hz)
    for i in range(1, steps + 1):
        alpha = i / steps
        send(start * (1.0 - alpha) + target * alpha)
        time.sleep(period)


def _projected_gravity_z(q: np.ndarray) -> float:
    rot = _rotation_matrix_from_zyx(_quat_to_zyx(q))
    return float((np.linalg.inv(rot) @ np.array([0.0, 0.0, -1.0], dtype=np.float32))[2])


def _quat_to_zyx(q: np.ndarray) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, s) if abs(s) >= 1.0 else math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([yaw, pitch, roll], dtype=np.float32)


def _rotation_matrix_from_zyx(e: np.ndarray) -> np.ndarray:
    z, y, x = [float(v) for v in e]
    c1, c2, c3 = np.cos(z), np.cos(y), np.cos(x)
    s1, s2, s3 = np.sin(z), np.sin(y), np.sin(x)
    return np.array([
        [c1 * c2, c1 * s2 * s3 - s1 * c3, c1 * s2 * c3 + s1 * s3],
        [s1 * c2, s1 * s2 * s3 + c1 * c3, s1 * s2 * c3 - c1 * s3],
        [-s2, c2 * s3, c2 * c3],
    ], dtype=np.float32)


if __name__ == "__main__":
    main()
