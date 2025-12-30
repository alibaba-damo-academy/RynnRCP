#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RealSense + XtrainerRobotController local collection, communicates with external nodes via LCM

LCM Input:
- rcp_request_feedback(act_request) + image_request(req_camera_image) paired within 1s:
    -> Capture a frame synchronously (three cameras trigger=min(latest_ts))
    -> Publish robot_feedback + image_response
- rcp_robotmotion(act_command):
    -> Parse actions (chunkSize, numJoint)
    -> Playback actions (with safety check servo_action_check)
    -> Publish state_feedback

LCM Output:
- robot_feedback
- image_response
- state_feedback
- camera_desc (camera_list_desc at regular intervals)

Cameras:
- RealSenseCamera multi-threaded capturing top/left/right (device_id from load_ini_data_camera())
- camera_desc from cameras.yaml (name must match image_request.camera_names)
"""


from __future__ import annotations

import io
import os
import sys
import gzip
import yaml
import time
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple, Any, List

import numpy as np

from lcm import LCM

lcm_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../../common/lcm")
)
sys.path.insert(0, lcm_root)

from lcmMotion.act_request import act_request
from lcmMotion.robot_feedback import robot_feedback
from lcmMotion.act_command import act_command
from lcmMotion.state_feedback import state_feedback

from lcmSensor.image_data import image_data
from lcmSensor.camera_desc import camera_desc
from lcmSensor.camera_list_desc import camera_list_desc
from lcmSensor.req_camera_image import req_camera_image
from lcmSensor.camera_image_response import camera_image_response

from dobot_control.cameras.realsense_camera import RealSenseCamera
from scripts.manipulate_utils import load_ini_data_camera

from dobot_control.agents.agent import BimanualAgent
from dobot_control.agents.dobot_agent import DobotAgent
from dobot_control.env import RobotEnv
from dobot_control.robots.robot_node import ZMQClientRobot
from scripts.manipulate_utils import (
    load_ini_data_hands,
    robot_pose_init,
    servo_action_check,
)
from scripts.function_util import wait_period


LCM_PAIR_TIMEOUT_S = 1.0


class XtrainerRobotController:
    """Xtrainer dual-arm robot controller"""

    def __init__(
        self,
        control_freq: float = 50.0,
        robot_host: str = "127.0.0.1",
        robot_port: int = 6001,
        frame_sleep: float = 0.0,
    ):
        """
        Initialize the robot controller

        Args:
            control_freq: Control frequency (Hz)
        """
        print(" Initializing Xtrainer dual-arm robot...")

        # Load configuration file
        _, hands_dict = load_ini_data_hands()

        # Initialize main hand reading channels
        print("   - Initializing main hand reading channels...")
        leader_left = DobotAgent(
            which_hand="HAND_LEFT", dobot_config=hands_dict["HAND_LEFT"]
        )
        leader_right = DobotAgent(
            which_hand="HAND_RIGHT", dobot_config=hands_dict["HAND_RIGHT"]
        )
        self.leader_agent = BimanualAgent(leader_left, leader_right)

        # Create control channel for subordinate hand
        print("   - Connecting to subordinate robot control cabinet...")
        self.robot_client = ZMQClientRobot(port=robot_port, host=robot_host)
        self.env = RobotEnv(
            robot=self.robot_client, control_rate_hz=control_freq, camera_dict={}
        )

        self.control_freq = control_freq
        self.frame_sleep = max(frame_sleep, 0.0)
        self.flag_in = np.array([1, 1])  # [Left Arm Enable, Right Arm Enable]

        print("✅ Robot initialization completed")

    def get_current_joints(self) -> np.ndarray:
        """Get the current joint positions"""
        obs = self.env.get_obs()
        return obs["joint_positions"]

    def get_leader_joints(self) -> np.ndarray:
        """Read the current joint positions of the main hand for debugging/safety checks"""
        return self.leader_agent.act({})

    def move_to_init_pose(self):
        """Move to initial pose"""
        print(" Moving to initial pose...")
        robot_pose_init(self.env)
        print("....")
        print("✅ Reached initial pose")

    def move_joints_smooth(
        self,
        target_joints: np.ndarray,
        max_step: float = 0.01,
        check_safety: bool = True,
    ):
        """
        Smoothly move to the target joint position

        Args:
            target_joints: Target joint positions (14 dimensions)
            max_step: Maximum joint change per step (rad)
            check_safety: Whether to perform safety checks
        """
        current_joints = self.get_current_joints()

        # Safety check
        if check_safety:
            err_code, checked_action = servo_action_check(
                target_joints, current_joints, self.flag_in, step_len=0.9
            )
            if err_code == 0:
                print(f"⚠️  Warning: Joint movement too large, skipping this frame")
                return False
            target_joints = checked_action

        # Calculate interpolation steps
        max_delta = np.abs(target_joints - current_joints).max()
        steps = max(int(max_delta / max_step), 1)

        # Smooth movement
        for jnt in np.linspace(current_joints, target_joints, steps):
            self.env.step(jnt, self.flag_in)
            tic = time.time()
            wait_period(self.control_freq, tic)
            if self.frame_sleep:
                time.sleep(self.frame_sleep)

        return True

    def execute_action(self, action: np.ndarray, check_safety: bool = True):
        """
        Execute a single action

        Args:
            action: Joint position (14 dimensions)
            check_safety: Whether to perform safety checks
        """
        current_joints = self.get_current_joints()

        # Safety check
        if check_safety:
            err_code, checked_action = servo_action_check(
                action, current_joints, self.flag_in, step_len=0.6
            )
            if err_code == 0:
                print(f"⚠️  Warning: Joint movement too large, skipping this frame")
                return False
            action = checked_action

        # Execute directly
        self.env.step(action, self.flag_in)
        if self.frame_sleep:
            time.sleep(self.frame_sleep)
        return True

    def close(self):
        """Close the robot connection"""
        print(" Closing robot connection")
        if hasattr(self, "robot_client"):
            try:
                self.robot_client.close()
            except Exception as exc:
                print(
                    f"⚠️  Exception occurred while closing control cabinet connection: {exc}"
                )


# ==================== Bridge Parameters ====================
@dataclass
class BridgeArgs:
    camera_desc_yaml_path: str = str(Path(__file__).resolve().parent / "cameras.yaml")
    camera_desc_period_s: float = 3.0

    robot_host: str = "127.0.0.1"
    robot_port: int = 6001
    control_freq: float = 15.0
    speed_scale: float = 1.0
    frame_sleep: float = 0.0
    action_delay: float = 0.05
    skip_init: bool = False
    dry_run: bool = False

    # The dimensions of actions under act_command need to match this
    joint_names: Tuple[str, ...] = (
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "left_gripper",
        "right_joint1",
        "right_joint2",
        "right_joint3",
        "right_joint4",
        "right_joint5",
        "right_joint6",
        "right_gripper",
    )


class CameraStreamer:
    def __init__(self, camera_cfg: Dict[str, str]):
        self.camera_cfg = camera_cfg
        self.frames: Dict[str, Optional[np.ndarray]] = {
            "top": None,
            "left": None,
            "right": None,
        }
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.threads = []
        self.cameras: Dict[str, RealSenseCamera] = {}

    def start(self):
        mapping = {
            "top": ("top", True),
            "left": ("left", False),
            "right": ("right", True),
        }
        for key, (cfg_key, flip) in mapping.items():
            device_id = self.camera_cfg.get(cfg_key)
            if device_id is None:
                raise ValueError(f"Camera '{cfg_key}' not found in configuration file")
            cam = RealSenseCamera(flip=flip, device_id=device_id)
            self.cameras[key] = cam
            th = threading.Thread(target=self._loop, args=(key, cam), daemon=True)
            th.start()
            self.threads.append(th)
        print(" Camera threads started.")

    def _loop(self, key: str, cam: RealSenseCamera):
        while not self.stop_event.is_set():
            image, _ = cam.read()
            if image is None:
                continue
            # run_control reads later which will convert color channels
            image_bgr = image[:, :, ::-1]
            with self.lock:
                self.frames[key] = image_bgr.copy()

    def get_frames(self) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        with self.lock:
            top = self.frames["top"]
            left = self.frames["left"]
            right = self.frames["right"]
        if top is None or left is None or right is None:
            return None
        return top, left, right

    def stop(self):
        self.stop_event.set()
        for th in self.threads:
            th.join(timeout=1.0)
        print(" Camera threads stopped.")


# ==================== Action Playback ====================
class RobotActionPlayer:
    def __init__(self, controller: Optional[XtrainerRobotController], args: BridgeArgs):
        self.controller = controller
        self.args = args
        self._last_action_time = time.time()

        # Automatically find the indices of left and right grippers based on joint_names
        self.left_gripper_idx = None
        self.right_gripper_idx = None
        try:
            self.left_gripper_idx = args.joint_names.index("left_gripper")
            self.right_gripper_idx = args.joint_names.index("right_gripper")
            print(
                f" Gripper indices: left_gripper={self.left_gripper_idx}, right_gripper={self.right_gripper_idx}"
            )
        except ValueError:
            print(
                "⚠️ left_gripper or right_gripper not found in joint_names, gripper values will not be printed."
            )

    def play(self, actions: np.ndarray):

        if actions.size == 0:
            return
        if self.controller is None:
            print("🛈 Dry-run: skip robot playback")
            return

        adjusted_freq = self.args.control_freq / max(self.args.speed_scale, 1e-3)

        for idx, action in enumerate(actions):

            if self.args.action_delay > 0:
                delay_start = time.time()
                print(f" Frame {idx} pre-delay {self.args.action_delay:.3f}s...")
                time.sleep(self.args.action_delay)
                actual_delay = time.time() - delay_start
                if actual_delay > self.args.action_delay * 1.1:  # Allow 10% error
                    print(f"⚠️  Actual delay {actual_delay:.3f}s exceeds expected limit")

            if self.left_gripper_idx is not None and self.right_gripper_idx is not None:
                left_g = float(action[self.left_gripper_idx])
                right_g = float(action[self.right_gripper_idx])
                print(
                    f" Frame {idx} Gripper values -> left_gripper={left_g:.4f}, right_gripper={right_g:.4f}"
                )

            # 2. Execute action
            tic = time.time()
            success = self.controller.execute_action(action, check_safety=True)
            if not success:
                print(f"⚠️  Action frame {idx} skipped due to safety check.")

            # 3. Control frequency wait
            wait_period(adjusted_freq, tic)

            # 4. Additional frame sleep (if configured)
            if self.args.frame_sleep > 0:
                time.sleep(self.args.frame_sleep)

            # Record last execution time
            self._last_action_time = time.time()

            # Display frame information
            frame_time = time.time() - tic
            print(
                f" Frame {idx} execution completed (duration {frame_time:.3f}s, total {time.time() - self._last_action_time + frame_time:.3f}s)"
            )


class XtrainerMotionNode:
    def __init__(self, args: BridgeArgs):
        self.args = args

        # Robot
        self.controller: Optional[XtrainerRobotController] = None
        if not args.dry_run:
            self.controller = XtrainerRobotController(
                control_freq=args.control_freq,
                robot_host=args.robot_host,
                robot_port=args.robot_port,
                frame_sleep=args.frame_sleep,
            )
            if not args.skip_init:
                self.controller.move_to_init_pose()

        self.player = RobotActionPlayer(self.controller, args)

        # Cameras
        camera_dict = load_ini_data_camera()  # {top/left/right: device_id}
        self.camera_streamer = CameraStreamer(camera_dict)
        self.camera_streamer.start()

        # LCM
        self.lcm = LCM()
        self._feedback_seq = 0

        self.pending_lock = threading.Lock()
        self.pending_act_request: Optional[act_request] = None
        self.pending_image_request: Optional[req_camera_image] = None
        self.pending_act_time: Optional[float] = None
        self.pending_image_time: Optional[float] = None

        self.lcm.subscribe("rcp_request_feedback", self._on_act_request)
        self.lcm.subscribe("image_request", self._on_image_request)
        self.lcm.subscribe("rcp_robotmotion", self._on_act_command)

        self.shutdown_event = threading.Event()
        self.lcm_thread = threading.Thread(target=self._lcm_listener, daemon=True)
        self.lcm_thread.start()

        self.timer_thread = threading.Thread(target=self._timers_loop, daemon=True)
        self.timer_thread.start()

        print("[Bridge] started.")

    def _lcm_listener(self):
        print("[LCM] listener thread started")
        while not self.shutdown_event.is_set():
            try:
                self.lcm.handle_timeout(50)
            except Exception as e:
                print(f"[LCM] handle error: {e}")
                time.sleep(0.01)

    def _timers_loop(self):
        last_desc = 0.0
        last_pair_check = 0.0
        while not self.shutdown_event.is_set():
            now = time.time()

            if now - last_pair_check >= 0.1:
                last_pair_check = now
                self._check_lcm_pair()

            if now - last_desc >= self.args.camera_desc_period_s:
                last_desc = now
                try:
                    self._publish_camera_desc()
                except Exception as e:
                    print(f"[Bridge] camera_desc publish error: {e}")

            time.sleep(0.005)

    # ---------- LCM callbacks ----------
    def _on_act_request(self, _channel: str, data: bytes):
        try:
            msg = act_request.decode(data)
        except Exception as e:
            print(f"[LCM] decode act_request failed: {e}")
            return
        with self.pending_lock:
            self.pending_act_request = msg
            self.pending_act_time = time.time()
        print("[LCM] got rcp_request_feedback")

    def _on_image_request(self, _channel: str, data: bytes):
        try:
            msg = req_camera_image.decode(data)
        except Exception as e:
            print(f"[LCM] decode image_request failed: {e}")
            return
        with self.pending_lock:
            self.pending_image_request = msg
            self.pending_image_time = time.time()
        print("[LCM] got image_request")

    def _on_act_command(self, _channel: str, data: bytes):
        try:
            msg = act_command.decode(data)
        except Exception as e:
            print(f"[LCM] decode act_command failed: {e}")
            return

        print(
            f"[LCM] got rcp_robotmotion: chunkSize={msg.chunkSize} numJoint={msg.numJoint}"
        )

        # parse actions
        try:
            expected = msg.chunkSize * msg.numJoint
            if msg.totalNumJoint != expected:
                raise ValueError(
                    f"totalNumJoint({msg.totalNumJoint}) != chunkSize*numJoint({expected})"
                )

            actions = np.array(
                [
                    msg.jointPos[i * msg.numJoint : (i + 1) * msg.numJoint]
                    for i in range(msg.chunkSize)
                ],
                dtype=np.float64,
            )
        except Exception as e:
            print(f"[LCM] action parse error: {e}")
            self._publish_state_feedback(False, str(e))
            return

        # execute
        try:
            self.player.play(actions)
            self._publish_state_feedback(True, "Action Succeeded")
        except Exception as e:
            print(f"[Bridge] action execution error: {e}")
            self._publish_state_feedback(False, str(e))

    # ---------- pairing ----------
    def _check_lcm_pair(self):
        now = time.time()
        with self.pending_lock:
            act_msg = self.pending_act_request
            img_msg = self.pending_image_request
            act_t = self.pending_act_time
            img_t = self.pending_image_time

            if act_msg is None and img_msg is None:
                return

            if act_msg is not None and img_msg is None:
                if act_t and (now - act_t) > LCM_PAIR_TIMEOUT_S:
                    print("[LCM] act_request timeout waiting image_request, drop")
                    self.pending_act_request = None
                    self.pending_act_time = None
                return

            if img_msg is not None and act_msg is None:
                if img_t and (now - img_t) > LCM_PAIR_TIMEOUT_S:
                    print("[LCM] image_request timeout waiting act_request, drop")
                    self.pending_image_request = None
                    self.pending_image_time = None
                return

            earliest = min(act_t or now, img_t or now)
            if (now - earliest) > LCM_PAIR_TIMEOUT_S:
                print("[LCM] pairing timeout, drop both")
                self.pending_act_request = None
                self.pending_act_time = None
                self.pending_image_request = None
                self.pending_image_time = None
                return

            act_req = self.pending_act_request
            img_req = self.pending_image_request
            self.pending_act_request = None
            self.pending_act_time = None
            self.pending_image_request = None
            self.pending_image_time = None

        self._handle_paired_requests(act_req, img_req)

    def _handle_paired_requests(self, _act_req: act_request, img_req: req_camera_image):
        obs = self._try_build_obs()
        if obs is None:
            print("[Bridge] sync/build obs failed")
            return
        self._publish_robot_feedback(obs["observation.state"])
        self._publish_image_response(img_req, obs)

    # ---------- local obs ----------
    def _try_build_obs(self) -> Optional[Dict[str, Any]]:
        synced = self.camera_streamer.get_frames()
        if synced is None:
            return None
        top_bgr, left_bgr, right_bgr = synced

        if self.controller is None:
            return None
        state = self.controller.get_current_joints()
        if state is None or state.size != len(self.args.joint_names):
            return None

        return {
            "observation.state": state.astype(np.float64),
            "observation.images.cam_main": top_bgr,
            "observation.images.cam_left_arm": left_bgr,
            "observation.images.cam_right_arm": right_bgr,
        }

    # ---------- LCM publishers ----------
    def _publish_robot_feedback(self, state: np.ndarray):
        msg = robot_feedback()
        now = time.time()
        msg.sec = int(now)
        msg.nanosec = int((now - int(now)) * 1e9)
        msg.utime = int(now * 1e6)

        msg.seq = self._feedback_seq
        self._feedback_seq += 1

        msg.numJoint = int(len(state))
        msg.qFb = list(map(float, state))
        msg.qdFb = [0.0] * msg.numJoint
        msg.tauFb = [0.0] * msg.numJoint

        msg.numGripper = 0
        msg.gripperPosFb = []
        msg.numFTsensor = 0
        msg.ftSensorFb = []
        msg.eePosFb = [0.0, 0.0, 0.0]
        msg.eeQuatFb = [1.0, 0.0, 0.0, 0.0]
        msg.odometry = [0.0, 0.0, 0.0]

        self.lcm.publish("robot_feedback", msg.encode())
        print("[LCM] published robot_feedback")

    def _publish_image_response(self, img_req: req_camera_image, obs: Dict[str, Any]):
        resp = camera_image_response()
        resp.camera_count = img_req.camera_count
        resp.seq = img_req.seq
        resp.images = []

        for i in range(img_req.camera_count):
            cam_key = str(img_req.camera_names[i])
            frame = obs.get(cam_key, None)
            if frame is None:
                print(f"[LCM] missing image for key={cam_key}, skip")
                continue

            h, w, c = frame.shape
            buf = io.BytesIO()
            np.save(buf, frame)
            compressed = gzip.compress(buf.getvalue(), compresslevel=1)

            img_info = image_data()
            img_info.camera_id = img_req.camera_ids[i]
            img_info.camera_name = cam_key
            img_info.width = int(w)
            img_info.height = int(h)
            img_info.channels = int(c)
            img_info.image_size = len(compressed)
            img_info.image_bytes = compressed
            resp.images.append(img_info)

        self.lcm.publish("image_response", resp.encode())
        print("[LCM] published image_response")

    def _publish_state_feedback(self, success: bool, msg: str):
        fb = state_feedback()
        now = time.time()
        fb.sec = int(now)
        fb.nanosec = int((now - int(now)) * 1e9)
        fb.utime = int(now * 1e6)

        fb.seq = self._feedback_seq
        self._feedback_seq += 1

        fb.act_status_type = (
            state_feedback.kSuccess if success else state_feedback.kFail
        )
        fb.error_msg = msg

        fb.stateID = state_feedback.kMove1
        fb.substateID = state_feedback.kExit
        fb.state_msg = "Executing Move 1"

        self.lcm.publish("state_feedback", fb.encode())
        print("[LCM] published state_feedback")

    def _publish_camera_desc(self):
        with open(self.args.camera_desc_yaml_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        cameras = cfg.get("cameras", [])
        if not isinstance(cameras, list):
            raise ValueError("cameras.yaml format error: cameras must be list")

        msg = camera_list_desc()
        msg.seq = 0
        msg.stamp = int(time.time() * 1e6)
        msg.n = len(cameras)
        msg.cameras = []

        for idx, cam in enumerate(cameras):
            item = camera_desc()
            item.name = str(cam.get("name", f"camera_{idx}"))
            item.id = int(cam.get("id", idx))
            item.product = str(cam.get("brand", cam.get("product", "realsense")))
            item.format = str(cam.get("format", "BGR8"))
            item.width = int(cam.get("width", 640))
            item.height = int(cam.get("height", 360))
            item.fps = int(cam.get("framerate", cam.get("fps", 30)))
            item.status = int(cam.get("status", 1))
            msg.cameras.append(item)

        self.lcm.publish("camera_desc", msg.encode())
        print(f"[LCM] published camera_desc n={msg.n}")

    def close(self):
        self.shutdown_event.set()
        try:
            self.camera_streamer.stop()
        except Exception:
            pass
        try:
            if self.controller:
                self.controller.close()
        except Exception:
            pass
        print("[Bridge] closed.")


def main():
    args = BridgeArgs(
        camera_desc_yaml_path="./config/cameras.yaml",
        robot_host=os.environ.get("ROBOT_HOST", "127.0.0.1"),
        robot_port=int(os.environ.get("ROBOT_PORT", "6001")),
    )

    bridge = XtrainerMotionNode(args)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
