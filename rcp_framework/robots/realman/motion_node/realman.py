#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import io
import cv2
import sys
import gzip
import yaml
import time
import threading
import numpy as np
from collections import deque
from typing import List, Optional, Tuple, Any, Dict, Deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import CompressedImage, JointState
from rm_ros_interfaces.msg import Sixforce, Liftpos
from cv_bridge import CvBridge

from lcm import LCM

lcm_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../../../common/lcm')
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

from .action_executor import ActionExecutor


# ==================== Configuration Constants ====================
BUFFER_DURATION_S = 2.0  # Buffer duration (seconds)
DEBUG_LOG = False  # Whether to print detailed debug logs


class SensorBufferManager:
    """Unified management of all sensor buffers, providing temporal alignment query interface"""

    def __init__(self, node: Node, max_duration=BUFFER_DURATION_S):
        self.node = node
        self.max_duration = max_duration
        self.lock = threading.Lock()

        # Buffer format: deque[(timestamp_sec: float, msg)]
        self.buffers = {
            'camera_head': deque(maxlen=200),
            'camera_left': deque(maxlen=200),
            'camera_right': deque(maxlen=200),
            'joint_left': deque(maxlen=500),
            'joint_right': deque(maxlen=500),
            'gripper_left': deque(maxlen=500),
            'gripper_right': deque(maxlen=500),
            'sixforce_left': deque(maxlen=500),
            'sixforce_right': deque(maxlen=500),
            'lift_pos': deque(maxlen=500),
        }

        # Record of the latest valid timestamps
        self.latest_ts = {key: 0.0 for key in self.buffers}

    def add_message(self, sensor_key: str, timestamp: float, msg):
        with self.lock:
            buffer = self.buffers[sensor_key]
            buffer.append((timestamp, msg))
            self._prune_old_data(sensor_key)
            self.latest_ts[sensor_key] = timestamp

    def _prune_old_data(self, sensor_key: str):
        """Remove data that exceeds the buffer duration"""
        now = time.time()
        buffer = self.buffers[sensor_key]
        while len(buffer) > 0 and (now - buffer[0][0]) > self.max_duration:
            buffer.popleft()

    def get_latest_timestamps(self) -> Dict[str, Optional[float]]:
        """Get the latest timestamp of each camera (only for calculating trigger_time)"""
        ts = {}
        with self.lock:
            for cam in ['camera_head', 'camera_left', 'camera_right']:
                buf = self.buffers[cam]
                ts[cam] = buf[-1][0] if len(buf) > 0 else None
        return ts

    def get_next_available(self, sensor_key: str, target_t: float) -> Optional[Any]:
        """Get the first available data >= target_t"""
        with self.lock:
            buffer = self.buffers[sensor_key]
            for t, msg in buffer:
                if t >= target_t:
                    return msg
            return None

    def get_next_available_joint(
        self, sensor_key: str, target_t: float
    ) -> Optional[np.ndarray]:
        msg = self.get_next_available(sensor_key, target_t)
        if msg is None or not hasattr(msg, 'position'):
            return None
        return np.array(msg.position, dtype=np.float32)

    def get_next_available_scalar(
        self, sensor_key: str, target_t: float
    ) -> Optional[float]:
        msg = self.get_next_available(sensor_key, target_t)
        if msg is None or len(msg.position) == 0:
            return None
        return float(msg.position[0])

    def get_next_available_sixforce(
        self, sensor_key: str, target_t: float
    ) -> Optional[np.ndarray]:
        msg = self.get_next_available(sensor_key, target_t)
        if msg is None:
            return None
        return np.array(
            [
                msg.force_fx,
                msg.force_fy,
                msg.force_fz,
                msg.force_mx,
                msg.force_my,
                msg.force_mz,
            ],
            dtype=np.float32,
        )

    def get_next_available_lift(
        self, sensor_key: str, target_t: float
    ) -> Optional[float]:
        msg = self.get_next_available(sensor_key, target_t)
        if msg is None:
            return None
        return float(msg.height) / 10.0  # mm → cm

    def get_buffer_length(self, sensor_key: str) -> int:
        with self.lock:
            return len(self.buffers[sensor_key])


class MultiSensorSynchronizer(Node):
    """Multi-Sensor Synchronizer"""

    def __init__(self):
        super().__init__('multi_sensor_synchronizer')

        self.bridge = CvBridge()
        self.sensor_mgr = SensorBufferManager(self)

        self.sync_count = 0

        self.latest_sync_data = None
        self.shutdown_event = threading.Event()

        self.camera_desc_yaml_path = os.path.join(
            os.path.dirname(__file__), '../config/cameras.yaml'
        )

        # ==================== ROS Subscription Section ====================
        # Create mutually exclusive callback group
        cb_group = MutuallyExclusiveCallbackGroup()
        qos = qos_profile_sensor_data

        # --- Image Subscriptions ---
        self.create_subscription(
            CompressedImage,
            '/camera_head/color/image_raw/compressed',
            self.camera_head_callback,
            qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            CompressedImage,
            '/camera_left/color/image_raw/compressed',
            self.camera_left_callback,
            qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            CompressedImage,
            '/camera_right/color/image_raw/compressed',
            self.camera_right_callback,
            qos,
            callback_group=cb_group,
        )

        # --- Joint & Force Control ---
        self.create_subscription(
            JointState,
            '/left_arm_controller/joint_states',
            self.joint_left_callback,
            qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            JointState,
            '/right_arm_controller/joint_states',
            self.joint_right_callback,
            qos,
            callback_group=cb_group,
        )

        self.create_subscription(
            JointState,
            '/left_arm_controller/rm_driver/gripper_pos',
            self.gripper_left_callback,
            qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            JointState,
            '/right_arm_controller/rm_driver/gripper_pos',
            self.gripper_right_callback,
            qos,
            callback_group=cb_group,
        )

        self.create_subscription(
            Sixforce,
            '/left_arm_controller/rm_driver/udp_six_force',
            self.sixforce_left_callback,
            qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            Sixforce,
            '/right_arm_controller/rm_driver/udp_six_force',
            self.sixforce_right_callback,
            qos,
            callback_group=cb_group,
        )

        self.create_subscription(
            Liftpos,
            '/right_arm_controller/rm_driver/udp_lift_pos',
            self.lift_pos_callback,
            qos,
            callback_group=cb_group,
        )

        # ==================== LCM Initialization ====================
        self.lcm = LCM()
        self.pending_act_request: Optional[act_request] = None
        self.pending_image_request: Optional[req_camera_image] = None
        self.pending_act_time: Optional[float] = None
        self.pending_image_time: Optional[float] = None
        self.pending_lock = threading.Lock()
        self._feedback_seq = 0

        # Maximum wait time for pairing (seconds)
        self.lcm_pair_timeout = 1.0

        # LCM Subscriptions
        self.lcm.subscribe("rcp_request_feedback", self.handle_act_request)
        self.lcm.subscribe("rcp_robotmotion", self.handle_act_command)
        self.lcm.subscribe("image_request", self.handle_request)

        # LCM Listener Thread
        self.lcm_thread = threading.Thread(target=self.lcm_listener, daemon=True)
        self.lcm_thread.start()

        self.get_logger().info("🟢 All sensors subscribed...")

        # ==================== Timer: Check paired status periodically ======
        self.lcm_pair_timer = self.create_timer(0.1, self.check_lcm_pair)

        # ==================== Timer: Send camera configuration =============
        self.camera_desc_timer = self.create_timer(
            3.0, self._camera_desc_timer_callback
        )

        # ==================== Action Executor Initialization ===============

        LEFT_IP = "192.168.10.110"
        LEFT_PORT = 8080
        RIGHT_IP = "192.168.10.111"
        RIGHT_PORT = 8080

        self.action_executor = ActionExecutor(
            LEFT_IP=LEFT_IP,
            LEFT_PORT=LEFT_PORT,
            RIGHT_IP=RIGHT_IP,
            RIGHT_PORT=RIGHT_PORT,
        )

    # ==================== Callback Functions ====================

    def _safe_add_msg(self, sensor_key: str, msg):
        if not self.is_valid_timestamp(msg.header.stamp):
            return
        t = self.ros_ts_to_sec(msg.header.stamp)
        self.sensor_mgr.add_message(sensor_key, t, msg)

    def camera_head_callback(self, msg: CompressedImage):
        self._safe_add_msg('camera_head', msg)

    def camera_left_callback(self, msg: CompressedImage):
        self._safe_add_msg('camera_left', msg)

    def camera_right_callback(self, msg: CompressedImage):
        self._safe_add_msg('camera_right', msg)

    def joint_left_callback(self, msg: JointState):
        self._safe_add_msg('joint_left', msg)

    def joint_right_callback(self, msg: JointState):
        self._safe_add_msg('joint_right', msg)

    def gripper_left_callback(self, msg: JointState):
        self._safe_add_msg('gripper_left', msg)

    def gripper_right_callback(self, msg: JointState):
        self._safe_add_msg('gripper_right', msg)

    def sixforce_left_callback(self, msg: Sixforce):
        self._safe_add_msg('sixforce_left', msg)

    def sixforce_right_callback(self, msg: Sixforce):
        self._safe_add_msg('sixforce_right', msg)

    def lift_pos_callback(self, msg: Liftpos):
        self._safe_add_msg('lift_pos', msg)

    # ==================== Core Synchronization Thread ====================

    def try_synchronize(self):
        """Core synchronization logic"""

        latest_ts = self.sensor_mgr.get_latest_timestamps()

        # Check if all three cameras have data
        if not all(latest_ts.values()):
            if DEBUG_LOG:
                missing = [k for k, v in latest_ts.items() if v is None]
                self.get_logger().debug(f"🟡 Waiting camera data: {missing}")
            return

        # Use the minimum timestamp among the latest from three cameras as the sync point
        trigger_time = min(latest_ts.values())

        # Get the closest future data from various sensors
        obs = {
            'observation.images.cam_head': self._decode_compressed_image(
                self.sensor_mgr.get_next_available('camera_head', trigger_time)
            ),
            'observation.images.cam_left': self._decode_compressed_image(
                self.sensor_mgr.get_next_available('camera_left', trigger_time)
            ),
            'observation.images.cam_right': self._decode_compressed_image(
                self.sensor_mgr.get_next_available('camera_right', trigger_time)
            ),
            'observation.joint_position.left': self.sensor_mgr.get_next_available_joint(
                'joint_left', trigger_time
            ),
            'observation.joint_position.right': self.sensor_mgr.get_next_available_joint(
                'joint_right', trigger_time
            ),
            'observation.gripper_position.left': self.sensor_mgr.get_next_available_scalar(
                'gripper_left', trigger_time
            ),
            'observation.gripper_position.right': self.sensor_mgr.get_next_available_scalar(
                'gripper_right', trigger_time
            ),
            'observation.force.left': self.sensor_mgr.get_next_available_sixforce(
                'sixforce_left', trigger_time
            ),
            'observation.force.right': self.sensor_mgr.get_next_available_sixforce(
                'sixforce_right', trigger_time
            ),
            'observation.state.lift': self.sensor_mgr.get_next_available_lift(
                'lift_pos', trigger_time
            ),
        }

        # Check for missing data
        if any(v is None for v in obs.values()):
            missing = [k for k, v in obs.items() if v is None]
            self.get_logger().warn(f"🟡 Skipping sync frame, missing data: {missing}")
            return

        self.latest_sync_data = obs

        # Check for missing data
        try:
            state = np.concatenate(
                [
                    obs['observation.joint_position.left'],
                    [obs['observation.gripper_position.left']],
                    obs['observation.joint_position.right'],
                    [obs['observation.gripper_position.right']],
                    [obs['observation.state.lift']],
                ]
            ).astype(np.float64)

            image_high = obs['observation.images.cam_head']
            image_left = obs['observation.images.cam_left']
            image_right = obs['observation.images.cam_right']

            obs_npy = {
                "observation.state": state,
                "observation.images.head_camera": image_high,
                "observation.images.left_hand_camera": image_left,
                "observation.images.right_hand_camera": image_right,
            }

            self.last_inference_time = time.time()
            self.sync_count += 1
            self.get_logger().info(f"✅ Sync #{self.sync_count} completed")

            return obs_npy

        except Exception as e:
            self.get_logger().error(f"❌ Sync failed: {e}")
            return None

    # ==================== Utility Functions ====================

    @staticmethod
    def is_valid_timestamp(stamp) -> bool:
        return stamp.sec != 0 or stamp.nanosec != 0

    @staticmethod
    def ros_ts_to_sec(ros_ts):
        return ros_ts.sec + ros_ts.nanosec * 1e-9

    @staticmethod
    def format_timestamp(sec_float):
        sec = int(sec_float)
        ms = int((sec_float - sec) * 1000)
        dt = time.gmtime(sec)
        return time.strftime("%H:%M:%S", dt) + f".{ms:03d}"

    def _decode_compressed_image(self, msg) -> Optional[np.ndarray]:
        if msg is None:
            return None
        try:
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
            if img is None:
                return None
            img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_LINEAR)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return img_rgb
        except Exception as e:
            self.get_logger().warn(f"Decoding failed: {e}")
            return None

    # ==================== LCM Listener Thread ====================
    def lcm_listener(self):
        self.get_logger().info("[LCM] lcm_listener thread started")
        while True:
            try:
                self.lcm.handle()
            except Exception as e:
                self.get_logger().error(f"[LCM] Exception occurred in handle: {e}")
                time.sleep(0.01)

    # ==================== LCM Callbacks ====================

    def handle_act_request(self, channel, data):
        """LCM: rcp_request_feedback"""
        try:
            msg = act_request.decode(data)
        except Exception as e:
            self.get_logger().error(f"Decoding act_request failed: {e}")
            return

        now = time.time()
        with self.pending_lock:
            self.pending_act_request = msg
            self.pending_act_time = now

        self.get_logger().info("📩 Received rcp_request_feedback")

    def handle_request(self, channel, data):
        """LCM: image_request"""
        try:
            msg = req_camera_image.decode(data)
        except Exception as e:
            self.get_logger().error(f"Decoding image_request failed: {e}")
            return

        now = time.time()
        with self.pending_lock:
            self.pending_image_request = msg
            self.pending_image_time = now

        self.get_logger().info("📩 Received image_request")

    def handle_act_command(self, channel, data):
        """LCM: rcp_robotmotion"""
        self.get_logger().info(
            "📩 Received rcp_robotmotion, chunkSize={msg.chunkSize} numJoint={msg.numJoint}"
        )

        try:
            msg = act_command.decode(data)
        except Exception as e:
            self.get_logger().error(f"Decoding act_command failed: {e}")
            return

        if msg.totalNumJoint == msg.chunkSize * msg.numJoint:
            actions = np.array(
                [
                    msg.jointPos[i * msg.numJoint : (i + 1) * msg.numJoint]
                    for i in range(msg.chunkSize)
                ]
            )
            min_steps = msg.chunkSize

            self.action_executor.infer(actions, min_steps)
        else:
            self.get_logger().error("Joint position data length exception")

        state_msg = state_feedback()
        now = time.time()
        state_msg.sec = int(now)
        state_msg.nanosec = int((now - int(now)) * 1e9)
        state_msg.utime = int(now * 1e6)
        state_msg.seq = self._feedback_seq
        self._feedback_seq += 1
        state_msg.act_status_type = state_feedback.kSuccess
        state_msg.error_msg = "Action Succeeded"
        state_msg.stateID = state_feedback.kMove1
        state_msg.substateID = state_feedback.kExit
        state_msg.state_msg = "Executing Move 1"

        self.lcm.publish("state_feedback", state_msg.encode())
        self.get_logger().info(f"[LCM] Send state_feedback Success")

    # ==================== LCM Pairing Check (1s Timeout) ====================

    def check_lcm_pair(self):
        """
        Called every 0.1 seconds:
        - If both requests are present and time is under 1 second -> Pairing processing;
        - If one request has waited over 1 second while the other has not arrived -> Discard it.
        """
        now = time.time()

        with self.pending_lock:
            act_msg = self.pending_act_request
            img_msg = self.pending_image_request
            act_t = self.pending_act_time
            img_t = self.pending_image_time

            # None of them
            if act_msg is None and img_msg is None:
                return

            # Only act_request
            if act_msg is not None and img_msg is None:
                if act_t is not None and now - act_t > self.lcm_pair_timeout:
                    self.get_logger().warn(
                        "⚠️ rcp_request_feedback waiting for image_request timed out, discarding this act_request"
                    )
                    self.pending_act_request = None
                    self.pending_act_time = None
                return

            # Only image_request
            if img_msg is not None and act_msg is None:
                if img_t is not None and now - img_t > self.lcm_pair_timeout:
                    self.get_logger().warn(
                        "⚠️ image_request waiting for rcp_request_feedback timed out, discarding this image_request"
                    )
                    self.pending_image_request = None
                    self.pending_image_time = None
                return

            # Both are present
            if act_msg is not None and img_msg is not None:
                earliest = min(act_t, img_t)
                if now - earliest > self.lcm_pair_timeout:
                    self.get_logger().warn(
                        "⚠️ rcp_request_feedback paired with image_request timed out, discarding both messages"
                    )
                    self.pending_act_request = None
                    self.pending_act_time = None
                    self.pending_image_request = None
                    self.pending_image_time = None
                    return

                # Both received before timeout -> can pair
                act_req = self.pending_act_request
                img_req = self.pending_image_request
                self.pending_act_request = None
                self.pending_act_time = None
                self.pending_image_request = None
                self.pending_image_time = None

        # Synchronize and send outside the lock
        self.handle_lcm_pair_with_obs(act_req, img_req)

    def handle_lcm_pair_with_obs(self, act_req: act_request, img_req: req_camera_image):
        """Successfully paired a single rcp_request_feedback + image_request, process with the same frame obs_npy"""
        obs_npy = self.try_synchronize()
        if obs_npy is None:
            self.get_logger().warn(
                "⚠️ Sync failed, cannot provide data for this LCM request"
            )
            return

        # Send robot_feedback using observation.state
        try:
            state = obs_npy["observation.state"]
            self.send_robot_feedback(state)
        except Exception as e:
            self.get_logger().error(f"[LCM] Error sending robot_feedback: {e}")

        # Generate image_response using obs_npy images + image_request
        try:
            self.build_camera_image_response(img_req, obs_npy)
        except Exception as e:
            self.get_logger().error(f"[LCM] Error sending image_response: {e}")

    # ==================== Constructing LCM Response Using obs_npy ====================
    def send_robot_feedback(self, state: np.ndarray):
        """Construct and send robot_feedback based on obs_npy['observation.state']"""
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
        self.get_logger().info("[LCM] Sent robot_feedback")

    def build_camera_image_response(
        self, img_req: req_camera_image, obs_npy: Dict[str, Any]
    ) -> camera_image_response:
        """
        Construct camera_image_response based on image_request and images in obs_npy.
        camera_names[i] directly corresponds to the keys in obs_npy:
          - "observation.image"
          - "observation.left_wrist_image"
          - "observation.right_wrist_image"

        Encoding method:
        - obs_npy is already a RGB formatted numpy array (H, W, 3)
        - Use np.save() to write to BytesIO, then compress using gzip.compress
        - Place the resulting compressed bytes into image_bytes
        """
        response = camera_image_response()
        response.camera_count = img_req.camera_count
        response.seq = img_req.seq
        response.images = []

        for i in range(img_req.camera_count):
            cam_key = str(img_req.camera_names[i])

            frame_rgb = obs_npy.get(cam_key, None)
            if frame_rgb is None:
                self.get_logger().warning(
                    f"[LCM] No image corresponding to key={cam_key} in obs_npy"
                )
                continue

            # frame_rgb: H x W x 3, uint8, RGB
            height, width, channels = frame_rgb.shape

            try:
                buf = io.BytesIO()
                np.save(buf, frame_rgb)
                buf.seek(0)
                compressed = gzip.compress(buf.getvalue(), compresslevel=1)
            except Exception as e:
                self.get_logger().error(
                    f"[LCM] Image encoding (np.save + gzip) failed: {cam_key}, error: {e}"
                )
                continue

            img_info = image_data()
            img_info.camera_id = img_req.camera_ids[i]
            img_info.camera_name = cam_key
            img_info.width = int(width)
            img_info.height = int(height)
            img_info.channels = int(channels)
            img_info.image_size = len(compressed)
            img_info.image_bytes = compressed

            response.images.append(img_info)

        self.lcm.publish("image_response", response.encode())
        self.get_logger().info("[LCM] Sent image_response")

    def _camera_desc_timer_callback(self):

        with open(self.camera_desc_yaml_path, "r") as f:
            cfg = yaml.safe_load(f)

        cameras = cfg.get("cameras", [])
        if not isinstance(cameras, list):
            self.get_logger().error("[LCM] camera_desc_yaml format is incorrect")
            return

        msg = camera_list_desc()
        msg.seq = 0
        msg.stamp = int(time.time() * 1e6)
        msg.n = len(cameras)
        msg.cameras = []

        for idx, cam in enumerate(cameras):
            item = camera_desc()
            item.name = str(cam.get("name", f"camera_{idx}"))
            item.id = idx

            item.product = str(cam.get("brand", ""))
            item.format = str(cam.get("format", ""))
            item.width = int(cam.get("width", 0))
            item.height = int(cam.get("height", 0))
            item.fps = int(cam.get("framerate", 30))
            item.status = 1

            msg.cameras.append(item)

        self.lcm.publish("camera_desc", msg.encode())
        self.get_logger().info(f"Published camera_list_desc to camera_desc, n={msg.n}")

    # ==================== External Trigger Interface ====================
    def destroy_node(self):
        self.get_logger().info("🛑 Starting to shut down multi-sensor synchronizer...")
        self.shutdown_event.set()
        if hasattr(self, 'lcm_thread') and self.lcm_thread.is_alive():
            self.lcm_thread.join(timeout=1.0)
        super().destroy_node()


# ==================== Main Function ====================
def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorSynchronizer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        node.get_logger().info("🚀 Multi-sensor synchronization system is running...")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🛑 Shutting down the system...")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
