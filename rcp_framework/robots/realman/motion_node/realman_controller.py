#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import json
import time
import re
from typing import List, Optional


class RealmanController:
    """
    Realman Dual-arm + Integrated Lift Controller
    Supports:
      - Left/Right Arm: movej (absolute positioning), movej_canfd (high/low follow)
      - Left/Right Gripper: hand_follow_pos
      - Lift Control: set_lift_speed
    """

    def __init__(self, LEFT_IP, LEFT_PORT, RIGHT_IP, RIGHT_PORT):
        # IP Configuration
        # self.LEFT_IP = "192.168.10.110"
        # self.RIGHT_IP = "192.168.10.111"
        # self.PORT = 8080
        self.LEFT_IP = LEFT_IP
        self.RIGHT_IP = RIGHT_IP
        self.LEFT_PORT = LEFT_PORT
        self.RIGHT_PORT = RIGHT_PORT

        # Timeout Configuration
        self.CONNECT_TIMEOUT = 2.0
        self.SEND_TIMEOUT = (
            0.005  # 5ms to ensure high frequency does not block the main loop
        )

        # Connection Handles
        self.left_sock: Optional[socket.socket] = None
        self.right_sock: Optional[socket.socket] = None

        # Establish Connection
        self.connect()

    def connect(self) -> bool:
        """Establish connections for left and right arms"""
        success = True
        if not self._connect_side("left"):
            success = False
        if not self._connect_side("right"):
            success = False
        return success

    def _connect_side(self, side: str) -> bool:
        """Connect to one side of the robotic arm"""
        ip = self.LEFT_IP if side == "left" else self.RIGHT_IP
        port = self.LEFT_PORT if side == "left" else self.RIGHT_PORT
        sock_attr = "left_sock" if side == "left" else "right_sock"

        try:
            sock = socket.create_connection((ip, port), timeout=self.CONNECT_TIMEOUT)
            sock.settimeout(self.SEND_TIMEOUT)
            setattr(self, sock_attr, sock)
            print(f"[✅] {side.upper()} arm connected: {ip}:{port}")
            return True
        except Exception as e:
            print(f"[❌] {side.upper()} arm connect failed: {e}")
            setattr(self, sock_attr, None)
            return False

    def _send(self, sock: socket.socket, cmd: dict):
        """Send command (internal method)"""
        try:
            message = json.dumps(cmd, ensure_ascii=True) + "\r\n"
            sock.send(message.encode('utf-8'))
            print(f"[📤] SENT: {cmd}")
        except Exception as e:
            print(f"[❌] Send failed: {e}")
            self._reconnect_sock(sock)

    def _reconnect_sock(self, old_sock: socket.socket):
        """Attempt to reconnect"""
        try:
            old_sock.close()
        except:
            pass
        # Simple reconnect logic, can implement retries
        print("[🔁] Attempting to reconnect...")
        self.connect()

    # ==================== 夹爪控制 ====================

    def set_left_gripper(self, pos: int):
        """Control left gripper (unit: internal encoding value)"""
        if not self.left_sock:
            print("[⚠️] Left arm not connected, cannot control gripper")
            return
        cmd = {"command": "hand_follow_pos", "hand_pos": [int(pos)]}
        self._send(self.left_sock, cmd)

    def set_right_gripper(self, pos: int):
        """Control right gripper"""
        if not self.right_sock:
            print("[⚠️] Right arm not connected, cannot control gripper")
            return
        cmd = {"command": "hand_follow_pos", "hand_pos": [int(pos)]}
        self._send(self.right_sock, cmd)

    # ==================== 升降控制 ====================

    def lift_up(self):
        """Move up"""
        if not self.right_sock:
            print("[⚠️] Right arm not connected, cannot control lift")
            return
        cmd = {"command": "set_lift_speed", "speed": 50}
        self._send(self.right_sock, cmd)

    def lift_down(self):
        """Move down"""
        cmd = {"command": "set_lift_speed", "speed": -50}
        self._send(self.right_sock, cmd)

    def lift_stop(self):
        """Stop"""
        cmd = {"command": "set_lift_speed", "speed": 0}
        self._send(self.right_sock, cmd)

    def lift_by_dir(self, direction: float):
        """Direction control: 1 = up, -1 = down, 0 = stop"""
        if direction > 0.8:
            self.lift_up()
        elif direction < -0.8:
            self.lift_down()
        else:
            self.lift_stop()

    # ==================== Arm Control ====================

    def movej_left(self, joints: List[float], velocity: int = 10, radius: int = 0):
        """Left arm movej (absolute positioning)"""
        if not self.left_sock:
            print("[⚠️] Left arm not connected")
            return
        cmd = {
            "command": "movej",
            "joint": [float(j) for j in joints],
            "v": velocity,
            "r": radius,
            "trajectory_connect": 0,
        }
        self._send(self.left_sock, cmd)

    def movej_right(self, joints: List[float], velocity: int = 10, radius: int = 0):
        """Right arm movej (absolute positioning)"""
        if not self.right_sock:
            print("[⚠️] Right arm not connected")
            return
        cmd = {
            "command": "movej",
            "joint": [float(j) for j in joints],
            "v": velocity,
            "r": radius,
            "trajectory_connect": 0,
        }
        self._send(self.right_sock, cmd)

    def movej_canfd_left(self, joints: List[float], follow: bool = False):
        """Left arm movej_canfd"""
        if not self.left_sock:
            print("[⚠️] Left arm not connected")
            return
        cmd = {
            "command": "movej_canfd",
            "joint": [float(j) for j in joints],
            "follow": bool(follow),
            "expand": 1000,
            "trajectory_mode": 1,
        }
        self._send(self.left_sock, cmd)

    def movej_canfd_right(self, joints: List[float], follow: bool = False):
        """Right arm movej_canfd"""
        if not self.right_sock:
            print("[⚠️] Right arm not connected")
            return
        cmd = {
            "command": "movej_canfd",
            "joint": [float(j) for j in joints],
            "follow": bool(follow),
            "expand": 1000,
            "trajectory_mode": 1,
        }
        self._send(self.right_sock, cmd)

    # ==================== Trajectory Playback ====================

    def parse_joints_from_txt(self, file_path: str) -> List[List[float]]:
        """Parse movej_canfd joint data from txt file"""
        all_joints = []
        replacements = [
            (r"'", '"'),
            (r"True", "true"),
            (r"False", "false"),
            (r"None", "null"),
        ]

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    cleaned = line
                    for old, new in replacements:
                        cleaned = re.sub(old, new, cleaned)
                    data = json.loads(cleaned)
                    if data.get("command") == "movej_canfd" and isinstance(
                        data["joint"], list
                    ):
                        all_joints.append([float(x) for x in data["joint"]])
        except Exception as e:
            print(f"[❌] Trajectory file parsing failed: {e}")

        return all_joints

    def playback_left_trajectory(
        self, file_path: str, mode: str = "low_follow", delay: float = 0.5
    ):
        """
        Playback left arm trajectory
        :param file_path: trajectory file path
        :param mode: "movej", "high_follow", "low_follow"
        :param delay: delay time (seconds)
        """
        joints_list = self.parse_joints_from_txt(file_path)
        if not joints_list:
            print("[⚠️] No valid trajectory data parsed")
            return

        print(
            f"[▶️] Starting playback of left arm trajectory, mode: {mode}, total {len(joints_list)} frames"
        )

        for i, joint in enumerate(joints_list):
            if i == 0 and mode == "movej":
                self.movej_left(joint)
                time.sleep(5)  # Wait for positioning
            else:
                if mode == "high_follow":
                    self.movej_canfd_left(joint, follow=True)
                    time.sleep(0.03)
                else:  # low_follow
                    self.movej_canfd_left(joint, follow=False)
                    time.sleep(delay)

    def playback_right_trajectory(
        self, file_path: str, mode: str = "low_follow", delay: float = 0.5
    ):
        """Playback right arm trajectory"""
        joints_list = self.parse_joints_from_txt(file_path)
        if not joints_list:
            print("[⚠️] No valid trajectory data parsed")
            return

        print(
            f"[▶️] Starting playback of right arm trajectory, mode: {mode}, total {len(joints_list)} frames"
        )

        for i, joint in enumerate(joints_list):
            if i == 0 and mode == "movej":
                self.movej_right(joint)
                time.sleep(5)
            else:
                if mode == "high_follow":
                    self.movej_canfd_right(joint, follow=True)
                    time.sleep(0.03)
                else:
                    self.movej_canfd_right(joint, follow=False)
                    time.sleep(delay)

    # ==================== Cleanup ====================

    def close(self):
        """Close all connections"""
        if self.left_sock:
            try:
                self.left_sock.close()
            except:
                pass
        if self.right_sock:
            try:
                self.right_sock.close()
            except:
                pass
        print("[🔌] All connections have been closed")
