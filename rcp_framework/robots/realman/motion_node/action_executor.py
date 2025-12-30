#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import numpy as np
from typing import List

from .realman_controller import RealmanController


class ActionExecutor:
    """
    Action Executor
    """

    def __init__(self, LEFT_IP, LEFT_PORT, RIGHT_IP, RIGHT_PORT):
        self.inference_count = 0

        self.LEFT_IP = LEFT_IP
        self.LEFT_PORT = LEFT_PORT
        self.RIGHT_IP = RIGHT_IP
        self.RIGHT_PORT = RIGHT_PORT
        self.ctrl = None
        self.init_components()

        print("🟢 Initialization successful")

    def init_components(self):
        """Initialize the controller"""
        self.ctrl = RealmanController(
            self.LEFT_IP, self.LEFT_PORT, self.RIGHT_IP, self.RIGHT_PORT
        )

        # Initial pose: both arms pointing straight down
        left_arm_joints_init = [
            float(joint_r) * 1000.0 * (180.0 / math.pi)
            for joint_r in [
                -7.8210898e-02,
                -2.1041908e00,
                8.1289077e-01,
                -1.7949070e-01,
                -2.0505147e00,
                -2.0704424e-01,
                1.8220417e00,
            ]
        ]
        right_arm_joints_init = [
            float(joint_r) * 1000.0 * (180.0 / math.pi)
            for joint_r in [
                1.9477689e-01,
                2.1103506e00,
                -3.6501908e-01,
                1.0431610e-01,
                2.1013290e-01,
                2.1446049e-02,
                -1.9781321e-01,
            ]
        ]
        left_gripper_position = int(1.0 * 12000)
        right_gripper_position = int(1.0 * 12000)
        lift_value = 0.0

        self.run_controller(
            "MOVEJ",
            left_arm_joints_init,
            left_gripper_position,
            right_arm_joints_init,
            right_gripper_position,
            lift_value,
        )
        time.sleep(3)
        print("🟢 Initial pose setup completed")

    def run_controller(
        self,
        move_type: str,
        left_arm_joints: List[float],
        left_gripper_position: int,
        right_arm_joints: List[float],
        right_gripper_position: int,
        lift_value: float,
    ):
        try:
            if move_type == "MOVEJ":
                self.ctrl.movej_left(left_arm_joints)
                self.ctrl.set_left_gripper(left_gripper_position)
                self.ctrl.movej_right(right_arm_joints)
                self.ctrl.set_right_gripper(right_gripper_position)
                self.ctrl.lift_by_dir(lift_value)
                time.sleep(5.0)
            elif move_type == "MOVEJ_CANFD":
                self.ctrl.movej_canfd_left(left_arm_joints, follow=False)
                self.ctrl.set_left_gripper(left_gripper_position)
                self.ctrl.movej_canfd_right(right_arm_joints, follow=False)
                self.ctrl.set_right_gripper(right_gripper_position)
                self.ctrl.lift_by_dir(lift_value)
                # time.sleep(0.005)
        except Exception as e:
            print(f"❌ Control failed: {e}")

    def infer(self, actions, min_steps=5):
        try:
            # for id_a in range(min(min_steps, len(actions))):  # Safety limit
            for id_a in range(min(min_steps, 10)):  # Safety limit
                action = actions[id_a]
                left_q = list(action[:7] * (180.0 / math.pi) * 1000.0)
                left_g = int((1.0 - action[7]) * 12000)
                right_q = list(action[8:15] * (180.0 / math.pi) * 1000.0)
                right_g = int((1.0 - action[15]) * 12000)
                lift_v = float(action[16])

                move_type = "MOVEJ" if self.inference_count == 0 else "MOVEJ_CANFD"
                self.run_controller(move_type, left_q, left_g, right_q, right_g, lift_v)
                self.inference_count += 1

            time.sleep(1)
            print(f"✅ Execution of {self.inference_count} completed")

        except Exception as e:
            print(f"❌ Execution failed: {e}")
