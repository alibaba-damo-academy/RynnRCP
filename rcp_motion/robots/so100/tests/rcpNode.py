#!/usr/bin/env python3

import time
import math
import lcm
import sys
import os
import numpy as np


from common.lcm.lcmMotion.act_command import act_command
from common.lcm.lcmMotion.act_request import act_request
from common.lcm.lcmMotion.state_feedback import state_feedback
from common.lcm.lcmMotion.robot_feedback import robot_feedback

# Test cases for VLA output
actions_vla = np.array(
    [
        [24.03303757, 94.46789526, 59.32535416, 63.61290231, -166.74484244, 3.55751272],
        [23.04750477, 94.8182611, 58.85384675, 63.84554083, -166.08341069, 4.23319221],
        [21.85856981, 95.22543199, 58.30731758, 64.15302194, -165.48935429, 4.99560129],
        [20.49728645, 95.66095188, 57.68064752, 64.54819448, -164.93710154, 5.29820931],
        [18.98123257, 96.1101937, 56.94247082, 65.0441596, -164.4018602, 5.79293347],
        [17.35040097, 96.52309385, 56.03693844, 65.68953836, -163.84778041, 6.10453296],
        [15.6480634, 96.84604216, 54.90410988, 66.56302001, -163.25600796, 6.56288157],
        [13.87864696, 97.0420848, 53.47681307, 67.74887845, -162.56975726, 7.14865427],
        [12.08581761, 97.1024169, 51.82454523, 69.25137664, -161.77841271, 7.870637],
        [10.26764458, 97.06132611, 50.00344555, 71.05895359, -160.87110636, 8.53408347],
        [8.43628744, 96.96093424, 48.13086704, 73.07983613, -159.84647787, 9.00863424],
        [6.59852019, 96.84858413, 46.30985648, 75.22735797, -158.6945439, 9.50507656],
        [4.77176818, 96.73388901, 44.59821408, 77.42185138, -157.40129042, 9.98172504],
        [2.97849186, 96.61334852, 43.02413519, 79.58938711, -155.97762939, 10.32685816],
        [1.24141658, 96.46895563, 41.58960289, 81.69024696, -154.4357273, 10.61228333],
        [-0.42547352, 96.2741998, 40.2568222, 83.70190511, -152.80292465, 10.87177012],
        [
            -1.99501868,
            96.00776286,
            39.01951931,
            85.59488712,
            -151.12216695,
            11.16823024,
        ],
        [-3.4423824, 95.66537277, 37.89735097, 87.31091292, -149.42984718, 11.34256669],
        [
            -4.77471213,
            95.24903219,
            36.89864111,
            88.83563414,
            -147.78570886,
            11.46810057,
        ],
        [-5.9919432, 94.76233062, 36.05320008, 90.13365552, -146.21560434, 11.77679474],
    ]
)
actions_vla_rad = np.radians(actions_vla)

# Test cases for sin
actions_sin = np.array(
    [
        [0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000],
        [0.75000000, 0.70000000, 0.65000000, 0.60000000, 0.55000000, 0.50000000],
        [1.29903811, 1.21243557, 1.12583302, 1.03923048, 0.95262794, 0.86602540],
        [1.50000000, 1.40000000, 1.30000000, 1.20000000, 1.10000000, 1.00000000],
        [1.29903811, 1.21243557, 1.12583302, 1.03923048, 0.95262794, 0.86602540],
        [0.75000000, 0.70000000, 0.65000000, 0.60000000, 0.55000000, 0.50000000],
        [0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000],
        [-0.75000000, -0.70000000, -0.65000000, -0.60000000, -0.55000000, -0.50000000],
        [-1.29903811, -1.21243557, -1.12583302, -1.03923048, -0.95262794, -0.86602540],
        [-1.50000000, -1.40000000, -1.30000000, -1.20000000, -1.10000000, -1.00000000],
        [-1.29903811, -1.21243557, -1.12583302, -1.03923048, -0.95262794, -0.86602540],
        [-0.75000000, -0.70000000, -0.65000000, -0.60000000, -0.55000000, -0.50000000],
        [0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000],
        [0.75000000, 0.70000000, 0.65000000, 0.60000000, 0.55000000, 0.50000000],
        [1.29903811, 1.21243557, 1.12583302, 1.03923048, 0.95262794, 0.86602540],
        [1.50000000, 1.40000000, 1.30000000, 1.20000000, 1.10000000, 1.00000000],
        [1.29903811, 1.21243557, 1.12583302, 1.03923048, 0.95262794, 0.86602540],
        [0.75000000, 0.70000000, 0.65000000, 0.60000000, 0.55000000, 0.50000000],
        [0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000],
        [-0.75000000, -0.70000000, -0.65000000, -0.60000000, -0.55000000, -0.50000000],
    ]
)


class RCPSender:
    def __init__(self):
        self.lcm = lcm.LCM()
        self.sequence_number = 0
        self.request_sequence_number = 0
        self.start_time = time.time()

    def handle_state_feedback(self, channel, data):
        msg = state_feedback.decode(data)
        status_text = "Unknown Act Status"
        act_status_map = {
            state_feedback.kIdle: "Idle",
            state_feedback.kSuccess: "Success",
            state_feedback.kExecuting: "Executing",
            state_feedback.kPaused: "Paused",
            state_feedback.kCollision: "Collision",
            state_feedback.kFail: "Fail",
        }
        status_text = act_status_map.get(msg.act_status_type, status_text)

        stateID_text = "Unknown StateID"
        stateID_map = {
            state_feedback.kGoStand: "GoStand",
            state_feedback.kGoHome: "GoHome",
            state_feedback.kMove1: "Move1",
            state_feedback.kMove2: "Move2",
            state_feedback.kError: "ErrorState",
        }
        stateID_text = stateID_map.get(msg.stateID, stateID_text)

        substateID_text = "Unknown SubStateID"
        substateID_map = {
            state_feedback.kEnter: "Enter",
            state_feedback.kDuring: "During",
            state_feedback.kExit: "Exit",
        }
        substateID_text = substateID_map.get(msg.substateID, substateID_text)

        print(
            f"Received State Feedback #{msg.seq}, "
            f"act_status: {msg.act_status_type} ({status_text}), error: '{msg.error_msg}', "
            f"stateID: {msg.stateID} ({stateID_text}), substateID: {msg.substateID} ({substateID_text}), state_msg: '{msg.state_msg}'"
        )

    def handle_robot_feedback(self, channel, data):
        msg = robot_feedback.decode(data)
        print(
            f"Received Robot Feedback #{msg.seq}, joints: {msg.numJoint}, grippers: {msg.numGripper}"
        )

    def send_chunked_commands(
        self, actions_val, chunk_size, num_joints, num_grippers=0
    ):
        msg = act_command()

        msg.sec = int(time.time())
        msg.nanosec = int((time.time() % 1) * 1e9)
        msg.utime = int(time.time() * 1000000)
        msg.seq = self.sequence_number
        self.sequence_number += 1

        msg.chunkSize = chunk_size
        msg.numJoint = num_joints
        msg.numGripper = num_grippers

        msg.totalNumJoint = chunk_size * num_joints
        msg.totalNumGripper = chunk_size * num_grippers
        msg.totalEePos = chunk_size * 3
        msg.totalEeQuat = chunk_size * 4

        msg.jointPos = [0.0] * msg.totalNumJoint
        msg.jointVel = [0.0] * msg.totalNumJoint
        msg.gripperPos = [0.0] * msg.totalNumGripper
        msg.eePos = [0.0] * msg.totalEePos
        msg.eeQuat = [0.0] * msg.totalEeQuat

        # Fixed values for each joint and chunk
        joint_base_values = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # Base values for each joint
        joint_vel_base_values = [
            0.01,
            0.02,
            0.03,
            0.04,
            0.05,
            0.06,
            0.07,
        ]  # Base velocities

        for chunk in range(chunk_size):
            for joint in range(num_joints):
                idx = chunk * num_joints + joint
                msg.jointPos[idx] = joint_base_values[joint] + actions_val[chunk][joint]
                msg.jointVel[idx] = joint_vel_base_values[joint] + (chunk * 0.01)

            for gripper in range(num_grippers):
                idx = chunk * num_grippers + gripper
                msg.gripperPos[idx] = actions_val[chunk][num_joints + gripper]

            ee_pos_base = chunk * 3
            msg.eePos[ee_pos_base + 0] = 0.5 + (chunk * 0.1)  # x
            msg.eePos[ee_pos_base + 1] = 0.0 + (chunk * 0.05)  # y
            msg.eePos[ee_pos_base + 2] = 0.3 + (chunk * 0.02)  # z

            ee_quat_base = chunk * 4
            msg.eeQuat[ee_quat_base + 0] = 1.0  # w
            msg.eeQuat[ee_quat_base + 1] = 0.0  # x
            msg.eeQuat[ee_quat_base + 2] = 0.0  # y
            msg.eeQuat[ee_quat_base + 3] = 0.0  # z

        msg.workMode = 1

        self.lcm.publish("rcp_robotmotion", msg.encode())

        elapsed_time = time.time() - self.start_time
        print(
            f"Send ACT #{msg.seq} with {chunk_size} chunks, {num_joints} joints, {num_grippers} grippers at time: {elapsed_time:.3f}s"
        )

    def send_request_feedback(self, request_type):
        msg = act_request()

        msg.sec = int(time.time())
        msg.nanosec = int((time.time() % 1) * 1e9)
        msg.utime = int(time.time() * 1000000)
        msg.seq = self.request_sequence_number
        self.request_sequence_number += 1

        msg.request_type = request_type

        self.lcm.publish("rcp_request_feedback", msg.encode())

        if request_type == act_request.kStateFeedbackRequest:
            request_name = "ACT Status"
        elif request_type == act_request.kRobotFeedbackRequest:
            request_name = "Robot Status"
        else:
            request_name = "Go Home Request"
        print(f"Send {request_name} Request #{msg.seq}")

    def subscribe_feedback(self):
        self.lcm.subscribe("state_feedback", self.handle_state_feedback)
        self.lcm.subscribe("robot_feedback", self.handle_robot_feedback)


def main():
    sender = RCPSender()

    print("Starting RCP Node with chunked commands... (Press Ctrl+C to stop)")
    print("Publishing every 2 seconds with varying chunk sizes...")
    print("Also sending feedback requests every 5 seconds...")
    print("Subscribing to feedback channels...")

    sender.subscribe_feedback()

    with open("sample_policy.txt", "r") as file:
        lines = file.readlines()

    total_lines = len(lines)

    chunk_size = 20

    num_joints = 5
    num_grippers = 1

    chunk_sizes = [20]
    chunk_index = 0

    request_types = [
        act_request.kStateFeedbackRequest,
        act_request.kRobotFeedbackRequest,
    ]
    request_index = 0

    last_request_time = time.time()
    send_command = True

    try:
        while True:

            if send_command:
                send_command = False
                start_index = chunk_index * chunk_size
                end_index = start_index + chunk_size

                if end_index > total_lines:
                    print("No more complete chunks to process. Exiting loop.")
                    break

                actions_val = []
                current_chunk = lines[start_index:end_index]
                for line in current_chunk:
                    line = line.strip()
                    joint_positions = list(map(float, line.split()))
                    actions_val.append(joint_positions)
                sender.send_chunked_commands(
                    actions_val, chunk_size, num_joints, num_grippers
                )
                chunk_index += 1
            else:
                send_command = True
                current_request_type = request_types[request_index % len(request_types)]
                sender.send_request_feedback(current_request_type)
                request_index += 1

            sender.lcm.handle_timeout(100)  # 100ms timeout

            time.sleep(0.6)

    except KeyboardInterrupt:
        print("\nStopping RCP Node...")


if __name__ == "__main__":
    main()
