"""
LCM Communication Handler

Handles all LCM communication for robot control including:
- Policy command reception (ACT commands)
- Feedback requests handling
- State and robot feedback publishing
"""

import time
import threading
import logging
import lcm
import numpy as np
from common.lcm.lcmMotion.act_command import act_command
from common.lcm.lcmMotion.act_request import act_request
from common.lcm.lcmMotion.robot_feedback import robot_feedback
from common.lcm.lcmMotion.state_feedback import state_feedback


class LCMHandler:
    """
    Handles all LCM communication for robot control.

    Separates communication logic from the main controller,
    making the code more modular and maintainable.
    """

    def __init__(self, logger=None):
        """
        Initialize LCM handler.

        Args:
            logger: Logger instance (optional)
        """
        self.logger = logger or logging.getLogger(__name__)
        self.lcm_instance = None

        self.first_timestamp = -1
        self.feedback_sequence_number = 0

        self.command_lock = threading.Lock()
        self.ACT_command_received = False
        self.current_ACT_command = None
        self.robot_feedback_requested = False
        self.state_feedback_requested = False
        self.gohome_requested = False

        self.last_command_time = time.time()
        self.has_received_command = False
        self.act_status_index = 0
        self.stateID_index = 0
        self.substateID_index = 0

        self.act_status_cycle = [
            state_feedback.kIdle,
            state_feedback.kSuccess,
            state_feedback.kExecuting,
            state_feedback.kPaused,
            state_feedback.kCollision,
            state_feedback.kFail,
        ]
        self.stateID_cycle = [
            state_feedback.kGoStand,
            state_feedback.kGoHome,
            state_feedback.kMove1,
            state_feedback.kMove2,
            state_feedback.kError,
        ]
        self.substateID_cycle = [
            state_feedback.kEnter,
            state_feedback.kDuring,
            state_feedback.kExit,
        ]

    def connect(self):
        """Initialize LCM connection and subscribe to channels."""
        try:
            self.lcm_instance = lcm.LCM()
            self.lcm_instance.subscribe("rcp_robotmotion", self.handle_policy_commands)
            self.lcm_instance.subscribe("rcp_request_feedback", self.handle_requests)
            self.logger.info("✓ LCM communication ready")
            return True
        except Exception as e:
            self.logger.warning(f"⚠ LCM setup failed: {e}")
            self.lcm_instance = None
            return False

    def disconnect(self):
        """Disconnect LCM."""
        if self.lcm_instance:
            self.lcm_instance = None
            self.logger.info("✓ LCM disconnected")

    def is_connected(self):
        """Check if LCM is connected."""
        return self.lcm_instance is not None

    def process_messages(self, timeout_ms=0):
        """Process incoming LCM messages (non-blocking)."""
        if self.lcm_instance:
            self.lcm_instance.handle_timeout(timeout_ms)

    def handle_policy_commands(self, channel, data):
        """Handle incoming policy commands, like ACT"""
        msg = act_command.decode(data)

        if self.first_timestamp == -1:
            self.first_timestamp = msg.utime
            self.logger.info("First LCM message received!")

        lcm_time = float(msg.utime - self.first_timestamp) * 1e-6
        self.logger.info(
            f"ACT #{msg.seq}, chunks: {msg.chunkSize}, "
            f"joints: {msg.numJoint}, time: {lcm_time:.3f}s"
        )

        with self.command_lock:
            self.current_ACT_command = msg
            self.ACT_command_received = True
            self.last_command_time = time.time()
            self.has_received_command = True

    def handle_requests(self, channel, data):
        """Handle feedback request messages."""
        msg = act_request.decode(data)

        if msg.request_type == act_request.kStateFeedbackRequest:
            self.state_feedback_requested = True
        elif msg.request_type == act_request.kRobotFeedbackRequest:
            self.robot_feedback_requested = True
        elif msg.request_type == act_request.kGoHomeRequest:
            self.gohome_requested = True
            self.logger.info("Go home request received")

    def get_latest_ACT(self):
        """
        Get the latest ACT command from the buffer (thread-safe).

        Returns:
            act_command or None: Latest ACT command, or None if no command available
        """
        with self.command_lock:
            if self.ACT_command_received:
                command = self.current_ACT_command
                self.ACT_command_received = False
                return command
            return None

    def publish_state_feedback(self):
        """Send state feedback message."""
        if not self.lcm_instance:
            return

        msg = state_feedback()
        msg.sec = int(time.time())
        msg.nanosec = int((time.time() % 1) * 1e9)
        msg.utime = int(time.time() * 1000000)
        msg.seq = self.feedback_sequence_number

        msg.act_status_type = self.act_status_cycle[self.act_status_index]
        msg.stateID = self.stateID_cycle[self.stateID_index % len(self.stateID_cycle)]
        msg.substateID = self.substateID_cycle[
            self.substateID_index % len(self.substateID_cycle)
        ]

        self.stateID_index += 1
        self.substateID_index += 1

        error_messages = {
            state_feedback.kIdle: "Action Idle",
            state_feedback.kSuccess: "Action Succeeded",
            state_feedback.kExecuting: "Action Executing",
            state_feedback.kPaused: "Action Paused",
            state_feedback.kCollision: "Collision Detected",
            state_feedback.kFail: "Action Failed",
        }
        msg.error_msg = error_messages.get(msg.act_status_type, "Unknown status")

        state_messages = {
            state_feedback.kGoStand: "Going to Stand",
            state_feedback.kGoHome: "Going Home",
            state_feedback.kMove1: "Executing Move 1",
            state_feedback.kMove2: "Executing Move 2",
            state_feedback.kError: "State Error Occurred",
        }
        msg.state_msg = state_messages.get(msg.stateID, "Current state unknown")

        self.lcm_instance.publish("state_feedback", msg.encode())
        self.logger.info(f"Sent state feedback #{msg.seq}")

    def publish_robot_feedback(self, joint_positions):
        """
        Send robot feedback message.

        Args:
            joint_positions: Current joint positions from the robot interface
        """
        if not self.lcm_instance:
            return

        msg = robot_feedback()
        msg.sec = int(time.time())
        msg.nanosec = int((time.time() % 1) * 1e9)
        msg.utime = int(time.time() * 1000000)
        msg.seq = self.feedback_sequence_number

        msg.numJoint = len(joint_positions) - 1
        msg.qFb = joint_positions[: msg.numJoint].tolist()
        msg.qdFb = [0.0] * msg.numJoint

        msg.numGripper = 1
        msg.gripperPosFb = joint_positions[msg.numJoint :].tolist()

        self.lcm_instance.publish("robot_feedback", msg.encode())
        self.logger.info(f"Sent robot feedback #{msg.seq}")

        self.feedback_sequence_number += 1

    def set_act_status(self, status_index):
        """Set the current action status index."""
        self.act_status_index = status_index

    def check_robot_feedback_request(self):
        """Check if robot feedback was requested and clear the flag."""
        if self.robot_feedback_requested:
            self.robot_feedback_requested = False
            return True
        return False

    def check_state_feedback_request(self):
        """Check if state feedback was requested and clear the flag."""
        if self.state_feedback_requested:
            self.state_feedback_requested = False
            return True
        return False

    def publish_state_feedback_with_status(self, status_index):
        """Publish state feedback with specific status."""
        old_status = self.act_status_index
        self.act_status_index = status_index
        self.publish_state_feedback()
        self.act_status_index = old_status

    def request_robot_feedback(self, joint_positions):
        """Handle robot feedback request with current joint positions."""
        self.publish_robot_feedback(joint_positions)

    def get_time_since_last_command(self):
        """Get time in seconds since last command was received."""
        if not self.has_received_command:
            return 0.0
        return time.time() - self.last_command_time

    def is_command_timeout(self, timeout_seconds):
        """Check if command timeout has occurred."""
        if not self.has_received_command:
            return False
        return self.get_time_since_last_command() > timeout_seconds

    def check_gohome_request(self):
        """Check if go home was requested and clear the flag."""
        if self.gohome_requested:
            return True
        return False
