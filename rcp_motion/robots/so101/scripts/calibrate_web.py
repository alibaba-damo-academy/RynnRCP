#!/usr/bin/env python3
# Copyright 2025 RynnRCP. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Web-based calibration controller for SO101 robot.

This module provides a non-interactive calibration interface that can be 
controlled via Web API. The calibration process is split into discrete steps
that can be triggered by the web frontend:

1. connect() - Connect to the robot and prepare for calibration
2. record_middle_position() - Record the homing offsets at middle position
3. start_recording_range() - Start recording joint range of motion
4. stop_recording_range() - Stop recording and save calibration
5. disconnect() - Disconnect from the robot

Usage:
    controller = WebCalibrationController(arm='follower', config_path='...')
    controller.connect()
    # User moves arm to middle position
    controller.record_middle_position()
    # User moves joints through range
    controller.start_recording_range()
    # ... user moves joints ...
    result = controller.stop_recording_range()
    controller.disconnect()
"""

from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import yaml

logger = logging.getLogger(__name__)


class CalibrationState(Enum):
    """Calibration state machine states."""
    IDLE = "idle"
    CONNECTED = "connected"
    MIDDLE_RECORDED = "middle_recorded"
    RECORDING_RANGE = "recording_range"
    DONE = "done"
    ERROR = "error"


@dataclass
class CalibrationResult:
    """Result of calibration process."""
    success: bool
    message: str
    calibration_path: Optional[str] = None
    calibration_data: Optional[Dict[str, Any]] = None


class WebCalibrationController:
    """
    Non-interactive calibration controller for SO101 robot.
    
    Designed to be controlled via Web API with discrete steps.
    """
    
    def __init__(
        self,
        arm: str = "follower",
        robot_type: str = "so101",
        config_path: Optional[str] = None,
        log_callback: Optional[Callable[[str, str], None]] = None,
    ):
        """
        Initialize the calibration controller.
        
        Args:
            arm: 'follower' or 'leader'
            robot_type: 'so101' or 'lekiwi'
            config_path: Path to config file (optional)
            log_callback: Callback for logging messages (level, message)
        """
        self.arm = arm
        self.robot_type = robot_type
        self.config_path = config_path
        self.log_callback = log_callback
        
        self._state = CalibrationState.IDLE
        self._device = None
        self._homing_offsets = {}
        self._recording_thread: Optional[threading.Thread] = None
        self._stop_recording = threading.Event()
        self._mins = {}
        self._maxes = {}
        self._current_positions = {}
        self._error_message = ""
        
        # Set language to Chinese by default
        self._setup_language()
    
    def _setup_language(self):
        """Setup language to Chinese."""
        try:
            from rcp_motion.robots.so101.scripts.lang import set_language
            set_language("zh")
        except Exception:
            pass
    
    def _log(self, level: str, message: str):
        """Log a message."""
        logger.info(f"[{level.upper()}] {message}")
        if self.log_callback:
            self.log_callback(level, message)
    
    @property
    def state(self) -> str:
        """Get current state."""
        return self._state.value
    
    @property
    def current_positions(self) -> Dict[str, int]:
        """Get current motor positions."""
        return self._current_positions.copy()
    
    @property
    def recorded_mins(self) -> Dict[str, int]:
        """Get recorded minimum positions."""
        return self._mins.copy()
    
    @property
    def recorded_maxes(self) -> Dict[str, int]:
        """Get recorded maximum positions."""
        return self._maxes.copy()
    
    def _get_config_path(self) -> str:
        """Get the path to the config file."""
        if self.config_path:
            return self.config_path
        
        import rcp_motion
        package_dir = Path(rcp_motion.__file__).parent
        config_filename = f"{self.robot_type}.yaml"
        return str(package_dir / "robots" / "so101" / "configs" / config_filename)
    
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        config_path = self._get_config_path()
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            return config if config else {}
        except Exception as e:
            self._log("error", f"加载配置文件失败: {e}")
            return {}
    
    def _create_device(self):
        """Create the robot device instance."""
        from rcp_motion.robots.so101.hardware.robots.so101_follower.config_so101_follower import (
            SO101FollowerConfig,
        )
        from rcp_motion.robots.so101.hardware.robots.so101_follower.so101_follower import (
            SO101Follower,
        )
        
        config = self._load_config()
        
        if self.arm == "follower":
            robot_config = config.get("robot", {})
            port = robot_config.get("port", "/dev/ttyACM0")
            calibration_dir = robot_config.get("calibration_dir")
        else:  # leader
            teleop_config = config.get("teleoperate", {})
            port = teleop_config.get("port", "/dev/ttyUSB0")
            calibration_dir = teleop_config.get("calibration_dir")
        
        # Extract ID from calibration_dir
        robot_id = Path(os.path.expanduser(calibration_dir)).name if calibration_dir else self.robot_type
        
        device_config = SO101FollowerConfig(port=port, id=robot_id)
        if calibration_dir:
            device_config.calibration_dir = Path(os.path.expanduser(calibration_dir))
        
        return SO101Follower(device_config)
    
    def connect(self) -> Dict[str, Any]:
        """
        Connect to the robot and prepare for calibration.
        
        Returns:
            Dict with success status and message
        """
        if self._state != CalibrationState.IDLE:
            return {"success": False, "message": f"无效状态: {self._state.value}"}
        
        try:
            self._log("info", "正在连接机械臂...")
            self._device = self._create_device()
            
            # Connect without auto-calibration
            self._device.bus.connect()
            
            # Disable torque and set operating mode
            self._log("info", "禁用扭矩，准备标定...")
            self._device.bus.disable_torque()
            
            from rcp_motion.robots.so101.hardware.motors.feetech import OperatingMode
            for motor in self._device.bus.motors:
                self._device.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            
            self._state = CalibrationState.CONNECTED
            self._log("info", "✅ 连接成功，请将机械臂移动到中间位置")
            
            return {
                "success": True,
                "message": "连接成功，请将机械臂移动到中间位置",
                "motors": list(self._device.bus.motors.keys())
            }
            
        except Exception as e:
            self._error_message = str(e)
            self._state = CalibrationState.ERROR
            self._log("error", f"连接失败: {e}")
            return {"success": False, "message": f"连接失败: {e}"}
    
    def record_middle_position(self) -> Dict[str, Any]:
        """
        Record the homing offsets at middle position.
        
        Returns:
            Dict with success status, message and homing offsets
        """
        if self._state != CalibrationState.CONNECTED:
            return {"success": False, "message": f"无效状态: {self._state.value}"}
        
        try:
            self._log("info", "正在记录中间位置...")
            
            # Reset calibration and record homing offsets
            self._device.bus.reset_calibration()
            self._homing_offsets = self._device.bus.set_half_turn_homings()
            
            self._state = CalibrationState.MIDDLE_RECORDED
            self._log("info", "✅ 中间位置已记录，请开始移动各关节到极限位置")
            
            return {
                "success": True,
                "message": "中间位置已记录，请开始移动各关节到极限位置",
                "homing_offsets": {k: int(v) for k, v in self._homing_offsets.items()}
            }
            
        except Exception as e:
            self._error_message = str(e)
            self._state = CalibrationState.ERROR
            self._log("error", f"记录中间位置失败: {e}")
            return {"success": False, "message": f"记录中间位置失败: {e}"}
    
    def start_recording_range(self) -> Dict[str, Any]:
        """
        Start recording joint range of motion.
        
        Returns:
            Dict with success status and message
        """
        if self._state != CalibrationState.MIDDLE_RECORDED:
            return {"success": False, "message": f"无效状态: {self._state.value}"}
        
        try:
            self._log("info", "开始记录运动范围...")
            
            motors = list(self._device.bus.motors.keys())
            start_positions = self._device.bus.sync_read("Present_Position", motors, normalize=False)
            self._mins = start_positions.copy()
            self._maxes = start_positions.copy()
            self._current_positions = start_positions.copy()
            
            self._stop_recording.clear()
            self._state = CalibrationState.RECORDING_RANGE
            
            # Start background thread to continuously record positions
            self._recording_thread = threading.Thread(
                target=self._recording_loop, daemon=True
            )
            self._recording_thread.start()
            
            self._log("info", "✅ 正在记录运动范围，请移动各关节到极限位置，完成后点击结束按钮")
            
            return {
                "success": True,
                "message": "正在记录运动范围，请移动各关节到极限位置",
                "motors": motors
            }
            
        except Exception as e:
            self._error_message = str(e)
            self._state = CalibrationState.ERROR
            self._log("error", f"开始记录失败: {e}")
            return {"success": False, "message": f"开始记录失败: {e}"}
    
    def _recording_loop(self):
        """Background loop to record motor positions."""
        motors = list(self._device.bus.motors.keys())
        
        while not self._stop_recording.is_set():
            try:
                positions = self._device.bus.sync_read("Present_Position", motors, normalize=False)
                self._current_positions = positions.copy()
                
                # Update mins and maxes
                for motor in motors:
                    self._mins[motor] = min(positions[motor], self._mins[motor])
                    self._maxes[motor] = max(positions[motor], self._maxes[motor])
                
                time.sleep(0.02)  # 50Hz
                
            except Exception as e:
                self._log("warning", f"读取位置失败: {e}")
                time.sleep(0.1)
    
    def get_recording_status(self) -> Dict[str, Any]:
        """
        Get current recording status.
        
        Returns:
            Dict with current positions, mins, and maxes
        """
        if self._state != CalibrationState.RECORDING_RANGE:
            return {"success": False, "message": f"未在记录状态: {self._state.value}"}
        
        return {
            "success": True,
            "current_positions": {k: int(v) for k, v in self._current_positions.items()},
            "mins": {k: int(v) for k, v in self._mins.items()},
            "maxes": {k: int(v) for k, v in self._maxes.items()}
        }
    
    def stop_recording_range(self) -> Dict[str, Any]:
        """
        Stop recording and save calibration.
        
        Returns:
            Dict with success status, message and calibration result
        """
        if self._state != CalibrationState.RECORDING_RANGE:
            return {"success": False, "message": f"无效状态: {self._state.value}"}
        
        try:
            self._log("info", "停止记录，正在保存标定数据...")
            
            # Stop recording thread
            self._stop_recording.set()
            if self._recording_thread:
                self._recording_thread.join(timeout=1.0)
            
            # Check if any motor has same min and max
            motors = list(self._device.bus.motors.keys())
            same_min_max = [motor for motor in motors if self._mins[motor] == self._maxes[motor]]
            if same_min_max:
                self._state = CalibrationState.ERROR
                msg = f"以下关节未移动: {same_min_max}"
                self._log("error", msg)
                return {"success": False, "message": msg}
            
            # Build calibration data
            from rcp_motion.robots.so101.hardware.motors import MotorCalibration
            
            self._device.calibration = {}
            for motor, m in self._device.bus.motors.items():
                self._device.calibration[motor] = MotorCalibration(
                    id=m.id,
                    drive_mode=0,
                    homing_offset=self._homing_offsets[motor],
                    range_min=self._mins[motor],
                    range_max=self._maxes[motor],
                )
            
            # Write calibration to motors and save to file
            self._device.bus.write_calibration(self._device.calibration)
            self._device._save_calibration()
            
            calibration_path = str(self._device.calibration_fpath)
            self._state = CalibrationState.DONE
            self._log("info", f"✅ 标定完成！标定文件已保存到: {calibration_path}")
            
            # Convert calibration data to serializable format
            calibration_data = {}
            for motor, calib in self._device.calibration.items():
                calibration_data[motor] = {
                    "id": calib.id,
                    "drive_mode": calib.drive_mode,
                    "homing_offset": int(calib.homing_offset),
                    "range_min": int(calib.range_min),
                    "range_max": int(calib.range_max),
                }
            
            return {
                "success": True,
                "message": f"标定完成！标定文件已保存到: {calibration_path}",
                "calibration_path": calibration_path,
                "calibration_data": calibration_data
            }
            
        except Exception as e:
            self._error_message = str(e)
            self._state = CalibrationState.ERROR
            self._log("error", f"保存标定失败: {e}")
            return {"success": False, "message": f"保存标定失败: {e}"}
    
    def disconnect(self) -> Dict[str, Any]:
        """
        Disconnect from the robot.
        
        Returns:
            Dict with success status and message
        """
        try:
            # Stop recording if still running
            self._stop_recording.set()
            if self._recording_thread:
                self._recording_thread.join(timeout=1.0)
            
            if self._device:
                self._log("info", "正在断开连接...")
                self._device.bus.disconnect()
                self._device = None
            
            self._state = CalibrationState.IDLE
            self._log("info", "✅ 已断开连接")
            
            return {"success": True, "message": "已断开连接"}
            
        except Exception as e:
            self._log("error", f"断开连接失败: {e}")
            return {"success": False, "message": f"断开连接失败: {e}"}
    
    def reset(self) -> Dict[str, Any]:
        """
        Reset the controller state.
        
        Returns:
            Dict with success status and message
        """
        self.disconnect()
        self._state = CalibrationState.IDLE
        self._homing_offsets = {}
        self._mins = {}
        self._maxes = {}
        self._current_positions = {}
        self._error_message = ""
        return {"success": True, "message": "已重置"}


# Global controller instances for each arm
_controllers: Dict[str, WebCalibrationController] = {}


def get_controller(arm: str, **kwargs) -> WebCalibrationController:
    """Get or create a calibration controller for the specified arm."""
    if arm not in _controllers:
        _controllers[arm] = WebCalibrationController(arm=arm, **kwargs)
    return _controllers[arm]


def reset_controller(arm: str):
    """Reset and remove the controller for the specified arm."""
    if arm in _controllers:
        _controllers[arm].reset()
        del _controllers[arm]
