#!/usr/bin/env python3
"""
SO101 Robot Web Configuration Tool

A web-based configuration tool for SO101 robot, providing:
- Device settings (RynnBot)
- Camera configuration with live preview
- Serial port detection with auto-refresh
- Calibration with real-time log streaming

Based on the TeleopWebUI architecture.
"""

from __future__ import annotations

import base64
import io
import logging
import os
import subprocess
import sys
import threading
import time
import webbrowser
import yaml
from pathlib import Path
from typing import Any, Dict, List, Optional, Callable

import cv2

from flask import Flask, jsonify, render_template_string, request
from flask_socketio import SocketIO

# Optional serial deps
try:
    from serial.tools import list_ports
    PYSERIAL_AVAILABLE = True
except Exception:
    PYSERIAL_AVAILABLE = False

# Import web calibration controller
try:
    from rcp_motion.robots.so101.scripts.calibrate_web import (
        WebCalibrationController,
        get_controller,
        reset_controller,
    )
    CALIB_CONTROLLER_AVAILABLE = True
except ImportError:
    CALIB_CONTROLLER_AVAILABLE = False
    WebCalibrationController = None
    get_controller = None
    reset_controller = None

# Logging setup
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    stream=sys.stdout,
)
logger = logging.getLogger(__name__)

# ----------------------------
# Paths (SO101)
# ----------------------------
SCRIPT_DIR = Path(__file__).parent.absolute()
RYNNBOT_CONFIG_PATH = SCRIPT_DIR / "config" / "rynnbot_config.yaml"
SO101_RCP_CONFIG_PATH = SCRIPT_DIR / "config" / "so101_config.yaml"
SO101_LOWLEVEL_CONFIG_PATH = (
    SCRIPT_DIR.parent.parent
    / "rcp_motion"
    / "robots"
    / "so101"
    / "configs"
    / "so101.yaml"
)
SO101_LEADER_LOWLEVEL_CONFIG_PATH = SO101_LOWLEVEL_CONFIG_PATH
SO101_ROBOT_DIR = SCRIPT_DIR.parent.parent / "rcp_motion" / "robots" / "so101"

# Camera output keys
CAM_OUT_KEYS = ["observation.images.front", "observation.images.wrist"]


# ============================================================================ #
# HTML Template
# ============================================================================ #

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SO101 配置向导 - Configuration Wizard</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            color: #e8e8e8;
        }
        .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
        header {
            text-align: center;
            padding: 20px 0;
            border-bottom: 1px solid #333;
            margin-bottom: 20px;
        }
        header h1 {
            font-size: 28px;
            background: linear-gradient(90deg, #00d4ff, #00ff88);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        header p { color: #888; margin-top: 8px; }
        
        /* Navigation */
        .nav-tabs {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }
        .nav-tab {
            padding: 12px 20px;
            background: #2a2a3e;
            border: none;
            border-radius: 8px;
            color: #888;
            cursor: pointer;
            transition: all 0.2s;
            font-size: 14px;
        }
        .nav-tab:hover { background: #3a3a4e; color: #ccc; }
        .nav-tab.active {
            background: linear-gradient(135deg, #00d4ff, #0099cc);
            color: white;
        }
        .nav-tab.completed {
            background: #1e3a2e;
            border: 1px solid #00ff88;
            color: #00ff88;
        }
        
        /* Sections */
        .config-section {
            display: none;
            background: #1e1e2f;
            border-radius: 12px;
            padding: 25px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        .config-section.active { display: block; }
        .section-title {
            font-size: 20px;
            color: #00d4ff;
            margin-bottom: 20px;
            padding-bottom: 10px;
            border-bottom: 1px solid #333;
        }
        
        /* Form styles */
        .form-group {
            margin-bottom: 20px;
        }
        .form-label {
            display: block;
            color: #888;
            font-size: 14px;
            margin-bottom: 8px;
        }
        .form-input {
            width: 100%;
            padding: 12px 15px;
            border-radius: 8px;
            border: 1px solid #444;
            background: #2a2a3e;
            color: #e8e8e8;
            font-size: 14px;
        }
        .form-input:focus {
            outline: none;
            border-color: #00d4ff;
        }
        
        /* Buttons */
        .btn {
            padding: 12px 24px;
            border: none;
            border-radius: 8px;
            font-size: 14px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn:disabled { opacity: 0.5; cursor: not-allowed; }
        .btn-primary { background: linear-gradient(135deg, #00d4ff, #0099cc); color: white; }
        .btn-primary:hover:not(:disabled) { transform: translateY(-2px); box-shadow: 0 4px 15px rgba(0, 212, 255, 0.4); }
        .btn-success { background: linear-gradient(135deg, #00ff88, #00cc6a); color: #1a1a2e; }
        .btn-success:hover:not(:disabled) { transform: translateY(-2px); box-shadow: 0 4px 15px rgba(0, 255, 136, 0.4); }
        .btn-warning { background: linear-gradient(135deg, #ffa500, #cc8400); color: #1a1a2e; }
        .btn-warning:hover:not(:disabled) { transform: translateY(-2px); box-shadow: 0 4px 15px rgba(255, 165, 0, 0.4); }
        .btn-danger { background: linear-gradient(135deg, #ff4757, #cc3a47); color: white; }
        .btn-danger:hover:not(:disabled) { transform: translateY(-2px); box-shadow: 0 4px 15px rgba(255, 71, 87, 0.4); }
        
        .button-group { display: flex; gap: 10px; margin-top: 20px; }
        
        /* Camera grid */
        .camera-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 15px;
            margin-top: 20px;
        }
        .camera-card {
            background: #2a2a3e;
            border-radius: 8px;
            overflow: hidden;
            border: 2px solid transparent;
            transition: border-color 0.2s;
        }
        .camera-card.selected { border-color: #00ff88; }
        .camera-card img {
            width: 100%;
            height: 200px;
            object-fit: cover;
            display: block;
            background: #1a1a2e;
        }
        .camera-info {
            padding: 12px;
        }
        .camera-name { font-weight: 600; color: #e8e8e8; }
        .camera-actions { margin-top: 10px; }
        .camera-btn {
            padding: 8px 16px;
            border: none;
            border-radius: 6px;
            font-size: 12px;
            cursor: pointer;
            margin-right: 8px;
            transition: all 0.2s;
        }
        .camera-btn.front { background: #00d4ff; color: #1a1a2e; }
        .camera-btn.wrist { background: #ffa500; color: #1a1a2e; }
        
        /* Serial ports */
        .serial-list {
            margin-top: 15px;
        }
        .serial-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 12px 15px;
            background: #2a2a3e;
            border-radius: 8px;
            margin-bottom: 10px;
            transition: all 0.2s;
        }
        .serial-item:hover { background: #3a3a4e; }
        .serial-item.new { border-left: 4px solid #00ff88; }
        .serial-info { flex: 1; }
        .serial-device { font-weight: 600; color: #e8e8e8; }
        .serial-desc { font-size: 12px; color: #888; margin-top: 4px; }
        
        /* Log display */
        .log-container {
            background: #0d0d1a;
            border-radius: 8px;
            padding: 15px;
            margin-top: 20px;
            max-height: 400px;
            overflow-y: auto;
            font-family: 'Fira Code', 'Consolas', monospace;
        }
        .log-line {
            font-size: 13px;
            line-height: 1.6;
            color: #00ff88;
            white-space: pre-wrap;
            word-break: break-all;
        }
        .log-line.error { color: #ff4757; }
        .log-line.warning { color: #ffa500; }
        .log-line.info { color: #00d4ff; }
        
        /* Status indicators */
        .status-badge {
            display: inline-block;
            padding: 4px 12px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: 600;
        }
        .status-badge.success { background: #1e3a2e; color: #00ff88; }
        .status-badge.warning { background: #3a3a1e; color: #ffa500; }
        .status-badge.error { background: #3a1e1e; color: #ff4757; }
        .status-badge.info { background: #1e2e3a; color: #00d4ff; }
        
        /* Toast */
        .toast {
            position: fixed;
            bottom: 20px;
            right: 20px;
            padding: 15px 25px;
            border-radius: 8px;
            color: white;
            font-weight: 500;
            transform: translateX(150%);
            transition: transform 0.3s;
            z-index: 1000;
        }
        .toast.show { transform: translateX(0); }
        .toast.success { background: #00cc6a; }
        .toast.error { background: #ff4757; }
        .toast.info { background: #00d4ff; }
        
        /* Mapping display */
        .mapping-display {
            background: #2a2a3e;
            border-radius: 8px;
            padding: 15px;
            margin-top: 15px;
        }
        .mapping-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 0;
            border-bottom: 1px solid #3a3a4e;
        }
        .mapping-item:last-child { border-bottom: none; }
        .mapping-key { color: #00d4ff; font-weight: 600; }
        .mapping-value { color: #00ff88; }
        .mapping-value.unset { color: #ff4757; }
        
        /* Progress bar */
        .progress-container {
            background: #2a2a3e;
            border-radius: 8px;
            padding: 15px;
            margin-top: 15px;
        }
        .progress-bar-wrapper {
            width: 100%;
            height: 8px;
            background: #1e1e2f;
            border-radius: 4px;
            overflow: hidden;
            margin-top: 10px;
        }
        .progress-bar {
            height: 100%;
            background: linear-gradient(90deg, #00d4ff, #00ff88);
            transition: width 0.3s;
        }
        .progress-text {
            font-size: 13px;
            color: #888;
        }
        
        /* Tips */
        .tip-box {
            background: #2a3a4e;
            border-left: 4px solid #00d4ff;
            padding: 15px;
            border-radius: 0 8px 8px 0;
            margin-bottom: 20px;
        }
        .tip-box h4 { color: #00d4ff; margin-bottom: 8px; }
        .tip-box p { color: #aaa; font-size: 14px; line-height: 1.6; }
        
        /* Calibration steps */
        .calib-steps {
            margin-top: 20px;
        }
        .calib-step {
            display: flex;
            align-items: flex-start;
            gap: 15px;
            padding: 15px;
            background: #2a2a3e;
            border-radius: 8px;
            margin-bottom: 10px;
        }
        .calib-step-number {
            width: 32px;
            height: 32px;
            background: #3a3a4e;
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: 600;
            color: #888;
            flex-shrink: 0;
        }
        .calib-step.active .calib-step-number {
            background: #00d4ff;
            color: #1a1a2e;
        }
        .calib-step.completed .calib-step-number {
            background: #00ff88;
            color: #1a1a2e;
        }
        .calib-step-content { flex: 1; }
        .calib-step-title { font-weight: 600; color: #e8e8e8; }
        .calib-step-desc { font-size: 13px; color: #888; margin-top: 4px; }
        
        /* Modal dialog */
        .modal-overlay {
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.7);
            display: none;
            justify-content: center;
            align-items: center;
            z-index: 2000;
        }
        .modal-overlay.show { display: flex; }
        .modal-dialog {
            background: #1e1e2f;
            border-radius: 12px;
            padding: 25px;
            min-width: 350px;
            max-width: 90%;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.5);
            border: 1px solid #3a3a4e;
        }
        .modal-title {
            font-size: 18px;
            color: #00d4ff;
            margin-bottom: 15px;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .modal-desc {
            color: #888;
            font-size: 14px;
            margin-bottom: 20px;
            line-height: 1.5;
        }
        .modal-input {
            width: 100%;
            padding: 12px 15px;
            border-radius: 8px;
            border: 1px solid #444;
            background: #2a2a3e;
            color: #e8e8e8;
            font-size: 14px;
            margin-bottom: 20px;
        }
        .modal-input:focus {
            outline: none;
            border-color: #00d4ff;
        }
        .modal-buttons {
            display: flex;
            justify-content: flex-end;
            gap: 10px;
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>🤖 SO101 配置向导 | Configuration Wizard</h1>
            <p>按步骤完成 SO101 机器人的配置 | Follow the steps to configure your SO101 robot</p>
        </header>
        
        <!-- Navigation Tabs -->
        <div class="nav-tabs">
            <button class="nav-tab active" onclick="showSection('device')" id="tab-device">1. 设备设置</button>
            <button class="nav-tab" onclick="showSection('camera')" id="tab-camera">2. 相机配置</button>
            <button class="nav-tab" onclick="showSection('follower-serial')" id="tab-follower-serial">3. 从臂串口</button>
            <button class="nav-tab" onclick="showSection('follower-calib')" id="tab-follower-calib">4. 从臂标定</button>
            <button class="nav-tab" onclick="showSection('leader-serial')" id="tab-leader-serial">5. 主臂串口</button>
            <button class="nav-tab" onclick="showSection('leader-calib')" id="tab-leader-calib">6. 主臂标定</button>
        </div>
        
        <!-- Section 1: Device Settings -->
        <div class="config-section active" id="section-device">
            <h2 class="section-title">📱 设备设置 (RynnBot) | Device Settings</h2>
            
            <div class="tip-box">
                <h4>💡 提示</h4>
                <p>配置 RynnBot 云平台连接参数，用于远程监控和控制。<br>
                Configure RynnBot cloud platform connection parameters for remote monitoring and control.</p>
            </div>
            
            <div class="form-group">
                <label class="form-label">Product Key</label>
                <input type="text" class="form-input" id="device-product-key" placeholder="请输入 product_key">
            </div>
            <div class="form-group">
                <label class="form-label">Device Name</label>
                <input type="text" class="form-input" id="device-name" placeholder="请输入 device_name">
            </div>
            <div class="form-group">
                <label class="form-label">Device Secret</label>
                <input type="text" class="form-input" id="device-secret" placeholder="请输入 device_secret">
            </div>
            <div class="form-group">
                <label class="form-label">HTTP URL</label>
                <input type="text" class="form-input" id="device-http-url" placeholder="请输入 http_url">
            </div>
            
            <div class="button-group">
                <button class="btn btn-success" onclick="saveDeviceSettings()">💾 保存设置</button>
                <button class="btn btn-primary" onclick="showSection('camera')">下一步 →</button>
            </div>
        </div>
        
        <!-- Section 2: Camera Settings -->
        <div class="config-section" id="section-camera">
            <h2 class="section-title">📷 相机配置 | Camera Configuration</h2>
            
            <div class="tip-box">
                <h4>💡 提示</h4>
                <p>点击相机下方的按钮将其绑定到对应的视角。<br>
                Click the buttons below each camera to bind it to the corresponding view.</p>
            </div>
            
            <div class="button-group" style="margin-bottom: 20px;">
                <button class="btn btn-primary" onclick="scanCameras()">🔍 扫描相机</button>
                <button class="btn btn-warning" onclick="clearCameraMapping()">🗑️ 清除绑定</button>
            </div>
            
            <div class="mapping-display">
                <h4 style="color: #888; margin-bottom: 10px;">当前绑定 | Current Mapping</h4>
                <div class="mapping-item">
                    <span class="mapping-key">observation.images.front</span>
                    <span class="mapping-value unset" id="mapping-front">(未绑定)</span>
                </div>
                <div class="mapping-item">
                    <span class="mapping-key">observation.images.wrist</span>
                    <span class="mapping-value unset" id="mapping-wrist">(未绑定)</span>
                </div>
            </div>
            
            <div class="camera-grid" id="camera-grid">
                <div style="color: #888; text-align: center; padding: 40px;">
                    点击"扫描相机"按钮检测可用相机<br>
                    Click "Scan Cameras" to detect available cameras
                </div>
            </div>
            
            <div class="button-group">
                <button class="btn btn-success" onclick="saveCameraSettings()" id="btn-save-camera" disabled>💾 保存配置</button>
                <button class="btn btn-primary" onclick="showSection('follower-serial')">下一步 →</button>
            </div>
        </div>
        
        <!-- Section 3: Follower Serial -->
        <div class="config-section" id="section-follower-serial">
            <h2 class="section-title">🔌 从臂串口配置 | Follower Arm Serial</h2>
            
            <div class="tip-box">
                <h4>💡 提示</h4>
                <p>1. 先拔掉从臂的串口设备，点击"记录基线"<br>
                2. 再插入从臂的串口设备，点击"检测新增"<br>
                3. 选择检测到的串口设备<br>
                <br>
                1. Unplug the follower arm serial device, click "Record Baseline"<br>
                2. Plug in the device, click "Detect New"<br>
                3. Select the detected serial port</p>
            </div>
            
            <div class="button-group" style="margin-bottom: 20px;">
                <button class="btn btn-warning" onclick="recordSerialBaseline('follower')">📋 记录基线</button>
                <button class="btn btn-primary" onclick="detectNewSerial('follower')">🔍 检测新增</button>
            </div>
            
            <div id="follower-serial-status" style="color: #888; margin-bottom: 15px;">
                当前状态: 等待操作...
            </div>
            
            <div class="serial-list" id="follower-serial-list">
                <div style="color: #888; text-align: center; padding: 20px;">
                    串口列表将在检测后显示<br>
                    Serial ports will be shown after detection
                </div>
            </div>
            
            <div style="margin-top: 20px; padding: 15px; background: #2a2a3e; border-radius: 8px;">
                <span style="color: #888;">已选择的串口:</span>
                <span id="follower-selected-port" style="color: #00ff88; font-weight: 600; margin-left: 10px;">(未选择)</span>
            </div>
            
            <div class="button-group">
                <button class="btn btn-success" onclick="saveFollowerSerial()" id="btn-save-follower-serial" disabled>💾 保存配置</button>
                <button class="btn btn-primary" onclick="showSection('follower-calib')">下一步 →</button>
            </div>
        </div>
        
        <!-- Section 4: Follower Calibration -->
        <div class="config-section" id="section-follower-calib">
            <h2 class="section-title">🎯 从臂标定 | Follower Arm Calibration</h2>
            
            <div class="tip-box">
                <h4>💡 标定流程说明</h4>
                <p>
                <strong>步骤1:</strong> 点击"连接机械臂"按钮<br>
                <strong>步骤2:</strong> 将机械臂各关节移动到中间位置，点击"记录中间位置"<br>
                <strong>步骤3:</strong> 点击"开始记录范围"，然后移动各关节到极限位置<br>
                <strong>步骤4:</strong> 完成后点击"结束并保存"<br>
                </p>
            </div>
            
            <div class="calib-steps">
                <div class="calib-step" id="follower-calib-step-1">
                    <div class="calib-step-number">1</div>
                    <div class="calib-step-content">
                        <div class="calib-step-title">连接机械臂</div>
                        <div class="calib-step-desc">建立与机械臂的通信连接</div>
                    </div>
                </div>
                <div class="calib-step" id="follower-calib-step-2">
                    <div class="calib-step-number">2</div>
                    <div class="calib-step-content">
                        <div class="calib-step-title">记录中间位置</div>
                        <div class="calib-step-desc">将各关节移动到中间位置后点击确认</div>
                    </div>
                </div>
                <div class="calib-step" id="follower-calib-step-3">
                    <div class="calib-step-number">3</div>
                    <div class="calib-step-content">
                        <div class="calib-step-title">记录运动范围</div>
                        <div class="calib-step-desc">移动各关节到极限位置</div>
                    </div>
                </div>
                <div class="calib-step" id="follower-calib-step-4">
                    <div class="calib-step-number">4</div>
                    <div class="calib-step-content">
                        <div class="calib-step-title">保存标定</div>
                        <div class="calib-step-desc">保存标定数据到文件</div>
                    </div>
                </div>
            </div>
            
            <div class="button-group" style="margin-top: 20px;">
                <button class="btn btn-primary" onclick="calibConnect('follower')" id="btn-follower-connect">🔌 连接机械臂</button>
                <button class="btn btn-warning" onclick="calibRecordMiddle('follower')" id="btn-follower-middle" disabled>📍 记录中间位置</button>
                <button class="btn btn-success" onclick="calibStartRange('follower')" id="btn-follower-start-range" disabled>▶️ 开始记录范围</button>
                <button class="btn btn-danger" onclick="calibStopRange('follower')" id="btn-follower-stop-range" disabled>⏹️ 结束并保存</button>
            </div>
            
            <div class="button-group" style="margin-top: 10px;">
                <button class="btn" style="background: #555;" onclick="calibDisconnect('follower')" id="btn-follower-disconnect" disabled>断开连接</button>
                <button class="btn" style="background: #555;" onclick="calibReset('follower')">🔄 重置</button>
            </div>
            
            <!-- Status display -->
            <div id="follower-calib-status" style="margin-top: 20px; padding: 15px; background: #2a2a3e; border-radius: 8px;">
                <span style="color: #888;">当前状态:</span>
                <span id="follower-calib-state" style="color: #00d4ff; font-weight: 600; margin-left: 10px;">等待连接</span>
            </div>
            
            <!-- Motor positions display -->
            <div id="follower-motor-positions" style="display: none; margin-top: 15px; padding: 15px; background: #2a2a3e; border-radius: 8px;">
                <h4 style="color: #888; margin-bottom: 10px;">📊 关节位置 (实时)</h4>
                <div style="font-family: 'Fira Code', monospace; font-size: 12px;">
                    <div style="display: grid; grid-template-columns: 120px 80px 80px 80px; gap: 5px; color: #888; margin-bottom: 5px;">
                        <span>关节名称</span><span>最小值</span><span>当前值</span><span>最大值</span>
                    </div>
                    <div id="follower-positions-grid"></div>
                </div>
            </div>
            
            <div class="log-container" id="follower-calib-log" style="margin-top: 15px;">
            </div>
            
            <div class="button-group">
                <button class="btn btn-primary" onclick="showSection('leader-serial')">下一步 →</button>
            </div>
        </div>
        
        <!-- Section 5: Leader Serial -->
        <div class="config-section" id="section-leader-serial">
            <h2 class="section-title">🔌 主臂串口配置 | Leader Arm Serial</h2>
            
            <div class="tip-box">
                <h4>💡 提示</h4>
                <p>1. 先拔掉主臂的串口设备，点击"记录基线"<br>
                2. 再插入主臂的串口设备，点击"检测新增"<br>
                3. 选择检测到的串口设备<br>
                <br>
                1. Unplug the leader arm serial device, click "Record Baseline"<br>
                2. Plug in the device, click "Detect New"<br>
                3. Select the detected serial port</p>
            </div>
            
            <div class="button-group" style="margin-bottom: 20px;">
                <button class="btn btn-warning" onclick="recordSerialBaseline('leader')">📋 记录基线</button>
                <button class="btn btn-primary" onclick="detectNewSerial('leader')">🔍 检测新增</button>
            </div>
            
            <div id="leader-serial-status" style="color: #888; margin-bottom: 15px;">
                当前状态: 等待操作...
            </div>
            
            <div class="serial-list" id="leader-serial-list">
                <div style="color: #888; text-align: center; padding: 20px;">
                    串口列表将在检测后显示<br>
                    Serial ports will be shown after detection
                </div>
            </div>
            
            <div style="margin-top: 20px; padding: 15px; background: #2a2a3e; border-radius: 8px;">
                <span style="color: #888;">已选择的串口:</span>
                <span id="leader-selected-port" style="color: #00ff88; font-weight: 600; margin-left: 10px;">(未选择)</span>
            </div>
            
            <div class="button-group">
                <button class="btn btn-success" onclick="saveLeaderSerial()" id="btn-save-leader-serial" disabled>💾 保存配置</button>
                <button class="btn btn-primary" onclick="showSection('leader-calib')">下一步 →</button>
            </div>
        </div>
        
        <!-- Section 6: Leader Calibration -->
        <div class="config-section" id="section-leader-calib">
            <h2 class="section-title">🎯 主臂标定 | Leader Arm Calibration</h2>
            
            <div class="tip-box">
                <h4>💡 标定流程说明</h4>
                <p>
                <strong>步骤1:</strong> 点击"连接机械臂"按钮<br>
                <strong>步骤2:</strong> 将机械臂各关节移动到中间位置，点击"记录中间位置"<br>
                <strong>步骤3:</strong> 点击"开始记录范围"，然后移动各关节到极限位置<br>
                <strong>步骤4:</strong> 完成后点击"结束并保存"<br>
                </p>
            </div>
            
            <div class="calib-steps">
                <div class="calib-step" id="leader-calib-step-1">
                    <div class="calib-step-number">1</div>
                    <div class="calib-step-content">
                        <div class="calib-step-title">连接机械臂</div>
                        <div class="calib-step-desc">建立与机械臂的通信连接</div>
                    </div>
                </div>
                <div class="calib-step" id="leader-calib-step-2">
                    <div class="calib-step-number">2</div>
                    <div class="calib-step-content">
                        <div class="calib-step-title">记录中间位置</div>
                        <div class="calib-step-desc">将各关节移动到中间位置后点击确认</div>
                    </div>
                </div>
                <div class="calib-step" id="leader-calib-step-3">
                    <div class="calib-step-number">3</div>
                    <div class="calib-step-content">
                        <div class="calib-step-title">记录运动范围</div>
                        <div class="calib-step-desc">移动各关节到极限位置</div>
                    </div>
                </div>
                <div class="calib-step" id="leader-calib-step-4">
                    <div class="calib-step-number">4</div>
                    <div class="calib-step-content">
                        <div class="calib-step-title">保存标定</div>
                        <div class="calib-step-desc">保存标定数据到文件</div>
                    </div>
                </div>
            </div>
            
            <div class="button-group" style="margin-top: 20px;">
                <button class="btn btn-primary" onclick="calibConnect('leader')" id="btn-leader-connect">🔌 连接机械臂</button>
                <button class="btn btn-warning" onclick="calibRecordMiddle('leader')" id="btn-leader-middle" disabled>📍 记录中间位置</button>
                <button class="btn btn-success" onclick="calibStartRange('leader')" id="btn-leader-start-range" disabled>▶️ 开始记录范围</button>
                <button class="btn btn-danger" onclick="calibStopRange('leader')" id="btn-leader-stop-range" disabled>⏹️ 结束并保存</button>
            </div>
            
            <div class="button-group" style="margin-top: 10px;">
                <button class="btn" style="background: #555;" onclick="calibDisconnect('leader')" id="btn-leader-disconnect" disabled>断开连接</button>
                <button class="btn" style="background: #555;" onclick="calibReset('leader')">🔄 重置</button>
            </div>
            
            <!-- Status display -->
            <div id="leader-calib-status" style="margin-top: 20px; padding: 15px; background: #2a2a3e; border-radius: 8px;">
                <span style="color: #888;">当前状态:</span>
                <span id="leader-calib-state" style="color: #00d4ff; font-weight: 600; margin-left: 10px;">等待连接</span>
            </div>
            
            <!-- Motor positions display -->
            <div id="leader-motor-positions" style="display: none; margin-top: 15px; padding: 15px; background: #2a2a3e; border-radius: 8px;">
                <h4 style="color: #888; margin-bottom: 10px;">📊 关节位置 (实时)</h4>
                <div style="font-family: 'Fira Code', monospace; font-size: 12px;">
                    <div style="display: grid; grid-template-columns: 120px 80px 80px 80px; gap: 5px; color: #888; margin-bottom: 5px;">
                        <span>关节名称</span><span>最小值</span><span>当前值</span><span>最大值</span>
                    </div>
                    <div id="leader-positions-grid"></div>
                </div>
            </div>
            
            <div class="log-container" id="leader-calib-log" style="margin-top: 15px;">
            </div>
            
            <div class="button-group">
                <button class="btn btn-success" onclick="finishConfig()">✅ 完成配置</button>
            </div>
        </div>
    </div>
    
    <!-- Sudo Password Modal -->
    <div class="modal-overlay" id="sudo-modal">
        <div class="modal-dialog">
            <div class="modal-title">🔐 需要 Sudo 权限</div>
            <div class="modal-desc">
                设置串口权限需要管理员权限，请输入您的 sudo 密码：<br>
                <small style="color: #666;">密码仅在本次操作中使用，不会被保存。</small>
            </div>
            <input type="password" class="modal-input" id="sudo-password" 
                   placeholder="请输入 sudo 密码" 
                   onkeypress="if(event.key==='Enter') confirmSudoPassword();">
            <div class="modal-buttons">
                <button class="btn btn-primary" onclick="closeSudoModal()" style="background: #555;">取消</button>
                <button class="btn btn-success" onclick="confirmSudoPassword()">确认</button>
            </div>
        </div>
    </div>
    
    <div id="toast" class="toast"></div>
    
    <script>
        const socket = io();
        
        // State
        let cameraMapping = { front: null, wrist: null };
        let followerSerialBaseline = [];
        let leaderSerialBaseline = [];
        let selectedFollowerPort = null;
        let selectedLeaderPort = null;
        
        // ================== Navigation ==================
        function showSection(name) {
            document.querySelectorAll('.config-section').forEach(s => s.classList.remove('active'));
            document.querySelectorAll('.nav-tab').forEach(t => t.classList.remove('active'));
            document.getElementById('section-' + name).classList.add('active');
            document.getElementById('tab-' + name).classList.add('active');
        }
        
        function markTabCompleted(name) {
            document.getElementById('tab-' + name).classList.add('completed');
        }
        
        // ================== Toast ==================
        function showToast(msg, type='info', duration=3000) {
            const t = document.getElementById('toast');
            t.textContent = msg;
            t.className = 'toast ' + type + ' show';
            setTimeout(() => t.classList.remove('show'), duration);
        }
        
        // ================== Sudo Modal ==================
        let _sudoCallback = null;
        
        function showSudoModal(callback) {
            _sudoCallback = callback;
            document.getElementById('sudo-password').value = '';
            document.getElementById('sudo-modal').classList.add('show');
            setTimeout(() => document.getElementById('sudo-password').focus(), 100);
        }
        
        function closeSudoModal() {
            document.getElementById('sudo-modal').classList.remove('show');
            _sudoCallback = null;
        }
        
        function confirmSudoPassword() {
            const password = document.getElementById('sudo-password').value;
            if (!password) {
                showToast('❌ 请输入密码', 'error');
                return;
            }
            // 先保存 callback，再关闭弹窗
            const callback = _sudoCallback;
            closeSudoModal();
            if (callback) {
                callback(password);
            }
        }
        
        // ================== Device Settings ==================
        // Load device settings on page load
        fetch('/api/device/load').then(r => r.json()).then(d => {
            if (d.success) {
                document.getElementById('device-product-key').value = d.data.product_key || '';
                document.getElementById('device-name').value = d.data.device_name || '';
                document.getElementById('device-secret').value = d.data.device_secret || '';
                document.getElementById('device-http-url').value = d.data.http_url || '';
            }
        });
        
        function saveDeviceSettings() {
            const data = {
                product_key: document.getElementById('device-product-key').value.trim(),
                device_name: document.getElementById('device-name').value.trim(),
                device_secret: document.getElementById('device-secret').value.trim(),
                http_url: document.getElementById('device-http-url').value.trim()
            };
            fetch('/api/device/save', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(data)
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    showToast('✅ 设备设置已保存', 'success');
                    markTabCompleted('device');
                } else {
                    showToast('❌ 保存失败: ' + d.message, 'error');
                }
            });
        }
        
        // ================== Camera Settings ==================
        function scanCameras() {
            showToast('🔍 正在扫描相机...', 'info');
            fetch('/api/camera/scan').then(r => r.json()).then(d => {
                if (d.success) {
                    renderCameras(d.cameras);
                    showToast('✅ 找到 ' + d.cameras.length + ' 个相机', 'success');
                } else {
                    showToast('❌ 扫描失败: ' + d.message, 'error');
                }
            });
        }
        
        function renderCameras(cameras) {
            const grid = document.getElementById('camera-grid');
            if (cameras.length === 0) {
                grid.innerHTML = '<div style="color: #888; text-align: center; padding: 40px;">未检测到相机</div>';
                return;
            }
            grid.innerHTML = '';
            cameras.forEach(cam => {
                const card = document.createElement('div');
                card.className = 'camera-card';
                card.id = 'cam-card-' + cam.index;
                card.innerHTML = `
                    <img id="cam-img-${cam.index}" src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==" alt="Camera ${cam.index}">
                    <div class="camera-info">
                        <div class="camera-name">Camera ${cam.index}</div>
                        <div class="camera-actions">
                            <button class="camera-btn front" onclick="bindCamera(${cam.index}, 'front')">📷 绑定前置</button>
                            <button class="camera-btn wrist" onclick="bindCamera(${cam.index}, 'wrist')">📷 绑定腕部</button>
                        </div>
                    </div>
                `;
                grid.appendChild(card);
            });
        }
        
        function bindCamera(index, type) {
            cameraMapping[type] = index;
            updateCameraMappingDisplay();
            
            // Update card highlight
            document.querySelectorAll('.camera-card').forEach(c => c.classList.remove('selected'));
            if (cameraMapping.front !== null) {
                const fc = document.getElementById('cam-card-' + cameraMapping.front);
                if (fc) fc.classList.add('selected');
            }
            if (cameraMapping.wrist !== null) {
                const wc = document.getElementById('cam-card-' + cameraMapping.wrist);
                if (wc) wc.classList.add('selected');
            }
            
            showToast('✅ 已绑定 Camera ' + index + ' -> ' + type, 'success');
        }
        
        function updateCameraMappingDisplay() {
            const frontEl = document.getElementById('mapping-front');
            const wristEl = document.getElementById('mapping-wrist');
            
            if (cameraMapping.front !== null) {
                frontEl.textContent = 'Camera ' + cameraMapping.front;
                frontEl.classList.remove('unset');
            } else {
                frontEl.textContent = '(未绑定)';
                frontEl.classList.add('unset');
            }
            
            if (cameraMapping.wrist !== null) {
                wristEl.textContent = 'Camera ' + cameraMapping.wrist;
                wristEl.classList.remove('unset');
            } else {
                wristEl.textContent = '(未绑定)';
                wristEl.classList.add('unset');
            }
            
            // Enable save button if both are mapped
            document.getElementById('btn-save-camera').disabled = 
                (cameraMapping.front === null || cameraMapping.wrist === null);
        }
        
        function clearCameraMapping() {
            cameraMapping = { front: null, wrist: null };
            updateCameraMappingDisplay();
            document.querySelectorAll('.camera-card').forEach(c => c.classList.remove('selected'));
            showToast('🗑️ 已清除绑定', 'info');
        }
        
        function saveCameraSettings() {
            if (cameraMapping.front === null || cameraMapping.wrist === null) {
                showToast('❌ 请先绑定两个相机', 'error');
                return;
            }
            fetch('/api/camera/save', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    front: cameraMapping.front,
                    wrist: cameraMapping.wrist
                })
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    showToast('✅ 相机配置已保存', 'success');
                    markTabCompleted('camera');
                } else {
                    showToast('❌ 保存失败: ' + d.message, 'error');
                }
            });
        }
        
        // Camera image updates via WebSocket
        socket.on('camera_frame', d => {
            const img = document.getElementById('cam-img-' + d.index);
            if (img) {
                img.src = 'data:image/jpeg;base64,' + d.frame;
            }
        });
        
        // ================== Serial Port Detection ==================
        function recordSerialBaseline(arm) {
            fetch('/api/serial/scan').then(r => r.json()).then(d => {
                if (d.success) {
                    if (arm === 'follower') {
                        followerSerialBaseline = d.ports.map(p => p.device);
                        document.getElementById('follower-serial-status').innerHTML = 
                            '<span class="status-badge info">基线已记录，检测到 ' + d.ports.length + ' 个端口</span>';
                    } else {
                        leaderSerialBaseline = d.ports.map(p => p.device);
                        document.getElementById('leader-serial-status').innerHTML = 
                            '<span class="status-badge info">基线已记录，检测到 ' + d.ports.length + ' 个端口</span>';
                    }
                    showToast('✅ 基线已记录，请插入设备后点击"检测新增"', 'success');
                }
            });
        }
        
        function detectNewSerial(arm) {
            fetch('/api/serial/scan').then(r => r.json()).then(d => {
                if (d.success) {
                    const baseline = arm === 'follower' ? followerSerialBaseline : leaderSerialBaseline;
                    const currentPorts = d.ports;
                    const newPorts = currentPorts.filter(p => !baseline.includes(p.device));
                    console.log('[DEBUG] detectNewSerial:', arm, 'baseline:', baseline.length, 'current:', currentPorts.length, 'new:', newPorts.length);
                    
                    renderSerialList(arm, currentPorts, newPorts);
                    
                    const statusEl = document.getElementById(arm + '-serial-status');
                    if (newPorts.length > 0) {
                        statusEl.innerHTML = '<span class="status-badge success">检测到 ' + newPorts.length + ' 个新端口</span>';
                        if (newPorts.length === 1) {
                            selectSerialPort(arm, newPorts[0].device);
                        }
                    } else {
                        statusEl.innerHTML = '<span class="status-badge warning">未检测到新端口，请确认设备已插入</span>';
                    }
                }
            });
        }
        
        function renderSerialList(arm, allPorts, newPorts) {
            const list = document.getElementById(arm + '-serial-list');
            if (allPorts.length === 0) {
                list.innerHTML = '<div style="color: #888; text-align: center; padding: 20px;">未检测到任何串口</div>';
                return;
            }
            
            const newDevices = newPorts.map(p => p.device);
            list.innerHTML = '';
            allPorts.forEach(port => {
                const isNew = newDevices.includes(port.device);
                const item = document.createElement('div');
                item.className = 'serial-item' + (isNew ? ' new' : '');
                item.innerHTML = `
                    <div class="serial-info">
                        <div class="serial-device">${port.device}</div>
                        <div class="serial-desc">${port.description || ''}</div>
                    </div>
                    <button class="btn btn-primary" style="padding: 8px 16px; font-size: 12px;" 
                            onclick="selectSerialPort('${arm}', '${port.device}')">
                        选择
                    </button>
                `;
                list.appendChild(item);
            });
        }
        
        function selectSerialPort(arm, device) {
            console.log('[DEBUG] selectSerialPort called:', arm, device);
            if (arm === 'follower') {
                selectedFollowerPort = device;
                document.getElementById('follower-selected-port').textContent = device;
                document.getElementById('btn-save-follower-serial').disabled = false;
            } else {
                selectedLeaderPort = device;
                document.getElementById('leader-selected-port').textContent = device;
                document.getElementById('btn-save-leader-serial').disabled = false;
            }
            showToast('✅ 已选择: ' + device, 'success');
        }
        
        function saveFollowerSerial() {
            console.log('[DEBUG] saveFollowerSerial called, selectedFollowerPort:', selectedFollowerPort);
            if (!selectedFollowerPort) {
                showToast('❌ 请先选择串口', 'error');
                return;
            }
            console.log('[DEBUG] Showing sudo modal...');
            // Show sudo modal first, then save with password
            showSudoModal((password) => {
                console.log('[DEBUG] Sudo modal callback, sending request...');
                fetch('/api/serial/save/follower', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ port: selectedFollowerPort, sudo_password: password })
                }).then(r => r.json()).then(d => {
                    if (d.success) {
                        showToast('✅ ' + (d.message || '从臂串口已保存'), 'success');
                        markTabCompleted('follower-serial');
                    } else {
                        showToast('❌ 保存失败: ' + d.message, 'error');
                    }
                });
            });
        }
        
        function saveLeaderSerial() {
            if (!selectedLeaderPort) {
                showToast('❌ 请先选择串口', 'error');
                return;
            }
            // Show sudo modal first, then save with password
            showSudoModal((password) => {
                fetch('/api/serial/save/leader', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ port: selectedLeaderPort, sudo_password: password })
                }).then(r => r.json()).then(d => {
                    if (d.success) {
                        showToast('✅ ' + (d.message || '主臂串口已保存'), 'success');
                        markTabCompleted('leader-serial');
                    } else {
                        showToast('❌ 保存失败: ' + d.message, 'error');
                    }
                });
            });
        }
        
        // Auto-refresh serial ports
        setInterval(() => {
            // Only refresh if the serial sections are active
            const followerActive = document.getElementById('section-follower-serial').classList.contains('active');
            const leaderActive = document.getElementById('section-leader-serial').classList.contains('active');
            if (followerActive || leaderActive) {
                // Just update the status indicator that auto-refresh is working
            }
        }, 1000);
        
        // ================== Calibration (Step-by-Step) ==================
        let calibState = { follower: 'idle', leader: 'idle' };
        
        function updateCalibUI(arm, state, message) {
            calibState[arm] = state;
            const stateEl = document.getElementById(arm + '-calib-state');
            const logContainer = document.getElementById(arm + '-calib-log');
            
            // Update state display
            const stateMap = {
                'idle': '等待连接',
                'connected': '已连接 - 请移动到中间位置',
                'middle_recorded': '中间位置已记录',
                'recording_range': '正在记录运动范围...',
                'done': '标定完成',
                'error': '错误'
            };
            stateEl.textContent = stateMap[state] || state;
            stateEl.style.color = state === 'error' ? '#ff4757' : (state === 'done' ? '#00ff88' : '#00d4ff');
            
            // Update buttons
            document.getElementById('btn-' + arm + '-connect').disabled = (state !== 'idle');
            document.getElementById('btn-' + arm + '-middle').disabled = (state !== 'connected');
            document.getElementById('btn-' + arm + '-start-range').disabled = (state !== 'middle_recorded');
            document.getElementById('btn-' + arm + '-stop-range').disabled = (state !== 'recording_range');
            document.getElementById('btn-' + arm + '-disconnect').disabled = (state === 'idle');
            
            // Update step indicators
            for (let i = 1; i <= 4; i++) {
                const step = document.getElementById(arm + '-calib-step-' + i);
                step.classList.remove('active', 'completed');
            }
            if (state === 'connected') {
                document.getElementById(arm + '-calib-step-1').classList.add('completed');
                document.getElementById(arm + '-calib-step-2').classList.add('active');
            } else if (state === 'middle_recorded') {
                document.getElementById(arm + '-calib-step-1').classList.add('completed');
                document.getElementById(arm + '-calib-step-2').classList.add('completed');
                document.getElementById(arm + '-calib-step-3').classList.add('active');
            } else if (state === 'recording_range') {
                document.getElementById(arm + '-calib-step-1').classList.add('completed');
                document.getElementById(arm + '-calib-step-2').classList.add('completed');
                document.getElementById(arm + '-calib-step-3').classList.add('active');
            } else if (state === 'done') {
                for (let i = 1; i <= 4; i++) {
                    document.getElementById(arm + '-calib-step-' + i).classList.add('completed');
                }
            }
            
            // Log message
            if (message) {
                addCalibLog(arm, message);
            }
        }
        
        function addCalibLog(arm, message, level = '') {
            const logContainer = document.getElementById(arm + '-calib-log');
            const line = document.createElement('div');
            line.className = 'log-line' + (level ? ' ' + level : '');
            line.textContent = '[' + new Date().toLocaleTimeString() + '] ' + message;
            logContainer.appendChild(line);
            logContainer.scrollTop = logContainer.scrollHeight;
        }
        
        function calibConnect(arm) {
            addCalibLog(arm, '正在连接机械臂...', 'info');
            fetch('/api/calibration/connect', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ arm: arm })
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    updateCalibUI(arm, 'connected', '✅ ' + d.message);
                    document.getElementById(arm + '-motor-positions').style.display = 'block';
                    showToast('✅ 连接成功', 'success');
                } else {
                    updateCalibUI(arm, 'error', '❌ ' + d.message);
                    showToast('❌ 连接失败: ' + d.message, 'error');
                }
            }).catch(e => {
                updateCalibUI(arm, 'error', '❌ 连接异常: ' + e);
                showToast('❌ 连接异常', 'error');
            });
        }
        
        function calibRecordMiddle(arm) {
            addCalibLog(arm, '正在记录中间位置...', 'info');
            fetch('/api/calibration/record_middle', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ arm: arm })
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    updateCalibUI(arm, 'middle_recorded', '✅ ' + d.message);
                    showToast('✅ 中间位置已记录', 'success');
                } else {
                    addCalibLog(arm, '❌ ' + d.message, 'error');
                    showToast('❌ ' + d.message, 'error');
                }
            });
        }
        
        function calibStartRange(arm) {
            addCalibLog(arm, '开始记录运动范围...', 'info');
            fetch('/api/calibration/start_range', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ arm: arm })
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    updateCalibUI(arm, 'recording_range', '✅ ' + d.message);
                    showToast('▶️ 开始记录，请移动各关节到极限位置', 'info');
                } else {
                    addCalibLog(arm, '❌ ' + d.message, 'error');
                    showToast('❌ ' + d.message, 'error');
                }
            });
        }
        
        function calibStopRange(arm) {
            addCalibLog(arm, '停止记录，正在保存标定数据...', 'info');
            fetch('/api/calibration/stop_range', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ arm: arm })
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    updateCalibUI(arm, 'done', '✅ ' + d.message);
                    showToast('🎉 标定完成！', 'success');
                    markTabCompleted(arm + '-calib');
                    if (d.calibration_path) {
                        addCalibLog(arm, '标定文件: ' + d.calibration_path, 'info');
                    }
                } else {
                    addCalibLog(arm, '❌ ' + d.message, 'error');
                    showToast('❌ 保存失败: ' + d.message, 'error');
                }
            });
        }
        
        function calibDisconnect(arm) {
            fetch('/api/calibration/disconnect', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ arm: arm })
            }).then(r => r.json()).then(d => {
                updateCalibUI(arm, 'idle', d.message);
                document.getElementById(arm + '-motor-positions').style.display = 'none';
                showToast('✅ 已断开连接', 'info');
            });
        }
        
        function calibReset(arm) {
            fetch('/api/calibration/reset', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ arm: arm })
            }).then(r => r.json()).then(d => {
                updateCalibUI(arm, 'idle', '已重置');
                document.getElementById(arm + '-motor-positions').style.display = 'none';
                document.getElementById(arm + '-calib-log').innerHTML = '';
                showToast('🔄 已重置', 'info');
            });
        }
        
        // Calibration position updates via WebSocket
        socket.on('calib_positions', d => {
            const grid = document.getElementById(d.arm + '-positions-grid');
            if (!grid) return;
            
            let html = '';
            const motors = Object.keys(d.current_positions || {}).sort();
            motors.forEach(motor => {
                const cur = d.current_positions[motor] || 0;
                const min = (d.mins && d.mins[motor]) || cur;
                const max = (d.maxes && d.maxes[motor]) || cur;
                html += `<div style="display: grid; grid-template-columns: 120px 80px 80px 80px; gap: 5px; color: #e8e8e8;">
                    <span style="color: #00d4ff;">${motor}</span>
                    <span style="color: #ffa500;">${min}</span>
                    <span>${cur}</span>
                    <span style="color: #00ff88;">${max}</span>
                </div>`;
            });
            grid.innerHTML = html;
        });
        
        socket.on('calib_log', d => {
            addCalibLog(d.arm, d.message, d.level || '');
        });
        
        // Initial state
        updateCameraMappingDisplay();
        
        // ================== Finish Config ==================
        function finishConfig() {
            showToast('🎉 配置完成，正在关闭程序...', 'success', 3000);
            fetch('/api/shutdown', { method: 'POST' })
                .then(() => { setTimeout(() => { window.close(); }, 1000); })
                .catch(() => {});
        }
    </script>
</body>
</html>
'''


# ============================================================================ #
# YAML Helpers
# ============================================================================ #

def load_yaml_config(filepath: Path) -> Dict[str, Any]:
    """Load a YAML configuration file."""
    if not filepath.exists():
        logger.warning(f"Config file not found: {filepath}")
        return {}
    try:
        with filepath.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        logger.error(f"Failed to load YAML {filepath}: {e}")
        return {}


def save_yaml_config(filepath: Path, config: Dict[str, Any]) -> None:
    """Save a YAML configuration file."""
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with filepath.open("w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    logger.info(f"Saved config: {filepath}")


# ============================================================================ #
# Camera Utilities
# ============================================================================ #

def probe_camera_indices(max_probe: int = 8) -> List[int]:
    """Probe available camera indices."""
    ok: List[int] = []
    for i in range(max_probe):
        cap = None
        try:
            if sys.platform.startswith("linux"):
                cap = cv2.VideoCapture(f"/dev/video{i}", cv2.CAP_V4L2)
            else:
                cap = cv2.VideoCapture(i)
            
            if cap is None or not cap.isOpened():
                continue
            
            # Try to read a frame
            good = False
            for _ in range(3):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    good = True
                    break
                time.sleep(0.03)
            
            if good:
                ok.append(i)
        except Exception:
            pass
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
    return ok


def capture_camera_frame(index: int) -> Optional[bytes]:
    """Capture a single frame from a camera and return as JPEG bytes."""
    cap = None
    try:
        if sys.platform.startswith("linux"):
            cap = cv2.VideoCapture(f"/dev/video{index}", cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(index)
        
        if cap is None or not cap.isOpened():
            return None
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        for _ in range(3):
            ret, frame = cap.read()
            if ret and frame is not None:
                _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                return jpg.tobytes()
            time.sleep(0.03)
        return None
    except Exception as e:
        logger.error(f"Error capturing from camera {index}: {e}")
        return None
    finally:
        if cap:
            cap.release()


# ============================================================================ #
# Serial Port Utilities
# ============================================================================ #

def scan_serial_ports() -> List[Dict[str, str]]:
    """Scan for available serial ports."""
    if not PYSERIAL_AVAILABLE:
        # Fallback to glob on Linux
        if sys.platform.startswith("linux"):
            devs = []
            devs += [str(p) for p in Path("/dev").glob("ttyACM*")]
            devs += [str(p) for p in Path("/dev").glob("ttyUSB*")]
            return [{"device": d, "description": "", "hwid": ""} for d in sorted(set(devs))]
        return []
    
    ports = []
    for p in list_ports.comports():
        ports.append({
            "device": p.device,
            "description": getattr(p, "description", "") or "",
            "hwid": getattr(p, "hwid", "") or "",
        })
    return sorted(ports, key=lambda x: x["device"])


# ============================================================================ #
# Web Server
# ============================================================================ #

class SO101ConfigWebUI:
    """Web UI server for SO101 configuration."""
    
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 5001,
        open_browser: bool = True,
    ) -> None:
        self._host = host
        self._port = port
        self._open_browser = open_browser
        self._stop_event = threading.Event()
        self._server_thread: Optional[threading.Thread] = None
        self._camera_thread: Optional[threading.Thread] = None
        
        # Camera streaming state
        self._camera_indices: List[int] = []
        self._streaming_cameras = False
        
        # Calibration state
        self._calib_process: Optional[subprocess.Popen] = None
        
        # Create Flask app
        self._app = Flask(__name__)
        self._app.config["SECRET_KEY"] = "so101-config-secret"
        self._socketio = SocketIO(
            self._app, cors_allowed_origins="*", async_mode="threading"
        )
        
        # Register routes
        self._register_routes()
    
    def _register_routes(self) -> None:
        """Register Flask routes."""
        
        @self._app.route("/")
        def index():
            return render_template_string(HTML_TEMPLATE)
        
        # ==================== Device Settings ====================
        @self._app.route("/api/device/load", methods=["GET"])
        def api_device_load():
            cfg = load_yaml_config(RYNNBOT_CONFIG_PATH)
            rb = cfg.get("rynnbot", {})
            return jsonify({
                "success": True,
                "data": {
                    "product_key": rb.get("product_key", ""),
                    "device_name": rb.get("device_name", ""),
                    "device_secret": rb.get("device_secret", ""),
                    "http_url": rb.get("http_url", ""),
                }
            })
        
        @self._app.route("/api/device/save", methods=["POST"])
        def api_device_save():
            try:
                data = request.get_json(silent=True) or {}
                cfg = load_yaml_config(RYNNBOT_CONFIG_PATH)
                
                if "rynnbot" not in cfg:
                    cfg["rynnbot"] = {}
                
                cfg["rynnbot"]["product_key"] = data.get("product_key", "")
                cfg["rynnbot"]["device_name"] = data.get("device_name", "")
                cfg["rynnbot"]["device_secret"] = data.get("device_secret", "")
                cfg["rynnbot"]["http_url"] = data.get("http_url", "")
                
                save_yaml_config(RYNNBOT_CONFIG_PATH, cfg)
                return jsonify({"success": True})
            except Exception as e:
                logger.error(f"Error saving device settings: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        # ==================== Camera Settings ====================
        @self._app.route("/api/camera/scan", methods=["GET"])
        def api_camera_scan():
            try:
                indices = probe_camera_indices(8)
                self._camera_indices = indices
                
                # Start camera streaming thread
                if not self._streaming_cameras and indices:
                    self._streaming_cameras = True
                    self._camera_thread = threading.Thread(
                        target=self._camera_stream_loop, daemon=True
                    )
                    self._camera_thread.start()
                
                return jsonify({
                    "success": True,
                    "cameras": [{"index": i} for i in indices]
                })
            except Exception as e:
                logger.error(f"Error scanning cameras: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        @self._app.route("/api/camera/save", methods=["POST"])
        def api_camera_save():
            try:
                data = request.get_json(silent=True) or {}
                front_idx = data.get("front")
                wrist_idx = data.get("wrist")
                
                if front_idx is None or wrist_idx is None:
                    return jsonify({"success": False, "message": "Both cameras must be mapped"})
                
                cfg = load_yaml_config(SO101_RCP_CONFIG_PATH)
                
                # Update camera device_id in config
                self._update_camera_device_id(cfg, "observation.images.front", str(front_idx))
                self._update_camera_device_id(cfg, "observation.images.wrist", str(wrist_idx))
                
                save_yaml_config(SO101_RCP_CONFIG_PATH, cfg)
                
                # Stop camera streaming
                self._streaming_cameras = False
                
                return jsonify({"success": True})
            except Exception as e:
                logger.error(f"Error saving camera settings: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        # ==================== Serial Port Settings ====================
        @self._app.route("/api/serial/scan", methods=["GET"])
        def api_serial_scan():
            try:
                ports = scan_serial_ports()
                logger.info(f"[串口扫描] 检测到 {len(ports)} 个端口: {[p['device'] for p in ports]}")
                return jsonify({"success": True, "ports": ports})
            except Exception as e:
                logger.error(f"Error scanning serial ports: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        @self._app.route("/api/serial/save/follower", methods=["POST"])
        def api_serial_save_follower():
            logger.info("[从臂串口] 收到保存请求")
            try:
                data = request.get_json(silent=True) or {}
                port = data.get("port")
                sudo_password = data.get("sudo_password", "")
                
                if not port:
                    return jsonify({"success": False, "message": "Port is required"})
                
                logger.info(f"[从臂串口] 保存配置: {port}")
                
                cfg = load_yaml_config(SO101_LOWLEVEL_CONFIG_PATH)
                
                if "robot" not in cfg:
                    cfg["robot"] = {}
                cfg["robot"]["port"] = port
                
                save_yaml_config(SO101_LOWLEVEL_CONFIG_PATH, cfg)
                
                permission_set = False
                
                # Set permissions on Linux using sudo with password
                if sys.platform.startswith("linux"):
                    
                    print("[从臂串口] 正在设置权限: sudo chmod 666", port)
                    
                    try:
                        logger.info(f"[从臂串口] 正在设置权限: sudo chmod 666 {port}")
                        # Use echo password | sudo -S to provide password
                        chmod_cmd = ["sudo", "-S", "chmod", "666", port]
                        result = subprocess.run(
                            chmod_cmd,
                            input=sudo_password + "\n",
                            capture_output=True,
                            text=True,
                            timeout=10
                        )
                        if result.returncode != 0:
                            error_msg = result.stderr.strip() or "权限设置失败，请检查密码是否正确"
                            logger.warning(f"[从臂串口] 权限设置失败: {error_msg}")
                            return jsonify({"success": False, "message": error_msg})
                        permission_set = True
                        logger.info(f"[从臂串口] ✅ 权限设置成功")
                    except subprocess.TimeoutExpired:
                        logger.error("[从臂串口] 权限设置超时")
                        return jsonify({"success": False, "message": "权限设置超时"})
                    except Exception as e:
                        logger.error(f"[从臂串口] 权限设置异常: {e}")
                        return jsonify({"success": False, "message": str(e)})
                
                logger.info(f"[从臂串口] ✅ 配置保存成功" + ("，权限已设置" if permission_set else ""))
                return jsonify({
                    "success": True, 
                    "permission_set": permission_set,
                    "message": "配置已保存" + ("，串口权限已设置" if permission_set else "")
                })
            except Exception as e:
                logger.error(f"[从臂串口] 保存失败: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        @self._app.route("/api/serial/save/leader", methods=["POST"])
        def api_serial_save_leader():
            logger.info("[主臂串口] 收到保存请求")
            try:
                data = request.get_json(silent=True) or {}
                port = data.get("port")
                sudo_password = data.get("sudo_password", "")
                
                if not port:
                    return jsonify({"success": False, "message": "Port is required"})
                
                logger.info(f"[主臂串口] 保存配置: {port}")
                
                cfg = load_yaml_config(SO101_LEADER_LOWLEVEL_CONFIG_PATH)
                
                if "teleoperate" not in cfg:
                    cfg["teleoperate"] = {}
                cfg["teleoperate"]["port"] = port
                
                save_yaml_config(SO101_LEADER_LOWLEVEL_CONFIG_PATH, cfg)
                
                permission_set = False
                
                # Set permissions on Linux using sudo with password
                if sys.platform.startswith("linux"):
                    
                    print("[主臂串口] 正在设置权限: sudo chmod 666", port)
                    
                    try:
                        logger.info(f"[主臂串口] 正在设置权限: sudo chmod 666 {port}")
                        # Use echo password | sudo -S to provide password
                        chmod_cmd = ["sudo", "-S", "chmod", "666", port]
                        result = subprocess.run(
                            chmod_cmd,
                            input=sudo_password + "\n",
                            capture_output=True,
                            text=True,
                            timeout=10
                        )
                        if result.returncode != 0:
                            error_msg = result.stderr.strip() or "权限设置失败，请检查密码是否正确"
                            logger.warning(f"[主臂串口] 权限设置失败: {error_msg}")
                            return jsonify({"success": False, "message": error_msg})
                        permission_set = True
                        logger.info(f"[主臂串口] ✅ 权限设置成功")
                    except subprocess.TimeoutExpired:
                        logger.error("[主臂串口] 权限设置超时")
                        return jsonify({"success": False, "message": "权限设置超时"})
                    except Exception as e:
                        logger.error(f"[主臂串口] 权限设置异常: {e}")
                        return jsonify({"success": False, "message": str(e)})
                
                logger.info(f"[主臂串口] ✅ 配置保存成功" + ("，权限已设置" if permission_set else ""))
                return jsonify({
                    "success": True, 
                    "permission_set": permission_set,
                    "message": "配置已保存" + ("，串口权限已设置" if permission_set else "")
                })
            except Exception as e:
                logger.error(f"[主臂串口] 保存失败: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        # ==================== Calibration ====================
        @self._app.route("/api/calibration/start", methods=["POST"])
        def api_calibration_start():
            try:
                data = request.get_json(silent=True) or {}
                arm = data.get("arm", "follower")
                
                if not SO101_ROBOT_DIR.exists():
                    return jsonify({"success": False, "message": f"Robot directory not found: {SO101_ROBOT_DIR}"})
                
                # Start calibration in a background thread
                threading.Thread(
                    target=self._run_calibration, args=(arm,), daemon=True
                ).start()
                
                return jsonify({"success": True})
            except Exception as e:
                logger.error(f"Error starting calibration: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        # ==================== Step-by-Step Calibration (Non-Interactive) ====================
        @self._app.route("/api/calibration/connect", methods=["POST"])
        def api_calibration_connect():
            """Connect to robot and prepare for calibration."""
            if not CALIB_CONTROLLER_AVAILABLE:
                return jsonify({"success": False, "message": "Calibration controller not available"})
            
            try:
                data = request.get_json(silent=True) or {}
                arm = data.get("arm", "follower")
                
                # Create log callback to emit via WebSocket
                def log_callback(level, message):
                    self._socketio.emit("calib_log", {
                        "arm": arm,
                        "message": message,
                        "level": level
                    })
                
                # Get or create controller
                controller = get_controller(
                    arm=arm,
                    robot_type="so101",
                    config_path=str(SO101_LOWLEVEL_CONFIG_PATH),
                    log_callback=log_callback
                )
                
                result = controller.connect()
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error connecting for calibration: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        @self._app.route("/api/calibration/record_middle", methods=["POST"])
        def api_calibration_record_middle():
            """Record the homing offsets at middle position."""
            if not CALIB_CONTROLLER_AVAILABLE:
                return jsonify({"success": False, "message": "Calibration controller not available"})
            
            try:
                data = request.get_json(silent=True) or {}
                arm = data.get("arm", "follower")
                
                controller = get_controller(arm=arm)
                result = controller.record_middle_position()
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error recording middle position: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        @self._app.route("/api/calibration/start_range", methods=["POST"])
        def api_calibration_start_range():
            """Start recording joint range of motion."""
            if not CALIB_CONTROLLER_AVAILABLE:
                return jsonify({"success": False, "message": "Calibration controller not available"})
            
            try:
                data = request.get_json(silent=True) or {}
                arm = data.get("arm", "follower")
                
                controller = get_controller(arm=arm)
                result = controller.start_recording_range()
                
                if result.get("success"):
                    # Start position streaming thread
                    threading.Thread(
                        target=self._calib_position_stream_loop,
                        args=(arm,),
                        daemon=True
                    ).start()
                
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error starting range recording: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        @self._app.route("/api/calibration/stop_range", methods=["POST"])
        def api_calibration_stop_range():
            """Stop recording and save calibration."""
            if not CALIB_CONTROLLER_AVAILABLE:
                return jsonify({"success": False, "message": "Calibration controller not available"})
            
            try:
                data = request.get_json(silent=True) or {}
                arm = data.get("arm", "follower")
                
                controller = get_controller(arm=arm)
                result = controller.stop_recording_range()
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error stopping range recording: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        @self._app.route("/api/calibration/disconnect", methods=["POST"])
        def api_calibration_disconnect():
            """Disconnect from the robot."""
            if not CALIB_CONTROLLER_AVAILABLE:
                return jsonify({"success": False, "message": "Calibration controller not available"})
            
            try:
                data = request.get_json(silent=True) or {}
                arm = data.get("arm", "follower")
                
                controller = get_controller(arm=arm)
                result = controller.disconnect()
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        @self._app.route("/api/calibration/reset", methods=["POST"])
        def api_calibration_reset():
            """Reset the calibration controller."""
            if not CALIB_CONTROLLER_AVAILABLE:
                return jsonify({"success": False, "message": "Calibration controller not available"})
            
            try:
                data = request.get_json(silent=True) or {}
                arm = data.get("arm", "follower")
                
                reset_controller(arm)
                return jsonify({"success": True, "message": "已重置"})
                
            except Exception as e:
                logger.error(f"Error resetting calibration: {e}")
                return jsonify({"success": False, "message": str(e)})
        
        # ==================== Shutdown ====================
        @self._app.route("/api/shutdown", methods=["POST"])
        def api_shutdown():
            """Shutdown the configuration server."""
            logger.info("配置完成，关闭服务器...")
            self._stop_event.set()
            def shutdown():
                time.sleep(0.5)
                os._exit(0)
            threading.Thread(target=shutdown, daemon=True).start()
            return jsonify({"success": True, "message": "正在关闭..."})
    
    def _update_camera_device_id(self, cfg: Dict[str, Any], out_key: str, device_id: str) -> None:
        """Update camera device_id in so101_config.yaml."""
        servers = cfg.get("servers")
        if not isinstance(servers, list):
            return
        
        for server in servers:
            if isinstance(server, dict) and server.get("name") == "sensor_server":
                inputs = server.get("inputs", [])
                for item in inputs:
                    if not isinstance(item, dict):
                        continue
                    params = item.get("params", {})
                    if params.get("out_key") == out_key:
                        if "init_args" not in params:
                            params["init_args"] = {}
                        params["init_args"]["device_id"] = device_id
                        return
    
    def _camera_stream_loop(self) -> None:
        """Stream camera frames via WebSocket."""
        while self._streaming_cameras and not self._stop_event.is_set():
            for idx in self._camera_indices:
                if not self._streaming_cameras:
                    break
                frame = capture_camera_frame(idx)
                if frame:
                    b64 = base64.b64encode(frame).decode("ascii")
                    self._socketio.emit("camera_frame", {"index": idx, "frame": b64})
            time.sleep(0.1)  # 10 FPS
    
    def _calib_position_stream_loop(self, arm: str) -> None:
        """Stream calibration motor positions via WebSocket."""
        if not CALIB_CONTROLLER_AVAILABLE:
            return
        
        try:
            controller = get_controller(arm=arm)
            
            while controller.state == "recording_range" and not self._stop_event.is_set():
                status = controller.get_recording_status()
                if status.get("success"):
                    self._socketio.emit("calib_positions", {
                        "arm": arm,
                        "current_positions": status.get("current_positions", {}),
                        "mins": status.get("mins", {}),
                        "maxes": status.get("maxes", {})
                    })
                time.sleep(0.05)  # 20 FPS
                
        except Exception as e:
            logger.error(f"Error in calibration position stream: {e}")
    
    def _run_calibration(self, arm: str) -> None:
        """Run calibration process and stream output."""
        try:
            cmd = [
                sys.executable,
                "-m",
                "scripts.calibrate",
                "--robot_type",
                "so101",
                "--arm",
                arm,
            ]
            
            self._socketio.emit("calib_log", {
                "arm": arm,
                "message": f"启动标定命令: {' '.join(cmd)}",
                "level": "info"
            })
            
            process = subprocess.Popen(
                cmd,
                cwd=str(SO101_ROBOT_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            
            self._calib_process = process
            
            progress = 0
            while True:
                line = process.stdout.readline()
                if not line and process.poll() is not None:
                    break
                if line:
                    line = line.strip()
                    # Determine log level
                    level = ""
                    if "error" in line.lower():
                        level = "error"
                    elif "warning" in line.lower():
                        level = "warning"
                    elif "success" in line.lower() or "完成" in line:
                        level = "info"
                    
                    self._socketio.emit("calib_log", {
                        "arm": arm,
                        "message": line,
                        "level": level
                    })
                    
                    # Update progress (rough estimation)
                    progress = min(progress + 5, 90)
                    self._socketio.emit("calib_progress", {
                        "arm": arm,
                        "progress": progress,
                        "message": "标定中..."
                    })
            
            returncode = process.wait()
            success = returncode == 0
            
            self._socketio.emit("calib_progress", {
                "arm": arm,
                "progress": 100,
                "message": "完成" if success else "失败"
            })
            
            self._socketio.emit("calib_complete", {
                "arm": arm,
                "success": success,
                "message": "" if success else f"标定失败，返回码: {returncode}"
            })
            
        except Exception as e:
            logger.error(f"Calibration error: {e}")
            self._socketio.emit("calib_complete", {
                "arm": arm,
                "success": False,
                "message": str(e)
            })
        finally:
            self._calib_process = None
    
    def start(self) -> None:
        """Start the web server."""
        if self._server_thread is not None:
            logger.warning("Server already running.")
            return
        
        self._stop_event.clear()
        
        def run_server():
            try:
                self._socketio.run(
                    self._app,
                    host=self._host,
                    port=self._port,
                    debug=False,
                    use_reloader=False,
                    allow_unsafe_werkzeug=True,
                )
            except Exception as e:
                logger.error(f"Server error: {e}")
        
        self._server_thread = threading.Thread(
            target=run_server, daemon=True, name="so101-config-web-server"
        )
        self._server_thread.start()
        
        logger.info(f"SO101 Config Web UI started at http://{self._host}:{self._port}")
        
        if self._open_browser:
            time.sleep(0.5)
            webbrowser.open(f"http://127.0.0.1:{self._port}")
    
    def stop(self) -> None:
        """Stop the web server."""
        self._stop_event.set()
        self._streaming_cameras = False
        if self._calib_process:
            self._calib_process.terminate()
        logger.info("SO101 Config Web UI stopped.")
    
    def run(self) -> None:
        """Run the web server (blocking)."""
        self.start()
        try:
            while not self._stop_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="SO101 Web Configuration Tool")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    parser.add_argument("--port", type=int, default=5001, help="Port to bind to")
    parser.add_argument("--no-browser", action="store_true", help="Don't open browser automatically")
    
    args = parser.parse_args()
    
    ui = SO101ConfigWebUI(
        host=args.host,
        port=args.port,
        open_browser=not args.no_browser,
    )
    ui.run()


if __name__ == "__main__":
    main()
