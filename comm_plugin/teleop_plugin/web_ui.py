# comm_plugin/teleop_plugin/web_ui.py

"""
Web UI server for teleoperation control.

This module provides a Flask + WebSocket server that:
- Renders images and state received from the follower arm
- Provides buttons for teleop control (start/stop/quit)
- Provides buttons for data collection (start/stop recording, export)
- Displays recording status and frame count

The server is started by TeleopPlugin when running in leader role with
``enable_web_ui=True``.
"""

from __future__ import annotations

import base64
import threading
import time
import webbrowser
from typing import TYPE_CHECKING, Any, Callable, Dict, Optional

from flask import Flask, jsonify, render_template_string, request
from flask_socketio import SocketIO

from rcp_core.common.utils.logger import server_logger

if TYPE_CHECKING:
    from .teleop_plugin import TeleopPlugin

logger = server_logger()

# ============================================================================ #
# HTML Template
# ============================================================================ #

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>遥操控制台 - Teleop Console</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            color: #e8e8e8;
        }
        .container { max-width: 1400px; margin: 0 auto; padding: 20px; }
        header {
            text-align: center;
            padding: 20px 0;
            border-bottom: 1px solid #333;
            margin-bottom: 20px;
            position: relative;
        }
        header h1 {
            font-size: 28px;
            background: linear-gradient(90deg, #00d4ff, #00ff88);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .main-content { display: grid; grid-template-columns: 2fr 1.5fr; gap: 20px; }
        .video-section {
            background: #1e1e2f;
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        .video-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 15px; }
        .video-container {
            background: #2a2a3e;
            border-radius: 8px;
            overflow: hidden;
            position: relative;
        }
        .video-container img { width: 100%; height: auto; display: block; }
        .video-label {
            position: absolute; top: 10px; left: 10px;
            background: rgba(0,0,0,0.7);
            padding: 5px 10px; border-radius: 4px;
            font-size: 12px; color: #00d4ff;
        }
        .control-panel {
            background: #1e1e2f;
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        .section-title {
            font-size: 16px; color: #888;
            text-transform: uppercase; letter-spacing: 1px;
            margin-bottom: 15px; padding-bottom: 10px;
            border-bottom: 1px solid #333;
        }
        .status-grid { display: grid; gap: 12px; margin-bottom: 20px; }
        .status-item {
            display: flex; justify-content: space-between; align-items: center;
            padding: 10px 15px; background: #2a2a3e; border-radius: 8px;
        }
        .status-label { color: #888; font-size: 14px; }
        .status-value { font-weight: 600; font-size: 14px; }
        .status-value.active { color: #00ff88; }
        .status-value.inactive { color: #ff6b6b; }
        .status-value.recording { color: #ff4757; animation: pulse 1s infinite; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        .button-group { display: grid; gap: 10px; margin-bottom: 20px; }
        .btn {
            padding: 12px 20px; border: none; border-radius: 8px;
            font-size: 14px; font-weight: 600; cursor: pointer;
            transition: all 0.2s; text-transform: uppercase; letter-spacing: 0.5px;
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
        .state-display { background: #2a2a3e; border-radius: 8px; padding: 15px; margin-top: 20px; }
        .state-display pre {
            font-family: 'Fira Code', monospace; font-size: 12px;
            color: #00ff88; white-space: pre-wrap; word-break: break-all;
        }
        .workflow-hint {
            background: #2a3a4e; border-left: 4px solid #00d4ff;
            padding: 15px; border-radius: 0 8px 8px 0; margin-top: 20px;
        }
        .workflow-hint h4 { color: #00d4ff; margin-bottom: 10px; }
        .workflow-hint ol { padding-left: 20px; line-height: 1.8; }
        .workflow-hint li { color: #aaa; }
        .workflow-hint li.current { color: #00ff88; font-weight: bold; }
        .toast {
            position: fixed; bottom: 20px; right: 20px;
            padding: 15px 25px; border-radius: 8px; color: white;
            font-weight: 500; transform: translateX(150%);
            transition: transform 0.3s; z-index: 1000;
        }
        .toast.show { transform: translateX(0); }
        .toast.success { background: #00cc6a; }
        .toast.error { background: #ff4757; }
        .toast.info { background: #00d4ff; }
        /* Data Management Styles */
        .data-coll-item { margin-bottom: 15px; border: 1px solid #444; border-radius: 8px; overflow: hidden; }
        .data-coll-header {
            background: #2a3a4e; padding: 10px 15px; cursor: pointer;
            display: flex; justify-content: space-between; align-items: center;
        }
        .data-coll-header:hover { background: #3a4a5e; }
        .data-coll-name { font-weight: 600; color: #00d4ff; }
        .data-coll-actions { display: flex; gap: 8px; }
        .data-coll-actions button {
            padding: 4px 10px; font-size: 11px; border: none; border-radius: 4px;
            cursor: pointer; background: #ff4757; color: white;
        }
        .task-prompt-item { margin: 8px 0 8px 20px; border-left: 2px solid #444; padding-left: 10px; }
        .task-prompt-name { font-size: 13px; color: #aaa; margin-bottom: 5px; }
        .episode-item {
            display: flex; align-items: center; gap: 10px; padding: 6px 10px;
            background: #1e1e2f; margin: 4px 0; border-radius: 4px; font-size: 12px;
        }
        .episode-item:hover { background: #252535; }
        .episode-item input[type="checkbox"] { cursor: pointer; }
        .episode-name { flex: 1; color: #ccc; }
        .episode-status {
            padding: 2px 8px; border-radius: 10px; font-size: 10px;
        }
        .episode-status.complete { background: #00cc6a; color: #1a1a2e; }
        .episode-status.incomplete { background: #ffa500; color: #1a1a2e; }
        .episode-status.invalid { background: #ff4757; color: white; }
        .episode-status.unknown { background: #666; color: #ccc; }
        .episode-frames { color: #888; font-size: 11px; }
        .episode-actions button {
            padding: 2px 8px; font-size: 10px; border: none; border-radius: 4px;
            cursor: pointer; background: #ff4757; color: white;
        }
        .exported-zips-section { margin-top: 15px; padding-top: 15px; border-top: 1px solid #444; }
        .exported-zip-item {
            display: flex; justify-content: space-between; align-items: center;
            padding: 8px 10px; background: #1e1e2f; margin: 4px 0; border-radius: 4px; font-size: 12px;
        }
        .zip-name { color: #00ff88; }
        .zip-size { color: #888; font-size: 11px; }
        /* Language toggle */
        .lang-toggle {
            position: absolute; right: 20px; top: 50%;
            transform: translateY(-50%);
            background: rgba(255,255,255,0.1); border: 1px solid #555;
            color: #e8e8e8; padding: 6px 14px; border-radius: 6px;
            cursor: pointer; font-size: 13px; font-weight: 600;
            transition: all 0.2s; letter-spacing: 0.5px;
        }
        .lang-toggle:hover { background: rgba(255,255,255,0.2); border-color: #00d4ff; color: #00d4ff; }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1 data-i18n="header-title">RynnRCP 遥操数采平台</h1>
            <button class="lang-toggle" id="lang-toggle" onclick="toggleLang()"><span id="lang-hint" style="font-size:11px;color:#888;font-weight:400;">Switch to </span>EN</button>
        </header>
        <div class="main-content">
            <div class="video-section">
                <h3 class="section-title" data-i18n="section-follower-view">从臂视野</h3>
                <div class="video-grid" id="video-grid">
                    <div class="video-container" style="display:flex;align-items:center;justify-content:center;min-height:200px;">
                        <span style="color:#555;" data-i18n="waiting-image">等待图像数据...</span>
                    </div>
                </div>
                <div class="state-display">
                    <h4 style="color: #888; margin-bottom: 10px;" data-i18n="follower-state-title">从臂状态</h4>
                    <pre id="state-display" data-i18n="waiting-data">等待数据...</pre>
                    <h4 style="color: #888; margin: 10px 0;" data-i18n="leader-state-title">主臂状态</h4>
                    <pre id="leader-state-display" data-i18n="waiting-data">等待数据...</pre>
                </div>
            </div>
            <div class="control-panel">
                <h3 class="section-title" data-i18n="section-status">状态监控</h3>
                <div class="status-grid" style="grid-template-columns: repeat(2, 1fr);">
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-teleop-status">遥操状态</span>
                        <span id="teleop-status" class="status-value inactive" data-i18n="teleop-off">未启动</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-follower-online">从臂在线</span>
                        <span id="follower-online" class="status-value inactive" data-i18n="follower-offline">离线</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-record-status">数采状态</span>
                        <span id="record-status" class="status-value inactive" data-i18n="record-off">未录制</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-round">当前轮次</span>
                        <span id="round-number" class="status-value">0</span>
                    </div>
                </div>
                <h3 class="section-title" data-i18n="section-teleop">遥操控制</h3>
                <div class="button-group" style="grid-template-columns: repeat(3, 1fr); gap: 8px;">
                    <button id="btn-start-teleop" class="btn btn-success" onclick="startTeleop()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-start-teleop">▶ 开始遥操</button>
                    <button id="btn-stop-teleop" class="btn btn-warning" onclick="stopTeleop()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-stop-teleop">⏸ 停止遥操</button>
                    <button id="btn-quit" class="btn btn-danger" onclick="quitTeleop()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-quit">⏹ 退出</button>
                </div>
                <h3 class="section-title" data-i18n="section-datacoll">数据采集</h3>
                <div class="status-grid" style="margin-bottom: 15px;">
                    <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:4px;">
                        <label class="status-label">Task Prompt</label>
                        <input id="input-task-prompt" type="text" value="teleop demo"
                               style="width:100%;padding:6px 10px;border-radius:6px;border:1px solid #444;background:#1e1e2f;color:#e8e8e8;font-size:13px;">
                    </div>
                    <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:4px;">
                        <label class="status-label">Task Description</label>
                        <input id="input-task-desc" type="text" value="Teleoperation data collection"
                               style="width:100%;padding:6px 10px;border-radius:6px;border:1px solid #444;background:#1e1e2f;color:#e8e8e8;font-size:13px;">
                    </div>
                    <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:4px;">
                        <label class="status-label">FPS</label>
                        <input id="input-fps" type="number" value="30" min="1" max="120" step="1"
                               style="width:100%;padding:6px 10px;border-radius:6px;border:1px solid #444;background:#1e1e2f;color:#e8e8e8;font-size:13px;">
                    </div>
                    <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:4px;">
                        <label class="status-label" data-i18n="label-data-coll-id">Data Collection ID (自动生成)</label>
                        <input id="input-data-coll-id" type="text" readonly
                               style="width:100%;padding:6px 10px;border-radius:6px;border:1px solid #555;background:#252535;color:#888;font-size:13px;">
                    </div>
                </div>
                <!-- Data Keys Selection -->
                <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:8px;margin-bottom:15px;">
                    <label class="status-label"><span data-i18n="label-data-keys">采集数据项</span> <span id="keys-status" style="color:#888;font-size:11px;" data-i18n="keys-status-waiting">(等待从臂数据...)</span></label>
                    <div id="record-keys-container" style="display:flex;flex-wrap:wrap;gap:10px;width:100%;">
                        <span style="color:#666;font-size:13px;" data-i18n="keys-empty">等待从臂发送可用数据项...</span>
                    </div>
                </div>
                <div class="button-group" style="grid-template-columns: repeat(3, 1fr); gap: 8px;">
                    <button id="btn-start-record" class="btn btn-success" onclick="startRecording()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-start-record">⏺ 开始数采</button>
                    <button id="btn-stop-record" class="btn btn-warning" onclick="stopRecording()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-stop-record">⏹ 停止数采</button>
                    <button id="btn-export" class="btn btn-primary" onclick="exportData()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-export">📦 导出数据</button>
                </div>
                <!-- Export Progress -->
                <div id="export-progress-container" style="display:none; margin-top: 15px; padding: 15px; background: #2a3a4e; border-radius: 8px;">
                    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom: 8px;">
                        <span style="color: #00d4ff; font-size: 13px;" data-i18n="export-progress-label">📦 正在导出数据...</span>
                        <span id="export-progress-text" style="color: #888; font-size: 12px;" data-i18n="processing">处理中</span>
                    </div>
                    <div style="width:100%; height: 6px; background: #1e1e2f; border-radius: 3px; overflow: hidden;">
                        <div id="export-progress-bar" style="width: 0%; height: 100%; background: linear-gradient(90deg, #00d4ff, #00ff88); transition: width 0.3s;"></div>
                    </div>
                </div>
                <!-- Export Result -->
                <div id="export-result-container" style="display:none; margin-top: 15px; padding: 15px; background: #2a3a4e; border-radius: 8px;">
                    <h4 style="color: #00ff88; margin-bottom: 10px; font-size: 14px;" data-i18n="export-complete-title">✅ 导出完成</h4>
                    <div style="font-size: 12px; color: #aaa; line-height: 1.6;">
                        <div id="export-result-message"></div>
                        <div style="margin-top: 8px; color: #ffaa00; font-size: 11px;" data-i18n="export-path-warning">⚠️ 以下路径为从臂工控机上的文件位置</div>
                        <div id="export-result-path" style="margin-top: 4px; word-break: break-all; color: #00d4ff;"></div>
                    </div>
                </div>
                <div class="workflow-hint">
                    <h4 data-i18n="workflow-title">📋 操作流程</h4>
                    <ol>
                        <li id="step1" data-i18n="step1">点击"开始遥操"启动主从臂通信</li>
                        <li id="step2" data-i18n="step2">点击"开始数采"开始录制</li>
                        <li id="step3" data-i18n="step3">操作机械臂执行任务</li>
                        <li id="step4" data-i18n="step4">点击"停止数采"结束录制</li>
                        <li id="step5" data-i18n="step5">重复2-4采集多条数据</li>
                        <li id="step6" data-i18n="step6">点击"导出数据"编码导出</li>
                    </ol>
                </div>

                <!-- Data Management Section -->
                <h3 class="section-title" style="margin-top: 25px;" data-i18n="section-datamgmt">📁 数据管理</h3>
                <div id="disk-usage-info" style="background: #2a3a4e; padding: 10px 15px; border-radius: 8px; margin-bottom: 15px; display: none;">
                    <div style="display: flex; justify-content: space-between; align-items: center;">
                        <span style="color: #888; font-size: 13px;" data-i18n="disk-usage-label">💾 总磁盘占用</span>
                        <span id="total-size" style="color: #00d4ff; font-weight: 600;">-</span>
                    </div>
                    <div id="export-dir-info" style="margin-top: 8px; padding-top: 8px; border-top: 1px solid #444; font-size: 12px; color: #aaa; display: none;">
                        <span data-i18n="export-dir-label">📂 导出位置（从臂）:</span> <span id="export-dir-path" style="color: #00ff88;"></span>
                    </div>
                </div>
                <div class="button-group" style="grid-template-columns: repeat(3, 1fr); gap: 8px; margin-bottom: 15px;">
                    <button id="btn-refresh-records" class="btn btn-primary" onclick="refreshRecords()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-refresh">🔄 刷新列表</button>
                    <button id="btn-encode-selected" class="btn btn-warning" onclick="encodeSelectedEpisodes()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-encode-selected">🔧 编码选中</button>
                    <button id="btn-export-selected" class="btn btn-success" onclick="exportSelectedEpisodes()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-export-selected">📦 导出选中</button>
                </div>
                <!-- Encode Progress -->
                <div id="encode-progress-container" style="display:none; margin-bottom: 15px; padding: 15px; background: #2a3a4e; border-radius: 8px;">
                    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom: 8px;">
                        <span style="color: #ffaa00; font-size: 13px;" data-i18n="encode-progress-label">🔧 正在编码数据...</span>
                        <span id="encode-progress-text" style="color: #888; font-size: 12px;" data-i18n="processing">处理中</span>
                    </div>
                    <div style="width:100%; height: 6px; background: #1e1e2f; border-radius: 3px; overflow: hidden;">
                        <div id="encode-progress-bar" style="width: 0%; height: 100%; background: linear-gradient(90deg, #ffaa00, #ffdd00); transition: width 0.3s;"></div>
                    </div>
                </div>
                <!-- Export Selected Progress -->
                <div id="export-selected-progress-container" style="display:none; margin-bottom: 15px; padding: 15px; background: #2a3a4e; border-radius: 8px;">
                    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom: 8px;">
                        <span style="color: #00d4ff; font-size: 13px;" data-i18n="export-sel-progress-label">📦 正在导出选中数据...</span>
                        <span id="export-selected-progress-text" style="color: #888; font-size: 12px;" data-i18n="processing">处理中</span>
                    </div>
                    <div style="width:100%; height: 6px; background: #1e1e2f; border-radius: 3px; overflow: hidden;">
                        <div id="export-selected-progress-bar" style="width: 0%; height: 100%; background: linear-gradient(90deg, #00d4ff, #00ff88); transition: width 0.3s;"></div>
                    </div>
                </div>
                <!-- Playback Progress -->
                <div id="playback-progress-container" style="display:none; margin-bottom: 15px; padding: 15px; background: #3a2a4e; border-radius: 8px; border: 1px solid #9C27B0;">
                    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom: 8px;">
                        <span style="color: #E1BEE7; font-size: 13px;" data-i18n="playback-label">▶ 真机回放中...</span>
                        <span id="playback-episode-name" style="color: #888; font-size: 11px; max-width: 150px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap;"></span>
                    </div>
                    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom: 8px;">
                        <span id="playback-status-text" style="color: #CE93D8; font-size: 12px;" data-i18n="playback-preparing">准备中...</span>
                        <span id="playback-progress-text" style="color: #888; font-size: 12px;">0%</span>
                    </div>
                    <div style="width:100%; height: 6px; background: #1e1e2f; border-radius: 3px; overflow: hidden;">
                        <div id="playback-progress-bar" style="width: 0%; height: 100%; background: linear-gradient(90deg, #9C27B0, #E1BEE7); transition: width 0.3s;"></div>
                    </div>
                    <div style="margin-top: 10px; text-align: center;">
                        <button id="btn-stop-playback" onclick="stopPlayback()" style="background: #f44336; color: white; border: none; padding: 6px 16px; border-radius: 4px; cursor: pointer; font-size: 12px;" data-i18n="btn-stop-playback">⏹ 停止回放</button>
                    </div>
                </div>
                <div id="records-container" style="max-height: 500px; overflow-y: auto; background: #2a2a3e; border-radius: 8px; padding: 10px;">
                    <div style="color: #888; text-align: center; padding: 20px;" data-i18n="records-empty">点击"刷新列表"加载本地数据</div>
                </div>
            </div>
        </div>
    </div>
    <div id="toast" class="toast"></div>
    <script>
        // ==================== i18n ====================
        const I18N = {
            zh: {
                'header-title': 'RynnRCP 遥操数采平台',
                'lang-btn': 'EN',
                'section-follower-view': '从臂视野',
                'waiting-image': '等待图像数据...',
                'follower-state-title': '从臂状态',
                'leader-state-title': '主臂状态',
                'waiting-data': '等待数据...',
                'section-status': '状态监控',
                'label-teleop-status': '遥操状态',
                'label-follower-online': '从臂在线',
                'label-record-status': '数采状态',
                'label-round': '当前轮次',
                'teleop-on': '运行中', 'teleop-off': '未启动',
                'follower-online': '在线', 'follower-offline': '离线',
                'record-on': '录制中...', 'record-off': '未录制',
                'section-teleop': '遥操控制',
                'btn-start-teleop': '▶ 开始遥操',
                'btn-stop-teleop': '⏸ 停止遥操',
                'btn-quit': '⏹ 退出',
                'section-datacoll': '数据采集',
                'label-data-coll-id': 'Data Collection ID (自动生成)',
                'label-data-keys': '采集数据项',
                'keys-status-waiting': '(等待从臂数据...)',
                'keys-empty': '等待从臂发送可用数据项...',
                'btn-start-record': '⏺ 开始数采',
                'btn-stop-record': '⏹ 停止数采',
                'btn-export': '📦 导出数据',
                'export-progress-label': '📦 正在导出数据...',
                'processing': '处理中',
                'export-complete-title': '✅ 导出完成',
                'export-path-warning': '⚠️ 以下路径为从臂工控机上的文件位置',
                'workflow-title': '📋 操作流程',
                'step1': '点击"开始遥操"启动主从臂通信',
                'step2': '点击"开始数采"开始录制',
                'step3': '操作机械臂执行任务',
                'step4': '点击"停止数采"结束录制',
                'step5': '重复2-4采集多条数据',
                'step6': '点击"导出数据"编码导出',
                'section-datamgmt': '📁 数据管理',
                'disk-usage-label': '💾 总磁盘占用',
                'export-dir-label': '📂 导出位置（从臂）:',
                'btn-refresh': '🔄 刷新列表',
                'btn-encode-selected': '🔧 编码选中',
                'btn-export-selected': '📦 导出选中',
                'encode-progress-label': '🔧 正在编码数据...',
                'export-sel-progress-label': '📦 正在导出选中数据...',
                'playback-label': '▶ 真机回放中...',
                'playback-preparing': '准备中...',
                'btn-stop-playback': '⏹ 停止回放',
                'records-empty': '点击"刷新列表"加载本地数据',
                'records-no-data': '暂无本地数据',
                'msg-teleop-started': '遥操已启动',
                'msg-start-failed': '启动失败: ',
                'msg-teleop-stopped': '遥操已停止',
                'msg-quitting': '正在退出...',
                'msg-confirm-quit': '确定要退出吗？',
                'msg-select-key': '请至少选择一个数据项',
                'msg-record-started': '数采已开始，轮次: ',
                'msg-record-confirm': '帧数据，是否保留？\\n\\n点击"确定"保留数据\\n点击"取消"丢弃数据',
                'msg-data-kept': '数据已保留，轮次: ',
                'msg-data-discarded': '本轮数据已丢弃，下次采集将覆盖',
                'msg-no-data': '本轮无有效数据',
                'msg-export-waiting': '等待从臂响应...',
                'msg-export-timeout': '导出超时：从臂未响应，请检查从臂是否在线',
                'msg-export-failed': '导出失败: ',
                'msg-exporting': '正在导出数据...',
                'msg-export-complete': '导出完成',
                'msg-records-refreshed': '数据列表已刷新',
                'msg-refresh-failed': '刷新失败: ',
                'msg-confirm-delete': '确定要删除这条数据吗？此操作不可恢复。',
                'msg-deleted': '已删除',
                'msg-delete-failed': '删除失败: ',
                'msg-confirm-delete-coll': '确定要删除整个数据集合吗？此操作将删除所有相关数据，不可恢复。',
                'msg-coll-deleted': '数据集合已删除',
                'msg-encoding': '正在编码...',
                'msg-confirm-encode': '确定要编码这条数据吗？这将把原始流文件转换为标准格式。',
                'msg-encode-success': '编码成功: ',
                'msg-encode-failed': '编码失败: ',
                'msg-confirm-export-ep': '确定要导出这条数据吗？',
                'msg-exporting-ep': '正在导出...',
                'msg-export-ep-success': '导出成功: ',
                'msg-export-ep-failed': '导出失败: ',
                'msg-select-one': '请至少选择一条数据',
                'msg-no-raw': '选中的数据中没有需要编码的（原始格式）数据',
                'msg-confirm-encode-n': '确定要编码选中的 {n} 条数据吗？',
                'msg-skip-encoded': '（跳过 {n} 条已编码数据）',
                'msg-encode-done': '编码完成: {ok} 条成功, {fail} 条失败',
                'msg-invalid-data': '选中的数据格式无效，建议删除',
                'msg-skip-invalid': '注意: 跳过 {n} 条无效数据',
                'msg-confirm-export-n': '确定导出 {n} 条数据？原始格式将自动编码并导出。',
                'msg-stop-teleop-first': '请先停止遥操再开始回放',
                'msg-confirm-playback': '确定要开始回放 "{name}" 吗？\\n\\n回放过程：\\n1. 机械臂将移动到起始位置（约3秒）\\n2. 停留2秒后开始回放\\n3. 按原始时间戳执行动作\\n\\n请确保机械臂周围无障碍物！',
                'msg-playback-started': '回放已开始',
                'msg-playback-failed': '回放启动失败: ',
                'msg-playback-stopped': '回放已停止',
                'msg-playback-stop-failed': '停止失败: ',
                'msg-playback-done': '回放完成',
                'msg-playback-error': '回放错误: ',
                'msg-keys-available': '项可用',
                'status-complete': '完整', 'status-incomplete': '未完成',
                'status-invalid': '无效', 'status-unknown': '未知',
                'btn-playback': '▶ 回放', 'btn-encode': '编码',
                'btn-export-ep': '导出', 'btn-delete': '删除',
                'btn-delete-all': '删除全部',
                'label-exported-files': '📦 已导出文件',
                'label-follower-machine': '📍 从臂机器',
                'frames': '帧',
                'playback-loading': '加载数据中...',
                'playback-pre-move': '移动到起始位置...',
                'playback-waiting': '准备开始...',
                'playback-playing': '回放中...',
                'playback-completed': '回放完成',
                'playback-stopped': '已停止',
                'playback-error-prefix': '错误: ',
                'msg-collected': '本轮采集了 ',
            },
            en: {
                'header-title': 'RynnRCP Teleop Platform',
                'lang-btn': '中文',
                'section-follower-view': 'Follower View',
                'waiting-image': 'Waiting for image...',
                'follower-state-title': 'Follower State',
                'leader-state-title': 'Leader State',
                'waiting-data': 'Waiting for data...',
                'section-status': 'Status',
                'label-teleop-status': 'Teleop',
                'label-follower-online': 'Follower',
                'label-record-status': 'Recording',
                'label-round': 'Round',
                'teleop-on': 'Running', 'teleop-off': 'Stopped',
                'follower-online': 'Online', 'follower-offline': 'Offline',
                'record-on': 'Recording...', 'record-off': 'Idle',
                'section-teleop': 'Teleop Control',
                'btn-start-teleop': '▶ Start',
                'btn-stop-teleop': '⏸ Stop',
                'btn-quit': '⏹ Quit',
                'section-datacoll': 'Data Collection',
                'label-data-coll-id': 'Data Collection ID (auto)',
                'label-data-keys': 'Data Keys',
                'keys-status-waiting': '(waiting for follower...)',
                'keys-empty': 'Waiting for follower to send available keys...',
                'btn-start-record': '⏺ Record',
                'btn-stop-record': '⏹ Stop Rec',
                'btn-export': '📦 Export',
                'export-progress-label': '📦 Exporting data...',
                'processing': 'Processing',
                'export-complete-title': '✅ Export Complete',
                'export-path-warning': '⚠️ Path below is on the follower machine',
                'workflow-title': '📋 Workflow',
                'step1': 'Click "Start" to begin teleop',
                'step2': 'Click "Record" to start recording',
                'step3': 'Operate the arm to perform task',
                'step4': 'Click "Stop Rec" to end recording',
                'step5': 'Repeat 2-4 for more episodes',
                'step6': 'Click "Export" to encode & export',
                'section-datamgmt': '📁 Data Management',
                'disk-usage-label': '💾 Total Disk Usage',
                'export-dir-label': '📂 Export Dir (follower):',
                'btn-refresh': '🔄 Refresh',
                'btn-encode-selected': '🔧 Encode',
                'btn-export-selected': '📦 Export Sel.',
                'encode-progress-label': '🔧 Encoding data...',
                'export-sel-progress-label': '📦 Exporting selected...',
                'playback-label': '▶ Playing back...',
                'playback-preparing': 'Preparing...',
                'btn-stop-playback': '⏹ Stop Playback',
                'records-empty': 'Click "Refresh" to load data',
                'records-no-data': 'No local data',
                'msg-teleop-started': 'Teleop started',
                'msg-start-failed': 'Start failed: ',
                'msg-teleop-stopped': 'Teleop stopped',
                'msg-quitting': 'Quitting...',
                'msg-confirm-quit': 'Are you sure you want to quit?',
                'msg-select-key': 'Please select at least one data key',
                'msg-record-started': 'Recording started, round: ',
                'msg-record-confirm': ' frames collected. Keep this data?\\n\\nOK = Keep\\nCancel = Discard',
                'msg-data-kept': 'Data kept, round: ',
                'msg-data-discarded': 'Data discarded',
                'msg-no-data': 'No valid data in this round',
                'msg-export-waiting': 'Waiting for follower...',
                'msg-export-timeout': 'Export timeout: follower not responding',
                'msg-export-failed': 'Export failed: ',
                'msg-exporting': 'Exporting data...',
                'msg-export-complete': 'Export complete',
                'msg-records-refreshed': 'Records refreshed',
                'msg-refresh-failed': 'Refresh failed: ',
                'msg-confirm-delete': 'Delete this episode? This cannot be undone.',
                'msg-deleted': 'Deleted',
                'msg-delete-failed': 'Delete failed: ',
                'msg-confirm-delete-coll': 'Delete entire collection? This cannot be undone.',
                'msg-coll-deleted': 'Collection deleted',
                'msg-encoding': 'Encoding...',
                'msg-confirm-encode': 'Encode this episode? This converts raw stream to standard format.',
                'msg-encode-success': 'Encoded: ',
                'msg-encode-failed': 'Encode failed: ',
                'msg-confirm-export-ep': 'Export this episode?',
                'msg-exporting-ep': 'Exporting...',
                'msg-export-ep-success': 'Exported: ',
                'msg-export-ep-failed': 'Export failed: ',
                'msg-select-one': 'Please select at least one episode',
                'msg-no-raw': 'No raw (unencoded) episodes selected',
                'msg-confirm-encode-n': 'Encode {n} selected episodes?',
                'msg-skip-encoded': '(skipping {n} already encoded)',
                'msg-encode-done': 'Encode done: {ok} succeeded, {fail} failed',
                'msg-invalid-data': 'Selected data is invalid, consider deleting',
                'msg-skip-invalid': 'Note: skipping {n} invalid episodes',
                'msg-confirm-export-n': 'Export {n} episodes? Raw data will be auto-encoded.',
                'msg-stop-teleop-first': 'Please stop teleop before playback',
                'msg-confirm-playback': 'Start playback of "{name}"?\\n\\nPlayback process:\\n1. Arm moves to start position (~3s)\\n2. Wait 2s then begin\\n3. Execute actions by timestamp\\n\\nEnsure no obstacles around the arm!',
                'msg-playback-started': 'Playback started',
                'msg-playback-failed': 'Playback failed: ',
                'msg-playback-stopped': 'Playback stopped',
                'msg-playback-stop-failed': 'Stop failed: ',
                'msg-playback-done': 'Playback complete',
                'msg-playback-error': 'Playback error: ',
                'msg-keys-available': ' keys available',
                'status-complete': 'Complete', 'status-incomplete': 'Raw',
                'status-invalid': 'Invalid', 'status-unknown': 'Unknown',
                'btn-playback': '▶ Play', 'btn-encode': 'Encode',
                'btn-export-ep': 'Export', 'btn-delete': 'Delete',
                'btn-delete-all': 'Delete All',
                'label-exported-files': '📦 Exported Files',
                'label-follower-machine': '📍 Follower',
                'frames': ' frames',
                'playback-loading': 'Loading data...',
                'playback-pre-move': 'Moving to start position...',
                'playback-waiting': 'About to start...',
                'playback-playing': 'Playing...',
                'playback-completed': 'Playback complete',
                'playback-stopped': 'Stopped',
                'playback-error-prefix': 'Error: ',
                'msg-collected': 'Collected ',
            }
        };
        let currentLang = localStorage.getItem('teleop_lang') || 'zh';
        function t(key) { return (I18N[currentLang] || I18N.zh)[key] || (I18N.zh[key] || key); }
        function applyLang() {
            document.querySelectorAll('[data-i18n]').forEach(el => {
                const key = el.getAttribute('data-i18n');
                const text = t(key);
                if (text) el.textContent = text;
            });
            const btn = document.getElementById('lang-toggle');
            const hint = document.getElementById('lang-hint');
            if (currentLang === 'zh') {
                hint.textContent = 'Switch to ';
                btn.lastChild.textContent = 'EN';
            } else {
                hint.textContent = '切换为';
                btn.lastChild.textContent = '中文';
            }
            document.title = currentLang === 'zh' ? '遥操控制台 - Teleop Console' : 'Teleop Console - RynnRCP';
        }
        function toggleLang() {
            currentLang = currentLang === 'zh' ? 'en' : 'zh';
            localStorage.setItem('teleop_lang', currentLang);
            applyLang();
            // Re-apply dynamic state text
            updateUI();
        }
        // ==================== end i18n ====================
        const socket = io();
        let teleopEnabled = false, recordingEnabled = false;
        let dataCollId = '';
        // Fetch data_coll_id from backend on page load
        fetch('/api/config').then(r=>r.json()).then(d => {
            dataCollId = d.data_coll_id || 'teleop_' + Math.floor(Date.now() / 1000);
            document.getElementById('input-data-coll-id').value = dataCollId;
        });
        function updateUI() {
            document.getElementById('btn-start-teleop').disabled = teleopEnabled;
            document.getElementById('btn-stop-teleop').disabled = !teleopEnabled;
            document.getElementById('btn-start-record').disabled = !teleopEnabled || recordingEnabled;
            document.getElementById('btn-stop-record').disabled = !teleopEnabled || !recordingEnabled;
            document.getElementById('teleop-status').textContent = teleopEnabled ? t('teleop-on') : t('teleop-off');
            document.getElementById('teleop-status').className = 'status-value ' + (teleopEnabled ? 'active' : 'inactive');
            document.getElementById('record-status').textContent = recordingEnabled ? t('record-on') : t('record-off');
            document.getElementById('record-status').className = 'status-value ' + (recordingEnabled ? 'recording' : 'inactive');
            document.querySelectorAll('.workflow-hint li').forEach(li => li.classList.remove('current'));
            if (!teleopEnabled) document.getElementById('step1').classList.add('current');
            else if (!recordingEnabled) document.getElementById('step2').classList.add('current');
            else document.getElementById('step3').classList.add('current');
        }
        function showToast(msg, type='info', duration=3000) {
            const t = document.getElementById('toast');
            t.textContent = msg; t.className = 'toast ' + type + ' show';
            setTimeout(() => t.classList.remove('show'), duration);
        }
        function startTeleop() {
            fetch('/api/teleop/start', {method:'POST'}).then(r=>r.json()).then(d => {
                if (d.success) { teleopEnabled = true; updateUI(); showToast(t('msg-teleop-started'),'success'); }
                else showToast(t('msg-start-failed')+d.message,'error');
            });
        }
        function stopTeleop() {
            if (recordingEnabled) stopRecording();
            fetch('/api/teleop/stop', {method:'POST'}).then(r=>r.json()).then(d => {
                if (d.success) { teleopEnabled = false; updateUI(); showToast(t('msg-teleop-stopped'),'info'); }
            });
        }
        function quitTeleop() {
            if (recordingEnabled) stopRecording();
            if (confirm(t('msg-confirm-quit'))) {
                fetch('/api/quit', {method:'POST'}).then(() => {
                    showToast(t('msg-quitting'),'info');
                    setTimeout(() => window.close(), 1000);
                });
            }
        }
        function startRecording() {
            const selectedKeys = Array.from(document.querySelectorAll('.record-key:checked')).map(cb => cb.value);
            if (selectedKeys.length === 0) {
                showToast(t('msg-select-key'), 'error');
                return;
            }
            const body = {
                task_prompt: document.getElementById('input-task-prompt').value.trim() || 'teleop demo',
                task_description: document.getElementById('input-task-desc').value.trim() || '',
                fps: parseFloat(document.getElementById('input-fps').value) || 30,
                data_coll_id: dataCollId,
                keys: selectedKeys,
            };
            fetch('/api/record/start', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)}).then(r=>r.json()).then(d => {
                if (d.success) { recordingEnabled = true; updateUI(); showToast(t('msg-record-started')+d.round,'success'); }
                else showToast(t('msg-start-failed')+d.message,'error');
            });
        }
        function stopRecording() {
            fetch('/api/record/stop', {method:'POST'}).then(r=>r.json()).then(d => {
                recordingEnabled = false; updateUI();
                const frames = d.frames_written || 0;
                if (frames > 0) {
                    if (confirm(t('msg-collected') + frames + ' ' + t('msg-record-confirm'))) {
                        showToast(t('msg-data-kept') + d.round, 'success');
                    } else {
                        fetch('/api/record/discard', {method:'POST'}).then(r=>r.json()).then(dd => {
                            showToast(t('msg-data-discarded'), 'info');
                        });
                    }
                } else {
                    showToast(t('msg-no-data'), 'info');
                }
            });
        }
        let exportStartTimeout = null;
        function exportData() {
            // Show progress UI
            document.getElementById('export-progress-container').style.display = 'block';
            document.getElementById('export-result-container').style.display = 'none';
            document.getElementById('export-progress-bar').style.width = '0%';
            document.getElementById('export-progress-text').textContent = t('msg-export-waiting');
            document.getElementById('btn-export').disabled = true;

            clearTimeout(exportStartTimeout);
            exportStartTimeout = setTimeout(() => {
                if (document.getElementById('export-progress-container').style.display !== 'none'
                        && document.getElementById('export-progress-text').textContent === t('msg-export-waiting')) {
                    document.getElementById('export-progress-container').style.display = 'none';
                    document.getElementById('btn-export').disabled = false;
                    showToast(t('msg-export-timeout'), 'error');
                }
            }, 10000);
        
            fetch('/api/data/export', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({data_coll_id: dataCollId})}).then(r=>r.json()).then(d => {
                if (!d.success) {
                    clearTimeout(exportStartTimeout);
                    document.getElementById('export-progress-container').style.display = 'none';
                    document.getElementById('btn-export').disabled = false;
                    showToast(t('msg-export-failed') + (d.message || ''), 'error');
                }
                // Real progress arrives via export_progress socket event
            });
        }
        socket.on('export_started', d => {
            clearTimeout(exportStartTimeout);
            document.getElementById('export-progress-text').textContent = t('msg-exporting');
        });
        socket.on('export_progress', d => {
            const pct = Math.min(d.progress || 0, 100);
            document.getElementById('export-progress-bar').style.width = pct.toFixed(1) + '%';
            document.getElementById('export-progress-text').textContent = d.message || (pct.toFixed(0) + '%');
        });
        // Encode progress for selected episodes
        socket.on('encode_progress', d => {
            const pct = Math.min(d.progress || 0, 100);
            document.getElementById('encode-progress-bar').style.width = pct.toFixed(1) + '%';
            document.getElementById('encode-progress-text').textContent = d.message || (pct.toFixed(0) + '%');
        });
        // Export selected progress
        socket.on('export_selected_progress', d => {
            const pct = Math.min(d.progress || 0, 100);
            document.getElementById('export-selected-progress-bar').style.width = pct.toFixed(1) + '%';
            document.getElementById('export-selected-progress-text').textContent = d.message || (pct.toFixed(0) + '%');
        });
        function onExportComplete(data) {
            document.getElementById('export-progress-bar').style.width = '100%';
            setTimeout(() => {
                document.getElementById('export-progress-container').style.display = 'none';
                document.getElementById('export-result-container').style.display = 'block';
                document.getElementById('btn-export').disabled = false;
        
                if (data.success) {
                    document.getElementById('export-result-message').textContent = data.message || (t('msg-export-complete') + ': ' + data.exported_count);
                    document.getElementById('export-result-path').textContent = data.zip_path || '';
                    showToast(t('msg-export-complete'), 'success');
                } else {
                    document.getElementById('export-result-message').textContent = t('msg-export-failed') + (data.message || '');
                    document.getElementById('export-result-path').textContent = '';
                    showToast(t('msg-export-failed') + (data.message || ''), 'error');
                }
            }, 300);
        }
        let videoGridInited = false;
        socket.on('image_update', d => {
            const grid = document.getElementById('video-grid');
            const keys = Object.keys(d).sort();
            if (!keys.length) return;
            if (!videoGridInited) { grid.innerHTML = ''; videoGridInited = true; }
            keys.forEach(k => {
                let img = document.getElementById('cam-' + k);
                if (!img) {
                    const wrap = document.createElement('div');
                    wrap.className = 'video-container';
                    wrap.innerHTML = '<span class="video-label">' + k + '</span>';
                    img = document.createElement('img');
                    img.id = 'cam-' + k;
                    img.alt = k;
                    wrap.appendChild(img);
                    grid.appendChild(wrap);
                }
                img.src = 'data:image/jpeg;base64,' + d[k];
            });
        });
        socket.on('state_update', d => {
            const p = (d.state || {})['observation.state'] || [];
            document.getElementById('state-display').textContent = 'Joint Positions: [' + p.map(v=>v.toFixed(3)).join(', ') + ']';
        });
        socket.on('leader_state_update', d => {
            const p = (d.state || {})['observation.state'] || [];
            document.getElementById('leader-state-display').textContent = 'Joint Positions: [' + p.map(v=>v.toFixed(3)).join(', ') + ']';
        });
        socket.on('record_status', d => {
            document.getElementById('round-number').textContent = d.round_number || 0;
            recordingEnabled = d.recording || false;
            updateUI();
        });
        socket.on('teleop_status', d => { teleopEnabled = d.enabled || false; updateUI(); });
        socket.on('follower_status', d => {
            const el = document.getElementById('follower-online');
            if (d.online) {
                el.textContent = t('follower-online');
                el.className = 'status-value active';
            } else {
                el.textContent = t('follower-offline');
                el.className = 'status-value inactive';
            }
        });
        // Dynamic buffer keys from follower
        let currentBufferKeys = [];
        let renderedBufferKeys = [];  // Track rendered keys to avoid re-rendering
        const defaultSelectedKeys = ['observation.state', 'observation.images.front', 'observation.images.wrist', 'action'];
        socket.on('buffer_keys', d => {
            const keys = d.keys || [];
            // Sort and compare to check if keys changed
            const sortedKeys = [...keys].sort();
            const sortedRendered = [...renderedBufferKeys].sort();
            const keysChanged = JSON.stringify(sortedKeys) !== JSON.stringify(sortedRendered);
            
            if (!keysChanged) {
                // Keys unchanged, only update count
                const statusEl = document.getElementById('keys-status');
                if (keys.length > 0) {
                    statusEl.textContent = `(${keys.length} ${t('msg-keys-available')})`;
                }
                return;
            }
            
            // Keys changed, re-render
            currentBufferKeys = keys;
            renderedBufferKeys = [...keys];
            const container = document.getElementById('record-keys-container');
            const statusEl = document.getElementById('keys-status');
            if (keys.length === 0) {
                container.innerHTML = '<span style="color:#666;font-size:13px;">' + t('keys-empty') + '</span>';
                statusEl.textContent = t('keys-status-waiting');
                return;
            }
            statusEl.textContent = `(${keys.length} ${t('msg-keys-available')})`;
            container.innerHTML = '';
            keys.forEach(key => {
                const isDefault = defaultSelectedKeys.includes(key);
                const isImage = key.startsWith('observation.images.');
                const label = document.createElement('label');
                label.style.cssText = 'display:flex;align-items:center;gap:5px;cursor:pointer;font-size:13px;color:#ccc;';
                const checkbox = document.createElement('input');
                checkbox.type = 'checkbox';
                checkbox.className = 'record-key';
                checkbox.value = key;
                checkbox.checked = isDefault || isImage;
                label.appendChild(checkbox);
                label.appendChild(document.createTextNode(key));
                container.appendChild(label);
            });
        });
        socket.on('export_result', d => { onExportComplete(d); });

        // Data Management Functions
        let currentRecords = { data_collections: [], exported_zips: [] };
        let selectedEpisodes = new Set();

        function formatBytes(bytes) {
            if (bytes === 0) return '0 B';
            const k = 1024;
            const sizes = ['B', 'KB', 'MB', 'GB'];
            const i = Math.floor(Math.log(bytes) / Math.log(k));
            return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
        }

        function refreshRecords() {
            fetch('/api/records').then(r => r.json()).then(d => {
                if (d.success) {
                    currentRecords = d.data;
                    renderRecords();
                    showToast(t('msg-records-refreshed'), 'success');
                } else {
                    showToast(t('msg-refresh-failed') + d.message, 'error');
                }
            }).catch(e => {
                showToast(t('msg-refresh-failed') + e, 'error');
            });
        }

        function renderRecords() {
            const container = document.getElementById('records-container');
            selectedEpisodes.clear();
            updateExportButton();

            // Update disk usage display
            const diskUsageEl = document.getElementById('disk-usage-info');
            const totalSizeEl = document.getElementById('total-size');
            const exportDirInfoEl = document.getElementById('export-dir-info');
            const exportDirPathEl = document.getElementById('export-dir-path');

            if (currentRecords.total_size_formatted) {
                totalSizeEl.textContent = currentRecords.total_size_formatted;
                diskUsageEl.style.display = 'block';
            }
            if (currentRecords.export_dir) {
                exportDirPathEl.textContent = currentRecords.export_dir;
                exportDirInfoEl.style.display = 'block';
            }

            if (currentRecords.data_collections.length === 0 && currentRecords.exported_zips.length === 0) {
                container.innerHTML = '<div style="color: #888; text-align: center; padding: 20px;">' + t('records-no-data') + '</div>';
                return;
            }

            let html = '';

            // Render data collections
            currentRecords.data_collections.forEach(dc => {
                html += `<div class="data-coll-item">
                    <div class="data-coll-header" onclick="toggleDataColl('${dc.data_coll_id}')">
                        <span class="data-coll-name">📁 ${dc.data_coll_id} <span style="color: #888; font-size: 11px;">(${dc.size_formatted || '0 B'})</span></span>
                        <div class="data-coll-actions" onclick="event.stopPropagation()">
                            <button onclick="deleteDataColl('${dc.data_coll_id}')">${t('btn-delete-all')}</button>
                        </div>
                    </div>
                    <div id="dc-${dc.data_coll_id}" style="display: none; padding: 10px;">`;

                dc.task_prompts.forEach(tp => {
                    html += `<div class="task-prompt-item">
                        <div class="task-prompt-name">📂 ${tp.task_prompt}</div>`;

                    tp.episodes.forEach(ep => {
                        const statusClass = ep.status;
                        const statusText = t('status-' + ep.status) || t('status-unknown');
                        const canEncode = ep.status === 'incomplete';
                        const canExport = ep.status === 'complete';
                        const canPlayback = ep.status === 'complete' || ep.status === 'incomplete';
                        const isInvalid = ep.status === 'invalid';
                        const canSelect = !isInvalid;
                        html += `<div class="episode-item">
                            <input type="checkbox" onchange="toggleEpisode('${ep.path}', this.checked)" ${canSelect ? '' : 'disabled'}>
                            <span class="episode-name">${ep.episode_name}</span>
                            <span class="episode-status ${statusClass}">${statusText}</span>
                            <span class="episode-frames">${ep.frames}${t('frames')}</span>
                            <div class="episode-actions">
                                ${canPlayback ? `<button onclick="playbackEpisode('${ep.path}', '${ep.episode_name}')" style="background: #9C27B0;">${t('btn-playback')}</button>` : ''}
                                ${canEncode ? `<button onclick="encodeEpisode('${ep.path}')" style="background: #ffa500;">${t('btn-encode')}</button>` : ''}
                                ${canExport ? `<button onclick="exportEpisode('${ep.path}')" style="background: #4CAF50;">${t('btn-export-ep')}</button>` : ''}
                                <button onclick="deleteEpisode('${ep.path}')">${t('btn-delete')}</button>
                            </div>
                        </div>`;
                    });

                    html += `</div>`;
                });

                html += `</div></div>`;
            });

            // Render exported zips
            if (currentRecords.exported_zips.length > 0) {
                const exportDir = currentRecords.export_dir || '';
                html += `<div class="exported-zips-section">
                    <div style="margin-bottom: 8px;">
                        <span style="color: #888; font-size: 13px;">${t('label-exported-files')}</span>
                        <span style="color: #555; font-size: 10px; margin-left: 6px;">${t('label-follower-machine')}</span>
                    </div>
                    <div style="color: #666; font-size: 11px; font-family: monospace; padding: 4px 8px; background: #1a1a2e; border-radius: 4px; margin-bottom: 8px; word-break: break-all;">${exportDir}</div>`;
                currentRecords.exported_zips.forEach(zip => {
                    html += `<div class="exported-zip-item" title="${zip.path}">
                        <span class="zip-name">${zip.name}</span>
                        <span class="zip-size">${formatBytes(zip.size)}</span>
                    </div>`;
                });
                html += `</div>`;
            }

            container.innerHTML = html;
        }

        function toggleDataColl(dcId) {
            const el = document.getElementById('dc-' + dcId);
            if (el) {
                el.style.display = el.style.display === 'none' ? 'block' : 'none';
            }
        }

        function toggleEpisode(path, checked) {
            if (checked) {
                selectedEpisodes.add(path);
            } else {
                selectedEpisodes.delete(path);
            }
            updateExportButton();
        }

        function updateExportButton() {
            const hasSelection = selectedEpisodes.size > 0;
            // Both buttons enabled when any episode is selected
            // The backend will handle filtering (skip already encoded for encode, skip raw for export)
            document.getElementById('btn-export-selected').disabled = !hasSelection;
            document.getElementById('btn-encode-selected').disabled = !hasSelection;
        }

        // Helper to get episode status from path
        function getEpisodeStatus(epPath) {
            for (const dc of currentRecords.data_collections || []) {
                for (const tp of dc.task_prompts || []) {
                    for (const ep of tp.episodes || []) {
                        if (ep.path === epPath) {
                            return ep.status;
                        }
                    }
                }
            }
            return 'unknown';
        }

        function deleteEpisode(path) {
            if (!confirm(t('msg-confirm-delete'))) return;
            fetch('/api/records/delete', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({path: path})
            }).then(r => r.json()).then(d => {
                const result = d.result || {};
                if (d.success && result.success) {
                    showToast(t('msg-deleted'), 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-delete-failed') + (result.message || d.message), 'error');
                }
            });
        }

        function deleteDataColl(dataCollId) {
            if (!confirm(t('msg-confirm-delete-coll'))) return;
            fetch('/api/records/delete_coll', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({data_coll_id: dataCollId})
            }).then(r => r.json()).then(d => {
                const result = d.result || {};
                if (d.success && result.success) {
                    showToast(t('msg-coll-deleted'), 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-delete-failed') + (result.message || d.message), 'error');
                }
            });
        }

        function encodeEpisode(path) {
            if (!confirm(t('msg-confirm-encode'))) return;

            showToast(t('msg-encoding'), 'info');
            fetch('/api/records/encode', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({episode_paths: [path]})
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    const encoded = d.result?.encoded || [];
                    showToast(t('msg-encode-success') + encoded.length, 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-encode-failed') + d.message, 'error');
                }
            }).catch(e => {
                showToast(t('msg-encode-failed') + e, 'error');
            });
        }

        function exportEpisode(path) {
            if (!confirm(t('msg-confirm-export-ep'))) return;

            showToast(t('msg-exporting-ep'), 'info');
            fetch('/api/records/export', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({episode_paths: [path]})
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    const exported = d.result?.exported || [];
                    const zipPath = d.result?.zip_path || '';
                    showToast(t('msg-export-ep-success') + exported.length, 'success');
                    if (zipPath) {
                        showToast('ZIP: ' + zipPath, 'info', 5001);
                    }
                    refreshRecords();
                } else {
                    showToast(t('msg-export-ep-failed') + d.message, 'error');
                }
            }).catch(e => {
                showToast(t('msg-export-ep-failed') + e, 'error');
            });
        }

        function encodeSelectedEpisodes() {
            if (selectedEpisodes.size === 0) {
                showToast(t('msg-select-one'), 'error');
                return;
            }
            const allPaths = Array.from(selectedEpisodes);
            const paths = allPaths.filter(p => getEpisodeStatus(p) === 'incomplete');
            const skipped = allPaths.length - paths.length;

            if (paths.length === 0) {
                showToast(t('msg-no-raw'), 'error');
                return;
            }

            let confirmMsg = t('msg-confirm-encode-n').replace('{n}', paths.length);
            if (skipped > 0) {
                confirmMsg += t('msg-skip-encoded').replace('{n}', skipped);
            }
            if (!confirm(confirmMsg)) return;

            document.getElementById('btn-encode-selected').disabled = true;
            document.getElementById('encode-progress-container').style.display = 'block';
            document.getElementById('encode-progress-bar').style.width = '0%';
            document.getElementById('encode-progress-text').textContent = t('playback-preparing');
            showToast(t('msg-encoding'), 'info');

            fetch('/api/records/encode', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({episode_paths: paths})
            }).then(r => r.json()).then(d => {
                document.getElementById('btn-encode-selected').disabled = false;
                document.getElementById('encode-progress-container').style.display = 'none';
                if (d.success) {
                    const encoded = d.result?.encoded || [];
                    const failed = d.result?.failed || [];
                    showToast(t('msg-encode-done').replace('{ok}', encoded.length).replace('{fail}', failed.length), 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-encode-failed') + d.message, 'error');
                }
            }).catch(e => {
                document.getElementById('btn-encode-selected').disabled = false;
                document.getElementById('encode-progress-container').style.display = 'none';
                showToast(t('msg-encode-failed') + e, 'error');
            });
        }

        function exportSelectedEpisodes() {
            if (selectedEpisodes.size === 0) {
                showToast(t('msg-select-one'), 'error');
                return;
            }
            const allPaths = Array.from(selectedEpisodes);
            const paths = allPaths.filter(p => getEpisodeStatus(p) !== 'invalid');
            const skipped = allPaths.length - paths.length;

            if (paths.length === 0) {
                showToast(t('msg-invalid-data'), 'error');
                return;
            }

            if (skipped > 0) {
                showToast(t('msg-skip-invalid').replace('{n}', skipped), 'info');
            }

            if (!confirm(t('msg-confirm-export-n').replace('{n}', paths.length))) return;

            document.getElementById('btn-export-selected').disabled = true;
            document.getElementById('export-selected-progress-container').style.display = 'block';
            document.getElementById('export-selected-progress-bar').style.width = '0%';
            document.getElementById('export-selected-progress-text').textContent = t('playback-preparing');
            showToast(t('msg-exporting-ep'), 'info');

            fetch('/api/records/export', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({episode_paths: paths})
            }).then(r => r.json()).then(d => {
                document.getElementById('btn-export-selected').disabled = false;
                document.getElementById('export-selected-progress-container').style.display = 'none';
                if (d.success) {
                    showToast(t('msg-export-ep-success') + d.result.zip_path, 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-export-ep-failed') + d.message, 'error');
                }
            }).catch(e => {
                document.getElementById('btn-export-selected').disabled = false;
                document.getElementById('export-selected-progress-container').style.display = 'none';
                showToast(t('msg-export-ep-failed') + e, 'error');
            });
        }

        // Playback Functions
        let currentPlaybackEpisode = '';
        let playbackCheckInterval = null;

        function playbackEpisode(path, episodeName) {
            if (teleopEnabled) {
                showToast(t('msg-stop-teleop-first'), 'error');
                return;
            }

            if (!confirm(t('msg-confirm-playback').replace('{name}', episodeName))) {
                return;
            }

            currentPlaybackEpisode = episodeName;
            document.getElementById('playback-episode-name').textContent = episodeName;
            document.getElementById('playback-progress-container').style.display = 'block';
            document.getElementById('playback-progress-bar').style.width = '0%';
            document.getElementById('playback-progress-text').textContent = '0%';
            document.getElementById('playback-status-text').textContent = t('playback-preparing');

            fetch('/api/playback/start', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({episode_path: path})
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    showToast(t('msg-playback-started'), 'success');
                    startPlaybackStatusPolling();
                } else {
                    showToast(t('msg-playback-failed') + d.message, 'error');
                    document.getElementById('playback-progress-container').style.display = 'none';
                }
            }).catch(e => {
                showToast(t('msg-playback-failed') + e, 'error');
                document.getElementById('playback-progress-container').style.display = 'none';
            });
        }

        function stopPlayback() {
            fetch('/api/playback/stop', {method: 'POST'})
                .then(r => r.json())
                .then(d => {
                    if (d.success) {
                        showToast(t('msg-playback-stopped'), 'info');
                    }
                    stopPlaybackStatusPolling();
                    document.getElementById('playback-progress-container').style.display = 'none';
                })
                .catch(e => {
                    showToast(t('msg-playback-stop-failed') + e, 'error');
                });
        }

        function startPlaybackStatusPolling() {
            if (playbackCheckInterval) clearInterval(playbackCheckInterval);
            playbackCheckInterval = setInterval(() => {
                fetch('/api/playback/status')
                    .then(r => r.json())
                    .then(d => {
                        if (d.success && d.data) {
                            updatePlaybackUI(d.data);
                            // Check if playback is complete or stopped
                            if (['completed', 'stopped', 'error'].includes(d.data.status)) {
                                stopPlaybackStatusPolling();
                                if (d.data.status === 'completed') {
                                    showToast(t('msg-playback-done'), 'success');
                                    setTimeout(() => {
                                        document.getElementById('playback-progress-container').style.display = 'none';
                                    }, 2000);
                                } else if (d.data.status === 'error') {
                                    showToast(t('msg-playback-error') + d.data.message, 'error');
                                }
                            }
                        }
                    })
                    .catch(e => console.error('Playback status poll error:', e));
            }, 500);  // Poll every 500ms
        }

        function stopPlaybackStatusPolling() {
            if (playbackCheckInterval) {
                clearInterval(playbackCheckInterval);
                playbackCheckInterval = null;
            }
        }

        function updatePlaybackUI(data) {
            const progress = data.progress || 0;
            const status = data.status || 'unknown';
            const message = data.message || '';

            document.getElementById('playback-progress-bar').style.width = progress + '%';
            document.getElementById('playback-progress-text').textContent = progress + '%';

            const statusMap = {
                'loading': t('playback-loading'),
                'pre_move': t('playback-pre-move'),
                'waiting': t('playback-waiting'),
                'playing': t('playback-playing'),
                'completed': t('playback-completed'),
                'stopped': t('playback-stopped'),
                'error': t('playback-error-prefix') + message,
            };
            document.getElementById('playback-status-text').textContent = statusMap[status] || message;
        }

        // Listen for playback status updates via WebSocket
        socket.on('playback_status', d => {
            updatePlaybackUI(d);
        });

        applyLang();
        updateUI();
    </script>
</body>
</html>
'''


class TeleopWebUI:
    """
    Web UI server for teleoperation control.

    Provides a Flask + WebSocket server that runs in a background thread and
    allows browser-based control of the TeleopPlugin.
    """

    def __init__(
        self,
        plugin: "TeleopPlugin",
        host: str = "0.0.0.0",
        port: int = 5001,
        open_browser: bool = True,
        quit_callback: Optional[Callable[[], None]] = None,
    ) -> None:
        """
        Initialize the Web UI server.

        :param plugin: The TeleopPlugin instance to control.
        :param host: Host to bind the server to.
        :param port: Port to bind the server to.
        :param open_browser: Whether to open a browser on startup.
        :param quit_callback: Callback to invoke when the user clicks "Quit".
        """
        self._plugin = plugin
        self._host = host
        self._port = port
        self._open_browser = open_browser
        self._quit_callback = quit_callback
        self._stop_event = threading.Event()
        self._server_thread: Optional[threading.Thread] = None
        self._pusher_thread: Optional[threading.Thread] = None

        # Create Flask app
        self._app = Flask(__name__)
        self._app.config["SECRET_KEY"] = "teleop-secret"
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

        @self._app.route("/api/config", methods=["GET"])
        def api_config():
            return jsonify({"data_coll_id": self._plugin._data_coll_id})

        @self._app.route("/api/teleop/start", methods=["POST"])
        def api_start_teleop():
            self._plugin.start_teleop()
            self._socketio.emit("teleop_status", {"enabled": True})
            return jsonify({"success": True})

        @self._app.route("/api/teleop/stop", methods=["POST"])
        def api_stop_teleop():
            self._plugin.stop_teleop()
            self._socketio.emit("teleop_status", {"enabled": False})
            return jsonify({"success": True})

        @self._app.route("/api/record/start", methods=["POST"])
        def api_start_record():
            if not self._plugin.teleop_enabled:
                return jsonify({"success": False, "message": "请先开始遥操"})
            body = request.get_json(silent=True) or {}
            ok = self._plugin.start_recording(
                task_prompt=body.get("task_prompt", "teleop demo"),
                task_description=body.get("task_description", ""),
                fps=float(body.get("fps", 30)),
                data_coll_id=body.get("data_coll_id", "teleop"),
                keys=body.get("keys"),
            )
            if ok:
                return jsonify({"success": True, "round": self._plugin._current_round})
            return jsonify({"success": False, "message": "启动录制失败"})

        @self._app.route("/api/record/stop", methods=["POST"])
        def api_stop_record():
            result = self._plugin.stop_recording()
            return jsonify({
                "success": True,
                "frames_written": result.get("frames_written", 0),
                "round": self._plugin._current_round,
            })

        @self._app.route("/api/record/discard", methods=["POST"])
        def api_discard_record():
            self._plugin.discard_last_recording()
            return jsonify({"success": True})

        @self._app.route("/api/data/export", methods=["POST"])
        def api_export_data():
            body = request.get_json(silent=True) or {}
            data_coll_id = body.get("data_coll_id") or self._plugin._data_coll_id
            result = self._plugin.export_data(data_coll_id=data_coll_id)
            return jsonify(result)

        @self._app.route("/api/quit", methods=["POST"])
        def api_quit():
            self._stop_event.set()
            if self._quit_callback:
                threading.Thread(target=self._quit_callback, daemon=True).start()
            return jsonify({"success": True})

        # Data Management APIs
        @self._app.route("/api/records", methods=["GET"])
        def api_get_records():
            """Get all local data collections and exported zips."""
            try:
                data = self._plugin.scan_local_records()
                return jsonify({"success": True, "data": data})
            except Exception as e:
                logger.error(f"[WebUI] scan_local_records error: {e}")
                return jsonify({"success": False, "message": str(e)})

        @self._app.route("/api/records/delete", methods=["POST"])
        def api_delete_record():
            """Delete a specific episode."""
            body = request.get_json(silent=True) or {}
            path = body.get("path")
            if not path:
                return jsonify({"success": False, "message": "path required"})
            result = self._plugin.delete_episode(path)
            return jsonify(result)

        @self._app.route("/api/records/delete_coll", methods=["POST"])
        def api_delete_data_coll():
            """Delete an entire data collection."""
            body = request.get_json(silent=True) or {}
            data_coll_id = body.get("data_coll_id")
            if not data_coll_id:
                return jsonify({"success": False, "message": "data_coll_id required"})
            result = self._plugin.delete_data_collection(data_coll_id)
            return jsonify(result)

        @self._app.route("/api/records/export", methods=["POST"])
        def api_export_records():
            """Export selected episodes with progress callback."""
            body = request.get_json(silent=True) or {}
            episode_paths = body.get("episode_paths", [])
            zip_name = body.get("zip_name")
            if not episode_paths:
                return jsonify({"success": False, "message": "episode_paths required"})

            # Progress callback that emits via WebSocket
            def progress_callback(current: float, total: float, message: str):
                self._socketio.emit("export_progress", {
                    "progress": current,
                    "total": total,
                    "message": message,
                })

            result = self._plugin.export_episodes(
                episode_paths, zip_name, _progress_callback=progress_callback
            )
            return jsonify(result)

        @self._app.route("/api/records/encode", methods=["POST"])
        def api_encode_records():
            """Encode (export in-place) selected episodes with progress callback."""
            body = request.get_json(silent=True) or {}
            episode_paths = body.get("episode_paths", [])
            if not episode_paths:
                return jsonify({"success": False, "message": "episode_paths required"})

            # Progress callback that emits via WebSocket
            def progress_callback(current: float, total: float, message: str):
                self._socketio.emit("encode_progress", {
                    "progress": current,
                    "total": total,
                    "message": message,
                })

            result = self._plugin.encode_episodes(
                episode_paths, _progress_callback=progress_callback
            )
            return jsonify(result)

        # Playback APIs
        @self._app.route("/api/playback/start", methods=["POST"])
        def api_start_playback():
            """Start episode playback on the real robot."""
            body = request.get_json(silent=True) or {}
            episode_path = body.get("episode_path")
            if not episode_path:
                return jsonify({"success": False, "message": "episode_path required"})

            # Check if teleop is enabled
            if self._plugin.teleop_enabled:
                return jsonify({"success": False, "message": "请先停止遥操再开始回放"})

            # Check if playback already in progress
            if self._plugin.playback_in_progress:
                return jsonify({"success": False, "message": "回放正在进行中"})

            result = self._plugin.start_playback(episode_path)
            return jsonify({"success": result})

        @self._app.route("/api/playback/stop", methods=["POST"])
        def api_stop_playback():
            """Stop episode playback."""
            result = self._plugin.stop_playback()
            return jsonify({"success": result})

        @self._app.route("/api/playback/status", methods=["GET"])
        def api_get_playback_status():
            """Get current playback status."""
            status = self._plugin.get_playback_status()
            return jsonify({"success": True, "data": status})

    def _push_data_loop(self) -> None:
        """Push images and state to connected clients via WebSocket."""
        last_image_id = None
        last_state_id = None
        last_leader_state_id = None

        while not self._stop_event.is_set():
            try:
                # Push images
                images = self._plugin.latest_follower_images
                if images and id(images) != last_image_id:
                    last_image_id = id(images)
                    img_data: Dict[str, str] = {}
                    for key, jpg_bytes in images.items():
                        if isinstance(jpg_bytes, (bytes, bytearray)):
                            b64 = base64.b64encode(jpg_bytes).decode("ascii")
                            img_data[key] = b64
                    self._socketio.emit("image_update", img_data)

                # Push state
                state = self._plugin.latest_follower_state
                if state and id(state) != last_state_id:
                    last_state_id = id(state)
                    self._socketio.emit("state_update", {"state": state})

                # Push leader's own state
                leader_state = self._plugin.latest_leader_state
                if leader_state and id(leader_state) != last_leader_state_id:
                    last_leader_state_id = id(leader_state)
                    self._socketio.emit("leader_state_update", {"state": leader_state})

                # Push record status
                status = self._plugin.record_status
                self._socketio.emit("record_status", status)

                # Push follower online status (heartbeat received within last 3 seconds)
                last_hb = self._plugin._last_heartbeat_time
                is_online = last_hb is not None and (time.time() - last_hb) < 3.0
                self._socketio.emit("follower_status", {"online": is_online})

                # Push available buffer keys from follower
                buffer_keys = self._plugin.available_buffer_keys
                if buffer_keys:
                    self._socketio.emit("buffer_keys", {"keys": buffer_keys})

                # Push export started event if follower just acked
                export_started_time = self._plugin._export_started_time
                if export_started_time is not None:
                    self._socketio.emit("export_started", {})

                # Push export progress if in progress
                export_progress = self._plugin._export_progress
                if export_progress is not None:
                    self._socketio.emit("export_progress", export_progress)

                # Push export result if available
                if self._plugin._export_result is not None:
                    self._socketio.emit("export_result", self._plugin._export_result)
                    # Clear after sending to avoid duplicate
                    self._plugin._export_result = None
                    self._plugin._export_in_progress = False

                # Push playback status if in progress
                if self._plugin.playback_in_progress:
                    playback_status = self._plugin.get_playback_status()
                    self._socketio.emit("playback_status", playback_status)

                time.sleep(0.05)  # 20 Hz
            except Exception as e:
                logger.error(f"[TeleopWebUI] push_data error: {e}")
                time.sleep(0.5)

    @staticmethod
    def _is_port_in_use(port: int) -> bool:
        """Check if a TCP port is already in use by another process.

        Sets SO_REUSEADDR to match Flask/Werkzeug behaviour so that
        sockets in TIME_WAIT (left over from a previous run) do not
        cause a false positive.
        """
        import socket as _socket
        with _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM) as s:
            s.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
            try:
                s.bind(("0.0.0.0", port))
                return False
            except OSError:
                return True

    def start(self) -> None:
        """Start the web server in a background thread."""
        if self._server_thread is not None:
            logger.warning("[TeleopWebUI] Server already running.")
            return

        if self._is_port_in_use(self._port):
            raise OSError(
                f"[TeleopWebUI] 端口 {self._port} 已被占用，Web UI 无法启动。"
                f"请修改 web_port 参数为其他可用端口（如 5001、8080 等），然后重新运行。"
            )

        self._stop_event.clear()

        # Start data pusher thread
        self._pusher_thread = threading.Thread(
            target=self._push_data_loop, daemon=True, name="teleop-web-pusher"
        )
        self._pusher_thread.start()

        # Start Flask server thread
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
                logger.error(f"[TeleopWebUI] Server error: {e}")

        self._server_thread = threading.Thread(
            target=run_server, daemon=True, name="teleop-web-server"
        )
        self._server_thread.start()

        logger.info(
            f"[TeleopWebUI] Web UI started at http://{self._host}:{self._port}"
        )

        # Open browser
        if self._open_browser:
            time.sleep(0.5)
            webbrowser.open(f"http://127.0.0.1:{self._port}")

    def stop(self) -> None:
        """Stop the web server."""
        self._stop_event.set()
        logger.info("[TeleopWebUI] Web UI stopped.")
