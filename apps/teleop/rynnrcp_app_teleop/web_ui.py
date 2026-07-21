"""
Web UI server for teleoperation control.

This module provides a Flask + WebSocket server that:
- Renders images and state received from the follower arm
- Provides buttons for teleop control (start/stop/quit)
- Provides buttons for data collection (start/stop recording, export)
- Displays recording status and frame count

The server is started by the standalone Teleop app.
"""

from __future__ import annotations

import threading
import time
import webbrowser
from typing import TYPE_CHECKING, Any, Callable, Dict, Optional

from flask import Flask, Response, jsonify, render_template_string, request
from flask_socketio import SocketIO

import logging
from rynnrcp.utils.web_urls import browser_urls, primary_browser_url

if TYPE_CHECKING:
    from .teleop_app import TeleopApp

logger = logging.getLogger(__name__)


def _browser_url(host: str, port: int) -> str:
    """Resolve a bind host into the URL we should show/open for users."""
    return primary_browser_url(host, port)


def _progress_payload(current: float, total: float, message: str) -> Dict[str, Any]:
    denominator = float(total) if total else 0.0
    progress = 0.0 if denominator <= 0 else min(100.0, max(0.0, (float(current) / denominator) * 100.0))
    return {
        "progress": progress,
        "current": current,
        "total": total,
        "message": message,
    }

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
        .status-value.warning { color: #ffaa00; }
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
        .joint-chart-block { margin-top: 10px; }
        .joint-chart-header {
            display: flex; justify-content: space-between; align-items: baseline;
            gap: 12px; margin-bottom: 8px;
        }
        .joint-chart-raw {
            color: #00ff88; font-family: 'Fira Code', monospace;
            font-size: 12px; white-space: normal; overflow-wrap: anywhere;
            line-height: 1.5; margin: 8px 0 2px;
        }
        .joint-chart-wrap {
            position: relative; height: 150px; background: #202437;
            border: 1px solid rgba(255,255,255,0.08); border-radius: 8px;
            overflow: hidden;
        }
        .joint-chart-wrap canvas { width: 100%; height: 100%; display: block; }
        .joint-chart-empty {
            position: absolute; inset: 0; display: flex; align-items: center; justify-content: center;
            color: #666; font-size: 13px; pointer-events: none;
        }
        .joint-chart-legend {
            display: flex; flex-wrap: wrap; gap: 8px 12px;
            margin: 8px 0 12px; color: #888; font-size: 11px;
        }
        .joint-chart-legend label {
            display: inline-flex; align-items: center; gap: 5px;
            cursor: pointer; user-select: none;
        }
        .joint-chart-legend input { accent-color: var(--joint-color); }
        .joint-chart-swatch {
            display: inline-block; width: 16px; height: 3px; border-radius: 999px;
            background: var(--joint-color);
        }
        .debug-panel {
            display: none; background: #151827; border: 1px solid #3a4560;
            border-radius: 8px; padding: 12px; margin: 12px 0 20px;
        }
        .debug-panel.active { display: block; }
        .debug-toolbar { display: flex; justify-content: space-between; gap: 8px; align-items: center; margin-bottom: 10px; }
        .debug-summary {
            display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; margin-bottom: 10px;
        }
        .debug-summary-item { background: #24283a; border-radius: 6px; padding: 8px 10px; font-size: 12px; }
        .debug-summary-label { color: #888; display: block; margin-bottom: 3px; }
        .debug-summary-value { color: #00d4ff; font-weight: 700; }
        .debug-columns { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; min-height: 260px; }
        .debug-card { background: #202437; border: 1px solid #343d58; border-radius: 8px; padding: 10px; }
        .debug-card h4 { color: #00d4ff; font-size: 13px; margin-bottom: 8px; }
        .debug-card.wide { grid-column: 1 / -1; }
        .debug-row { display: flex; justify-content: space-between; gap: 10px; padding: 4px 0; border-bottom: 1px solid rgba(255,255,255,0.06); font-size: 12px; }
        .debug-row:last-child { border-bottom: none; }
        .debug-key { color: #888; }
        .debug-value { color: #d8e8ff; text-align: right; word-break: break-word; }
        .debug-subtle { color: #888; font-size: 11px; margin-left: 4px; }
        .debug-pill { display: inline-block; padding: 2px 7px; border-radius: 999px; font-size: 11px; font-weight: 700; }
        .debug-pill.ok { background: rgba(0,255,136,0.15); color: #00ff88; }
        .debug-pill.warn { background: rgba(255,170,0,0.18); color: #ffaa00; }
        .debug-pill.bad { background: rgba(255,71,87,0.18); color: #ff6b6b; }
        .debug-raw { margin-top: 10px; }
        .debug-raw summary { cursor: pointer; color: #888; font-size: 12px; margin-bottom: 6px; }
        .debug-pre {
            max-height: 260px; overflow: auto; background: #0d1117; border-radius: 6px;
            padding: 10px; color: #bde5ff; font-size: 11px; line-height: 1.45;
            white-space: pre-wrap; word-break: break-word;
        }
        @media (max-width: 900px) { .debug-columns { grid-template-columns: 1fr; } }
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
        .data-toolbar { display:flex; gap:8px; flex-wrap:wrap; margin-bottom:12px; }
        .data-toolbar .btn { width:auto; min-width:132px; padding:9px 14px; font-size:13px; }
        #export-dir-info {
            margin-bottom:10px; padding:8px 10px; font-size:12px; color:#aaa;
            background:rgba(255,255,255,.04); border:1px solid rgba(255,255,255,.08); border-radius:6px;
            display:none; align-items:center; gap:6px; min-width:0;
        }
        #export-dir-path { color:#00ff88; min-width:0; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
        .records-container { max-height:500px; overflow-y:auto; background:#242438; border-radius:8px; padding:10px; }
        .records-location { margin:12px 0; padding:12px; border:1px solid rgba(255,255,255,.08); border-radius:8px; background:rgba(255,255,255,.03); }
        .records-location-head { display:grid; grid-template-columns:minmax(0,1fr) auto; gap:10px; align-items:start; margin-bottom:10px; }
        .records-location-title { color:#00d4ff; font-size:17px; font-weight:800; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
        .records-location-detail { color:#aaa; font-size:12px; margin-top:3px; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
        .records-location-status { text-align:right; font-size:12px; font-weight:700; white-space:nowrap; }
        .records-section-title { margin:12px 0 7px; color:#999; font-size:12px; font-weight:800; letter-spacing:0; }
        .data-coll-item { margin-bottom:10px; border:1px solid rgba(255,255,255,.12); border-radius:8px; overflow:hidden; background:rgba(255,255,255,.02); }
        .data-coll-header {
            background:#2a3a4e; padding:10px 12px; cursor:pointer;
            display:grid; grid-template-columns:minmax(0,1fr) auto; gap:10px; align-items:center;
        }
        .data-coll-header:hover { background: #3a4a5e; }
        .data-coll-title { min-width:0; }
        .data-coll-name { font-weight:700; color:#00d4ff; min-width:0; overflow-wrap:anywhere; word-break:break-all; line-height:1.2; font-size:16px; }
        .data-coll-meta { margin-top:3px; color:#888; font-size:11px; white-space:nowrap; }
        .data-coll-actions { display:flex; gap:8px; flex-shrink:0; }
        .data-coll-actions button {
            padding:6px 10px; font-size:11px; border:none; border-radius:4px; white-space:nowrap;
            cursor: pointer; background: #ff4757; color: white;
        }
        .task-prompt-item { margin:8px 0; border-left:2px solid #444; padding-left:10px; }
        .task-prompt-name { font-size: 13px; color: #aaa; margin-bottom: 5px; }
        .episode-item {
            display:grid; grid-template-columns:18px minmax(0,1fr) auto auto; align-items:center; gap:8px; padding:7px 9px;
            background: #1e1e2f; margin: 4px 0; border-radius: 4px; font-size: 12px;
        }
        .episode-item:hover { background: #252535; }
        .episode-item input[type="checkbox"] { cursor: pointer; }
        .episode-name { min-width:0; color:#ccc; overflow-wrap:anywhere; word-break:break-all; line-height:1.25; }
        .episode-status {
            padding:2px 8px; border-radius:10px; font-size:10px; white-space:nowrap;
        }
        .episode-status.complete { background: #00cc6a; color: #1a1a2e; }
        .episode-status.incomplete { background: #ffa500; color: #1a1a2e; }
        .episode-status.invalid { background: #ff4757; color: white; }
        .episode-status.unknown { background: #666; color: #ccc; }
        .episode-frames { color: #888; font-size: 11px; }
        .episode-actions { display:flex; gap:6px; flex-shrink:0; }
        .episode-actions button {
            padding:4px 8px; font-size:10px; border:none; border-radius:4px; white-space:nowrap;
            cursor: pointer; background: #ff4757; color: white;
        }
        .exported-zips-section { margin-top:12px; padding-top:12px; border-top:1px solid rgba(255,255,255,.12); }
        .exported-zip-item {
            display:grid; grid-template-columns:minmax(0,1fr) auto; align-items:center; gap:10px;
            padding: 8px 10px; background: #1e1e2f; margin: 4px 0; border-radius: 4px; font-size: 12px;
        }
        .zip-name { color:#00ff88; min-width:0; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
        .zip-size { color: #888; font-size: 11px; }
        .zip-time { color: #777; font-size: 11px; margin-left: 8px; }
        .zip-actions { display: flex; align-items: center; gap: 8px; flex-shrink: 0; }
        .zip-actions button {
            padding:4px 8px; font-size:10px; border:none; border-radius:4px; white-space:nowrap;
            cursor: pointer; background: #ff4757; color: white;
        }
        .log-group-title { margin:10px 0 6px; color:#aaa; font-size:12px; font-weight:800; }
        .log-actions { display:flex; justify-content:flex-end; gap:6px; }
        .log-actions button {
            padding:4px 8px; font-size:10px; border:none; border-radius:4px; white-space:nowrap;
            cursor:pointer; color:white; background:#607D8B;
        }
        @media (max-width: 720px) {
            .data-coll-header, .episode-item, .exported-zip-item, .records-location-head { grid-template-columns:1fr; }
            .data-coll-actions, .episode-actions, .zip-actions { justify-content:flex-start; flex-wrap:wrap; }
            .records-location-status { text-align:left; }
        }
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
        .header-actions {
            position: absolute; left: 20px; top: 50%;
            transform: translateY(-50%);
            display: flex; align-items: center; gap: 8px;
        }
        .header-actions .btn { padding: 7px 14px; font-size: 12px; }
        .preview-key-row { display:flex; flex-wrap:wrap; gap:6px; margin:6px 0 10px; }
        .preview-key-chip {
            display:inline-flex; align-items:center; gap:5px; padding:4px 8px;
            border:1px solid #444; border-radius:6px; background:#1e1e2f;
            color:#bbb; font-size:11px; cursor:pointer; user-select:none;
        }
        .preview-key-chip input { margin:0; accent-color:#00d4ff; }
        .preview-key-chip:has(input:checked) { border-color:#00d4ff; color:#e8e8e8; background:#24364a; }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <div class="header-actions">
                <button id="btn-quit" class="btn btn-danger" onclick="quitTeleop()" data-i18n="btn-quit">⏹ 退出</button>
            </div>
            <h1 data-i18n="header-title">RynnRCP 遥操数采平台</h1>
            <button class="lang-toggle" id="lang-toggle" onclick="toggleLang()"><span id="lang-hint" style="font-size:11px;color:#888;font-weight:400;">Switch to </span>EN</button>
        </header>
        <div class="main-content">
            <div class="video-section">
                <h3 class="section-title" data-i18n="section-follower-view">执行端视野</h3>
                <div id="target-image-key-selector" class="preview-key-row"></div>
                <div class="video-grid" id="video-grid">
                    <div class="video-container" style="display:flex;align-items:center;justify-content:center;min-height:200px;">
                        <span style="color:#555;" data-i18n="waiting-image">等待图像数据...</span>
                    </div>
                </div>
                <div class="state-display">
                    <div class="joint-chart-block">
                        <div class="joint-chart-header">
                            <h4 style="color: #888;" data-i18n="follower-state-title">执行端状态</h4>
                        </div>
                        <div id="target-state-key-selector" class="preview-key-row"></div>
                        <div id="follower-state-cards" style="display:flex;flex-direction:column;gap:10px;"></div>
                    </div>
                    <div class="joint-chart-block">
                        <div class="joint-chart-header">
                            <h4 style="color: #888;" data-i18n="leader-state-title">控制端状态</h4>
                        </div>
                        <div id="source-state-key-selector" class="preview-key-row"></div>
                        <div id="leader-state-cards" style="display:flex;flex-direction:column;gap:10px;"></div>
                    </div>
                </div>
            </div>
            <div class="control-panel">
                <h3 class="section-title" data-i18n="section-robot-binding">设备选择</h3>
                <div class="status-grid" style="margin-bottom: 15px;">
                    <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:6px;">
                        <label class="status-label" data-i18n="label-leader-robot">控制端</label>
                        <select id="source-server-select"
                                onchange="renderControlFlows()"
                                style="width:100%;padding:6px 10px;border-radius:6px;border:1px solid #444;background:#1e1e2f;color:#e8e8e8;font-size:13px;"></select>
                    </div>
                    <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:6px;">
                        <label class="status-label" data-i18n="label-follower-robot">执行端</label>
                        <select id="target-server-select"
                                onchange="renderControlFlows()"
                                style="width:100%;padding:6px 10px;border-radius:6px;border:1px solid #444;background:#1e1e2f;color:#e8e8e8;font-size:13px;"></select>
                    </div>
                </div>
                <div class="button-group" style="grid-template-columns: 1fr; gap: 8px;">
                    <button class="btn btn-primary" onclick="discoverServers()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-refresh-devices">🔎 刷新设备</button>
                </div>
                <h3 class="section-title" data-i18n="section-control-flow">控制流</h3>
                <div id="control-flow-list" style="display:flex;flex-direction:column;gap:8px;margin-bottom:10px;"></div>
                <div class="button-group" style="grid-template-columns: 1fr; gap: 8px;">
                    <button class="btn btn-primary" onclick="addControlFlow()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-add-control-flow">＋ 添加控制流</button>
                </div>
                <h3 class="section-title" data-i18n="section-status">状态监控</h3>
                <div class="status-grid" style="grid-template-columns: repeat(2, 1fr);">
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-teleop-status">遥操状态</span>
                        <span id="teleop-status" class="status-value inactive" data-i18n="teleop-off">未启动</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-follower-online">执行端在线</span>
                        <span id="follower-online" class="status-value inactive" data-i18n="follower-offline">离线</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-source-conn">控制端连接</span>
                        <span id="source-connection" class="status-value inactive">未绑定</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-target-conn">执行端连接</span>
                        <span id="target-connection" class="status-value inactive">未绑定</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-record-status">数采状态</span>
                        <span id="record-status" class="status-value inactive" data-i18n="record-off">未录制</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-episode">当前 Episode</span>
                        <span id="episode-number" class="status-value">0</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-command-fps">控制指令帧率</span>
                        <span id="command-fps" class="status-value">0.0 Hz</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-preview-fps">执行端图像帧率</span>
                        <span id="preview-fps" class="status-value">0.0 Hz</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-transport">通信方式</span>
                        <span id="transport-mode" class="status-value">-</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label" data-i18n="label-control-loop">控制循环（含状态等待）</span>
                        <span id="action-latency" class="status-value">-</span>
                    </div>
                    <div class="status-item" style="grid-column: 1 / -1;">
                        <span class="status-label">链路耗时</span>
                        <span id="rpc-latency-summary" class="status-value">-</span>
                    </div>
                    <div class="status-item" style="grid-column: 1 / -1;">
                        <span class="status-label">状态错误</span>
                        <span id="runtime-last-error" class="status-value inactive">-</span>
                    </div>
                </div>
                <h3 class="section-title" data-i18n="section-teleop">遥操控制</h3>
                <div class="button-group" style="grid-template-columns: repeat(2, 1fr); gap: 8px;">
                    <button id="btn-start-teleop" class="btn btn-success" onclick="startTeleop()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-start-teleop">▶ 开始遥操</button>
                    <button id="btn-stop-teleop" class="btn btn-warning" onclick="stopTeleop()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-stop-teleop">⏸ 停止遥操</button>
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
                        <label class="status-label" data-i18n="label-export-fps">导出 FPS</label>
                        <input id="input-fps" type="number" value="30" min="1" max="120" step="1"
                               style="width:100%;padding:6px 10px;border-radius:6px;border:1px solid #444;background:#1e1e2f;color:#e8e8e8;font-size:13px;">
                    </div>
                    <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:4px;">
                        <label class="status-label" data-i18n="label-rynnbot-mapping">RynnBot 映射</label>
                        <label style="display:flex;align-items:center;gap:8px;color:#d8d8d8;font-size:13px;">
                            <input id="input-rynnbot-mapping" type="checkbox" checked>
                            <span data-i18n="label-rynnbot-mapping-on">导出兼容 key</span>
                        </label>
                    </div>
                    <div class="status-item" style="flex-direction:column;align-items:flex-start;gap:4px;">
                        <label class="status-label" data-i18n="label-max-record-min">最大采集时长（分钟）</label>
                        <input id="input-max-record-min" type="number" value="10" min="1" max="240" step="1"
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
                    <label class="status-label"><span data-i18n="label-data-keys">采集数据项</span> <span id="keys-status" style="color:#888;font-size:11px;" data-i18n="keys-status-waiting">(等待执行端数据...)</span></label>
                    <div id="record-keys-container" style="width:100%;">
                        <span style="color:#666;font-size:13px;" data-i18n="keys-empty">等待执行端发送可用数据项...</span>
                    </div>
                </div>
                <div class="button-group" style="grid-template-columns: repeat(3, 1fr); gap: 8px;">
                    <button id="btn-start-record" class="btn btn-success" onclick="startRecording()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-start-record">⏺ 开始数采</button>
                    <button id="btn-stop-record" class="btn btn-warning" onclick="stopRecording()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-stop-record">⏹ 停止数采</button>
                    <button id="btn-export" class="btn btn-primary" onclick="exportData()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-export">📦 导出数据</button>
                </div>
                <div class="workflow-hint">
                    <h4 data-i18n="workflow-title">📋 操作流程</h4>
                    <ol>
                        <li id="step1" data-i18n="step1">点击"开始遥操"启动控制端到执行端通信</li>
                        <li id="step2" data-i18n="step2">点击"开始数采"开始录制</li>
                        <li id="step3" data-i18n="step3">操作机械臂执行任务</li>
                        <li id="step4" data-i18n="step4">点击"停止数采"结束录制</li>
                        <li id="step5" data-i18n="step5">重复2-4采集多条数据</li>
                        <li id="step6" data-i18n="step6">在数据管理中查看或回放采集结果</li>
                    </ol>
                </div>

                <!-- Data Management Section -->
                <h3 class="section-title" style="margin-top: 25px;" data-i18n="section-datamgmt">📁 数据管理</h3>
                <div id="disk-usage-info" style="background: #2a3a4e; padding: 10px 15px; border-radius: 8px; margin-bottom: 15px; display: none;">
                    <div style="display:flex;flex-direction:column;gap:6px;">
                        <span style="color: #888; font-size: 13px;" data-i18n="disk-usage-label">💾 磁盘占用</span>
                        <div id="disk-usage-breakdown" style="display:flex;flex-direction:column;gap:4px;"></div>
                    </div>
                </div>
                <div class="data-toolbar">
                    <button id="btn-refresh-records" class="btn btn-primary" onclick="refreshRecords()" data-i18n="btn-refresh">🔄 刷新列表</button>
                    <button id="btn-export-selected" class="btn btn-success" onclick="exportSelectedEpisodes()" disabled data-i18n="btn-export-selected">📦 导出选中</button>
                </div>
                <div id="export-dir-info">
                    <span data-i18n="export-dir-label">📂 导出位置（执行端）:</span> <span id="export-dir-path"></span>
                </div>
                <div id="export-selected-progress-container" style="display:none; margin-bottom: 15px; padding: 15px; background: #2a3a4e; border-radius: 8px;">
                    <span id="export-selected-progress-text" style="color: #00d4ff; font-size: 12px;" data-i18n="processing">处理中</span>
                </div>
                <!-- Playback Progress -->
                <div id="playback-progress-container" style="display:none; margin-bottom: 15px; padding: 15px; background: #3a2a4e; border-radius: 8px; border: 1px solid #9C27B0;">
                    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom: 8px;">
                        <span id="playback-title" style="color: #E1BEE7; font-size: 13px;" data-i18n="playback-label">▶ 真机回放中...</span>
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
                <div id="records-container" class="records-container">
                    <div style="color: #888; text-align: center; padding: 20px;" data-i18n="records-empty">点击"刷新列表"加载本地数据</div>
                </div>
                <div id="debug-root" style="display:none;">
                    <h3 class="section-title" style="margin-top: 25px;" data-i18n="section-debug">Debug 排查</h3>
                    <div class="button-group" style="grid-template-columns: repeat(2, 1fr); gap: 8px; margin-bottom: 12px;">
                        <button id="btn-debug-toggle" class="btn btn-primary" onclick="toggleDebugPanel()" style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-debug-open">🧪 开启 Debug</button>
                        <button id="btn-debug-refresh" class="btn btn-warning" onclick="refreshDebugStatus()" disabled style="padding: 10px 14px; font-size: 14px;" data-i18n="btn-debug-refresh">刷新 Debug</button>
                    </div>
                    <div id="debug-panel" class="debug-panel">
                        <div class="debug-toolbar">
                            <span style="color:#00d4ff;font-weight:700;font-size:13px;" data-i18n="debug-title">Runtime Debug</span>
                            <span id="debug-updated-at" style="color:#888;font-size:11px;"></span>
                        </div>
                        <div class="debug-summary">
                            <div class="debug-summary-item">
                                <span class="debug-summary-label" data-i18n="debug-local-role">本端角色</span>
                                <span id="debug-local-role" class="debug-summary-value">-</span>
                            </div>
                            <div class="debug-summary-item">
                                <span class="debug-summary-label" data-i18n="debug-peer-role">对端角色</span>
                                <span id="debug-peer-role" class="debug-summary-value">-</span>
                            </div>
                            <div class="debug-summary-item">
                                <span class="debug-summary-label" data-i18n="debug-local-runner">本端 Runner</span>
                                <span id="debug-local-runner" class="debug-summary-value">-</span>
                            </div>
                            <div class="debug-summary-item">
                                <span class="debug-summary-label" data-i18n="debug-peer-runner">对端 Runner</span>
                                <span id="debug-peer-runner" class="debug-summary-value">-</span>
                            </div>
                        </div>
                        <div id="debug-parsed" class="debug-columns">
                            <div class="debug-card"><h4 data-i18n="debug-waiting">点击刷新 Debug 查看 runtime/device 状态。</h4></div>
                        </div>
                        <details class="debug-raw">
                            <summary data-i18n="debug-raw-json">原始 JSON</summary>
                            <pre id="debug-json" class="debug-pre">{}</pre>
                        </details>
                    </div>
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
                'section-follower-view': '执行端视野',
                'waiting-image': '等待图像数据...',
                'section-robot-binding': '设备选择',
                'btn-refresh-devices': '🔎 刷新设备',
                'section-control-flow': '控制流',
                'btn-add-control-flow': '＋ 添加控制流',
                'btn-remove-control-flow': '删除',
                'label-source-obs': '控制源 Observation',
                'label-target-action': '执行目标 Action',
                'control-flow-empty': '请添加至少一条控制流',
                'label-leader-robot': '控制端',
                'label-follower-robot': '执行端',
                'follower-state-title': '执行端状态',
                'leader-state-title': '控制端状态',
                'waiting-data': '等待数据...',
                'section-status': '状态监控',
                'label-teleop-status': '遥操状态',
                'label-follower-online': '执行端在线',
                'label-source-conn': '控制端连接',
                'label-target-conn': '执行端连接',
                'label-record-status': '数采状态',
                'label-episode': '当前 Episode',
                'label-command-fps': '控制指令帧率',
                'label-control-loop': '控制循环（含状态等待）',
                'label-transport': '通信方式',
                'label-preview-fps': '执行端图像帧率',
                'label-export-fps': '导出 FPS',
                'label-rynnbot-mapping': 'RynnBot 映射',
                'label-rynnbot-mapping-on': '导出兼容 key',
                'section-debug': 'Debug 排查',
                'btn-debug-open': '🧪 开启 Debug',
                'btn-debug-close': '关闭 Debug',
                'btn-debug-refresh': '刷新 Debug',
                'debug-title': 'Runtime Debug',
                'debug-local-role': '本端角色',
                'debug-peer-role': '对端角色',
                'debug-local-runner': '本端 Runner',
                'debug-peer-runner': '对端 Runner',
                'debug-waiting': '点击刷新 Debug 查看 runtime/device 状态。',
                'debug-loading': '正在读取 Debug 状态...',
                'debug-load-failed': 'Debug 读取失败: ',
                'debug-raw-json': '原始 JSON',
                'teleop-on': '运行中', 'teleop-off': '未启动',
                'follower-online': '在线', 'follower-offline': '离线',
                'conn-unbound': '未绑定',
                'conn-online': '在线',
                'conn-offline': '离线',
                'conn-reconnectable': '可重连',
                'conn-reconnecting': '重连中',
                'record-on': '录制中...', 'record-off': '未录制',
                'section-teleop': '遥操控制',
                'btn-start-teleop': '▶ 开始遥操',
                'btn-stop-teleop': '⏸ 停止遥操',
                'btn-quit': '⏹ 退出',
                'section-datacoll': '数据采集',
                'label-max-record-min': '最大采集时长（分钟）',
                'label-data-coll-id': 'Data Collection ID (自动生成)',
                'label-data-keys': '采集数据项',
                'keys-status-waiting': '(等待执行端数据...)',
                'keys-empty': '等待执行端发送可用数据项...',
                'btn-start-record': '⏺ 开始数采',
                'btn-stop-record': '⏹ 停止数采',
                'btn-export': '📦 导出数据',
                'export-progress-label': '📦 正在导出数据...',
                'processing': '处理中',
                'msg-export-starting': '正在准备导出任务...',
                'export-complete-title': '✅ 导出完成',
                'export-path-warning': '⚠️ 以下路径为执行端机器上的文件位置',
                'workflow-title': '📋 操作流程',
                'step1': '点击"开始遥操"启动控制端到执行端通信',
                'step2': '点击"开始数采"开始录制',
                'step3': '操作机械臂执行任务',
                'step4': '点击"停止数采"结束录制',
                'step5': '重复2-4采集多条数据',
                'step6': '点击"导出数据"编码导出',
                'section-datamgmt': '📁 数据管理',
                'disk-usage-label': '💾 磁盘占用',
                'export-dir-label': '📂 导出位置（执行端）:',
                'btn-refresh': '🔄 刷新列表',
                'btn-export-selected': '📦 导出选中',
                'export-sel-progress-label': '📦 正在导出选中数据...',
                'playback-label': '▶ 真机回放中...',
                'playback-preparing': '准备中...',
                'btn-stop-playback': '⏹ 停止回放',
                'records-empty': '点击"刷新列表"加载本地数据',
                'records-no-data': '暂无本地数据',
                'msg-teleop-started': '遥操已启动',
                'msg-reconnect-started': '正在重连...',
                'msg-confirm-reconnect': '检测到服务端已重新上线，是否现在重连？\\n\\n重连后不会自动恢复遥操、数采或回放，需要手动重新开始。',
                'msg-reconnect-complete': '重连完成，请手动重新开始遥操或数采',
                'msg-reconnect-failed': '重连失败: ',
                'msg-start-failed': '启动失败: ',
                'msg-select-servers': '请先选择控制端和执行端',
                'msg-select-control-flow': '请至少配置一条控制流',
                'msg-teleop-stopped': '遥操已停止',
                'msg-quitting': '正在退出...',
                'msg-confirm-quit': '确定要退出吗？',
                'msg-select-key': '请至少选择一个数据项',
                'msg-record-started': '数采已开始，Episode: ',
                'msg-record-confirm': '帧数据，是否保留？\\n\\n点击"确定"保留数据\\n点击"取消"丢弃数据',
                'msg-data-kept': '数据已保留，Episode: ',
                'msg-data-discarded': '本轮数据已丢弃，下次采集将覆盖',
                'msg-discard-failed': '丢弃数据失败: ',
                'msg-no-data': '本个 episode 无有效数据',
                'msg-export-waiting': '等待执行端响应...',
                'msg-export-timeout': '导出超时：执行端未响应，请检查执行端是否在线',
                'msg-export-failed': '导出失败: ',
                'msg-exporting': '正在导出数据...',
                'msg-export-complete': '导出完成',
                'msg-confirm-delete-exported-episodes': '导出已完成，是否删除本次已导出的 {n} 条 episode 原始数据？\\n\\nZIP 导出文件会保留，只删除 raw/encoded episode 目录。',
                'msg-exported-episodes-deleted': '已删除已导出的 episode: ',
                'msg-records-refreshed': '数据列表已刷新',
                'msg-refresh-failed': '刷新失败: ',
                'msg-confirm-delete': '确定要删除这条数据吗？此操作不可恢复。',
                'msg-deleted': '已删除',
                'msg-delete-failed': '删除失败: ',
                'msg-confirm-delete-coll': '确定要删除整个数据集合吗？此操作将删除所有相关数据，不可恢复。',
                'msg-coll-deleted': '数据集合已删除',
                'msg-confirm-export-coll': '确定要导出整个数据集 "{name}" 吗？',
                'msg-no-episodes-in-coll': '该数据集没有可导出的 episode',
                'msg-confirm-export-ep': '确定要导出这条数据吗？',
                'msg-exporting-ep': '正在导出...',
                'msg-export-ep-success': '导出成功: ',
                'msg-export-ep-failed': '导出失败: ',
                'msg-select-one': '请至少选择一条数据',
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
                'label-dataset': '数据集',
                'label-episodes': '条',
                'msg-keys-available': '项可用',
                'table-select': '选择',
                'table-component': '组件',
                'table-name': '名称',
                'table-type': '类型',
                'table-category': '类别',
                'table-fps': '帧率',
                'status-complete': '完整', 'status-incomplete': '未完成',
                'status-remote': '远端原始数据',
                'status-invalid': '无效', 'status-unknown': '未知',
                'btn-playback': '▶ 回放',
                'btn-export-ep': '导出', 'btn-delete': '删除',
                'btn-delete-all': '删除全部',
                'btn-export-all': '导出全部',
                'label-exported-files': '📦 已导出文件',
                'label-log-resources': '日志资源',
                'label-view': '查看',
                'label-download': '下载',
                'table-source': '来源',
                'table-log-type': '类型',
                'table-file': '文件',
                'table-size': '大小',
                'label-follower-machine': '📍 执行端机器',
                'label-export-time': '导出时间',
                'msg-confirm-delete-export': '确定要删除这个导出文件吗？此操作不可恢复。',
                'msg-export-deleted': '导出文件已删除',
                'frames': '帧',
                'playback-loading': '加载数据中...',
                'playback-pre-move': '移动到起始位置...',
                'playback-waiting': '准备开始...',
                'playback-playing': '回放中...',
                'playback-returning': '回到初始位置...',
                'playback-completed': '回放完成',
                'playback-stopped': '已停止',
                'playback-failed': '回放失败',
                'playback-error-prefix': '错误: ',
                'msg-collected': '本轮采集了 ',
            },
            en: {
                'header-title': 'RynnRCP Teleop Platform',
                'lang-btn': '中文',
                'section-follower-view': 'Controlled Robot View',
                'waiting-image': 'Waiting for image...',
                'section-robot-binding': 'Device Selection',
                'btn-refresh-devices': '🔎 Refresh Devices',
                'section-control-flow': 'Control Flows',
                'btn-add-control-flow': '+ Add Control Flow',
                'btn-remove-control-flow': 'Remove',
                'label-source-obs': 'Source Observation',
                'label-target-action': 'Target Action',
                'control-flow-empty': 'Add at least one control flow',
                'label-leader-robot': 'Control Device',
                'label-follower-robot': 'Controlled Robot',
                'follower-state-title': 'Controlled Robot State',
                'leader-state-title': 'Control Device State',
                'waiting-data': 'Waiting for data...',
                'section-status': 'Status',
                'label-teleop-status': 'Teleop',
                'label-follower-online': 'Controlled Robot',
                'label-source-conn': 'Control Device',
                'label-target-conn': 'Controlled Robot',
                'label-record-status': 'Recording',
                'label-episode': 'Episode',
                'label-command-fps': 'Command FPS',
                'label-control-loop': 'Control Loop (incl. state wait)',
                'label-transport': 'Transport',
                'label-preview-fps': 'Controlled Image FPS',
                'label-export-fps': 'Export FPS',
                'label-rynnbot-mapping': 'RynnBot Mapping',
                'label-rynnbot-mapping-on': 'Export compatible keys',
                'section-debug': 'Debug',
                'btn-debug-open': '🧪 Debug On',
                'btn-debug-close': 'Debug Off',
                'btn-debug-refresh': 'Refresh Debug',
                'debug-title': 'Runtime Debug',
                'debug-local-role': 'Local Role',
                'debug-peer-role': 'Peer Role',
                'debug-local-runner': 'Local Runner',
                'debug-peer-runner': 'Peer Runner',
                'debug-waiting': 'Click refresh to inspect runtime/device status.',
                'debug-loading': 'Loading debug status...',
                'debug-load-failed': 'Debug load failed: ',
                'debug-raw-json': 'Raw JSON',
                'teleop-on': 'Running', 'teleop-off': 'Stopped',
                'follower-online': 'Online', 'follower-offline': 'Offline',
                'conn-unbound': 'Unbound',
                'conn-online': 'Online',
                'conn-offline': 'Offline',
                'conn-reconnectable': 'Reconnectable',
                'conn-reconnecting': 'Reconnecting',
                'record-on': 'Recording...', 'record-off': 'Idle',
                'section-teleop': 'Teleop Control',
                'btn-start-teleop': '▶ Start',
                'btn-stop-teleop': '⏸ Stop',
                'btn-quit': '⏹ Quit',
                'section-datacoll': 'Data Collection',
                'label-max-record-min': 'Max Duration (minutes)',
                'label-data-coll-id': 'Data Collection ID (auto)',
                'label-data-keys': 'Data Keys',
                'keys-status-waiting': '(waiting for controlled robot...)',
                'keys-empty': 'Waiting for controlled robot to send available keys...',
                'btn-start-record': '⏺ Record',
                'btn-stop-record': '⏹ Stop Rec',
                'btn-export': '📦 Export',
                'export-progress-label': '📦 Exporting data...',
                'processing': 'Processing',
                'msg-export-starting': 'Preparing export...',
                'export-complete-title': '✅ Export Complete',
                'export-path-warning': '⚠️ Path below is on the controlled robot machine',
                'workflow-title': '📋 Workflow',
                'step1': 'Click "Start" to begin teleop',
                'step2': 'Click "Record" to start recording',
                'step3': 'Operate the arm to perform task',
                'step4': 'Click "Stop Rec" to end recording',
                'step5': 'Repeat 2-4 for more episodes',
                'step6': 'Click "Export" to encode & export',
                'section-datamgmt': '📁 Data Management',
                'disk-usage-label': '💾 Disk Usage',
                'export-dir-label': '📂 Export Dir (controlled):',
                'btn-refresh': '🔄 Refresh',
                'btn-export-selected': '📦 Export Sel.',
                'export-sel-progress-label': '📦 Exporting selected...',
                'playback-label': '▶ Playing back...',
                'playback-preparing': 'Preparing...',
                'btn-stop-playback': '⏹ Stop Playback',
                'records-empty': 'Click "Refresh" to load data',
                'records-no-data': 'No local data',
                'msg-teleop-started': 'Teleop started',
                'msg-reconnect-started': 'Reconnecting...',
                'msg-confirm-reconnect': 'A server is available for reconnect. Reconnect now?\\n\\nTeleop, recording, and playback will not resume automatically.',
                'msg-reconnect-complete': 'Reconnected. Start teleop or recording manually.',
                'msg-reconnect-failed': 'Reconnect failed: ',
                'msg-start-failed': 'Start failed: ',
                'msg-select-servers': 'Please select a control device and controlled robot',
                'msg-select-control-flow': 'Please configure at least one control flow',
                'msg-teleop-stopped': 'Teleop stopped',
                'msg-quitting': 'Quitting...',
                'msg-confirm-quit': 'Are you sure you want to quit?',
                'msg-select-key': 'Please select at least one data key',
                'msg-record-started': 'Recording started, episode: ',
                'msg-record-confirm': ' frames collected. Keep this data?\\n\\nOK = Keep\\nCancel = Discard',
                'msg-data-kept': 'Data kept, episode: ',
                'msg-data-discarded': 'Data discarded',
                'msg-discard-failed': 'Failed to discard data: ',
                'msg-no-data': 'No valid data in this episode',
                'msg-export-waiting': 'Waiting for controlled robot...',
                'msg-export-timeout': 'Export timeout: controlled robot not responding',
                'msg-export-failed': 'Export failed: ',
                'msg-exporting': 'Exporting data...',
                'msg-export-complete': 'Export complete',
                'msg-confirm-delete-exported-episodes': 'Export complete. Delete the {n} exported source episode(s)?\\n\\nThe ZIP export will be kept; only raw/encoded episode folders are deleted.',
                'msg-exported-episodes-deleted': 'Deleted exported episodes: ',
                'msg-records-refreshed': 'Records refreshed',
                'msg-refresh-failed': 'Refresh failed: ',
                'msg-confirm-delete': 'Delete this episode? This cannot be undone.',
                'msg-deleted': 'Deleted',
                'msg-delete-failed': 'Delete failed: ',
                'msg-confirm-delete-coll': 'Delete entire collection? This cannot be undone.',
                'msg-coll-deleted': 'Collection deleted',
                'msg-confirm-export-coll': 'Export entire dataset "{name}"?',
                'msg-no-episodes-in-coll': 'No exportable episodes in this dataset',
                'msg-confirm-export-ep': 'Export this episode?',
                'msg-exporting-ep': 'Exporting...',
                'msg-export-ep-success': 'Exported: ',
                'msg-export-ep-failed': 'Export failed: ',
                'msg-select-one': 'Please select at least one episode',
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
                'label-dataset': 'Dataset',
                'label-episodes': 'episodes',
                'msg-keys-available': ' keys available',
                'table-select': 'Select',
                'table-component': 'Component',
                'table-name': 'Name',
                'table-type': 'Type',
                'table-category': 'Category',
                'table-fps': 'FPS',
                'status-complete': 'Complete', 'status-incomplete': 'Raw',
                'status-remote': 'Remote Raw',
                'status-invalid': 'Invalid', 'status-unknown': 'Unknown',
                'btn-playback': '▶ Play',
                'btn-export-ep': 'Export', 'btn-delete': 'Delete',
                'btn-delete-all': 'Delete All',
                'btn-export-all': 'Export All',
                'label-exported-files': '📦 Exported Files',
                'label-log-resources': 'Log Resources',
                'label-view': 'View',
                'label-download': 'Download',
                'table-source': 'Source',
                'table-log-type': 'Type',
                'table-file': 'File',
                'table-size': 'Size',
                'label-follower-machine': '📍 Controlled Robot',
                'label-export-time': 'Exported',
                'msg-confirm-delete-export': 'Delete this exported file? This cannot be undone.',
                'msg-export-deleted': 'Export file deleted',
                'frames': ' frames',
                'playback-loading': 'Loading data...',
                'playback-pre-move': 'Moving to start position...',
                'playback-waiting': 'About to start...',
                'playback-playing': 'Playing...',
                'playback-returning': 'Returning to start pose...',
                'playback-completed': 'Playback complete',
                'playback-stopped': 'Stopped',
                'playback-failed': 'Playback failed',
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
            const debugToggle = document.getElementById('btn-debug-toggle');
            if (debugToggle) debugToggle.textContent = debugEnabled ? t('btn-debug-close') : t('btn-debug-open');
        }
        function toggleLang() {
            currentLang = currentLang === 'zh' ? 'en' : 'zh';
            localStorage.setItem('teleop_lang', currentLang);
            applyLang();
            // Re-apply dynamic state text
            updateUI();
        }
        // ==================== end i18n ====================
        const socket = io({
            autoConnect: false,
            reconnection: true,
            reconnectionAttempts: Infinity,
            reconnectionDelay: 300,
            reconnectionDelayMax: 2000,
            timeout: 5000,
        });
        let teleopEnabled = false, recordingEnabled = false;
        let transportMode = '-';
        let collectionId = '';
        let maxRecordDurationS = 600;
        let collectionConfigLocked = false;
        let debugStatusEnabled = false;
        let debugUiConfigured = false;
        let debugUrlToggleAllowed = true;
        let debugPollIntervalMs = 1000;
        let latestConnectionStatus = {};
        const JOINT_HISTORY_LIMIT = 200;
        const JOINT_RENDER_HZ = 30;
        const JOINT_RENDER_INTERVAL_MS = 1000 / JOINT_RENDER_HZ;
        const JOINT_COLORS = ['#00ff88', '#00d4ff', '#ffaa00', '#ff6b6b', '#b388ff', '#f8f871', '#ff8bd1', '#7df9ff', '#b8ff7a', '#ffcf70'];
        const followerStateHistory = [];
        const leaderStateHistory = [];
        const stateChartStores = new Map();
        let latestFollowerJointPositions = [];
        let latestLeaderJointPositions = [];
        const selectedJointIndices = new Set();
        let jointDimensionCount = 0;
        let lastImageUpdateAt = 0;
        let lastStateUpdateAt = 0;
        let lastLeaderStateUpdateAt = 0;
        const imageObjectUrls = new Map();
        const collectionConfigInputs = [
            'input-task-prompt',
            'input-task-desc',
            'input-fps',
            'input-max-record-min',
        ];
        let reconnectPromptKey = '';
        let discoveredServers = [];
        let controlFlowRows = [{source_observation: '', target_action: ''}];
        let previewKeyConfig = {options: {target_images: [], target_states: [], source_states: []}, selected: {target_images: [], target_states: [], source_states: []}};
        const PREFERRED_CONTROL_SOURCE = 'observation.robot.joint_state';
        const PREFERRED_CONTROL_TARGET = 'action.robot.joint_position';
        let latestRecordStatus = {};
        async function discoverServers() {
            const resp = await fetch('/api/discover');
            const data = await resp.json();
            if (!data.success) {
                showToast('Discover failed: ' + (data.message || ''), 'error');
                return;
            }
            const source = document.getElementById('source-server-select');
            const target = document.getElementById('target-server-select');
            source.innerHTML = '';
            target.innerHTML = '';
            discoveredServers = data.servers || [];
            discoveredServers.forEach(s => {
                const endpoint = endpointLabel(s.endpoint_source);
                const label = `${s.robot_name || s.robot_id} (${s.embodiment_type || '-'})${endpoint ? ' · ' + endpoint : ''}`;
                if (s.capabilities && s.capabilities.observations) {
                    source.add(new Option(label, s.robot_id));
                }
                if (s.capabilities && s.capabilities.actions) {
                    target.add(new Option(label, s.robot_id));
                }
            });
            selectPreferredServers();
            renderControlFlows();
            showToast(`Discovered ${(data.servers || []).length} robot(s)`, 'info');
        }
        function selectPreferredServers() {
            const sourceSelect = document.getElementById('source-server-select');
            const targetSelect = document.getElementById('target-server-select');
            const sourcePreferred = discoveredServers.find(s => serverSearchText(s).includes('leader') && s.capabilities?.observations);
            const targetPreferred = discoveredServers.find(s => serverSearchText(s).includes('follower') && s.capabilities?.actions);
            if (sourcePreferred) sourceSelect.value = sourcePreferred.robot_id;
            if (targetPreferred) targetSelect.value = targetPreferred.robot_id;
        }
        function serverSearchText(server) {
            return `${server?.robot_name || ''} ${server?.robot_id || ''}`.toLowerCase();
        }
        function selectedServer(id) {
            const robotId = document.getElementById(id)?.value || '';
            return discoveredServers.find(s => s.robot_id === robotId) || null;
        }
        function controlSourceOptions() {
            const server = selectedServer('source-server-select');
            return ((server || {}).observations || []).filter(item => item.name && item.type !== 'image');
        }
        function controlTargetOptions() {
            const server = selectedServer('target-server-select');
            return ((server || {}).actions || []).filter(item => item.name);
        }
        function canDriveAction(sourceType, targetType) {
            return (
                (sourceType === 'joint_state' && targetType === 'joint_position') ||
                (sourceType === 'ee_pose' && targetType === 'ee_pose') ||
                (sourceType === 'gripper_state' && targetType === 'gripper')
            );
        }
        function compatibleTargetOptions(sourceItem, targetOptions) {
            if (!sourceItem) return [];
            return targetOptions.filter(item => canDriveAction(sourceItem.type || '', item.type || ''));
        }
        function preferredSourceOption(sourceOptions) {
            return sourceOptions.find(item => item.name === PREFERRED_CONTROL_SOURCE) || sourceOptions[0] || null;
        }
        function preferredTargetOption(targetOptions) {
            return targetOptions.find(item => item.name === PREFERRED_CONTROL_TARGET) || targetOptions[0] || null;
        }
        function shortProtocolName(item) {
            const raw = String(item.name || item.protocol_name || '');
            const parts = raw.split('.');
            return parts[parts.length - 1] || raw;
        }
        function optionLabel(item) {
            const component = item.component_name ? item.component_name + ' · ' : '';
            const type = item.type ? ' (' + item.type + ')' : '';
            return component + shortProtocolName(item) + type;
        }
        function applyPreviewKeys(config) {
            if (!config) return;
            previewKeyConfig = {
                options: config.options || previewKeyConfig.options,
                selected: config.selected || previewKeyConfig.selected,
            };
            renderPreviewKeySelectors();
            prunePreviewDisplay();
        }
        function renderPreviewKeySelectors() {
            renderPreviewKeySelector('target-image-key-selector', 'target_images');
            renderPreviewKeySelector('target-state-key-selector', 'target_states');
            renderPreviewKeySelector('source-state-key-selector', 'source_states');
        }
        function renderPreviewKeySelector(elementId, group) {
            const root = document.getElementById(elementId);
            if (!root) return;
            const options = (previewKeyConfig.options || {})[group] || [];
            const selected = new Set((previewKeyConfig.selected || {})[group] || []);
            if (!options.length) {
                root.innerHTML = '';
                return;
            }
            root.innerHTML = options.map(item => {
                const name = String(item.protocol_name || item.name || '');
                return `<label class="preview-key-chip" title="${esc(name)}">
                    <input type="checkbox" data-preview-group="${esc(group)}" value="${esc(name)}" ${selected.has(name) ? 'checked' : ''} onchange="updatePreviewKeysFromUI()">
                    <span>${esc(optionLabel(item))}</span>
                </label>`;
            }).join('');
        }
        function selectedPreviewNames(group) {
            return Array.from(document.querySelectorAll(`input[data-preview-group="${group}"]:checked`)).map(el => el.value);
        }
        function updatePreviewKeysFromUI() {
            const payload = {
                target_images: selectedPreviewNames('target_images'),
                target_states: selectedPreviewNames('target_states'),
                source_states: selectedPreviewNames('source_states'),
            };
            fetch('/api/preview_keys', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(payload)
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    applyPreviewKeys(d.preview_keys);
                } else {
                    showToast('更新预览数据项失败: ' + (d.message || ''), 'error');
                }
            }).catch(e => showToast('更新预览数据项失败: ' + e, 'error'));
        }
        function prunePreviewDisplay() {
            const selectedImages = new Set((previewKeyConfig.selected || {}).target_images || []);
            imageObjectUrls.forEach((url, key) => {
                if (!selectedImages.has(key)) {
                    URL.revokeObjectURL(url);
                    imageObjectUrls.delete(key);
                }
            });
            document.querySelectorAll('#video-grid .video-container img[id^="cam-"]').forEach(img => {
                const key = img.id.slice(4);
                if (!selectedImages.has(key)) img.closest('.video-container')?.remove();
            });
            const grid = document.getElementById('video-grid');
            if (grid && !selectedImages.size) {
                grid.innerHTML = '<div class="video-container" style="display:flex;align-items:center;justify-content:center;min-height:200px;"><span style="color:#555;">未选择图像数据项</span></div>';
                videoGridInited = false;
            }
            pruneStateCards('follower-state-cards', new Set((previewKeyConfig.selected || {}).target_states || []), 'follower');
            pruneStateCards('leader-state-cards', new Set((previewKeyConfig.selected || {}).source_states || []), 'leader');
        }
        function pruneStateCards(containerId, selected, side) {
            const root = document.getElementById(containerId);
            if (!root) return;
            Array.from(root.children).forEach(child => {
                const key = String(child.title || '');
                if (key && !selected.has(key)) {
                    stateChartStores.delete(`${side}:${key}`);
                    child.remove();
                }
            });
        }
        function addControlFlow() {
            controlFlowRows.push({source_observation: '', target_action: ''});
            renderControlFlows();
        }
        function removeControlFlow(index) {
            controlFlowRows.splice(index, 1);
            renderControlFlows();
        }
        function renderControlFlows() {
            const root = document.getElementById('control-flow-list');
            if (!root) return;
            const sourceOptions = controlSourceOptions();
            const targetOptions = controlTargetOptions();
            if (!controlFlowRows.length) {
                root.innerHTML = '<span style="color:#666;font-size:13px;">' + t('control-flow-empty') + '</span>';
                return;
            }
            root.innerHTML = '';
            controlFlowRows.forEach((flow, index) => {
                let sourceItem = sourceOptions.find(item => item.name === flow.source_observation)
                    || preferredSourceOption(sourceOptions);
                flow.source_observation = sourceItem ? sourceItem.name : '';
                let targetsForSource = compatibleTargetOptions(sourceItem, targetOptions);
                let targetItem = targetsForSource.find(item => item.name === flow.target_action)
                    || preferredTargetOption(targetsForSource);
                flow.target_action = targetItem ? targetItem.name : '';
                const row = document.createElement('div');
                row.style.cssText = 'display:grid;grid-template-columns:minmax(0,1fr) auto minmax(0,1fr) auto;gap:8px;align-items:center;background:#252535;border:1px solid #333;border-radius:8px;padding:8px;';
                const sourceSelect = document.createElement('select');
                sourceSelect.style.cssText = 'min-width:0;width:100%;padding:6px 8px;border-radius:6px;border:1px solid #444;background:#1e1e2f;color:#e8e8e8;font-size:12px;';
                sourceOptions.forEach(item => sourceSelect.add(new Option(optionLabel(item), item.name)));
                sourceSelect.value = flow.source_observation;
                sourceSelect.onchange = () => {
                    controlFlowRows[index].source_observation = sourceSelect.value;
                    controlFlowRows[index].target_action = '';
                    renderControlFlows();
                };
                const arrow = document.createElement('span');
                arrow.textContent = '→';
                arrow.style.cssText = 'color:#00ff88;font-weight:700;';
                const targetSelect = document.createElement('select');
                targetSelect.style.cssText = sourceSelect.style.cssText;
                targetsForSource.forEach(item => targetSelect.add(new Option(optionLabel(item), item.name)));
                targetSelect.value = flow.target_action;
                targetSelect.onchange = () => { controlFlowRows[index].target_action = targetSelect.value; };
                const remove = document.createElement('button');
                remove.className = 'btn btn-danger';
                remove.type = 'button';
                remove.textContent = t('btn-remove-control-flow');
                remove.style.cssText = 'padding:6px 10px;font-size:12px;';
                remove.onclick = () => removeControlFlow(index);
                row.appendChild(sourceSelect);
                row.appendChild(arrow);
                row.appendChild(targetSelect);
                row.appendChild(remove);
                root.appendChild(row);
            });
        }
        function collectControlFlows() {
            return controlFlowRows
                .map(flow => ({
                    source_observation: String(flow.source_observation || '').trim(),
                    target_action: String(flow.target_action || '').trim(),
                }))
                .filter(flow => flow.source_observation && flow.target_action);
        }
        function endpointLabel(source) {
            if (source === 'static') return currentLang === 'zh' ? '配置地址' : 'configured';
            if (source === 'local_registry') return currentLang === 'zh' ? '本机' : 'local';
            if (source === 'mdns') return currentLang === 'zh' ? '局域网' : 'LAN';
            return '';
        }
        async function reconnectServers() {
            showToast(t('msg-reconnect-started'), 'info');
            const resp = await fetch('/api/reconnect', {method: 'POST'});
            const data = await resp.json();
            if (!data.success) {
                showToast(t('msg-reconnect-failed') + (data.message || ''), 'error');
                return;
            }
            if (data.connection_status) applyConnectionStatus(data.connection_status);
            if (data.preview_keys) applyPreviewKeys(data.preview_keys);
            resetRunControlsAfterReconnect();
            showToast(t('msg-reconnect-complete'), 'success');
        }
        function resetRunControlsAfterReconnect() {
            teleopEnabled = false;
            recordingEnabled = false;
            stopPlaybackStatusPolling();
            const playbackContainer = document.getElementById('playback-progress-container');
            if (playbackContainer) playbackContainer.style.display = 'none';
            const stopPlaybackBtn = document.getElementById('btn-stop-playback');
            if (stopPlaybackBtn) stopPlaybackBtn.style.display = 'none';
            updateUI();
        }
        // Fetch collection_id from backend on page load
        fetch('/api/config').then(r=>r.json()).then(d => {
            collectionId = d.collection_id || 'teleop_' + Math.floor(Date.now() / 1000);
            maxRecordDurationS = d.max_record_duration_s || 600;
            teleopEnabled = d.teleop_enabled || false;
            recordingEnabled = d.recording || false;
            transportMode = formatTransportMode(d.transport || '-');
            collectionConfigLocked = d.collection_config_locked || false;
            debugStatusEnabled = !!d.debug_enabled;
            debugUiConfigured = !!d.debug_ui;
            debugUrlToggleAllowed = d.debug_ui_allow_url_toggle !== false;
            debugPollIntervalMs = Math.max(500, Number(d.debug_poll_interval_s || 1.0) * 1000);
            document.getElementById('input-data-coll-id').value = collectionId;
            document.getElementById('input-max-record-min').value = Math.max(1, Math.round(maxRecordDurationS / 60));
            document.getElementById('input-rynnbot-mapping').checked = d.export_rynnbot_mapping !== false;
            if (d.collection_config) {
                if (d.collection_config.task_prompt != null) document.getElementById('input-task-prompt').value = d.collection_config.task_prompt;
                if (d.collection_config.task_description != null) document.getElementById('input-task-desc').value = d.collection_config.task_description;
                if (d.collection_config.fps != null) document.getElementById('input-fps').value = d.collection_config.fps;
                if (d.collection_config.max_duration_s != null) document.getElementById('input-max-record-min').value = Math.max(1, Math.round(d.collection_config.max_duration_s / 60));
            }
            configureDebugUi();
            discoverServers().catch(() => {});
            updateUI();
        });
        function updateUI() {
            document.getElementById('btn-start-teleop').disabled = teleopEnabled;
            document.getElementById('btn-stop-teleop').disabled = !teleopEnabled;
            document.getElementById('btn-start-record').disabled = !teleopEnabled || recordingEnabled;
            document.getElementById('btn-stop-record').disabled = !teleopEnabled || !recordingEnabled;
            const lockCollectionInputs = collectionConfigLocked || recordingEnabled;
            collectionConfigInputs.forEach(id => {
                const input = document.getElementById(id);
                if (!input) return;
                input.disabled = lockCollectionInputs;
                input.style.opacity = lockCollectionInputs ? '0.65' : '1';
                input.style.cursor = lockCollectionInputs ? 'not-allowed' : '';
            });
            document.getElementById('teleop-status').textContent = teleopEnabled ? t('teleop-on') : t('teleop-off');
            document.getElementById('teleop-status').className = 'status-value ' + (teleopEnabled ? 'active' : 'inactive');
            document.getElementById('record-status').textContent = recordingEnabled ? t('record-on') : t('record-off');
            document.getElementById('record-status').className = 'status-value ' + (recordingEnabled ? 'recording' : 'inactive');
            document.getElementById('transport-mode').textContent = transportMode;
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
        function formatTransportMode(value) {
            const mode = String(value || '').toLowerCase();
            if (mode === 'shm' || mode === 'shared_memory') return 'SHM';
            if (mode === 'udp') return 'UDP';
            return value || '-';
        }
        const DEBUG_STORAGE_KEY = 'teleop_debug_ui';
        let debugEnabled = false;
        let debugTimer = null;
        function configureDebugUi() {
            if (!debugStatusEnabled) {
                localStorage.removeItem(DEBUG_STORAGE_KEY);
                const root = document.getElementById('debug-root');
                if (root) root.style.display = 'none';
                setDebugPanelEnabled(false, { persist: false });
                return;
            }
            const params = new URLSearchParams(window.location.search);
            if (debugUrlToggleAllowed && params.has('debug')) {
                const value = String(params.get('debug') || '').toLowerCase();
                if (['1', 'true', 'yes', 'on'].includes(value)) {
                    localStorage.setItem(DEBUG_STORAGE_KEY, '1');
                } else if (['0', 'false', 'no', 'off'].includes(value)) {
                    localStorage.removeItem(DEBUG_STORAGE_KEY);
                }
            }
            const requestedByUrl = debugUrlToggleAllowed && localStorage.getItem(DEBUG_STORAGE_KEY) === '1';
            const visible = debugUiConfigured || requestedByUrl;
            const root = document.getElementById('debug-root');
            if (root) root.style.display = visible ? 'block' : 'none';
            setDebugPanelEnabled(visible && requestedByUrl, { persist: false });
        }
        function toggleDebugPanel() {
            setDebugPanelEnabled(!debugEnabled);
        }
        function setDebugPanelEnabled(enabled, options={persist: true}) {
            debugEnabled = !!enabled;
            const panel = document.getElementById('debug-panel');
            const refresh = document.getElementById('btn-debug-refresh');
            const toggle = document.getElementById('btn-debug-toggle');
            if (!panel || !refresh || !toggle) return;
            panel.classList.toggle('active', debugEnabled);
            refresh.disabled = !debugEnabled;
            toggle.textContent = debugEnabled ? t('btn-debug-close') : t('btn-debug-open');
            if (options.persist && debugUrlToggleAllowed) {
                if (debugEnabled) localStorage.setItem(DEBUG_STORAGE_KEY, '1');
                else localStorage.removeItem(DEBUG_STORAGE_KEY);
            }
            if (debugEnabled) {
                refreshDebugStatus();
                if (debugTimer) clearInterval(debugTimer);
                debugTimer = setInterval(refreshDebugStatus, debugPollIntervalMs);
            } else if (debugTimer) {
                clearInterval(debugTimer);
                debugTimer = null;
            }
        }
        function refreshDebugStatus() {
            if (!debugEnabled || !debugStatusEnabled) return;
            const refresh = document.getElementById('btn-debug-refresh');
            const updated = document.getElementById('debug-updated-at');
            refresh.disabled = true;
            updated.textContent = t('debug-loading');
            const url = debugUiConfigured ? '/api/debug/status' : '/api/debug/status?debug=1';
            fetch(url)
                .then(r => r.json())
                .then(d => {
                    if (!d.success) throw new Error(d.message || 'unknown');
                    renderDebugStatus(d.data || {});
                })
                .catch(e => {
                    updated.textContent = t('debug-load-failed') + e.message;
                    showToast(t('debug-load-failed') + e.message, 'error');
                })
                .finally(() => {
                    refresh.disabled = !debugEnabled;
                });
        }
        function renderDebugStatus(data) {
            const local = data.local || {};
            const peer = data.peer || {};
            document.getElementById('debug-local-role').textContent = local.role || '-';
            document.getElementById('debug-peer-role').textContent = peer.role || data.peer_error || '-';
            document.getElementById('debug-local-runner').textContent = summarizeRunner(local);
            document.getElementById('debug-peer-runner').textContent = peer.role ? summarizeRunner(peer) : (data.peer_error || '-');
            document.getElementById('debug-updated-at').textContent = new Date().toLocaleTimeString();
            document.getElementById('debug-parsed').innerHTML = buildParsedDebug(data);
            document.getElementById('debug-json').textContent = JSON.stringify(data, null, 2);
        }
        function summarizeRunner(side) {
            const runtime = ((side.get_runtime_status || {}).result || {});
            const runners = (runtime.runners || {}).processes || {};
            const alive = Object.entries(runners).map(([k, v]) => `${k}:${v ? 'ok' : 'down'}`);
            const errors = (runtime.recent_errors || []).length;
            if (!alive.length && !errors) return '-';
            return alive.join(', ') + (errors ? `, errors:${errors}` : '');
        }
        function buildParsedDebug(data) {
            const local = data.local || {};
            const peer = data.peer || null;
            const cards = [
                sideDebugCard(currentLang === 'zh' ? '本端' : 'Local', local),
                peer ? sideDebugCard(currentLang === 'zh' ? '对端' : 'Peer', peer)
                     : debugCard(currentLang === 'zh' ? '对端' : 'Peer', [
                        debugRow(currentLang === 'zh' ? '状态' : 'Status', pill(data.peer_error || 'timeout', 'bad'))
                     ]),
                runnerKeyFpsCard(currentLang === 'zh' ? '本端 Runner Key 帧率' : 'Local Runner Key FPS', local),
                peer ? runnerKeyFpsCard(currentLang === 'zh' ? '对端 Runner Key 帧率' : 'Peer Runner Key FPS', peer)
                     : debugCard(currentLang === 'zh' ? '对端 Runner Key 帧率' : 'Peer Runner Key FPS', [
                        debugRow(currentLang === 'zh' ? '状态' : 'Status', pill(data.peer_error || 'timeout', 'bad'))
                     ], true),
                actionLatencyCard(data),
                linkDebugCard(data),
                errorDebugCard(data),
            ];
            return cards.join('');
        }
        function sideDebugCard(title, side) {
            const runtimeResp = side.get_runtime_status || {};
            const deviceResp = side.get_device_info || {};
            const runtime = runtimeResp.result || {};
            const device = deviceResp.result || {};
            const metrics = side.metrics || {};
            const rows = [
                debugRow('role', esc(side.role || '-')),
                debugRow('peer', side.peer_connected ? pill('connected', 'ok') : pill('not connected', side.role === 'follower' ? 'warn' : 'bad')),
                debugRow('heartbeat', side.last_heartbeat_age_s == null ? '-' : fmt(side.last_heartbeat_age_s, 2) + 's'),
                debugRow('robot', esc(device.robot_id || device.arm_info || '-')),
                debugRow('runtime', esc((runtime.runtime || {}).runner_mode || '-')),
                debugRow('runners', runnerPills((runtime.runners || {}).processes || {})),
                debugRow('cmd tx/rx', `${fmt(metrics.command_tx_fps, 1)} / ${fmt(metrics.command_rx_fps, 1)} Hz`),
                debugRow('img tx/rx', `${fmt(metrics.image_tx_fps, 1)} / ${fmt(metrics.image_rx_fps, 1)} Hz`),
            ];
            return debugCard(title, rows);
        }
        function runnerKeyFpsCard(title, side) {
            const runtime = (((side || {}).get_runtime_status || {}).result || {});
            const rows = buildRunnerKeyRows(runtime);
            if (!rows.length) {
                rows.push(debugRow(currentLang === 'zh' ? '状态' : 'Status', pill('no runner stats', 'warn')));
            }
            return debugCard(title, rows, true);
        }
        function buildRunnerKeyRows(runtime) {
            const scheduler = ((runtime.runners || {}).scheduler || {});
            return buildRecordingRows(scheduler);
        }
        function buildRecordingRows(scheduler) {
            const recording = (scheduler || {}).recording || {};
            const components = recording.components || {};
            const names = Object.keys(components).sort((a, b) => {
                if (a === 'recording.loop') return -1;
                if (b === 'recording.loop') return 1;
                return a.localeCompare(b);
            });
            const rows = [];
            names.forEach(name => {
                const stats = components[name];
                if (!stats) return;
                const shortName = name.startsWith('recording.') ? name.slice('recording.'.length) : name;
                rows.push(debugRow(`recording:${shortName}`, formatRecordingFpsValue(stats, shortName)));
            });
            return rows;
        }
        function formatRecordingFpsValue(stats, shortName) {
            const observed = Number(stats.observed_hz);
            const target = Number(stats.target_hz);
            const overrun = Number(stats.dropped_runs || 0);
            const runs = Number(stats.total_runs || 0);
            const error = stats.last_error;
            let text = `${Number.isFinite(observed) ? observed.toFixed(1) : '-'} Hz`;
            if (Number.isFinite(target) && target > 0) text += ` / ${target.toFixed(target >= 10 ? 0 : 1)}`;
            if (runs > 0) text += `, ${currentLang === 'zh' ? '写入' : 'writes'} ${runs}`;
            if (overrun > 0) text += `, ${currentLang === 'zh' ? '超时' : 'overrun'} ${overrun}`;
            if (error) text += ', error';
            let kind = 'ok';
            if (error) kind = 'bad';
            else if (Number.isFinite(target) && target > 0 && Number.isFinite(observed) && observed > 0 && observed < target * 0.9) kind = 'warn';
            const mode = shortName === 'loop'
                ? (currentLang === 'zh' ? '主循环' : 'loop')
                : (currentLang === 'zh' ? '落盘' : 'write');
            // For recording.loop, append per-stage breakdown so users can see
            // where each tick spends its time (sample / write / book / sleep).
            let breakdown = '';
            if (shortName === 'loop' && stats.stage_avg_ms && typeof stats.stage_avg_ms === 'object') {
                const s = stats.stage_avg_ms;
                const fmt = (v) => Number.isFinite(Number(v)) ? Number(v).toFixed(2) : '-';
                const labels = currentLang === 'zh'
                    ? { sample: '采样', write: '写盘', book: '统计', sleep: '休眠' }
                    : { sample: 'sample', write: 'write', book: 'book', sleep: 'sleep' };
                breakdown = ` · ${labels.sample} ${fmt(s.sample)}ms · ${labels.write} ${fmt(s.write)}ms · ${labels.book} ${fmt(s.book)}ms · ${labels.sleep} ${fmt(s.sleep)}ms`;
            }
            return `${pill(text, kind)}<span class="debug-subtle">${esc(shortName)} · ${esc(mode)}${esc(breakdown)}</span>`;
        }
        function actionLatencyCard(data) {
            const local = (data.local || {}).get_action_latency || {};
            const peer = (data.peer || {}).get_action_latency || {};
            const localSummary = ((local.result || {}).summary || {});
            const peerSummary = ((peer.result || {}).summary || {});
            const rows = [];
            rows.push(debugRow(currentLang === 'zh' ? '本端样本' : 'Local samples', String(localSummary.count || 0)));
            rows.push(debugRow(currentLang === 'zh' ? '对端样本' : 'Peer samples', String(peerSummary.count || 0)));
            appendLatencyRows(rows, currentLang === 'zh' ? '本端' : 'Local', localSummary);
            appendLatencyRows(rows, currentLang === 'zh' ? '对端' : 'Peer', peerSummary);
            // Hardware-slow watchdog: surface explicit "驱动调用慢" rows so
            // users can see at a glance that the stall is in the downstream
            // hardware/driver layer, not in our software pipeline. Toast
            // once per new event.
            appendHardwareSlowRow(rows, currentLang === 'zh' ? '本端' : 'Local', localSummary);
            appendHardwareSlowRow(rows, currentLang === 'zh' ? '对端' : 'Peer', peerSummary);
            maybeNotifyHardwareSlow('local', localSummary);
            maybeNotifyHardwareSlow('peer', peerSummary);
            if (rows.length <= 2) {
                rows.push(debugRow(currentLang === 'zh' ? '状态' : 'Status', pill('waiting traces', 'warn')));
            }
            return debugCard(currentLang === 'zh' ? 'Action 延迟' : 'Action Latency', rows, true);
        }
        function appendHardwareSlowRow(rows, prefix, summary) {
            const slow = summary && summary.hardware_slow;
            if (!slow) return;
            const latest = Number(slow.latest_ms || 0);
            const max = Number(slow.max_ms || 0);
            const total = Number(slow.total || 0);
            const inWindow = Number(slow.window_count || 0);
            const threshold = Number(slow.threshold_ms || 0);
            const text = currentLang === 'zh'
                ? `${latest.toFixed(0)}ms / max ${max.toFixed(0)}ms · 累计 ${total} 次`
                : `${latest.toFixed(0)}ms / max ${max.toFixed(0)}ms · total ${total}`;
            const note = currentLang === 'zh'
                ? `驱动调用慢 (>${threshold.toFixed(0)}ms)，下游硬件层问题`
                : `slow driver call (>${threshold.toFixed(0)}ms), downstream hardware issue`;
            rows.push(debugRow(
                `${prefix} ${currentLang === 'zh' ? '驱动慢' : 'driver slow'}`,
                `${pill(text, 'bad')}<span class="debug-subtle">${esc(note)} · ${inWindow}/${Number(slow.window_size || 0)}</span>`,
            ));
        }
        const _hardwareSlowSeen = { local: 0, peer: 0 };
        function maybeNotifyHardwareSlow(side, summary) {
            const slow = summary && summary.hardware_slow;
            if (!slow) return;
            const total = Number(slow.total || 0);
            if (total <= _hardwareSlowSeen[side]) return;  // de-dupe across refreshes
            const delta = total - _hardwareSlowSeen[side];
            _hardwareSlowSeen[side] = total;
            // Skip the very first observation so we don't toast on page load
            // when historical traces are replayed.
            if (delta === total) return;
            const sideLabel = side === 'local'
                ? (currentLang === 'zh' ? '本端' : 'Local')
                : (currentLang === 'zh' ? '对端' : 'Peer');
            const msg = currentLang === 'zh'
                ? `${sideLabel}驱动调用慢 ${Number(slow.latest_ms || 0).toFixed(0)}ms (下游硬件层问题，非软件问题)`
                : `${sideLabel} slow driver call ${Number(slow.latest_ms || 0).toFixed(0)}ms (downstream hardware, not software)`;
            try { showToast(msg, 'error', 4000); } catch (_) {}
        }
        function appendLatencyRows(rows, prefix, summary) {
            const fields = [
                ['command_latency_ms', currentLang === 'zh' ? '指令延迟' : 'command'],
                ['end_to_end_ms', currentLang === 'zh' ? '全链路' : 'e2e'],
                ['leader_state_read_ms', currentLang === 'zh' ? '读state' : 'read'],
                ['leader_to_follower_ms', currentLang === 'zh' ? '传输' : 'wire'],
                ['follower_enqueue_ms', currentLang === 'zh' ? '入队' : 'enqueue'],
                ['action_queue_ms', currentLang === 'zh' ? '排队' : 'queue'],
                ['driver_call_ms', currentLang === 'zh' ? '驱动调用' : 'driver'],
            ];
            fields.forEach(([field, label]) => {
                const value = summary[field];
                if (!value) return;
                rows.push(debugRow(`${prefix} ${label}`, latencyValue(value)));
            });
        }
        function latencyValue(value) {
            const latest = Number(value.latest);
            const avg = Number(value.avg);
            const max = Number(value.max);
            const text = `${fmt(latest, 1)}ms avg ${fmt(avg, 1)} max ${fmt(max, 1)}`;
            const kind = latest > 80 ? 'bad' : (latest > 40 ? 'warn' : 'ok');
            return pill(text, kind);
        }
        function linkDebugCard(data) {
            const local = data.local || {};
            const peer = data.peer || {};
            const lm = local.metrics || {};
            const pm = peer.metrics || {};
            const localRuntime = ((local.get_runtime_status || {}).result || {});
            const peerRuntime = ((peer.get_runtime_status || {}).result || {});
            const rows = [
                debugRow('leader tx', fmt(lm.command_tx_fps, 1) + ' Hz'),
                debugRow('follower rx', fmt(pm.command_rx_fps, 1) + ' Hz'),
                debugRow('leader errors', recentErrorPill(localRuntime.recent_errors || [])),
                debugRow('follower errors', recentErrorPill(peerRuntime.recent_errors || [])),
                debugRow('判断', diagnoseLink(data)),
            ];
            return debugCard(currentLang === 'zh' ? '链路检查' : 'Link Check', rows);
        }
        function errorDebugCard(data) {
            const localErrors = ((((data.local || {}).get_runtime_status || {}).result || {}).recent_errors || []);
            const peerErrors = ((((data.peer || {}).get_runtime_status || {}).result || {}).recent_errors || []);
            const rows = [];
            [...localErrors.map(e => ['local', e]), ...peerErrors.map(e => ['peer', e])].slice(0, 8).forEach(([side, e]) => {
                rows.push(debugRow(side + ':' + (e.component || '-'), esc(e.last_error || '-')));
            });
            if (!rows.length) rows.push(debugRow(currentLang === 'zh' ? '最近错误' : 'Recent Errors', pill('none', 'ok')));
            return debugCard(currentLang === 'zh' ? '最近错误' : 'Recent Errors', rows);
        }
        function diagnoseLink(data) {
            const local = data.local || {};
            const peer = data.peer || {};
            const lm = local.metrics || {};
            const pm = peer.metrics || {};
            if (!peer.role) return pill(currentLang === 'zh' ? '对端无响应' : 'peer timeout', 'bad');
            if ((lm.command_tx_fps || 0) > 20 && (pm.command_rx_fps || 0) < 10) {
                return pill(currentLang === 'zh' ? '通信/接收慢' : 'interface/rx slow', 'warn');
            }
            if ((pm.command_rx_fps || 0) > 20) {
                return pill(currentLang === 'zh' ? '指令到达执行端' : 'commands arrived', 'ok');
            }
            return pill(currentLang === 'zh' ? '等待遥操数据' : 'waiting commands', 'warn');
        }
        function runnerPills(runners) {
            const entries = Object.entries(runners || {});
            if (!entries.length) return '-';
            return entries.map(([k, v]) => pill(k, v ? 'ok' : 'bad')).join(' ');
        }
        function recentErrorPill(errors) {
            return (errors || []).length ? pill(String(errors.length), 'bad') : pill('0', 'ok');
        }
        function debugCard(title, rows, wide=false) {
            return `<div class="debug-card${wide ? ' wide' : ''}"><h4>${esc(title)}</h4>${rows.join('')}</div>`;
        }
        function debugRow(key, value) {
            return `<div class="debug-row"><span class="debug-key">${esc(key)}</span><span class="debug-value">${value}</span></div>`;
        }
        function pill(text, kind) {
            return `<span class="debug-pill ${kind}">${esc(text)}</span>`;
        }
        function fmt(value, digits) {
            const n = Number(value || 0);
            return Number.isFinite(n) ? n.toFixed(digits) : '-';
        }
        function esc(value) {
            return String(value == null ? '' : value)
                .replace(/&/g, '&amp;')
                .replace(/</g, '&lt;')
                .replace(/>/g, '&gt;')
                .replace(/"/g, '&quot;')
                .replace(/'/g, '&#39;');
        }
        async function startTeleop() {
            const source = document.getElementById('source-server-select').value;
            const target = document.getElementById('target-server-select').value;
            if (!source || !target) {
                showToast(t('msg-select-servers'), 'error');
                return;
            }
            const controlFlows = collectControlFlows();
            if (controlFlows.length === 0) {
                showToast(t('msg-select-control-flow'), 'error');
                return;
            }
            const btn = document.getElementById('btn-start-teleop');
            if (btn) btn.disabled = true;
            try {
                const resp = await fetch('/api/teleop/start', {
                    method:'POST',
                    headers:{'Content-Type':'application/json'},
                    body: JSON.stringify({
                        source_robot_id: source,
                        target_robot_id: target,
                        control_flows: controlFlows,
                    })
                });
                const d = await resp.json();
                if (d.connection_status) applyConnectionStatus(d.connection_status);
                if (d.preview_keys) applyPreviewKeys(d.preview_keys);
                if (d.success) {
                    teleopEnabled = true;
                    updateUI();
                    showToast(t('msg-teleop-started'),'success');
                } else {
                    showToast(t('msg-start-failed') + (d.message || ''), 'error', 5000);
                    updateUI();
                }
            } catch (e) {
                showToast(t('msg-start-failed') + e, 'error', 5000);
                updateUI();
            }
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
                max_duration_s: Math.max(1, (parseFloat(document.getElementById('input-max-record-min').value) || 10) * 60),
                collection_id: collectionId,
                keys: selectedKeys,
            };
            fetch('/api/record/start', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)}).then(r=>r.json()).then(d => {
                if (d.success) {
                    recordingEnabled = true;
                    collectionConfigLocked = true;
                    updateUI();
                    showToast(t('msg-record-started') + d.episode, 'success');
                } else {
                    showToast(t('msg-start-failed')+d.message,'error');
                }
            });
        }
        function stopRecording() {
            fetch('/api/record/stop', {method:'POST'}).then(r=>r.json()).then(d => {
                recordingEnabled = false; updateUI();
                const frames = d.frames_written || 0;
                if (frames > 0) {
                    if (confirm(t('msg-collected') + frames + ' ' + t('msg-record-confirm'))) {
                        showToast(t('msg-data-kept') + d.episode, 'success');
                        refreshRecords();
                    } else {
                        fetch('/api/record/discard', {method:'POST'}).then(r=>r.json()).then(x => {
                            if (x.success) {
                                showToast(t('msg-data-discarded'), 'info');
                                refreshRecords();
                            } else {
                                showToast(t('msg-discard-failed') + (x.message || ''), 'error');
                            }
                        });
                    }
                } else {
                    showToast(t('msg-no-data'), 'info');
                }
            });
        }

        function exportData() {
            const paths = collectionEpisodePaths(collectionId);
            if (!paths.length) {
                showToast(t('msg-no-episodes-in-coll'), 'error');
                return;
            }
            exportPaths(paths);
        }
        let videoGridInited = false;
        function normalizeJointPositions(value) {
            if (!Array.isArray(value)) return [];
            return value.map(v => Number(v)).filter(v => Number.isFinite(v));
        }
        function appendJointSample(history, positions) {
            if (!positions.length) return;
            history.push(positions.slice());
            if (history.length > JOINT_HISTORY_LIMIT) {
                history.splice(0, history.length - JOINT_HISTORY_LIMIT);
            }
        }
        function jointLabel(index) {
            return `#${index}`;
        }
        function ensureJointSelector(positions) {
            if (!positions.length || positions.length <= jointDimensionCount) return;
            const oldCount = jointDimensionCount;
            jointDimensionCount = positions.length;
            if (selectedJointIndices.size === 0) {
                for (let i = 0; i < Math.min(jointDimensionCount, 6); i++) {
                    selectedJointIndices.add(i);
                }
            }
            renderJointLegend();
            if (oldCount > 0) redrawJointCharts();
        }
        function renderJointLegend() {
            const legend = document.getElementById('joint-chart-legend');
            if (!legend) return;
            legend.innerHTML = '';
            for (let i = 0; i < jointDimensionCount; i++) {
                const label = document.createElement('label');
                label.style.setProperty('--joint-color', JOINT_COLORS[i % JOINT_COLORS.length]);
                const checkbox = document.createElement('input');
                checkbox.type = 'checkbox';
                checkbox.checked = selectedJointIndices.has(i);
                checkbox.addEventListener('change', () => {
                    if (checkbox.checked) selectedJointIndices.add(i);
                    else selectedJointIndices.delete(i);
                    redrawJointCharts();
                });
                const swatch = document.createElement('span');
                swatch.className = 'joint-chart-swatch';
                const text = document.createElement('span');
                text.textContent = jointLabel(i);
                label.appendChild(checkbox);
                label.appendChild(swatch);
                label.appendChild(text);
                legend.appendChild(label);
            }
        }
        function updateJointChart(kind, positions) {
            ensureJointSelector(positions);
            if (kind === 'leader') latestLeaderJointPositions = positions.slice();
            else latestFollowerJointPositions = positions.slice();
        }
        function tickJointCharts() {
            let changed = false;
            if (latestFollowerJointPositions.length) {
                appendJointSample(followerStateHistory, latestFollowerJointPositions);
                changed = true;
            }
            if (latestLeaderJointPositions.length) {
                appendJointSample(leaderStateHistory, latestLeaderJointPositions);
                changed = true;
            }
            if (changed) redrawJointCharts();
        }
        function drawJointChart(canvasId, emptyId, history, selectedIndices = selectedJointIndices) {
            const canvas = document.getElementById(canvasId);
            const empty = document.getElementById(emptyId);
            if (!canvas) return;
            if (empty) empty.style.display = history.length ? 'none' : 'flex';
            const rect = canvas.getBoundingClientRect();
            const cssW = Math.max(1, Math.floor(rect.width));
            const cssH = Math.max(1, Math.floor(rect.height));
            const dpr = Math.max(1, window.devicePixelRatio || 1);
            if (canvas.width !== Math.floor(cssW * dpr) || canvas.height !== Math.floor(cssH * dpr)) {
                canvas.width = Math.floor(cssW * dpr);
                canvas.height = Math.floor(cssH * dpr);
            }
            const ctx = canvas.getContext('2d');
            ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            ctx.clearRect(0, 0, cssW, cssH);
            ctx.fillStyle = '#202437';
            ctx.fillRect(0, 0, cssW, cssH);
            if (!history.length) return;

            const padL = 36, padR = 8, padT = 12, padB = 22;
            const plotW = Math.max(1, cssW - padL - padR);
            const plotH = Math.max(1, cssH - padT - padB);
            const values = [];
            const selected = Array.from(selectedIndices || []).sort((a, b) => a - b);
            if (!selected.length) return;
            history.forEach(row => selected.forEach(i => {
                const v = row[i];
                if (Number.isFinite(v)) values.push(v);
            }));
            let min = Math.min(...values);
            let max = Math.max(...values);
            if (!Number.isFinite(min) || !Number.isFinite(max)) return;
            if (Math.abs(max - min) < 1e-6) {
                min -= 0.1;
                max += 0.1;
            } else {
                const padding = (max - min) * 0.12;
                min -= padding;
                max += padding;
            }
            const yOf = v => padT + (max - v) / (max - min) * plotH;
            const xOf = i => {
                const denom = Math.max(JOINT_HISTORY_LIMIT - 1, 1);
                const offset = JOINT_HISTORY_LIMIT - history.length;
                return padL + (offset + i) / denom * plotW;
            };

            ctx.strokeStyle = 'rgba(255,255,255,0.08)';
            ctx.lineWidth = 1;
            ctx.beginPath();
            for (let i = 0; i <= 3; i++) {
                const y = padT + (plotH * i / 3);
                ctx.moveTo(padL, y);
                ctx.lineTo(cssW - padR, y);
            }
            ctx.stroke();
            ctx.fillStyle = '#888';
            ctx.font = '10px monospace';
            ctx.fillText(max.toFixed(2), 4, padT + 3);
            ctx.fillText(min.toFixed(2), 4, padT + plotH);
            ctx.fillText('-200', padL, cssH - 7);
            ctx.fillText('now', cssW - padR - 24, cssH - 7);

            selected.forEach(joint => {
                ctx.strokeStyle = JOINT_COLORS[joint % JOINT_COLORS.length];
                ctx.lineWidth = 2;
                ctx.beginPath();
                let started = false;
                history.forEach((row, i) => {
                    const v = row[joint];
                    if (!Number.isFinite(v)) return;
                    const x = xOf(i);
                    const y = yOf(v);
                    if (!started) {
                        ctx.moveTo(x, y);
                        started = true;
                    } else {
                        ctx.lineTo(x, y);
                    }
                });
                ctx.stroke();
                if (history.length === 1 && started) {
                    const v = history[0][joint];
                    if (Number.isFinite(v)) {
                        ctx.fillStyle = JOINT_COLORS[joint % JOINT_COLORS.length];
                        ctx.beginPath();
                        ctx.arc(xOf(0), yOf(v), 3, 0, Math.PI * 2);
                        ctx.fill();
                    }
                }
            });
        }
        function redrawJointCharts() {
            drawJointChart('follower-state-chart', 'follower-state-empty', followerStateHistory);
            drawJointChart('leader-state-chart', 'leader-state-empty', leaderStateHistory);
            redrawStateCardCharts();
        }
        window.addEventListener('resize', redrawJointCharts);
        setInterval(() => requestAnimationFrame(tickJointCharts), JOINT_RENDER_INTERVAL_MS);
        function applyImageUpdate(d) {
            const grid = document.getElementById('video-grid');
            const keys = Object.keys(d || {}).sort();
            if (!keys.length) return;
            lastImageUpdateAt = Date.now();
            if (!videoGridInited) { grid.innerHTML = ''; videoGridInited = true; }
            keys.forEach(k => {
                let img = document.getElementById('cam-' + k);
                if (!img) {
                    const wrap = document.createElement('div');
                    wrap.className = 'video-container';
                    const item = recordItemFromName(k);
                    wrap.innerHTML = '<span class="video-label">' + (item.component_name || k) + ' · ' + (item.type || item.name || 'image') + '</span>';
                    img = document.createElement('img');
                    img.id = 'cam-' + k;
                    img.alt = k;
                    wrap.appendChild(img);
                    grid.appendChild(wrap);
                }
                const url = imagePayloadUrl(d[k]);
                if (!url) return;
                const previous = imageObjectUrls.get(k);
                img.src = url;
                if (previous) URL.revokeObjectURL(previous);
                if (url.startsWith('blob:')) imageObjectUrls.set(k, url);
                else imageObjectUrls.delete(k);
            });
        }
        function imagePayloadUrl(payload) {
            if (!payload) return '';
            if (typeof payload === 'string') {
                if (payload.startsWith('data:')) return payload;
                return 'data:image/jpeg;base64,' + payload;
            }
            let blob = null;
            if (payload instanceof Blob) {
                blob = payload;
            } else if (payload instanceof ArrayBuffer) {
                blob = new Blob([payload], {type: 'image/jpeg'});
            } else if (ArrayBuffer.isView(payload)) {
                const view = payload;
                blob = new Blob(
                    [view.buffer.slice(view.byteOffset, view.byteOffset + view.byteLength)],
                    {type: 'image/jpeg'}
                );
            }
            return blob ? URL.createObjectURL(blob) : '';
        }
        function releaseImageObjectUrls() {
            imageObjectUrls.forEach(url => URL.revokeObjectURL(url));
            imageObjectUrls.clear();
        }
        function firstJointPositions(state) {
            const values = Object.values(state || {});
            for (const value of values) {
                const positions = normalizeJointPositions((value || {}).joint_positions || []);
                if (positions.length) return positions;
            }
            return [];
        }
        function renderStateList(elementId, state, items) {
            const el = document.getElementById(elementId);
            if (!el) return;
            const entries = Object.entries(state || {}).sort((a, b) => a[0].localeCompare(b[0]));
            if (!entries.length) {
                el.textContent = t('waiting-data');
                return;
            }
            const descriptors = new Map((items || []).map(item => [String(item.protocol_name || item.name || ''), item]));
            el.innerHTML = '';
            entries.forEach(([key, value]) => {
                const item = descriptors.get(key) || recordItemFromName(key);
                const row = document.createElement('div');
                row.title = key;
                row.style.cssText = 'display:grid;grid-template-columns:minmax(120px,0.8fr) minmax(160px,1.2fr);gap:8px;align-items:start;padding:4px 0;border-bottom:1px solid rgba(255,255,255,0.06);';
                const name = document.createElement('span');
                name.style.cssText = 'color:#00d4ff;font-weight:600;overflow-wrap:anywhere;';
                name.textContent = `${item.component_name || '-'} · ${item.name || item.type || '-'}`;
                const body = document.createElement('span');
                body.style.cssText = 'color:#d8d8d8;overflow-wrap:anywhere;';
                body.textContent = formatStateValue(value);
                row.appendChild(name);
                row.appendChild(body);
                el.appendChild(row);
            });
        }
        function formatStateValue(value) {
            if (value && Array.isArray(value.joint_positions)) {
                return value.joint_positions.map((v, i) => `${jointLabel(i)}:${Number(v).toFixed(3)}`).join('  ');
            }
            if (value && typeof value === 'object') {
                return JSON.stringify(value);
            }
            return String(value == null ? '' : value);
        }
        function safeDomId(text) {
            return String(text || '').replace(/[^a-zA-Z0-9_-]/g, '_');
        }
        function descriptorMap(items) {
            return new Map((items || []).map(item => [String(item.protocol_name || item.name || ''), item]));
        }
        function isPoseValue(value) {
            return value
                && Array.isArray(value.position)
                && Array.isArray(value.orientation_quat_xyzw);
        }
        function formatNumberList(values, digits = 4) {
            return (values || []).map(v => {
                const num = Number(v);
                return Number.isFinite(num) ? num.toFixed(digits) : String(v);
            }).join(', ');
        }
        function renderPoseSummary(root, value) {
            const pos = value && Array.isArray(value.position) ? value.position : [];
            const quat = value && Array.isArray(value.orientation_quat_xyzw) ? value.orientation_quat_xyzw : [];
            root.innerHTML = `
                <div style="display:grid;grid-template-columns:72px 1fr;gap:6px 10px;font-family:monospace;font-size:12px;color:#d8d8d8;">
                    <span style="color:#888;">position</span>
                    <span>${formatNumberList(pos)}</span>
                    <span style="color:#888;">quat xyzw</span>
                    <span>${formatNumberList(quat)}</span>
                </div>
            `;
        }
        function getStateChartStore(storeKey) {
            let store = stateChartStores.get(storeKey);
            if (!store) {
                store = {
                    history: [],
                    selected: new Set(),
                    dimensionCount: 0,
                    canvasId: '',
                    emptyId: '',
                };
                stateChartStores.set(storeKey, store);
            }
            return store;
        }
        function ensureChartDimensions(store, positions) {
            if (!positions.length || positions.length <= store.dimensionCount) return false;
            const oldCount = store.dimensionCount;
            store.dimensionCount = positions.length;
            if (store.selected.size === 0) {
                for (let i = 0; i < Math.min(store.dimensionCount, 6); i++) {
                    store.selected.add(i);
                }
            }
            return oldCount !== store.dimensionCount;
        }
        function renderCardJointLegend(legend, store, redraw) {
            if (!legend) return;
            legend.innerHTML = '';
            for (let i = 0; i < store.dimensionCount; i++) {
                const label = document.createElement('label');
                label.style.setProperty('--joint-color', JOINT_COLORS[i % JOINT_COLORS.length]);
                const checkbox = document.createElement('input');
                checkbox.type = 'checkbox';
                checkbox.checked = store.selected.has(i);
                checkbox.addEventListener('change', () => {
                    if (checkbox.checked) store.selected.add(i);
                    else store.selected.delete(i);
                    redraw();
                });
                const swatch = document.createElement('span');
                swatch.className = 'joint-chart-swatch';
                const text = document.createElement('span');
                text.textContent = jointLabel(i);
                label.appendChild(checkbox);
                label.appendChild(swatch);
                label.appendChild(text);
                legend.appendChild(label);
            }
        }
        function redrawStateCardCharts() {
            stateChartStores.forEach(store => {
                if (store.canvasId && store.emptyId) {
                    drawJointChart(store.canvasId, store.emptyId, store.history, store.selected);
                }
            });
        }
        function renderStateCards(side, containerId, state, items) {
            const root = document.getElementById(containerId);
            if (!root) return;
            const descriptors = descriptorMap(items);
            const orderedKeys = [];
            (items || []).forEach(item => {
                const key = String(item.protocol_name || item.name || '');
                if (key && !orderedKeys.includes(key)) orderedKeys.push(key);
            });
            Object.keys(state || {}).sort((a, b) => a.localeCompare(b)).forEach(key => {
                if (!orderedKeys.includes(key)) orderedKeys.push(key);
            });
            if (!orderedKeys.length) {
                root.innerHTML = '<span style="color:#666;font-size:13px;">' + t('waiting-data') + '</span>';
                return;
            }
            const seen = new Set();
            orderedKeys.forEach(key => {
                const value = (state || {})[key];
                seen.add(key);
                const item = descriptors.get(key) || recordItemFromName(key);
                const cardId = `${side}-state-card-${safeDomId(key)}`;
                let card = document.getElementById(cardId);
                if (!card) {
                    card = document.createElement('div');
                    card.id = cardId;
                    card.style.cssText = 'background:#252535;border:1px solid #333;border-radius:8px;padding:10px;';
                    card.innerHTML = `
                        <div style="display:flex;justify-content:space-between;gap:10px;align-items:center;margin-bottom:8px;">
                            <span class="state-card-title" style="color:#00d4ff;font-weight:700;overflow-wrap:anywhere;"></span>
                            <span class="state-card-type" style="color:#888;font-size:12px;"></span>
                        </div>
                        <div class="joint-chart-wrap" style="display:none;">
                            <canvas></canvas>
                            <div class="joint-chart-empty" data-i18n="waiting-data">等待数据...</div>
                        </div>
                        <div class="joint-chart-legend" style="display:none;"></div>
                        <div class="pose-summary-wrap" style="display:none;"></div>
                        <div class="joint-chart-raw"></div>
                    `;
                    root.appendChild(card);
                }
                card.title = key;
                card.querySelector('.state-card-title').textContent = `${item.component_name || '-'} · ${item.name || item.type || '-'}`;
                card.querySelector('.state-card-type').textContent = item.type || '';
                const raw = card.querySelector('.joint-chart-raw');
                raw.textContent = value == null ? t('waiting-data') : formatStateValue(value);
                const positions = normalizeJointPositions((value || {}).joint_positions || []);
                const chartWrap = card.querySelector('.joint-chart-wrap');
                const legend = card.querySelector('.joint-chart-legend');
                const poseWrap = card.querySelector('.pose-summary-wrap');
                if (poseWrap) poseWrap.style.display = 'none';
                if (legend) legend.style.display = 'none';
                if (positions.length) {
                    chartWrap.style.display = 'block';
                    const storeKey = `${side}:${key}`;
                    const store = getStateChartStore(storeKey);
                    const dimensionsChanged = ensureChartDimensions(store, positions);
                    appendJointSample(store.history, positions);
                    const canvas = chartWrap.querySelector('canvas');
                    const empty = chartWrap.querySelector('.joint-chart-empty');
                    const canvasId = `${cardId}-canvas`;
                    const emptyId = `${cardId}-empty`;
                    canvas.id = canvasId;
                    empty.id = emptyId;
                    store.canvasId = canvasId;
                    store.emptyId = emptyId;
                    if (legend) {
                        legend.style.display = 'flex';
                        if (dimensionsChanged || !legend.children.length) {
                            renderCardJointLegend(legend, store, () => drawJointChart(canvasId, emptyId, store.history, store.selected));
                        }
                    }
                    drawJointChart(canvasId, emptyId, store.history, store.selected);
                } else if (isPoseValue(value)) {
                    chartWrap.style.display = 'none';
                    if (poseWrap) {
                        poseWrap.style.display = 'block';
                        renderPoseSummary(poseWrap, value);
                    }
                    raw.textContent = '';
                } else {
                    chartWrap.style.display = 'none';
                }
            });
            Array.from(root.children).forEach(child => {
                const key = String(child.title || '');
                if (key && !seen.has(key)) {
                    stateChartStores.delete(`${side}:${key}`);
                    child.remove();
                }
            });
        }
        function applyStateUpdate(d) {
            const state = d.state || {};
            const p = firstJointPositions(state);
            if (p.length) lastStateUpdateAt = Date.now();
            renderStateCards('follower', 'follower-state-cards', state, d.items || []);
        }
        function applyLeaderStateUpdate(d) {
            const state = d.state || {};
            const p = firstJointPositions(state);
            if (p.length) lastLeaderStateUpdateAt = Date.now();
            renderStateCards('leader', 'leader-state-cards', state, d.items || []);
        }
        function applyRecordStatus(d) {
            latestRecordStatus = d || {};
            document.getElementById('episode-number').textContent = d.episode_number || 0;
            recordingEnabled = d.recording || false;
            updateRecordKeyFps();
            updateUI();
        }
        function applyTeleopMetrics(d) {
            const num = value => {
                const parsed = Number(value);
                return Number.isFinite(parsed) ? parsed : 0;
            };
            const role = d.role || 'leader';
            const commandFps = role === 'follower'
                ? num(d.command_rx_fps)
                : num(d.command_tx_fps);
            const previewFps = role === 'follower'
                ? num(d.image_tx_fps)
                : num(d.image_rx_fps);
            document.getElementById('command-fps').textContent = commandFps.toFixed(1) + ' Hz';
            document.getElementById('preview-fps').textContent = previewFps.toFixed(1) + ' Hz';
            const latencyEl = document.getElementById('action-latency');
            if (latencyEl) {
                const last = num(d.action_latency_last_ms);
                latencyEl.textContent = last > 0
                    ? `${last.toFixed(1)} ms (${(1000 / last).toFixed(1)} Hz)`
                    : '-';
            }
            const summaryEl = document.getElementById('rpc-latency-summary');
            if (summaryEl) {
                const sourceLast = num(d.source_state_rpc_last_ms);
                const targetLast = num(d.target_state_rpc_last_ms);
                const imageLast = num(d.image_rpc_last_ms);
                const actionLast = num(d.action_rpc_last_ms);
                summaryEl.textContent = `状态 ${sourceLast.toFixed(1)}/${targetLast.toFixed(1)} · 图像 ${imageLast.toFixed(1)} · 动作 ${actionLast.toFixed(1)} ms`;
            }
            const errorEl = document.getElementById('runtime-last-error');
            if (errorEl) {
                const error = String(d.last_error || '').trim();
                errorEl.textContent = error || '-';
                errorEl.className = 'status-value ' + (error ? 'inactive' : 'active');
            }
        }
        function applyFollowerStatus(d) {
            const el = document.getElementById('follower-online');
            if (d.online) {
                el.textContent = t('follower-online');
                el.className = 'status-value active';
            } else {
                el.textContent = t('follower-offline');
                el.className = 'status-value inactive';
            }
        }
        function connectionLabel(status) {
            return t('conn-' + (status || 'unbound')) || status || '-';
        }
        function applyConnectionStatus(d) {
            const source = d.source || {};
            const target = d.target || {};
            updateConnectionEl('source-connection', source.status);
            updateConnectionEl('target-connection', target.status);
            maybePromptReconnect(d);
        }
        function updateConnectionEl(id, status) {
            const el = document.getElementById(id);
            if (!el) return;
            el.textContent = connectionLabel(status);
            const active = status === 'online';
            const warn = status === 'reconnectable' || status === 'reconnecting';
            el.className = 'status-value ' + (active ? 'active' : (warn ? 'warning' : 'inactive'));
        }
        function maybePromptReconnect(status) {
            const data = status || latestConnectionStatus || {};
            latestConnectionStatus = data;
            const states = [data.source?.status, data.target?.status];
            if (!states.includes('reconnectable')) {
                if (states.every(status => status === 'online' || status === 'unbound')) reconnectPromptKey = '';
                return;
            }
            const key = `${data.source?.status || ''}:${data.target?.status || ''}:${data.source?.robot_id || ''}:${data.target?.robot_id || ''}`;
            if (key === reconnectPromptKey) return;
            reconnectPromptKey = key;
            setTimeout(() => {
                if (confirm(t('msg-confirm-reconnect'))) {
                    reconnectServers();
                }
            }, 0);
        }
        socket.on('image_update', applyImageUpdate);
        socket.on('state_update', applyStateUpdate);
        socket.on('leader_state_update', applyLeaderStateUpdate);
        socket.on('record_status', applyRecordStatus);
        socket.on('teleop_metrics', applyTeleopMetrics);
        socket.on('teleop_status', d => { teleopEnabled = d.enabled || false; updateUI(); });
        socket.on('follower_status', applyFollowerStatus);
        socket.on('connection_status', applyConnectionStatus);
        socket.on('preview_keys', applyPreviewKeys);
        socket.on('export_selected_progress', d => {
            const container = document.getElementById('export-selected-progress-container');
            const text = document.getElementById('export-selected-progress-text');
            if (!container || !text) return;
            container.style.display = 'block';
            const progress = formatProgressPercent(normalizeProgressPercent(d.progress));
            text.textContent = `${progress} ${d.message || t('processing')}`;
        });
        // Dynamic buffer keys from follower
        let currentBufferKeys = [];
        let renderedBufferKeys = [];  // Track rendered keys to avoid re-rendering
        function applyBufferKeys(d) {
            const items = normalizeRecordItems(d);
            const keys = items.map(item => item.protocol_name).filter(Boolean);
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
            container.innerHTML = `
                <table style="width:100%;border-collapse:collapse;background:rgba(255,255,255,0.03);border:1px solid rgba(255,255,255,0.08);border-radius:6px;overflow:hidden;font-size:12px;color:#d8d8d8;">
                    <thead>
                        <tr style="background:rgba(0,212,255,0.08);color:#999;text-align:left;">
                            <th style="width:56px;padding:7px 8px;font-weight:600;">${t('table-select')}</th>
                            <th style="padding:7px 8px;font-weight:600;">${t('table-component')}</th>
                            <th style="padding:7px 8px;font-weight:600;">${t('table-name')}</th>
                            <th style="padding:7px 8px;font-weight:600;">${t('table-type')}</th>
                            <th style="padding:7px 8px;font-weight:600;">${t('table-category')}</th>
                            <th style="width:90px;padding:7px 8px;font-weight:600;text-align:right;">${t('table-fps')}</th>
                        </tr>
                    </thead>
                    <tbody></tbody>
                </table>
            `;
            const tbody = container.querySelector('tbody');
            items.forEach(item => {
                const row = document.createElement('tr');
                row.title = item.protocol_name;
                row.style.cssText = 'border-top:1px solid rgba(255,255,255,0.06);cursor:pointer;';
                const checkbox = document.createElement('input');
                checkbox.type = 'checkbox';
                checkbox.className = 'record-key';
                checkbox.value = item.protocol_name;
                checkbox.checked = defaultRecordKeySelected(item);
                const selectCell = document.createElement('td');
                selectCell.style.cssText = 'padding:7px 8px;text-align:center;';
                selectCell.appendChild(checkbox);
                row.appendChild(selectCell);
                [
                    {text: item.component_name || '-', style: 'color:#00d4ff;font-weight:700;'},
                    {text: item.name || '-', style: 'color:#e8e8e8;font-weight:700;'},
                    {text: item.type || '-', style: 'color:#aaa;'},
                    {text: item.category || '-', style: 'color:#aaa;'},
                ].forEach(cell => {
                    const td = document.createElement('td');
                    td.textContent = cell.text;
                    td.style.cssText = 'padding:7px 8px;overflow-wrap:anywhere;' + cell.style;
                    row.appendChild(td);
                });
                const fpsCell = document.createElement('td');
                fpsCell.className = 'record-key-fps';
                fpsCell.dataset.key = item.protocol_name;
                fpsCell.textContent = '-';
                fpsCell.style.cssText = 'padding:7px 8px;text-align:right;color:#00ff88;font-weight:700;font-variant-numeric:tabular-nums;';
                row.appendChild(fpsCell);
                row.onclick = event => {
                    if (event.target !== checkbox) checkbox.checked = !checkbox.checked;
                };
                tbody.appendChild(row);
            });
            updateRecordKeyFps();
        }
        function updateRecordKeyFps() {
            const status = latestRecordStatus || {};
            const streamStats = status.stream_stats || {};
            document.querySelectorAll('.record-key-fps').forEach(cell => {
                const key = cell.dataset.key || '';
                const stats = streamStats[key] || {};
                if (!stats || stats.fps == null) {
                    cell.textContent = status.recording ? 'ERR' : '-';
                    cell.style.color = status.recording ? '#ff5b73' : '#00ff88';
                    return;
                }
                const streamFps = Number(stats.fps || 0);
                cell.style.color = '#00ff88';
                cell.textContent = streamFps > 0 ? `${streamFps.toFixed(1)} Hz` : '-';
            });
        }
        function normalizeRecordItems(d) {
            d = d || {};
            const items = Array.isArray(d.items) ? d.items : [];
            if (items.length) return items.map(item => ({
                protocol_name: String(item.protocol_name || item.name || ''),
                category: String(item.category || ''),
                component_name: String(item.component_name || ''),
                name: String(item.name || item.object_name || ''),
                type: String(item.type || ''),
                description: String(item.description || ''),
            })).filter(item => item.protocol_name);
            return (d.keys || []).map(recordItemFromName);
        }
        function defaultRecordKeySelected(item) {
            if (String(item.category || '').toLowerCase() !== 'action') return true;
            const type = String(item.type || '').toLowerCase();
            return type !== 'prearranged' && type !== 'custom';
        }
        function recordItemFromName(name) {
            const text = String(name || '');
            const parts = text.split('.');
            return {
                protocol_name: text,
                category: parts[0] || '',
                component_name: parts.length >= 3 ? parts[1] : '',
                name: parts.length >= 3 ? parts[2] : (parts[parts.length - 1] || text),
                type: parts.length >= 3 ? parts[2] : '',
                description: '',
            };
        }
        socket.on('buffer_keys', applyBufferKeys);

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

        function formatBytesCompact(bytes) {
            if (bytes === 0) return '0 B';
            const k = 1024;
            const sizes = ['B', 'KB', 'MB', 'GB'];
            const i = Math.floor(Math.log(bytes) / Math.log(k));
            return Math.round(bytes / Math.pow(k, i)) + ' ' + sizes[i];
        }

        function copyText(value) {
            const text = String(value || '');
            if (!text) return;
            const done = () => showToast('已复制路径', 'success');
            const failed = () => showToast('复制失败', 'error');
            if (navigator.clipboard && navigator.clipboard.writeText) {
                navigator.clipboard.writeText(text).then(done).catch(failed);
                return;
            }
            const input = document.createElement('textarea');
            input.value = text;
            document.body.appendChild(input);
            input.select();
            try {
                document.execCommand('copy') ? done() : failed();
            } catch (e) {
                failed();
            } finally {
                document.body.removeChild(input);
            }
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
            const diskBreakdownEl = document.getElementById('disk-usage-breakdown');
            const exportDirInfoEl = document.getElementById('export-dir-info');
            const exportDirPathEl = document.getElementById('export-dir-path');

            const localRecords = currentRecords.local || currentRecords;
            const locations = currentRecords.locations || [
                localRecords,
                currentRecords.source_server || {key: 'source', label: '控制端 Server', online: false, data_collections: [], logs: []},
                currentRecords.target_server || currentRecords.server || {key: 'target', label: '被控端 Server', online: false, data_collections: [], logs: []},
            ];

            const diskRows = locations
                .filter(loc => Number(loc.total_size || 0) > 0 || loc.location_type === 'app' || loc.online)
                .map(loc => {
                    const label = loc.label || loc.key || '-';
                    const detail = loc.robot_id ? ' · ' + loc.robot_id : '';
                    const size = loc.total_size_formatted || formatBytes(Number(loc.total_size || 0));
                    return `<div style="display:flex;justify-content:space-between;gap:10px;font-size:13px;">
                        <span style="color:#bbb;min-width:0;overflow-wrap:anywhere;">${esc(label)}${esc(detail)}</span>
                        <span style="color:#00d4ff;font-weight:700;flex-shrink:0;">${esc(size)}</span>
                    </div>`;
                });
            if (diskRows.length) {
                diskBreakdownEl.innerHTML = diskRows.join('');
                diskUsageEl.style.display = 'block';
            } else {
                diskUsageEl.style.display = 'none';
            }
            if (localRecords.export_dir) {
                exportDirPathEl.textContent = localRecords.export_dir;
                exportDirInfoEl.style.display = 'flex';
            } else {
                exportDirInfoEl.style.display = 'none';
            }

            const hasAny = locations.some(loc =>
                (loc.data_collections || []).length || (loc.exported_zips || []).length || (loc.logs || []).length
            );
            if (!hasAny) {
                container.innerHTML = '<div style="color: #888; text-align: center; padding: 20px;">' + t('records-no-data') + '</div>';
                return;
            }

            let html = '';
            locations.forEach(loc => {
                html += renderLocationSection(loc);
            });

            container.innerHTML = html;
        }

        function renderLocationSection(loc) {
            const key = loc.key || loc.side || loc.location_type || 'location';
            const isServer = loc.location_type === 'server' || key === 'source' || key === 'target';
            const label = loc.label || (isServer ? 'Server' : 'App 本机');
            const detail = isServer
                ? [loc.robot_id || '', loc.endpoint || ''].filter(Boolean).join(' · ')
                : (loc.endpoint || 'local filesystem');
            const onlineText = isServer ? (loc.online ? '在线' : `离线${loc.message ? ' · ' + loc.message : ''}`) : '本机';
            const onlineColor = isServer && !loc.online ? '#ff5b73' : '#24ff83';
            const sizeText = loc.total_size_formatted || formatBytes(Number(loc.total_size || 0));
            const hasContent = (loc.data_collections || []).length || (loc.exported_zips || []).length || (loc.logs || []).length;
            const hint = loc.message ? `<div style="color:#ffd166;font-size:12px;margin-top:5px;">${esc(loc.message)}</div>` : '';
            let html = `<div class="records-location">
                <div class="records-location-head">
                    <div style="min-width:0;">
                        <div class="records-location-title" title="${esc(label)}">${esc(label)}</div>
                        <div class="records-location-detail" title="${esc(detail)}">${esc(detail)}</div>
                        ${hint}
                    </div>
                    <div class="records-location-status">
                        <div style="color:${onlineColor};font-size:12px;font-weight:700;">${esc(onlineText)}</div>
                        <div style="color:#00d4ff;font-size:12px;font-weight:700;margin-top:4px;">${esc(sizeText)}</div>
                    </div>
                </div>`;
            if (!hasContent) {
                html += `<div style="color:#777;padding:10px 0;">暂无资源</div></div>`;
                return html;
            }
            html += renderCollectionSection('原始采集数据', loc.data_collections || [], key);
            if (!isServer) {
                html += renderExportedZips(loc.exported_zips || []);
            }
            html += renderLogResources(t('label-log-resources'), loc.logs || []);
            html += `</div>`;
            return html;
        }

        function renderLogResources(title, logs) {
            if (!logs.length) return '';
            let html = `<div class="exported-zips-section">
                <div style="margin-bottom: 8px;">
                    <span style="color: #888; font-size: 13px;">${esc(title)}</span>
                </div>`;
            const groups = groupLogResources(logs);
            Object.keys(groups).sort().forEach(group => {
                html += `<div class="log-group-title">${esc(group)}</div>
                <div style="overflow-x:auto;border:1px solid rgba(255,255,255,0.08);border-radius:6px;background:rgba(255,255,255,0.03);">
                    <table style="width:100%;border-collapse:collapse;table-layout:fixed;font-size:12px;color:#d8d8d8;">
                        <colgroup>
                            <col style="width:150px;">
                            <col>
                            <col style="width:64px;">
                            <col style="width:118px;">
                        </colgroup>
                        <thead>
                            <tr style="background:rgba(0,212,255,0.08);color:#999;text-align:left;">
                                <th style="padding:8px 10px;font-weight:700;">${t('table-log-type')}</th>
                                <th style="padding:8px 10px;font-weight:700;">${t('table-file')}</th>
                                <th style="padding:8px 10px;font-weight:700;text-align:right;white-space:nowrap;">${t('table-size')}</th>
                                <th style="padding:8px 10px;font-weight:700;text-align:right;white-space:nowrap;"></th>
                            </tr>
                        </thead>
                        <tbody>`;
                groups[group].forEach(resource => {
                const info = logResourceInfo(resource);
                const url = serverLogUrl(resource, false);
                const downloadUrl = serverLogUrl(resource, true);
                const sideArg = JSON.stringify(resource.side || '');
                const resourceIdArg = JSON.stringify(resource.resource_id || '');
                html += `<tr style="border-top:1px solid rgba(255,255,255,0.06);" title="${esc(resource.name || resource.resource_id)}">
                    <td style="padding:9px 10px;color:#d8d8d8;vertical-align:middle;">
                        <div style="font-weight:700;white-space:nowrap;">${esc(info.type)}</div>
                        ${info.detail ? `<div style="margin-top:2px;color:#8d93a3;font-size:11px;white-space:nowrap;">${esc(info.detail)}</div>` : ''}
                    </td>
                    <td style="padding:9px 10px;color:#e8e8e8;vertical-align:middle;min-width:0;">
                        ${info.dir ? `<div style="color:#8d93a3;font-size:11px;line-height:1.25;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;">${esc(info.dir)}/</div>` : ''}
                        <div style="font-weight:700;line-height:1.35;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;">${esc(info.base)}</div>
                    </td>
                    <td style="padding:9px 8px;text-align:right;color:#00ff88;font-weight:800;font-variant-numeric:tabular-nums;white-space:nowrap;vertical-align:middle;">${formatBytesCompact(Number(resource.size_bytes || 0))}</td>
                    <td style="padding:9px 8px;vertical-align:middle;">
                        <div class="log-actions">
                            ${url ? `<button onclick="window.open('${esc(url)}', '_blank')">${t('label-view')}</button>` : ''}
                            ${downloadUrl ? `<button onclick="window.open('${esc(downloadUrl)}', '_blank')" style="background:#4CAF50;">${t('label-download')}</button>` : ''}
                            <button onclick='deleteServerLog(${sideArg}, ${resourceIdArg})' style="background:#ff4757;">${t('btn-delete')}</button>
                        </div>
                    </td>
                </tr>`;
                });
                html += `</tbody></table></div>`;
            });
            html += `</div>`;
            return html;
        }

        function groupLogResources(logs) {
            const groups = {};
            logs.forEach(resource => {
                const info = logResourceInfo(resource);
                const key = `${info.source} · ${info.type}`;
                (groups[key] ||= []).push(resource);
            });
            return groups;
        }

        function serverLogUrl(resource, download) {
            const side = resource.side || '';
            const resourceId = resource.resource_id || '';
            const name = String(resource.name || resource.resource_id || 'log.txt').split('/').pop();
            if (!side || !resourceId) return '';
            return `/api/server_resources/log?side=${encodeURIComponent(side)}&resource_id=${encodeURIComponent(resourceId)}&name=${encodeURIComponent(name)}${download ? '&download=1' : ''}`;
        }

        function logResourceInfo(resource) {
            const file = String(resource.name || resource.resource_id || '');
            const source = resource.side === 'source'
                ? '控制端'
                : (resource.side === 'target' ? '被控端' : '本机');
            let type = 'Log';
            let detail = '';
            const base = file.split('/').pop() || file;
            const dir = file.includes('/') ? file.split('/').slice(0, -1).join('/') : '';
            if (base === 'server.log') {
                type = 'Server';
            } else if (base.startsWith('process_') && base.endsWith('.log')) {
                type = 'Component';
                detail = base.replace(/^process_/, '').replace(/\.log$/, '');
            } else if (file.includes('/apps/')) {
                type = 'App';
            }
            return {source, type, detail, dir, base, file};
        }

        function renderExportedZips(zips) {
            if (!zips.length) return '';
            let html = `<div class="exported-zips-section">
                <div style="margin-bottom: 8px;">
                    <span style="color: #888; font-size: 13px;">${t('label-exported-files')}</span>
                </div>`;
            zips.forEach(zip => {
                const zipPathArg = JSON.stringify(zip.path || '');
                html += `<div class="exported-zip-item" title="${esc(zip.path)}">
                    <span class="zip-name">${esc(zip.name)}</span>
                    <span class="zip-actions">
                        <span class="zip-time">${t('label-export-time')}: ${esc(zip.exported_at || '-')}</span>
                        <span class="zip-size">${formatBytes(zip.size)}</span>
                        <button onclick='copyText(${zipPathArg})' style="background:#607D8B;">复制路径</button>
                        <button onclick='deleteExportedZip(${zipPathArg})'>${t('btn-delete')}</button>
                    </span>
                </div>`;
            });
            html += `</div>`;
            return html;
        }

        function renderCollectionSection(title, collections, location) {
            if (!collections.length) return '';
            let html = `<div class="records-section-title">${esc(title)}</div>`;
            collections.forEach(dc => {
                const sectionId = `${location}-${dc.collection_id}`.replace(/[^0-9a-zA-Z_-]/g, '_');
                const episodeCount = countCollectionEpisodes(dc);
                html += `<div class="data-coll-item">
                    <div class="data-coll-header" onclick="toggleDataColl('${sectionId}')">
                        <div class="data-coll-title" title="${esc(dc.collection_id)}">
                            <div class="data-coll-name">📁 ${esc(dc.collection_id)}</div>
                            <div class="data-coll-meta">${esc(dc.size_formatted || '0 B')} · ${episodeCount} ${t('label-episodes')}</div>
                        </div>
                        <div class="data-coll-actions" onclick="event.stopPropagation()">
                            <button onclick='exportDataCollection(${JSON.stringify(dc.collection_id)})' style="background:#4CAF50;">${t('btn-export-all')}</button>
                            <button onclick='deleteDataCollectionEpisodes(${JSON.stringify(dc.collection_id)})'>${t('btn-delete-all')}</button>
                        </div>
                    </div>
                    <div id="dc-${sectionId}" style="display: none; padding: 10px;">`;

                (dc.task_prompts || []).forEach(tp => {
                    html += `<div class="task-prompt-item">
                        <div class="task-prompt-name">📂 ${esc(tp.task_prompt)}</div>`;

                    (tp.episodes || []).forEach(ep => {
                        const statusClass = ep.status;
                        const statusText = t('status-' + ep.status) || t('status-unknown');
                        const isRemote = ep.location === 'remote' || ep.location === 'server';
                        const canExport = ep.status !== 'invalid';
                        const canPlayback = ep.status === 'complete' || ep.status === 'incomplete';
                        const isInvalid = ep.status === 'invalid';
                        const canSelect = !isInvalid;
                        const side = ep.side || dc.side || location;
                        const remoteTag = isRemote ? ` · ${esc(side === 'source' ? '控制端' : '被控端')}` : '';
                        const episodePathArg = JSON.stringify(ep.path || '');
                        const episodeNameArg = JSON.stringify(ep.episode_name || '');
                        const resourceIdArg = JSON.stringify(ep.resource_id || '');
                        const sideArg = JSON.stringify(side || '');
                        const epFullName = String(ep.episode_name || ep.episode_id || '');
                        html += `<div class="episode-item">
                            <input type="checkbox" onchange='toggleEpisode(${episodePathArg}, this.checked)' ${canSelect ? '' : 'disabled'}>
                            <span class="episode-name" title="${esc(epFullName)}${remoteTag}">${esc(epFullName)}${remoteTag}</span>
                            <span class="episode-status ${statusClass}">${statusText}</span>
                            <div class="episode-actions">
                                ${canPlayback ? `<button onclick='playbackEpisode(${episodePathArg}, ${episodeNameArg})' style="background: #9C27B0;">${t('btn-playback')}</button>` : ''}
                                ${canExport ? `<button onclick='exportEpisode(${episodePathArg})' style="background: #4CAF50;">${t('btn-export-ep')}</button>` : ''}
                                ${isRemote ? `<button onclick='deleteServerCollection(${sideArg}, ${resourceIdArg})'>${t('btn-delete')}</button>` : `<button onclick='deleteEpisode(${episodePathArg})'>${t('btn-delete')}</button>`}
                            </div>
                        </div>`;
                    });

                    html += `</div>`;
                });

                html += `</div></div>`;
            });
            return html;
        }

        function countCollectionEpisodes(dc) {
            return (dc.task_prompts || []).reduce((total, tp) => total + ((tp.episodes || []).length), 0);
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
            const exportBtn = document.getElementById('btn-export-selected');
            if (exportBtn) exportBtn.disabled = !hasSelection;
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
                const result = d.result || d;
                if (result.success) {
                    showToast(t('msg-deleted'), 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-delete-failed') + (result.message || d.message), 'error');
                }
            });
        }

        function collectionEpisodePaths(collectionId) {
            const paths = [];
            const seen = new Set();
            const locations = currentRecords.locations || [{data_collections: currentRecords.data_collections || []}];
            locations.forEach(loc => {
                (loc.data_collections || []).forEach(dc => {
                    if (String(dc.collection_id || '') !== String(collectionId || '')) return;
                    (dc.task_prompts || []).forEach(tp => {
                        (tp.episodes || []).forEach(ep => {
                            if (ep.status === 'invalid' || !ep.path || seen.has(ep.path)) return;
                            seen.add(ep.path);
                            paths.push(ep.path);
                        });
                    });
                });
            });
            return paths;
        }

        function exportDataCollection(collectionId) {
            const paths = collectionEpisodePaths(collectionId);
            if (!paths.length) {
                showToast(t('msg-no-episodes-in-coll'), 'error');
                return;
            }
            if (!confirm(t('msg-confirm-export-coll').replace('{name}', collectionId))) return;
            exportPaths(paths);
        }

        function deleteDataCollectionEpisodes(collectionId) {
            const paths = collectionEpisodePaths(collectionId);
            if (!paths.length) {
                showToast(t('msg-no-episodes-in-coll'), 'error');
                return;
            }
            if (!confirm(t('msg-confirm-delete-coll'))) return;
            Promise.all(paths.map(path => fetch('/api/records/delete', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({path})
            }).then(r => r.json()).catch(e => ({success: false, message: String(e)}))))
              .then(results => {
                  const deleted = results.filter(item => item && item.success).length;
                  const failed = results.length - deleted;
                  if (deleted) showToast(t('msg-coll-deleted') + ` (${deleted})`, 'success');
                  if (failed) showToast(t('msg-delete-failed') + failed, 'error');
                  refreshRecords();
              });
        }

        function deleteServerCollection(side, resourceId) {
            if (!confirm('确认删除 server 端原始数据？')) return;
            fetch('/api/server_resources/delete_collection', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({side: side, resource_id: resourceId})
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    showToast('server 数据已删除', 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-delete-failed') + (d.message || ''), 'error');
                }
            }).catch(e => showToast(t('msg-delete-failed') + e, 'error'));
        }

        function deleteServerLog(side, resourceId) {
            if (!confirm(t('msg-confirm-delete'))) return;
            fetch('/api/server_resources/delete_log', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({side: side, resource_id: resourceId})
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    showToast(t('msg-deleted'), 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-delete-failed') + (d.message || ''), 'error');
                }
            }).catch(e => showToast(t('msg-delete-failed') + e, 'error'));
        }

        function deleteExportedZip(path) {
            if (!confirm(t('msg-confirm-delete-export'))) return;
            fetch('/api/records/delete_export', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({path: path})
            }).then(r => r.json()).then(d => {
                if (d.success) {
                    showToast(t('msg-export-deleted'), 'success');
                    refreshRecords();
                } else {
                    showToast(t('msg-delete-failed') + d.message, 'error');
                }
            });
        }

        function exportedEpisodePaths(result) {
            return result.episode_paths
                || result.exported_episode_paths
                || result.result?.episode_paths
                || result.result?.exported_episode_paths
                || [];
        }

        function maybeDeleteExportedEpisodes(result) {
            const paths = exportedEpisodePaths(result).filter(Boolean);
            if (!paths.length) {
                refreshRecords();
                return;
            }
            if (!confirm(t('msg-confirm-delete-exported-episodes').replace('{n}', paths.length))) {
                refreshRecords();
                return;
            }
            Promise.all(paths.map(path => fetch('/api/records/delete', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({path})
            }).then(r => r.json()).catch(e => ({success: false, message: String(e)}))))
              .then(results => {
                  const deleted = results.filter(item => item && item.success).length;
                  if (deleted) {
                      selectedEpisodes.clear();
                      showToast(t('msg-exported-episodes-deleted') + deleted, 'success');
                  }
                  const failed = results.length - deleted;
                  if (failed) showToast(t('msg-delete-failed') + failed, 'error');
                  refreshRecords();
              });
        }

        function exportEpisode(path) {
            exportPaths([path]);
        }

        function exportSelectedEpisodes() {
            if (selectedEpisodes.size === 0) return showToast(t('msg-select-one'), 'error');
            exportPaths(Array.from(selectedEpisodes));
        }

        function exportPaths(paths) {
            document.getElementById('export-selected-progress-container').style.display = 'block';
            document.getElementById('export-selected-progress-text').textContent = t('msg-export-starting');
            fetch('/api/records/export', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    episode_paths: paths,
                    fps: parseFloat(document.getElementById('input-fps').value) || 30,
                    export_rynnbot_mapping: document.getElementById('input-rynnbot-mapping').checked
                })
            }).then(r => r.json()).then(d => {
                document.getElementById('export-selected-progress-container').style.display = 'none';
                if (d.success) {
                    showToast(t('msg-export-ep-success') + (d.result?.zip_path || d.zip_path || ''), 'success');
                    maybeDeleteExportedEpisodes(d);
                } else {
                    showToast(t('msg-export-ep-failed') + d.message, 'error');
                }
            }).catch(e => {
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
            document.getElementById('playback-title').textContent = t('playback-label');
            document.getElementById('btn-stop-playback').style.display = 'inline-block';

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
                        document.getElementById('playback-status-text').textContent = t('playback-returning');
                        document.getElementById('btn-stop-playback').style.display = 'none';
                        startPlaybackStatusPolling();
                    } else {
                        showToast(t('msg-playback-stop-failed') + (d.message || ''), 'error');
                    }
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
                                } else if (d.data.status === 'stopped') {
                                    showToast(t('msg-playback-stopped'), 'info');
                                    setTimeout(() => {
                                        document.getElementById('playback-progress-container').style.display = 'none';
                                    }, 2000);
                                } else if (d.data.status === 'error') {
                                    showToast(t('msg-playback-error') + d.data.message, 'error');
                                    document.getElementById('btn-stop-playback').style.display = 'none';
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

        function normalizeProgressPercent(value) {
            const numeric = Number(value || 0);
            if (!Number.isFinite(numeric)) return 0;
            const percent = numeric <= 1 ? numeric * 100 : numeric;
            return Math.min(100, Math.max(0, percent));
        }

        function formatProgressPercent(percent) {
            if (percent <= 0 || percent >= 100) return Math.round(percent) + '%';
            return percent.toFixed(1) + '%';
        }

        function updatePlaybackUI(data) {
            const progress = normalizeProgressPercent(data.progress);
            const status = data.status || 'unknown';
            const message = data.message || '';

            document.getElementById('playback-progress-bar').style.width = progress + '%';
            document.getElementById('playback-progress-text').textContent = formatProgressPercent(progress);

            const statusMap = {
                'loading': t('playback-loading'),
                'pre_move': t('playback-pre-move'),
                'returning': t('playback-returning'),
                'waiting': t('playback-waiting'),
                'playing': t('playback-playing'),
                'completed': t('playback-completed'),
                'stopped': t('playback-stopped'),
                'error': t('playback-error-prefix') + message,
            };
            const titleMap = {
                'completed': t('playback-completed'),
                'stopped': t('playback-stopped'),
                'error': t('playback-failed'),
            };
            document.getElementById('playback-title').textContent = titleMap[status] || t('playback-label');
            document.getElementById('btn-stop-playback').style.display = ['completed', 'stopped', 'error'].includes(status) ? 'none' : 'inline-block';
            document.getElementById('playback-status-text').textContent = statusMap[status] || message;
        }

        // Listen for playback status updates via WebSocket
        socket.on('playback_status', d => {
            updatePlaybackUI(d);
        });
        function applySnapshot(data) {
            if (!data || data.success === false) return;
            if (data.images) applyImageUpdate(data.images);
            if (data.state || data.target_state_items) {
                applyStateUpdate({state: data.state || {}, items: data.target_state_items || []});
            }
            if (data.leader_state || data.source_state_items) {
                applyLeaderStateUpdate({state: data.leader_state || {}, items: data.source_state_items || []});
            }
            if (data.record_status) applyRecordStatus(data.record_status);
            if (data.metrics) applyTeleopMetrics(data.metrics);
            if (data.follower_status) applyFollowerStatus(data.follower_status);
            if (data.preview_keys) applyPreviewKeys(data.preview_keys);
            if (data.buffer_keys) applyBufferKeys(data.buffer_keys);
        }
        window.addEventListener('pagehide', () => {
            releaseImageObjectUrls();
            try { socket.disconnect(); } catch (_) {}
        });
        window.addEventListener('beforeunload', () => {
            releaseImageObjectUrls();
            try { socket.disconnect(); } catch (_) {}
        });

        applyLang();
        updateUI();
        socket.connect();
    </script>
</body>
</html>
'''




class TeleopWebUI:
    """
    Web UI server for teleoperation control.

    Provides a Flask + WebSocket server that runs in a background thread and
    allows browser-based control of the Teleop app.
    """

    def __init__(
        self,
        plugin: "TeleopApp",
        host: str = "0.0.0.0",
        port: int = 28402,
        open_browser: bool = True,
        quit_callback: Optional[Callable[[], None]] = None,
    ) -> None:
        """
        Initialize the Web UI server.

        :param plugin: The Teleop app instance to control.
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
        self._client_lock = threading.RLock()
        self._client_sids: set[str] = set()

        # Create Flask app
        self._app = Flask(__name__)
        self._app.config["SECRET_KEY"] = "teleop-secret"
        self._socketio = SocketIO(
            self._app,
            cors_allowed_origins="*",
            async_mode="threading",
            ping_interval=2,
            ping_timeout=5,
        )

        # Register routes
        self._register_routes()
        self._register_socket_handlers()

    def _debug_ui_configured(self) -> bool:
        return self._debug_status_enabled() and bool(getattr(self._plugin, "config", {}).get("debug_ui", False))

    def _debug_status_enabled(self) -> bool:
        return bool(getattr(self._plugin, "config", {}).get("debug_enabled", False))

    def _debug_url_toggle_allowed(self) -> bool:
        return bool(getattr(self._plugin, "config", {}).get("debug_ui_allow_url_toggle", True))

    def _debug_poll_interval_s(self) -> float:
        try:
            return max(0.5, float(getattr(self._plugin, "config", {}).get("debug_poll_interval_s", 1.0)))
        except (TypeError, ValueError):
            return 1.0

    def _is_debug_request_allowed(self) -> bool:
        if not self._debug_status_enabled():
            return False
        if self._debug_ui_configured():
            return True
        if not self._debug_url_toggle_allowed():
            return False
        value = str(request.args.get("debug") or "").lower()
        return value in {"1", "true", "yes", "on"}

    def _register_routes(self) -> None:
        """Register Flask routes."""

        @self._app.route("/")
        def index():
            return render_template_string(HTML_TEMPLATE)

        @self._app.route("/api/config", methods=["GET"])
        def api_config():
            record_status = self._plugin.record_status or {}
            return jsonify({
                "collection_id": self._plugin._collection_id,
                "max_record_duration_s": getattr(self._plugin, "_record_max_duration_s", 600),
                "transport": getattr(self._plugin, "config", {}).get("transport", "udp"),
                "teleop_enabled": self._plugin.teleop_enabled,
                "recording": bool(record_status.get("recording")),
                "collection_config_locked": bool(getattr(self._plugin, "_collection_config_locked", False)),
                "collection_config": getattr(self._plugin, "_collection_config", {}) or {},
                "export_rynnbot_mapping": bool(getattr(self._plugin, "config", {}).get("export_rynnbot_mapping", True)),
                "debug_enabled": self._debug_status_enabled(),
                "debug_ui": self._debug_ui_configured(),
                "debug_ui_allow_url_toggle": self._debug_url_toggle_allowed(),
                "debug_poll_interval_s": self._debug_poll_interval_s(),
            })

        @self._app.route("/api/snapshot", methods=["GET"])
        def api_snapshot():
            return jsonify(self._snapshot_payload())

        @self._app.route("/api/preview_keys", methods=["POST"])
        def api_preview_keys():
            body = request.get_json(silent=True) or {}
            try:
                result = self._plugin.set_preview_key_config(
                    target_images=body.get("target_images"),
                    target_states=body.get("target_states"),
                    source_states=body.get("source_states"),
                )
                self._socketio.emit("preview_keys", result)
                return jsonify({"success": True, "preview_keys": result})
            except Exception as exc:
                return jsonify({"success": False, "message": str(exc)})

        @self._app.route("/api/discover", methods=["GET"])
        def api_discover():
            try:
                servers = []
                for item in self._plugin.discover():
                    servers.append({
                        "robot_id": item.robot_id,
                        "robot_name": item.robot_name,
                        "embodiment_type": item.embodiment_type,
                        "observations": item.observations,
                        "actions": item.actions,
                        "capabilities": item.capabilities,
                        "address": item.endpoint.address if item.endpoint else "",
                        "endpoint_source": item.endpoint.source if item.endpoint else "",
                    })
                return jsonify({"success": True, "servers": servers})
            except Exception as exc:
                return jsonify({
                    "success": False,
                    "message": str(exc),
                    "connection_status": self._plugin.connection_status,
                    "preview_keys": self._plugin.preview_key_config(),
                })

        @self._app.route("/api/reconnect", methods=["POST"])
        def api_reconnect():
            try:
                binding = self._plugin.reconnect_servers()
                return jsonify({
                    "success": True,
                "binding": {
                    "source_robot_id": binding.source.robot_id,
                    "target_robot_id": binding.target.robot_id,
                    "embodiment_type": binding.source.embodiment_type,
                    "source_observation": self._plugin.teleop_observation_name,
                    "target_observation": self._plugin.target_observation_name,
                    "action": self._plugin.teleop_action_name,
                    "control_flows": self._plugin.control_flows,
                },
                    "connection_status": self._plugin.connection_status,
                })
            except Exception as exc:
                return jsonify({"success": False, "message": str(exc)})

        @self._app.route("/api/teleop/start", methods=["POST"])
        def api_start_teleop():
            body = request.get_json(silent=True) or {}
            try:
                source_robot_id = str(body.get("source_robot_id") or "")
                target_robot_id = str(body.get("target_robot_id") or "")
                control_flows = body.get("control_flows")
                if source_robot_id and target_robot_id:
                    self._plugin.bind_servers(source_robot_id, target_robot_id, control_flows=control_flows)
                self._plugin.start_teleop()
                self._socketio.emit("teleop_status", {"enabled": True})
                return jsonify({
                    "success": True,
                    "connection_status": self._plugin.connection_status,
                    "preview_keys": self._plugin.preview_key_config(),
                    "binding": {
                        "source_observation": self._plugin.teleop_observation_name,
                        "target_observation": self._plugin.target_observation_name,
                        "action": self._plugin.teleop_action_name,
                        "control_flows": self._plugin.control_flows,
                    },
                })
            except Exception as exc:
                return jsonify({
                    "success": False,
                    "message": str(exc),
                    "connection_status": self._plugin.connection_status,
                })

        @self._app.route("/api/teleop/stop", methods=["POST"])
        def api_stop_teleop():
            try:
                self._plugin.stop_teleop()
                self._socketio.emit("teleop_status", {"enabled": False})
                return jsonify({"success": True})
            except Exception as exc:
                return jsonify({"success": False, "message": str(exc)})

        @self._app.route("/api/debug/status", methods=["GET"])
        def api_debug_status():
            if not self._is_debug_request_allowed():
                return jsonify({"success": False, "message": "debug disabled"}), 403
            try:
                data = self._plugin.get_debug_status(include_peer=True, timeout_s=1.0)
                return jsonify({"success": True, "data": data})
            except Exception as exc:
                logger.error("[WebUI] debug status error: %s", exc, exc_info=True)
                return jsonify({"success": False, "message": str(exc)})

        @self._app.route("/api/record/start", methods=["POST"])
        def api_start_record():
            if not self._plugin.teleop_enabled:
                return jsonify({"success": False, "message": "请先开始遥操"})
            body = request.get_json(silent=True) or {}
            try:
                ok = self._plugin.start_record(
                    keys=body.get("keys"),
                    task_prompt=body.get("task_prompt", "teleop demo"),
                    task_description=body.get("task_description", ""),
                    fps=float(body.get("fps", 30)),
                    max_duration_s=float(body.get("max_duration_s", 600)),
                    collection_id=body.get("collection_id", "teleop"),
                )
                if ok:
                    return jsonify({"success": True, "episode": self._plugin._current_episode})
                return jsonify({"success": False, "message": "启动录制失败"})
            except Exception as exc:
                return jsonify({"success": False, "message": str(exc)})

        @self._app.route("/api/record/stop", methods=["POST"])
        def api_stop_record():
            try:
                result = self._plugin.stop_record()
                return jsonify({
                    "success": True,
                    "frames_written": result.get("frames_written", 0),
                    "episode": self._plugin._current_episode,
                })
            except Exception as exc:
                return jsonify({"success": False, "message": str(exc)})

        @self._app.route("/api/record/discard", methods=["POST"])
        def api_discard_record():
            return jsonify(self._plugin.discard_record())

        @self._app.route("/api/data/export", methods=["POST"])
        def api_export_data():
            body = request.get_json(silent=True) or {}
            collection_id = body.get("collection_id") or self._plugin._collection_id
            result = self._plugin.export_data(
                collection_id=collection_id,
                fps=float(body.get("fps", 30)),
                export_rynnbot_mapping=bool(body.get("export_rynnbot_mapping", True)),
                progress_callback=self._emit_export_progress,
            )
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
            """Get protocol collections."""
            try:
                data = self._plugin.scan_records()
                return jsonify({"success": True, "data": data})
            except Exception as e:
                logger.error("[WebUI] scan_records error: %s", e, exc_info=True)
                return jsonify({"success": False, "message": str(e)})

        @self._app.route("/api/records/delete", methods=["POST"])
        def api_delete_record():
            """Delete a specific episode."""
            body = request.get_json(silent=True) or {}
            path = body.get("path")
            if not path:
                return jsonify({"success": False, "message": "path required"})
            result = self._plugin.delete_episode_ref(str(path))
            return jsonify(result)

        @self._app.route("/api/server_resources/log", methods=["GET"])
        def api_read_server_log():
            side = str(request.args.get("side") or "")
            resource_id = str(request.args.get("resource_id") or "")
            if side not in ("source", "target"):
                return Response("side must be source or target", status=400, mimetype="text/plain")
            if not resource_id:
                return Response("resource_id required", status=400, mimetype="text/plain")
            try:
                data = self._plugin.read_server_resource(side, resource_id)
            except Exception as exc:
                return Response(str(exc), status=500, mimetype="text/plain")
            name = str(request.args.get("name") or resource_id or "log.txt").split("/")[-1] or "log.txt"
            disposition = "attachment" if request.args.get("download") else "inline"
            return Response(
                data,
                mimetype="text/plain; charset=utf-8",
                headers={"Content-Disposition": f'{disposition}; filename="{name}"'},
            )

        @self._app.route("/api/server_resources/delete_collection", methods=["POST"])
        def api_delete_server_collection():
            body = request.get_json(silent=True) or {}
            side = body.get("side")
            resource_id = body.get("resource_id")
            if side not in ("source", "target"):
                return jsonify({"success": False, "message": "side must be source or target"})
            if not resource_id:
                return jsonify({"success": False, "message": "resource_id required"})
            result = self._plugin.delete_server_collection(str(side), str(resource_id))
            return jsonify(result)

        @self._app.route("/api/server_resources/delete_log", methods=["POST"])
        def api_delete_server_log():
            body = request.get_json(silent=True) or {}
            side = body.get("side")
            resource_id = body.get("resource_id")
            if side not in ("source", "target"):
                return jsonify({"success": False, "message": "side must be source or target"})
            if not resource_id:
                return jsonify({"success": False, "message": "resource_id required"})
            result = self._plugin.delete_server_log_resource(str(side), str(resource_id))
            return jsonify(result)

        @self._app.route("/api/records/delete_export", methods=["POST"])
        def api_delete_exported_zip():
            body = request.get_json(silent=True) or {}
            path = body.get("path")
            if not path:
                return jsonify({"success": False, "message": "path required"})
            result = self._plugin.records.delete_exported_zip(path)
            return jsonify(result)

        @self._app.route("/api/records/export", methods=["POST"])
        def api_export_records():
            body = request.get_json(silent=True) or {}
            episode_paths = body.get("episode_paths", [])
            zip_name = body.get("zip_name")
            if not episode_paths:
                return jsonify({"success": False, "message": "episode_paths required"})

            result = self._plugin.export_episode_paths(
                episode_paths,
                zip_name,
                fps=float(body.get("fps", 30)),
                export_rynnbot_mapping=bool(body.get("export_rynnbot_mapping", True)),
                _progress_callback=self._emit_export_progress,
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

    def _register_socket_handlers(self) -> None:
        """Register WebSocket handlers."""

        @self._socketio.on("connect")
        def _on_connect():
            with self._client_lock:
                self._client_sids.add(str(request.sid))
                has_clients = bool(self._client_sids)
            self._plugin.set_preview_client_demand(has_clients)
            self._emit_current_snapshot(to=request.sid)

        @self._socketio.on("disconnect")
        def _on_disconnect():
            with self._client_lock:
                self._client_sids.discard(str(request.sid))
                has_clients = bool(self._client_sids)
            self._plugin.set_preview_client_demand(has_clients)

        @self._socketio.on("request_snapshot")
        def _on_request_snapshot(_data=None):
            self._emit_current_snapshot(to=request.sid)

    def _emit_export_progress(self, current: float, total: float, message: str) -> None:
        self._socketio.emit("export_selected_progress", _progress_payload(current, total, message))

    def _current_image_payload(self, images: Optional[Dict[str, bytes]] = None) -> Dict[str, bytes]:
        if images is None:
            images = self._plugin.latest_follower_images
        if not images:
            return {}
        img_data: Dict[str, bytes] = {}
        for key, jpg_bytes in images.items():
            if isinstance(jpg_bytes, (bytes, bytearray)):
                img_data[str(key)] = bytes(jpg_bytes)
            elif isinstance(jpg_bytes, memoryview):
                img_data[str(key)] = jpg_bytes.tobytes()
        return img_data

    def _emit_current_snapshot(self, to: Optional[str] = None) -> None:
        """Send the latest UI state to a newly connected browser."""
        snapshot = self._snapshot_payload(include_images=False)
        img_data = self._current_image_payload()
        if img_data:
            self._socketio.emit("image_update", img_data, to=to)

        state = snapshot["state"]
        if state or snapshot["target_state_items"]:
            self._socketio.emit("state_update", {"state": state, "items": snapshot["target_state_items"]}, to=to)

        leader_state = snapshot["leader_state"]
        if leader_state or snapshot["source_state_items"]:
            self._socketio.emit("leader_state_update", {"state": leader_state, "items": snapshot["source_state_items"]}, to=to)

        self._socketio.emit("record_status", snapshot["record_status"], to=to)
        self._socketio.emit("teleop_metrics", snapshot["metrics"], to=to)

        self._socketio.emit("follower_status", snapshot["follower_status"], to=to)
        self._socketio.emit("connection_status", snapshot["connection_status"], to=to)
        self._socketio.emit("preview_keys", snapshot["preview_keys"], to=to)

        if snapshot["buffer_keys"]["keys"]:
            self._socketio.emit("buffer_keys", snapshot["buffer_keys"], to=to)

    def _snapshot_payload(self, include_images: bool = False) -> Dict[str, Any]:
        """Return the latest UI state for socket reconnect and HTTP fallback."""
        return {
            "success": True,
            "images": self._current_image_payload() if include_images else {},
            "state": self._plugin.latest_follower_state or {},
            "leader_state": self._plugin.latest_leader_state or {},
            "target_state_items": self._plugin.target_state_items or [],
            "source_state_items": self._plugin.source_state_items or [],
            "record_status": self._plugin.record_status or {},
            "metrics": self._plugin.metrics or {},
            "follower_status": {"online": self._is_follower_online()},
            "connection_status": self._plugin.connection_status,
            "preview_keys": self._plugin.preview_key_config(),
            "buffer_keys": {
                "keys": self._plugin.available_buffer_keys or [],
                "items": self._plugin.available_buffer_items or [],
            },
            "server_time": time.time(),
        }

    def _has_connected_clients(self) -> bool:
        with self._client_lock:
            return bool(self._client_sids)

    def _push_data_loop(self) -> None:
        """Push images and state to connected clients via WebSocket."""
        state_interval = 1.0 / 30.0
        image_interval = 1.0 / 10.0
        status_interval = 0.5
        last_state_emit = 0.0
        last_image_emit = 0.0
        last_status_emit = 0.0
        last_image_seq = -1
        last_state_seq = -1
        last_leader_state_seq = -1
        last_record_status: Dict[str, Any] | None = None
        last_metrics: Dict[str, Any] | None = None
        last_follower_online: bool | None = None
        last_connection_status: Dict[str, Any] | None = None
        last_buffer_keys: tuple[str, ...] | None = None

        while not self._stop_event.is_set():
            try:
                if not self._has_connected_clients():
                    time.sleep(0.05)
                    continue

                now = time.monotonic()

                if now - last_image_emit >= image_interval:
                    last_image_emit = now
                    image_seq = int(getattr(self._plugin, "latest_follower_images_seq", -1) or -1)
                    images = self._plugin.latest_follower_images
                    if images:
                        should_emit = image_seq < 0 or image_seq != last_image_seq
                        if should_emit:
                            img_data = self._current_image_payload(images)
                        else:
                            img_data = {}
                        if img_data:
                            last_image_seq = image_seq
                            self._socketio.emit("image_update", img_data)

                if now - last_state_emit >= state_interval:
                    last_state_emit = now

                    raw_state_seq = getattr(self._plugin, "latest_follower_state_seq", -1)
                    state_seq = int(raw_state_seq if raw_state_seq is not None else -1)
                    state = self._plugin.latest_follower_state
                    target_items = self._plugin.target_state_items or []
                    if (state or target_items) and (state_seq < 0 or state_seq != last_state_seq):
                        last_state_seq = state_seq
                        self._socketio.emit("state_update", {"state": state or {}, "items": target_items})

                    raw_leader_state_seq = getattr(self._plugin, "latest_leader_state_seq", -1)
                    leader_state_seq = int(raw_leader_state_seq if raw_leader_state_seq is not None else -1)
                    leader_state = self._plugin.latest_leader_state
                    source_items = self._plugin.source_state_items or []
                    if (leader_state or source_items) and (leader_state_seq < 0 or leader_state_seq != last_leader_state_seq):
                        last_leader_state_seq = leader_state_seq
                        self._socketio.emit("leader_state_update", {"state": leader_state or {}, "items": source_items})

                if now - last_status_emit >= status_interval:
                    last_status_emit = now

                    # Push record status
                    status = self._plugin.record_status
                    if status != last_record_status:
                        self._socketio.emit("record_status", status)
                        last_record_status = dict(status)

                    # Push realtime transport metrics
                    metrics = self._plugin.metrics
                    if metrics != last_metrics:
                        self._socketio.emit("teleop_metrics", metrics)
                        last_metrics = dict(metrics)

                    # Push follower online status (heartbeat received within last 3 seconds)
                    is_online = self._is_follower_online()
                    if is_online != last_follower_online:
                        self._socketio.emit("follower_status", {"online": is_online})
                        last_follower_online = is_online

                    connection_status = self._plugin.connection_status
                    if connection_status != last_connection_status:
                        self._socketio.emit("connection_status", connection_status)
                        last_connection_status = {
                            side: dict(value)
                            for side, value in connection_status.items()
                        }

                    # Push available buffer keys from follower
                    buffer_items = self._plugin.available_buffer_items
                    buffer_keys = [str(item.get("protocol_name") or "") for item in buffer_items if item.get("protocol_name")]
                    buffer_keys_key = tuple(buffer_keys)
                    if buffer_items and buffer_keys_key != last_buffer_keys:
                        self._socketio.emit("buffer_keys", {"keys": buffer_keys, "items": buffer_items})
                        last_buffer_keys = buffer_keys_key

                    # Push playback status if in progress
                    if self._plugin.playback_in_progress:
                        playback_status = self._plugin.get_playback_status()
                        self._socketio.emit("playback_status", playback_status)

                next_due = min(
                    last_state_emit + state_interval,
                    last_image_emit + image_interval,
                    last_status_emit + status_interval,
                )
                time.sleep(max(0.001, min(0.05, next_due - time.monotonic())))
            except Exception as e:
                logger.error("[TeleopWebUI] push_data error: %s", e, exc_info=True)
                time.sleep(0.5)

    def _is_follower_online(self) -> bool:
        target_status = self._plugin.connection_status.get("target", {}).get("status")
        if target_status == "online":
            return True
        if target_status in ("offline", "reconnectable", "reconnecting", "unbound"):
            return False
        now = time.time()
        last_hb = float(getattr(self._plugin, "_last_heartbeat_recv", 0.0) or 0.0)
        if last_hb > 0 and now - last_hb < 3.0:
            return True
        last_state = float(getattr(self._plugin, "latest_follower_state_at", 0.0) or 0.0)
        if last_state > 0 and now - last_state < 3.0:
            return True
        last_images = float(getattr(self._plugin, "latest_follower_images_at", 0.0) or 0.0)
        return last_images > 0 and now - last_images < 3.0

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
                f"请修改 web_port 参数为其他可用端口（如 28412、28422 等），然后重新运行。"
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
                logger.error("[TeleopWebUI] Server error: %s", e, exc_info=True)

        self._server_thread = threading.Thread(
            target=run_server, daemon=True, name="teleop-web-server"
        )
        self._server_thread.start()

        urls = browser_urls(self._host, self._port)
        browser_url = urls[0]
        logger.info("[TeleopWebUI] Web UI bind address: http://%s:%s", self._host, self._port)
        logger.info("[TeleopWebUI] Web UI Local: %s", urls[0])
        for url in urls[1:]:
            logger.info("[TeleopWebUI] Web UI LAN:   %s", url)

        # Open browser
        if self._open_browser:
            time.sleep(0.5)
            webbrowser.open(browser_url)

    def stop(self) -> None:
        """Stop the web server."""
        self._stop_event.set()
        logger.info("[TeleopWebUI] Web UI stopped.")
