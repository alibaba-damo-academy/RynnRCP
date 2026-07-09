#!/usr/bin/env python3
"""Browser-based SO101 configuration helper for RynnRCP."""

from __future__ import annotations

import argparse
import base64
import glob
import json
import logging
import os
import re
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping


LOGGER = logging.getLogger("rynnrcp.so101.configure_web")

PACKAGE_DIR = Path(__file__).resolve().parent
SO101_PROJECT_ROOT = PACKAGE_DIR.parent
REPO_ROOT = PACKAGE_DIR.parents[2]
CONFIG_DIR = PACKAGE_DIR / "config"
CONFIG_FILES = {
    "follower_server": CONFIG_DIR / "so101_follower_server.yaml",
    "leader_server": CONFIG_DIR / "so101_leader_server.yaml",
    "rynnbot_app": CONFIG_DIR / "so101_rynnbot_app.yaml",
}

HTML_TEMPLATE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SO101 配置向导 - RynnRCP</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      min-height: 100vh;
      color: #e8e8e8;
      background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
      font: 14px/1.55 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
    header {
      position: relative;
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
      letter-spacing: 0;
    }
    header p { color: #9aa4b2; margin-top: 8px; }
    .nav-tabs { display: flex; gap: 10px; margin-bottom: 20px; flex-wrap: wrap; }
    .nav-tab {
      padding: 12px 18px;
      background: #2a2a3e;
      border: 1px solid transparent;
      border-radius: 8px;
      color: #9aa4b2;
      cursor: pointer;
      transition: all .18s;
      font-weight: 650;
    }
    .nav-tab:hover { background: #3a3a4e; color: #d0d5dd; }
    .nav-tab.active { background: linear-gradient(135deg, #00d4ff, #0099cc); color: white; }
    .nav-tab.completed { background: #1e3a2e; border-color: #00ff88; color: #00ff88; }
    .config-section {
      display: none;
      background: #1e1e2f;
      border: 1px solid #303044;
      border-radius: 12px;
      padding: 25px;
      box-shadow: 0 4px 20px rgba(0,0,0,.3);
    }
    .config-section.active { display: block; }
    .section-title { font-size: 20px; color: #00d4ff; margin-bottom: 18px; padding-bottom: 10px; border-bottom: 1px solid #333; }
    .tip-box {
      background: #24364d;
      border-left: 4px solid #00d4ff;
      padding: 14px 15px;
      border-radius: 0 8px 8px 0;
      margin-bottom: 20px;
    }
    .tip-box h4 { color: #00d4ff; margin-bottom: 6px; }
    .tip-box p { color: #b8c0cc; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(260px, 1fr)); gap: 16px; }
    .form-group { margin-bottom: 18px; }
    .form-label { display: block; color: #a6afbd; font-size: 13px; margin-bottom: 7px; font-weight: 650; }
    .form-input {
      width: 100%;
      padding: 12px 14px;
      border-radius: 8px;
      border: 1px solid #44485a;
      background: #2a2a3e;
      color: #e8e8e8;
      font: inherit;
    }
    .form-input:focus { outline: none; border-color: #00d4ff; }
    .check-row { display: flex; align-items: center; gap: 10px; min-height: 43px; color: #cbd5e1; }
    .check-row input { width: 18px; height: 18px; }
    .button-group { display: flex; gap: 10px; margin-top: 20px; flex-wrap: wrap; }
    .btn {
      padding: 12px 22px;
      border: 0;
      border-radius: 8px;
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
      transition: all .18s;
    }
    .btn:disabled { opacity: .5; cursor: not-allowed; }
    .btn:hover:not(:disabled) { transform: translateY(-1px); }
    .btn-primary { background: linear-gradient(135deg, #00d4ff, #0099cc); color: white; }
    .btn-success { background: linear-gradient(135deg, #00ff88, #00cc6a); color: #101828; }
    .btn-warning { background: linear-gradient(135deg, #ffa500, #cc8400); color: #101828; }
    .btn-danger { background: linear-gradient(135deg, #ff4757, #cc3a47); color: white; }
    .btn-plain { background: #555; color: white; }
    .exit-button {
      position: absolute;
      right: 0;
      top: 20px;
    }
    .mapping-display, .status-panel {
      background: #2a2a3e;
      border: 1px solid #38384d;
      border-radius: 8px;
      padding: 15px;
      margin-top: 15px;
    }
    .mapping-item {
      display: flex;
      justify-content: space-between;
      gap: 14px;
      padding: 10px 0;
      border-bottom: 1px solid #3a3a4e;
    }
    .mapping-item:last-child { border-bottom: 0; }
    .mapping-key { color: #00d4ff; font-weight: 700; }
    .mapping-value { color: #00ff88; font-weight: 700; overflow-wrap: anywhere; }
    .mapping-value.unset { color: #ff4757; }
    .camera-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 15px; margin-top: 20px; }
    .camera-card {
      background: #2a2a3e;
      border-radius: 8px;
      overflow: hidden;
      border: 2px solid transparent;
      transition: border-color .18s;
    }
    .camera-card.selected { border-color: #00ff88; }
    .camera-card img { width: 100%; aspect-ratio: 16/9; object-fit: cover; display: block; background: #101828; }
    .camera-info { padding: 12px; }
    .camera-name { font-weight: 700; }
    .camera-actions { display: flex; gap: 8px; margin-top: 10px; flex-wrap: wrap; }
    .camera-btn { padding: 8px 12px; border: 0; border-radius: 6px; font-size: 12px; font-weight: 700; cursor: pointer; }
    .camera-btn.front { background: #00d4ff; color: #101828; }
    .camera-btn.wrist { background: #ffa500; color: #101828; }
    .progress-container { background: #2a2a3e; border: 1px solid #38384d; border-radius: 8px; padding: 14px; margin-top: 15px; }
    .progress-text { color: #cbd5e1; font-size: 13px; margin-bottom: 8px; }
    .progress-bar-wrapper { height: 8px; background: #17172a; border-radius: 999px; overflow: hidden; }
    .progress-bar { height: 100%; width: 0%; background: linear-gradient(90deg, #00d4ff, #00ff88); transition: width .18s; }
    .serial-list { margin-top: 15px; }
    .serial-item {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 14px;
      padding: 12px 15px;
      background: #2a2a3e;
      border-radius: 8px;
      margin-bottom: 10px;
      border-left: 4px solid transparent;
    }
    .serial-item.new { border-left-color: #00ff88; }
    .serial-device { font-weight: 750; color: #f2f4f7; overflow-wrap: anywhere; }
    .serial-desc { font-size: 12px; color: #9aa4b2; margin-top: 4px; overflow-wrap: anywhere; }
    .status-badge { display: inline-block; padding: 4px 11px; border-radius: 18px; font-size: 12px; font-weight: 750; }
    .status-badge.success { background: #1e3a2e; color: #00ff88; }
    .status-badge.warning { background: #3a331e; color: #ffa500; }
    .status-badge.error { background: #3a1e1e; color: #ff4757; }
    .status-badge.info { background: #1e2e3a; color: #00d4ff; }
    .calib-steps { margin-top: 20px; }
    .calib-step { display: flex; align-items: flex-start; gap: 14px; padding: 14px; background: #2a2a3e; border-radius: 8px; margin-bottom: 10px; }
    .calib-step-number {
      width: 32px;
      height: 32px;
      background: #3a3a4e;
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      font-weight: 800;
      color: #98a2b3;
      flex: 0 0 auto;
    }
    .calib-step.active .calib-step-number { background: #00d4ff; color: #101828; }
    .calib-step.completed .calib-step-number { background: #00ff88; color: #101828; }
    .calib-step-title { color: #f2f4f7; font-weight: 750; }
    .calib-step-desc { color: #9aa4b2; font-size: 13px; margin-top: 3px; }
    .calib-hint {
      margin-top: 10px;
      padding: 10px 12px;
      border-radius: 8px;
      border: 1px solid #3a331e;
      background: #241f13;
      color: #ffd166;
      font-weight: 700;
    }
    .calib-hint.ready {
      border-color: #1e3a2e;
      background: #10251d;
      color: #00ff88;
    }
    .log-container {
      background: #0d0d1a;
      border-radius: 8px;
      padding: 14px;
      margin-top: 18px;
      max-height: 360px;
      overflow-y: auto;
      font-family: "SFMono-Regular", Consolas, monospace;
    }
    .log-line { font-size: 13px; line-height: 1.6; color: #00ff88; white-space: pre-wrap; word-break: break-word; }
    .log-line.info { color: #00d4ff; }
    .log-line.warning { color: #ffa500; }
    .log-line.error { color: #ff4757; }
    .range-table { width: 100%; border-collapse: collapse; margin-top: 8px; }
    .range-table th, .range-table td { border-bottom: 1px solid #3a3a4e; padding: 7px 8px; text-align: right; font-family: "SFMono-Regular", Consolas, monospace; }
    .range-table th:first-child, .range-table td:first-child { text-align: left; }
    .range-table th { color: #00d4ff; font-size: 12px; }
    .range-progress { height: 7px; min-width: 90px; background: #17172a; border-radius: 999px; overflow: hidden; }
    .range-progress span { display: block; height: 100%; background: linear-gradient(90deg, #00d4ff, #00ff88); }
    pre {
      margin: 6px 0 0;
      background: #0d0d1a;
      border: 1px solid #34344a;
      border-radius: 8px;
      color: #dbeafe;
      padding: 12px;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      font-family: "SFMono-Regular", Consolas, monospace;
      font-size: 13px;
    }
    .toast {
      position: fixed;
      bottom: 20px;
      right: 20px;
      max-width: min(520px, calc(100vw - 40px));
      padding: 14px 20px;
      border-radius: 8px;
      color: white;
      font-weight: 700;
      transform: translateX(150%);
      transition: transform .25s;
      z-index: 1000;
      box-shadow: 0 10px 28px rgba(0,0,0,.35);
    }
    .toast.show { transform: translateX(0); }
    .toast.success { background: #00a86b; }
    .toast.error { background: #ff4757; }
    .toast.info { background: #0099cc; }
    .shutdown-screen {
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 24px;
      text-align: center;
    }
    .shutdown-screen h1 { color: #00d4ff; font-size: 28px; margin-bottom: 10px; }
    .shutdown-screen p { color: #b8c0cc; font-size: 16px; }
    @media (max-width: 760px) {
      .container { padding: 14px; }
      header { text-align: left; padding-right: 96px; }
      .exit-button { right: 0; top: 18px; }
      .grid { grid-template-columns: 1fr; }
      .nav-tab { flex: 1 1 160px; }
      .mapping-item, .serial-item { align-items: flex-start; flex-direction: column; }
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>SO101 配置向导 | Configuration Wizard</h1>
      <p>按步骤完成 RynnRCP 的 SO101 真机配置</p>
      <button class="btn btn-danger exit-button" onclick="exitWizard()">退出</button>
    </header>

    <div class="nav-tabs">
      <button class="nav-tab active" id="tab-device" onclick="showSection('device')">1. 设备设置</button>
      <button class="nav-tab" id="tab-camera" onclick="showSection('camera')">2. 相机配置</button>
      <button class="nav-tab" id="tab-follower-serial" onclick="showSection('follower-serial')">3. 从臂串口</button>
      <button class="nav-tab" id="tab-follower-calib" onclick="showSection('follower-calib')">4. 从臂标定</button>
      <button class="nav-tab" id="tab-leader-serial" onclick="showSection('leader-serial')">5. 主臂串口</button>
      <button class="nav-tab" id="tab-leader-calib" onclick="showSection('leader-calib')">6. 主臂标定</button>
    </div>

    <section class="config-section active" id="section-device">
      <h2 class="section-title">设备设置 (RynnBot / MCP / Teleop)</h2>
      <div class="tip-box">
        <h4>提示</h4>
        <p>这一步配置机器人 ID 和云端设备凭证。机器人 ID 会写入协议 Manifest 的 robot_id，用于区分不同机器人。</p>
      </div>
      <div class="grid">
        <div class="form-group">
          <label class="form-label">机器人 ID / Robot ID</label>
          <input class="form-input" id="robot-id" placeholder="so101_follower">
        </div>
        <div class="form-group">
          <label class="form-label">Product Key</label>
          <input class="form-input" id="device-product-key" placeholder="请输入 product_key">
        </div>
        <div class="form-group">
          <label class="form-label">Device Name</label>
          <input class="form-input" id="device-name" placeholder="请输入 device_name">
        </div>
        <div class="form-group">
          <label class="form-label">Device Secret</label>
          <input class="form-input" id="device-secret" placeholder="请输入 device_secret">
        </div>
        <div class="form-group">
          <label class="form-label">HTTP URL</label>
          <input class="form-input" id="device-http-url" placeholder="https://robot-access.damo-academy.com">
        </div>
        <div class="form-group">
          <label class="form-label">图片上传编码 / Image Upload Codec</label>
          <select class="form-input" id="device-image-codec">
            <option value="jpeg">jpeg</option>
            <option value="npy_gzip">npy_gzip</option>
          </select>
        </div>
      </div>
      <div class="button-group">
        <button class="btn btn-success" onclick="saveDeviceSettings()">保存设置</button>
        <button class="btn btn-primary" onclick="showSection('camera')">下一步</button>
      </div>
    </section>

    <section class="config-section" id="section-camera">
      <h2 class="section-title">相机配置</h2>
      <div class="tip-box">
        <h4>提示</h4>
        <p>点击扫描相机后，把画面绑定到 front 和 wrist。保存后会同步写入 MCP、RynnBot 和 Teleop follower 配置。</p>
      </div>
      <div class="button-group">
        <button class="btn btn-primary" onclick="scanCameras()">扫描相机</button>
        <button class="btn btn-warning" onclick="clearCameraMapping()">清除绑定</button>
      </div>
      <div class="progress-container" id="camera-scan-progress" style="display:none;">
        <div class="progress-text" id="camera-scan-text">等待扫描</div>
        <div class="progress-bar-wrapper"><div class="progress-bar" id="camera-scan-bar"></div></div>
      </div>
      <div class="mapping-display">
        <div class="mapping-item"><span class="mapping-key">observation.images.front</span><span class="mapping-value unset" id="mapping-front">(未绑定)</span></div>
        <div class="mapping-item"><span class="mapping-key">observation.images.wrist</span><span class="mapping-value unset" id="mapping-wrist">(未绑定)</span></div>
      </div>
      <div class="camera-grid" id="camera-grid">
        <div style="color:#98a2b3;text-align:center;padding:40px;">点击“扫描相机”检测可用相机</div>
      </div>
      <div class="button-group">
        <button class="btn btn-success" onclick="saveCameraSettings()" id="btn-save-camera" disabled>保存配置</button>
        <button class="btn btn-primary" onclick="showSection('follower-serial')">下一步</button>
      </div>
    </section>

    <section class="config-section" id="section-follower-serial">
      <h2 class="section-title">从臂串口配置</h2>
      <div class="tip-box">
        <h4>提示</h4>
        <p>先拔掉从臂串口并记录基线，再插入从臂并检测新增。若无法拔插，也可以直接在列表里选择已有串口。</p>
      </div>
      <div class="button-group">
        <button class="btn btn-warning" onclick="recordSerialBaseline('follower')">记录基线</button>
        <button class="btn btn-primary" onclick="detectNewSerial('follower')">检测新增</button>
        <button class="btn btn-plain" onclick="scanSerialOnly('follower')">刷新列表</button>
      </div>
      <div id="follower-serial-status" class="status-panel">当前状态: 等待操作...</div>
      <div class="serial-list" id="follower-serial-list">
        <div style="color:#98a2b3;text-align:center;padding:20px;">串口列表将在检测后显示</div>
      </div>
      <div class="mapping-display">
        <div class="mapping-item"><span class="mapping-key">已选择的从臂串口</span><span class="mapping-value unset" id="follower-selected-port">(未选择)</span></div>
      </div>
      <div class="button-group">
        <button class="btn btn-success" onclick="saveFollowerSerial()" id="btn-save-follower-serial" disabled>保存配置</button>
        <button class="btn btn-primary" onclick="showSection('follower-calib')">下一步</button>
      </div>
    </section>

    <section class="config-section" id="section-follower-calib">
      <h2 class="section-title">从臂标定</h2>
      <div class="tip-box">
        <h4>SO101 标定流程</h4>
        <p>点击“启动标定”后，后端会启动 RynnRCP SO101 标定进程，并在同一个串口连接内读取各关节 Present_Position。先把所有关节移动到中间位置并点击“记录中间位置”，然后缓慢推动每个关节走完整范围，观察 Min / 当前 / Max 是否持续变化，最后点击“结束并保存”。标定期间不要同时运行 MCP、Teleop 或 RynnBot。</p>
      </div>
      <div class="calib-steps">
        <div class="calib-step" id="follower-calib-step-1"><div class="calib-step-number">1</div><div><div class="calib-step-title">启动标定进程</div><div class="calib-step-desc">连接 follower 串口并启动 SO101 calibration。</div></div></div>
        <div class="calib-step" id="follower-calib-step-2"><div class="calib-step-number">2</div><div><div class="calib-step-title">记录中间位置</div><div class="calib-step-desc">把所有关节放在运动范围中间后确认。</div></div></div>
        <div class="calib-step" id="follower-calib-step-3"><div class="calib-step-number">3</div><div><div class="calib-step-title">记录完整范围</div><div class="calib-step-desc">缓慢推动每个关节通过最小/最大范围。</div></div></div>
        <div class="calib-step" id="follower-calib-step-4"><div class="calib-step-number">4</div><div><div class="calib-step-title">保存完成</div><div class="calib-step-desc">SO101 写入 calibration cache。</div></div></div>
      </div>
      <div class="button-group">
        <button class="btn btn-primary" id="follower-calib-start-btn" onclick="calibStart('follower')">启动标定</button>
        <button class="btn btn-warning" id="follower-calib-middle-btn" onclick="calibRecordMiddle('follower')" disabled>记录中间位置</button>
        <button class="btn btn-success" id="follower-calib-finish-btn" onclick="calibFinishRange('follower')" disabled>结束并保存</button>
        <button class="btn btn-danger" id="follower-calib-stop-btn" onclick="calibStop('follower')" disabled>停止</button>
      </div>
      <div class="status-panel">
        <span style="color:#98a2b3;">当前阶段:</span>
        <span id="follower-calib-phase" style="color:#00d4ff;font-weight:700;margin-left:8px;">未启动</span>
        <div class="calib-hint" id="follower-calib-hint">点击“启动标定”后，请等待进入“等待确认中间位置”阶段，再点击“记录中间位置”。</div>
      </div>
      <div class="status-panel" id="follower-range-panel" style="display:none;">
        <div class="mapping-key">关节 Min / 当前 / Max</div>
        <div id="follower-range-table"></div>
      </div>
      <div class="log-container" id="follower-calib-log"></div>
      <div class="button-group">
        <button class="btn btn-primary" onclick="showSection('leader-serial')">下一步</button>
      </div>
    </section>

    <section class="config-section" id="section-leader-serial">
      <h2 class="section-title">主臂串口配置</h2>
      <div class="tip-box">
        <h4>提示</h4>
        <p>主臂用于 Teleop leader。先拔掉主臂串口记录基线，再插入主臂检测新增。leader runtime 会按 read-only 策略连接。</p>
      </div>
      <div class="button-group">
        <button class="btn btn-warning" onclick="recordSerialBaseline('leader')">记录基线</button>
        <button class="btn btn-primary" onclick="detectNewSerial('leader')">检测新增</button>
        <button class="btn btn-plain" onclick="scanSerialOnly('leader')">刷新列表</button>
      </div>
      <div id="leader-serial-status" class="status-panel">当前状态: 等待操作...</div>
      <div class="serial-list" id="leader-serial-list">
        <div style="color:#98a2b3;text-align:center;padding:20px;">串口列表将在检测后显示</div>
      </div>
      <div class="mapping-display">
        <div class="mapping-item"><span class="mapping-key">已选择的主臂串口</span><span class="mapping-value unset" id="leader-selected-port">(未选择)</span></div>
      </div>
      <div class="button-group">
        <button class="btn btn-success" onclick="saveLeaderSerial()" id="btn-save-leader-serial" disabled>保存配置</button>
        <button class="btn btn-primary" onclick="showSection('leader-calib')">下一步</button>
      </div>
    </section>

    <section class="config-section" id="section-leader-calib">
      <h2 class="section-title">主臂标定 / 完成配置</h2>
      <div class="tip-box">
        <h4>SO101 标定流程</h4>
        <p>点击“启动标定”后，后端会启动 RynnRCP SO101 标定进程，并在同一个串口连接内读取各关节 Present_Position。leader 标定同样需要先确认中间位置，再推动完整关节范围并保存；标定时请观察 Min / 当前 / Max 是否覆盖每个关节的真实运动范围。标定期间不要同时运行 Teleop leader。</p>
      </div>
      <div class="calib-steps">
        <div class="calib-step" id="leader-calib-step-1"><div class="calib-step-number">1</div><div><div class="calib-step-title">启动标定进程</div><div class="calib-step-desc">连接 leader 串口并启动 SO101 calibration。</div></div></div>
        <div class="calib-step" id="leader-calib-step-2"><div class="calib-step-number">2</div><div><div class="calib-step-title">记录中间位置</div><div class="calib-step-desc">把所有关节放在运动范围中间后确认。</div></div></div>
        <div class="calib-step" id="leader-calib-step-3"><div class="calib-step-number">3</div><div><div class="calib-step-title">记录完整范围</div><div class="calib-step-desc">缓慢推动每个关节通过最小/最大范围。</div></div></div>
        <div class="calib-step" id="leader-calib-step-4"><div class="calib-step-number">4</div><div><div class="calib-step-title">保存完成</div><div class="calib-step-desc">SO101 写入 calibration cache。</div></div></div>
      </div>
      <div class="button-group">
        <button class="btn btn-primary" id="leader-calib-start-btn" onclick="calibStart('leader')">启动标定</button>
        <button class="btn btn-warning" id="leader-calib-middle-btn" onclick="calibRecordMiddle('leader')" disabled>记录中间位置</button>
        <button class="btn btn-success" id="leader-calib-finish-btn" onclick="calibFinishRange('leader')" disabled>结束并保存</button>
        <button class="btn btn-danger" id="leader-calib-stop-btn" onclick="calibStop('leader')" disabled>停止</button>
        <button class="btn btn-success" onclick="finishConfig()">完成配置</button>
      </div>
      <div class="status-panel">
        <span style="color:#98a2b3;">当前阶段:</span>
        <span id="leader-calib-phase" style="color:#00d4ff;font-weight:700;margin-left:8px;">未启动</span>
        <div class="calib-hint" id="leader-calib-hint">点击“启动标定”后，请等待进入“等待确认中间位置”阶段，再点击“记录中间位置”。</div>
      </div>
      <div class="status-panel" id="leader-range-panel" style="display:none;">
        <div class="mapping-key">关节 Min / 当前 / Max</div>
        <div id="leader-range-table"></div>
      </div>
      <div class="log-container" id="leader-calib-log"></div>
    </section>
  </div>

  <div id="toast" class="toast"></div>

  <script>
    let currentConfig = null;
    let cameraMapping = { front: null, wrist: null };
    let serialBaseline = { follower: [], leader: [] };
    let selectedPorts = { follower: null, leader: null };
    let calibrationPhases = { follower: "idle", leader: "idle" };
    const calibrationJoints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"];

    const ids = {
      robotId: "robot-id",
      productKey: "device-product-key",
      deviceName: "device-name",
      deviceSecret: "device-secret",
      httpUrl: "device-http-url",
      imageCodec: "device-image-codec"
    };

    function $(id) { return document.getElementById(id); }

    async function exitWizard() {
      showToast("正在退出配置程序...", "info");
      try {
        await api("/api/shutdown", { method: "POST" });
      } catch (error) {
        console.warn("shutdown request failed", error);
      }
      document.body.innerHTML = `
        <main class="shutdown-screen">
          <div>
            <h1>SO101 配置程序已退出</h1>
            <p>如果页面没有自动关闭，可以直接关闭这个标签页。</p>
          </div>
        </main>`;
      setTimeout(() => window.close(), 150);
    }

    function showSection(name) {
      document.querySelectorAll(".config-section").forEach(s => s.classList.remove("active"));
      document.querySelectorAll(".nav-tab").forEach(t => t.classList.remove("active"));
      $("section-" + name).classList.add("active");
      $("tab-" + name).classList.add("active");
    }

    function markTabCompleted(name) {
      $("tab-" + name).classList.add("completed");
    }

    function showToast(message, type = "info", duration = 3000) {
      const toast = $("toast");
      toast.textContent = message;
      toast.className = "toast " + type + " show";
      setTimeout(() => toast.classList.remove("show"), duration);
    }

    async function api(url, options = {}) {
      const response = await fetch(url, options);
      const data = await response.json();
      if (!response.ok || data.ok === false || data.success === false) {
        throw new Error(data.error || data.message || "request failed");
      }
      return data;
    }

    async function loadConfig() {
      const data = await api("/api/config");
      currentConfig = data.config;
      fillConfig(currentConfig);
      showToast("配置已加载", "success", 1600);
    }

    function fillConfig(config) {
      $(ids.robotId).value = config.server.id || "";
      $(ids.productKey).value = config.rynnbot.product_key || "";
      $(ids.deviceName).value = config.rynnbot.device_name || "";
      $(ids.deviceSecret).value = config.rynnbot.device_secret || "";
      $(ids.httpUrl).value = config.rynnbot.http_url || "";
      $(ids.imageCodec).value = config.rynnbot.image_upload_codec || "jpeg";
      cameraMapping.front = Number(config.hardware.front_camera ?? 0);
      cameraMapping.wrist = Number(config.hardware.wrist_camera ?? 1);
      selectedPorts.follower = config.hardware.follower_port || null;
      selectedPorts.leader = config.hardware.leader_port || null;
      updateCameraMappingDisplay();
      updateSelectedPort("follower");
      updateSelectedPort("leader");
    }

    async function postConfig(partial) {
      const data = await api("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(partial)
      });
      currentConfig = data.config;
      fillConfig(currentConfig);
      return data;
    }

    async function saveDeviceSettings() {
      try {
        await postConfig({
          server: {
            id: $(ids.robotId).value.trim()
          },
          rynnbot: {
            product_key: $(ids.productKey).value.trim(),
            device_name: $(ids.deviceName).value.trim(),
            device_secret: $(ids.deviceSecret).value.trim(),
            http_url: $(ids.httpUrl).value.trim(),
            image_upload_codec: $(ids.imageCodec).value
          }
        });
        markTabCompleted("device");
        showToast("设备设置已保存", "success");
      } catch (error) {
        showToast("保存失败: " + error.message, "error", 5000);
      }
    }

    async function scanCameras() {
      const maxIndex = 20;
      const found = [];
      const progress = $("camera-scan-progress");
      const progressText = $("camera-scan-text");
      const progressBar = $("camera-scan-bar");
      const grid = $("camera-grid");
      try {
        showToast("正在扫描相机 0-" + maxIndex + "...", "info");
        progress.style.display = "block";
        progressBar.style.width = "0%";
        progressText.textContent = "准备扫描 0-" + maxIndex;
        grid.innerHTML = '<div style="color:#98a2b3;text-align:center;padding:40px;">正在扫描相机...</div>';
        for (let index = 0; index <= maxIndex; index += 1) {
          const pct = Math.round((index / (maxIndex + 1)) * 100);
          progressBar.style.width = pct + "%";
          progressText.textContent = "正在扫描 Camera " + index + " / " + maxIndex + "，已发现 " + found.length + " 个";
          try {
            const data = await api("/api/camera/" + index + "/probe");
            if (data.ok && data.camera) {
              found.push(data.camera);
              renderCameras(found, true);
            }
          } catch (e) {
            // 单个相机探测失败，跳过继续扫描下一个
          }
        }
        progressBar.style.width = "100%";
        progressText.textContent = "扫描完成：0-" + maxIndex + "，找到 " + found.length + " 个相机";
        renderCameras(found);
        showToast("找到 " + found.length + " 个相机", found.length ? "success" : "info");
      } catch (error) {
        progressText.textContent = "扫描失败: " + error.message;
        showToast("相机扫描失败: " + error.message, "error", 5000);
      }
    }

    function renderCameras(cameras, scanning = false) {
      const grid = $("camera-grid");
      if (!cameras.length) {
        grid.innerHTML = '<div style="color:#98a2b3;text-align:center;padding:40px;">' + (scanning ? "正在扫描，暂未发现相机" : "未检测到相机") + '</div>';
        return;
      }
      grid.innerHTML = "";
      cameras.forEach(cam => {
        const card = document.createElement("div");
        card.className = "camera-card";
        card.id = "cam-card-" + cam.index;
        card.innerHTML = `
          <img src="${cam.image || ""}" alt="Camera ${cam.index}">
          <div class="camera-info">
            <div class="camera-name">Camera ${cam.index} ${cam.width ? "(" + cam.width + "x" + cam.height + ")" : ""}</div>
            <div class="camera-actions">
              <button class="camera-btn front" onclick="bindCamera(${cam.index}, 'front')">绑定前置</button>
              <button class="camera-btn wrist" onclick="bindCamera(${cam.index}, 'wrist')">绑定腕部</button>
            </div>
          </div>`;
        grid.appendChild(card);
      });
      updateCameraCardHighlight();
    }

    function bindCamera(index, type) {
      cameraMapping[type] = index;
      updateCameraMappingDisplay();
      updateCameraCardHighlight();
      showToast("已绑定 Camera " + index + " -> " + type, "success");
    }

    function updateCameraMappingDisplay() {
      setMappingValue("mapping-front", cameraMapping.front, "Camera ");
      setMappingValue("mapping-wrist", cameraMapping.wrist, "Camera ");
      $("btn-save-camera").disabled = cameraMapping.front === null || cameraMapping.wrist === null;
    }

    function updateCameraCardHighlight() {
      document.querySelectorAll(".camera-card").forEach(c => c.classList.remove("selected"));
      [cameraMapping.front, cameraMapping.wrist].forEach(index => {
        const card = $("cam-card-" + index);
        if (card) card.classList.add("selected");
      });
    }

    function clearCameraMapping() {
      cameraMapping = { front: null, wrist: null };
      updateCameraMappingDisplay();
      updateCameraCardHighlight();
      showToast("已清除相机绑定", "info");
    }

    async function saveCameraSettings() {
      try {
        await postConfig({ hardware: { front_camera: Number(cameraMapping.front), wrist_camera: Number(cameraMapping.wrist) } });
        markTabCompleted("camera");
        showToast("相机配置已保存", "success");
      } catch (error) {
        showToast("保存失败: " + error.message, "error", 5000);
      }
    }

    async function scanSerial() {
      const data = await api("/api/serial");
      return data.ports || [];
    }

    async function recordSerialBaseline(arm) {
      try {
        const ports = await scanSerial();
        serialBaseline[arm] = ports.map(p => p.device);
        setSerialStatus(arm, '<span class="status-badge info">基线已记录，检测到 ' + ports.length + ' 个端口</span>');
        renderSerialList(arm, ports, []);
        showToast("基线已记录，请插入设备后检测新增", "success");
      } catch (error) {
        showToast("串口扫描失败: " + error.message, "error", 5000);
      }
    }

    async function detectNewSerial(arm) {
      try {
        const ports = await scanSerial();
        const base = serialBaseline[arm] || [];
        const newPorts = ports.filter(p => !base.includes(p.device));
        renderSerialList(arm, ports, newPorts);
        if (newPorts.length === 1) selectSerialPort(arm, newPorts[0].device);
        setSerialStatus(arm, newPorts.length
          ? '<span class="status-badge success">检测到 ' + newPorts.length + ' 个新端口</span>'
          : '<span class="status-badge warning">未检测到新端口，可直接从列表选择</span>');
      } catch (error) {
        showToast("串口检测失败: " + error.message, "error", 5000);
      }
    }

    async function scanSerialOnly(arm) {
      try {
        const ports = await scanSerial();
        renderSerialList(arm, ports, []);
        setSerialStatus(arm, '<span class="status-badge info">已刷新，检测到 ' + ports.length + ' 个端口</span>');
      } catch (error) {
        showToast("串口扫描失败: " + error.message, "error", 5000);
      }
    }

    function renderSerialList(arm, ports, newPorts) {
      const list = $(arm + "-serial-list");
      if (!ports.length) {
        list.innerHTML = '<div style="color:#98a2b3;text-align:center;padding:20px;">未检测到任何串口</div>';
        return;
      }
      const newSet = new Set(newPorts.map(p => p.device));
      list.innerHTML = "";
      ports.forEach(port => {
        const item = document.createElement("div");
        item.className = "serial-item" + (newSet.has(port.device) ? " new" : "");
        item.innerHTML = `
          <div class="serial-info">
            <div class="serial-device">${escapeHtml(port.device)}</div>
            <div class="serial-desc">${escapeHtml(port.description || port.hwid || "")}</div>
          </div>
          <button class="btn btn-primary" style="padding:8px 15px;font-size:12px" onclick="selectSerialPort('${arm}', '${escapeAttr(port.device)}')">选择</button>`;
        list.appendChild(item);
      });
    }

    function selectSerialPort(arm, device) {
      selectedPorts[arm] = device;
      updateSelectedPort(arm);
      showToast("已选择: " + device, "success");
    }

    function updateSelectedPort(arm) {
      const id = arm + "-selected-port";
      const value = selectedPorts[arm];
      setMappingValue(id, value, "");
      $(arm === "follower" ? "btn-save-follower-serial" : "btn-save-leader-serial").disabled = !value;
    }

    function setSerialStatus(arm, html) {
      $(arm + "-serial-status").innerHTML = "当前状态: " + html;
    }

    async function saveFollowerSerial() {
      await saveSerial("follower", "follower-serial");
    }

    async function saveLeaderSerial() {
      await saveSerial("leader", "leader-serial");
    }

    async function saveSerial(arm, tabName) {
      if (!selectedPorts[arm]) {
        showToast("请先选择串口", "error");
        return;
      }
      try {
        const hardware = {};
        hardware[arm + "_port"] = selectedPorts[arm];
        await postConfig({ hardware });
        markTabCompleted(tabName);
        showToast((arm === "follower" ? "从臂" : "主臂") + "串口已保存", "success");
      } catch (error) {
        showToast("保存失败: " + error.message, "error", 5000);
      }
    }

    async function calibStart(arm) {
      const log = arm + "-calib-log";
      clearLog(log);
      addLog(log, "启动 RynnRCP SO101 标定进程...", "info");
      updateCalibrationPhase(arm, "starting", true, null);
      try {
        const data = await api("/api/calibration/start", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ arm })
        });
        renderCalibrationStatus(arm, data.status);
        completeCalibSteps(arm, 1);
        pollCalibration(arm);
        showToast("标定已启动", "success");
      } catch (error) {
        updateCalibrationPhase(arm, "failed", false, 1);
        addLog(log, "启动失败: " + error.message, "error");
        showToast("启动失败: " + error.message, "error", 6000);
      }
    }

    async function calibRecordMiddle(arm) {
      if (calibrationPhases[arm] !== "middle_position") {
        showToast("标定进程还没准备好，请等待阶段变成“等待确认中间位置”后再点击。", "info", 5000);
        return;
      }
      await calibSendEnter(arm, "middle position captured", 2);
    }

    async function calibFinishRange(arm) {
      if (calibrationPhases[arm] !== "recording_range") {
        showToast("请先记录中间位置，并等待进入 Min / Max 记录阶段。", "info", 5000);
        return;
      }
      await calibSendEnter(arm, "range motion captured", 3);
    }

    async function calibSendEnter(arm, label, step) {
      const log = arm + "-calib-log";
      try {
        const data = await api("/api/calibration/enter", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ arm, label })
        });
        renderCalibrationStatus(arm, data.status);
        pollCalibration(arm);
      } catch (error) {
        addLog(log, "发送确认失败: " + error.message, "error");
        showToast("发送确认失败: " + error.message, "error", 6000);
      }
    }

    async function calibStop(arm) {
      try {
        const data = await api("/api/calibration/stop", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ arm })
        });
        renderCalibrationStatus(arm, data.status);
        showToast("已请求停止标定", "info");
      } catch (error) {
        showToast("停止失败: " + error.message, "error", 6000);
      }
    }

    async function pollCalibration(arm) {
      try {
        const data = await api("/api/calibration/status?arm=" + encodeURIComponent(arm));
        renderCalibrationStatus(arm, data.status);
        if (data.status && data.status.running) {
          setTimeout(() => pollCalibration(arm), 1000);
        } else if (data.status && data.status.returncode === 0) {
          completeCalibSteps(arm, 4);
          markTabCompleted(arm + "-calib");
          showToast((arm === "follower" ? "从臂" : "主臂") + "标定完成", "success", 5000);
        }
      } catch (error) {
        addLog(arm + "-calib-log", "状态刷新失败: " + error.message, "error");
      }
    }

    function renderCalibrationStatus(arm, status) {
      const log = $(arm + "-calib-log");
      if (!status || !Array.isArray(status.logs)) return;
      updateCalibrationPhase(arm, status.phase, status.running, status.returncode);
      renderRangeTable(arm, status.ranges || {}, status);
      log.innerHTML = "";
      status.logs.forEach(item => {
        const line = document.createElement("div");
        line.className = "log-line " + (item.type || "info");
        line.textContent = "[" + (item.time || "--:--:--") + "] " + item.text;
        log.appendChild(line);
      });
      log.scrollTop = log.scrollHeight;
    }

    function updateCalibrationPhase(arm, phase, running, returncode) {
      const phaseText = {
        idle: "未启动",
        starting: "正在启动标定进程",
        connecting: "正在连接机械臂",
        existing_cache_prompt: "检测到已有缓存，正在切换为重新标定",
        middle_position: "等待确认中间位置",
        recording_range: "正在记录 Min / Max，请移动所有关节",
        saving: "正在保存标定",
        saved: "标定文件已保存",
        stopping: "正在停止",
        failed: "标定失败",
        finished: "标定完成"
      };
      const el = $(arm + "-calib-phase");
      if (el) {
        el.textContent = phaseText[phase] || phase || "未知";
      }
      calibrationPhases[arm] = phase || "idle";
      updateCalibrationControls(arm, phase || "idle", !!running, returncode);
      if (phase === "middle_position") completeCalibSteps(arm, 1);
      if (phase === "recording_range") completeCalibSteps(arm, 2);
      if (phase === "saving" || phase === "saved") completeCalibSteps(arm, 3);
      if (!running && returncode === 0) completeCalibSteps(arm, 4);
    }

    function updateCalibrationControls(arm, phase, running, returncode) {
      const startBtn = $(arm + "-calib-start-btn");
      const middleBtn = $(arm + "-calib-middle-btn");
      const finishBtn = $(arm + "-calib-finish-btn");
      const stopBtn = $(arm + "-calib-stop-btn");
      const hint = $(arm + "-calib-hint");
      const readyForMiddle = phase === "middle_position";
      const readyForFinish = phase === "recording_range";
      if (startBtn) startBtn.disabled = !!running;
      if (middleBtn) middleBtn.disabled = !readyForMiddle;
      if (finishBtn) finishBtn.disabled = !readyForFinish;
      if (stopBtn) stopBtn.disabled = !running;
      if (!hint) return;
      hint.classList.toggle("ready", readyForMiddle || readyForFinish || phase === "saved");
      if (phase === "starting" || phase === "connecting" || phase === "existing_cache_prompt") {
        hint.textContent = "标定进程正在启动并读取第一帧关节数据，CPU 较慢时可能需要几秒。请等待按钮变亮后再记录中间位置。";
      } else if (readyForMiddle) {
        hint.textContent = "已准备好。请把所有关节放在运动范围中间，然后点击“记录中间位置”。";
      } else if (readyForFinish) {
        hint.textContent = "正在记录关节范围。请缓慢推动每个关节走完整范围，完成后点击“结束并保存”。";
      } else if (phase === "saving") {
        hint.textContent = "正在保存标定文件，请等待完成。";
      } else if (phase === "saved" || (!running && returncode === 0)) {
        hint.textContent = "标定已保存。";
      } else if (phase === "failed") {
        hint.textContent = "标定失败，请查看下方日志并确认串口未被其他程序占用。";
      } else {
        hint.textContent = "点击“启动标定”后，请等待进入“等待确认中间位置”阶段，再点击“记录中间位置”。";
      }
    }

    function renderRangeTable(arm, ranges, status) {
      const panel = $(arm + "-range-panel");
      const target = $(arm + "-range-table");
      const rows = Object.entries(ranges || {});
      if (!panel || !target) {
        return;
      }
      if (rows.length === 0) {
        const phase = status && status.phase ? status.phase : "idle";
        if (!status || (!status.running && phase === "idle")) {
          panel.style.display = "none";
          return;
        }
        panel.style.display = "block";
        const placeholderRows = calibrationJoints.map(name => `<tr>
          <td>${escapeHtml(name)}</td>
          <td>--</td>
          <td>--</td>
          <td>--</td>
          <td><div class="range-progress"><span style="width:0%"></span></div></td>
        </tr>`).join("");
        target.innerHTML = `<div style="color:#98a2b3;margin:4px 0 10px;">
          等待第一帧关节数据。如果长时间没有数值，请确认 ${arm === "leader" ? "主臂" : "从臂"} 串口正确、未被 Teleop/MCP/RynnBot 占用，并等待日志进入中间位置阶段。
        </div>
        <table class="range-table">
          <thead><tr><th>Joint</th><th>Min</th><th>Pos</th><th>Max</th><th>Range</th></tr></thead>
          <tbody>${placeholderRows}</tbody>
        </table>`;
        return;
      }
      panel.style.display = "block";
      const body = rows.map(([name, item]) => {
        const minValue = Number(item.min);
        const posValue = Number(item.position);
        const maxValue = Number(item.max);
        const span = Math.max(1, maxValue - minValue);
        const pct = Math.max(0, Math.min(100, ((posValue - minValue) / span) * 100));
        return `<tr>
          <td>${escapeHtml(name)}</td>
          <td>${minValue}</td>
          <td>${posValue}</td>
          <td>${maxValue}</td>
          <td><div class="range-progress"><span style="width:${pct.toFixed(1)}%"></span></div></td>
        </tr>`;
      }).join("");
      target.innerHTML = `<table class="range-table">
        <thead><tr><th>Joint</th><th>Min</th><th>Pos</th><th>Max</th><th>Range</th></tr></thead>
        <tbody>${body}</tbody>
      </table>`;
    }

    function finishConfig() {
      completeCalibSteps("leader", 4);
      markTabCompleted("leader-calib");
      addLog("leader-calib-log", "配置流程已完成。", "info");
      showToast("SO101 配置流程完成，可以开始真机测试", "success", 5000);
    }

    function completeCalibSteps(arm, count) {
      for (let i = 1; i <= 4; i++) {
        const step = $(arm + "-calib-step-" + i);
        if (!step) continue;
        step.classList.toggle("completed", i <= count);
        step.classList.toggle("active", i === count + 1 && count < 4);
      }
    }

    function clearLog(id) {
      $(id).innerHTML = "";
    }

    function addLog(id, text, type = "info") {
      const line = document.createElement("div");
      line.className = "log-line " + type;
      const now = new Date().toLocaleTimeString();
      line.textContent = "[" + now + "] " + text;
      $(id).appendChild(line);
      $(id).scrollTop = $(id).scrollHeight;
    }

    function setMappingValue(id, value, prefix) {
      const el = $(id);
      if (value === null || value === undefined || value === "") {
        el.textContent = "(未选择)";
        el.classList.add("unset");
      } else {
        el.textContent = prefix + value;
        el.classList.remove("unset");
      }
    }

    function escapeHtml(value) {
      return String(value).replace(/[&<>"']/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
    }

    function escapeAttr(value) {
      return String(value).replace(/\\/g, "\\\\").replace(/'/g, "\\'");
    }

    loadConfig().catch(error => showToast("加载配置失败: " + error.message, "error", 6000));
  </script>
</body>
</html>
"""

COMMANDS = {
    "configure": "rynnrcp-so101-configure",
    "server": "rynnrcp-server",
    "mcp": "rynnrcp-mcp-app",
    "rynnbot": "rynnrcp-rynnbot-app",
    "teleop": "rynnrcp-teleop-app",
}

CALIBRATION_LOG_LIMIT = 500
ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
RANGE_ROW_RE = re.compile(r"^\s*([A-Za-z0-9_]+)\s+\|\s+(-?\d+)\s+\|\s+(-?\d+)\s+\|\s+(-?\d+)")
CALIBRATION_EVENT_PREFIX = "RCP_CALIB_JSON "
EXISTING_CALIBRATION_PROMPT = "type 'c' and press ENTER to run calibration:"
MIDDLE_POSITION_PROMPT = "middle of its range of motion and press ENTER"


def _normalize_arm(arm: str) -> str:
    normalized = str(arm).strip().lower()
    if normalized not in {"follower", "leader"}:
        raise ValueError("arm must be 'follower' or 'leader'")
    return normalized


class CalibrationJob:
    """Run one SO101 calibration process and expose browser-friendly controls."""

    def __init__(self, arm: str) -> None:
        self.arm = _normalize_arm(arm)
        self._process: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._logs: list[Dict[str, str]] = []
        self._command: list[str] = []
        self._phase = "idle"
        self._ranges: dict[str, Dict[str, int]] = {}
        self._forced_recalibration = False
        self._lock = threading.RLock()

    def start(self) -> Dict[str, Any]:
        with self._lock:
            if self.is_running():
                raise RuntimeError(f"{self.arm} calibration is already running")
            configs = load_all_configs()
            self._command = build_calibration_command(self.arm, configs)
            self._logs = []
            self._phase = "starting"
            self._ranges = {}
            self._forced_recalibration = False
            self._append("info", "Starting calibration process")
            self._append("info", "Command: " + " ".join(self._command))

            env = os.environ.copy()
            env.setdefault("PYTHONUNBUFFERED", "1")
            self._process = subprocess.Popen(
                self._command,
                cwd=str(REPO_ROOT),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
            )
            self._reader = threading.Thread(target=self._read_output_loop, daemon=True)
            self._reader.start()
            return self.status()

    def send_enter(self, label: str = "continue") -> Dict[str, Any]:
        with self._lock:
            if not self.is_running() or self._process is None or self._process.stdin is None:
                raise RuntimeError(f"{self.arm} calibration is not running")
            self._send_text("\n", f"Browser confirmed: {label}")
            return self.status()

    def stop(self, timeout_s: float = 2.0) -> Dict[str, Any]:
        with self._lock:
            if self.is_running() and self._process is not None:
                self._append("warning", "Stopping calibration process")
                self._phase = "stopping"
                process = self._process
                process.terminate()
                try:
                    process.wait(timeout=timeout_s)
                except subprocess.TimeoutExpired:
                    self._append("warning", "Calibration process ignored terminate; killing it")
                    process.kill()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        self._append("error", "Calibration process is still alive after kill")
            return self.status()

    def reset(self) -> Dict[str, Any]:
        with self._lock:
            if self.is_running():
                raise RuntimeError(f"{self.arm} calibration is still running")
            self._logs = []
            self._command = []
            self._phase = "idle"
            self._ranges = {}
            self._forced_recalibration = False
            self._process = None
            self._reader = None
            return self.status()

    def status(self) -> Dict[str, Any]:
        with self._lock:
            returncode = self._process.poll() if self._process is not None else None
            running = self._process is not None and returncode is None
            return {
                "arm": self.arm,
                "running": running,
                "returncode": returncode,
                "phase": self._phase,
                "ranges": dict(self._ranges),
                "command": list(self._command),
                "logs": list(self._logs),
            }

    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _read_output_loop(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        try:
            buffer = ""
            while True:
                char = process.stdout.read(1)
                if char == "":
                    break
                buffer += char
                if (
                    EXISTING_CALIBRATION_PROMPT in buffer
                    and not self._forced_recalibration
                ):
                    self._handle_output_line(buffer)
                    buffer = ""
                    self._forced_recalibration = True
                    self._phase = "middle_position"
                    self._send_text(
                        "c\n",
                        "Existing calibration cache detected; browser selected 'c' to run a fresh calibration",
                    )
                elif MIDDLE_POSITION_PROMPT in buffer:
                    self._handle_output_line(buffer)
                    buffer = ""
                elif char == "\n":
                    self._handle_output_line(buffer)
                    buffer = ""
            if buffer:
                self._handle_output_line(buffer)
            returncode = process.wait()
            if returncode == 0:
                self._phase = "finished"
                self._append("info", "Calibration process finished successfully")
            else:
                self._phase = "failed"
                self._append("error", f"Calibration process exited with code {returncode}")
        except Exception as exc:
            LOGGER.exception("Calibration output reader failed for %s", self.arm)
            self._phase = "failed"
            self._append("error", f"Calibration output reader failed: {exc}")

    def _handle_output_line(self, line: str) -> None:
        clean = ANSI_RE.sub("", line).strip()
        if not clean:
            return
        if clean.startswith(CALIBRATION_EVENT_PREFIX):
            self._handle_structured_event(clean[len(CALIBRATION_EVENT_PREFIX):])
            return
        self._update_phase_from_output(clean)
        self._update_ranges_from_output(clean)
        self._append(_log_type(clean), clean)

    def _handle_structured_event(self, text: str) -> None:
        try:
            event = json.loads(text)
        except json.JSONDecodeError:
            self._append("warning", f"Invalid calibration event: {text}")
            return
        kind = event.get("event")
        if "phase" in event:
            self._phase = str(event["phase"])
        if kind == "ranges":
            ranges = event.get("ranges") or {}
            if isinstance(ranges, dict):
                self._ranges = {
                    str(name): {
                        "min": int(values["min"]),
                        "position": int(values["position"]),
                        "max": int(values["max"]),
                    }
                    for name, values in ranges.items()
                    if isinstance(values, dict)
                }
        elif kind in {"phase", "log"}:
            message = str(event.get("message") or event.get("phase") or "")
            level = str(event.get("level") or "info")
            if message:
                self._append(level, message)

    def _update_phase_from_output(self, line: str) -> None:
        lower = line.lower()
        if EXISTING_CALIBRATION_PROMPT.lower() in lower:
            self._phase = "existing_cache_prompt"
        elif "move" in lower and "middle of its range" in lower:
            self._phase = "middle_position"
        elif "recording positions" in lower or "entire ranges of motion" in lower:
            self._phase = "recording_range"
        elif "calibration saved to" in lower:
            self._phase = "saved"

    def _update_ranges_from_output(self, line: str) -> None:
        match = RANGE_ROW_RE.match(line)
        if not match:
            return
        motor, min_value, position, max_value = match.groups()
        if motor == "NAME":
            return
        self._ranges[motor] = {
            "min": int(min_value),
            "position": int(position),
            "max": int(max_value),
        }

    def _send_text(self, text: str, log_message: str) -> None:
        if self._process is None or self._process.stdin is None:
            raise RuntimeError(f"{self.arm} calibration process has no stdin")
        self._process.stdin.write(text)
        self._process.stdin.flush()
        self._append("info", log_message)

    def _append(self, kind: str, text: str) -> None:
        with self._lock:
            self._logs.append(
                {
                    "time": time.strftime("%H:%M:%S"),
                    "type": kind,
                    "text": text,
                }
            )
            if len(self._logs) > CALIBRATION_LOG_LIMIT:
                self._logs = self._logs[-CALIBRATION_LOG_LIMIT:]


_CALIBRATION_JOBS = {
    "follower": CalibrationJob("follower"),
    "leader": CalibrationJob("leader"),
}


def create_app():
    """Create the Flask app.

    Flask is imported lazily so the module can still be inspected and tested in
    environments that have not installed the web UI dependencies yet.
    """
    try:
        from flask import Flask, jsonify, render_template_string, request
    except ImportError as exc:  # pragma: no cover - exercised by the CLI path
        raise RuntimeError(
            "SO101 web configuration requires Flask. Install the SO101 package "
            "dependencies, for example: uv pip install -e robots/so101"
        ) from exc

    app = Flask(__name__)

    @app.get("/")
    def index():
        return render_template_string(_load_html_template())

    @app.get("/api/config")
    def api_get_config():
        return jsonify({"ok": True, "config": build_snapshot(load_all_configs()), "paths": _path_snapshot()})

    @app.post("/api/config")
    def api_save_config():
        payload = request.get_json(silent=True) or {}
        if not isinstance(payload, dict):
            return jsonify({"ok": False, "error": "request body must be a JSON object"}), 400

        try:
            snapshot = normalize_snapshot(payload)
            configs = load_all_configs()
            apply_snapshot_to_configs(configs, snapshot)
            for name, config in configs.items():
                save_yaml_config(CONFIG_FILES[name], config)
        except Exception as exc:
            LOGGER.exception("Failed to save SO101 web configuration")
            return jsonify({"ok": False, "error": str(exc)}), 400

        return jsonify({"ok": True, "config": build_snapshot(load_all_configs())})

    @app.get("/api/serial")
    def api_scan_serial():
        return jsonify({"ok": True, "ports": scan_serial_ports()})

    @app.get("/api/cameras")
    def api_scan_cameras():
        max_index = _to_int(request.args.get("max_index", 20), 20)
        return jsonify(scan_cameras(max_index=max_index))

    @app.get("/api/camera/<int:index>/probe")
    def api_camera_probe(index: int):
        return jsonify(probe_camera(index))

    @app.get("/api/camera/<int:index>/preview")
    def api_camera_preview(index: int):
        return jsonify(capture_camera_preview(index))

    @app.get("/api/validate")
    def api_validate():
        return jsonify(validate_configs())

    @app.get("/api/commands")
    def api_commands():
        return jsonify({"ok": True, "commands": COMMANDS, "cwd": str(REPO_ROOT)})

    @app.post("/api/shutdown")
    def api_shutdown():
        for job in _CALIBRATION_JOBS.values():
            try:
                job.stop(timeout_s=1.0)
            except Exception:
                LOGGER.exception("Failed to stop %s calibration during shutdown", job.arm)

        LOGGER.info("SO101 configuration UI shutdown requested")
        threading.Timer(0.25, lambda: os._exit(0)).start()
        return jsonify({"ok": True, "message": "SO101 configuration UI is shutting down"})

    @app.post("/api/calibration/start")
    def api_calibration_start():
        try:
            arm = _request_arm(request.get_json(silent=True) or {})
            return jsonify({"ok": True, "status": _CALIBRATION_JOBS[arm].start()})
        except Exception as exc:
            LOGGER.exception("Failed to start SO101 calibration")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/calibration/enter")
    def api_calibration_enter():
        try:
            payload = request.get_json(silent=True) or {}
            arm = _request_arm(payload)
            label = str(payload.get("label") or "continue")
            return jsonify({"ok": True, "status": _CALIBRATION_JOBS[arm].send_enter(label)})
        except Exception as exc:
            LOGGER.exception("Failed to send SO101 calibration input")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/calibration/stop")
    def api_calibration_stop():
        try:
            arm = _request_arm(request.get_json(silent=True) or {})
            return jsonify({"ok": True, "status": _CALIBRATION_JOBS[arm].stop()})
        except Exception as exc:
            LOGGER.exception("Failed to stop SO101 calibration")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/calibration/reset")
    def api_calibration_reset():
        try:
            arm = _request_arm(request.get_json(silent=True) or {})
            return jsonify({"ok": True, "status": _CALIBRATION_JOBS[arm].reset()})
        except Exception as exc:
            LOGGER.exception("Failed to reset SO101 calibration")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.get("/api/calibration/status")
    def api_calibration_status():
        try:
            arm = _normalize_arm(request.args.get("arm", "follower"))
            return jsonify({"ok": True, "status": _CALIBRATION_JOBS[arm].status()})
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    return app


def build_calibration_command(arm: str, configs: Mapping[str, Mapping[str, Any]]) -> list[str]:
    return [sys.executable, "-m", "rynnrcp_robot_so101.configure_so101_web", "--calibrate"] + calibration_args(arm, configs)


def calibration_args(arm: str, configs: Mapping[str, Mapping[str, Any]]) -> list[str]:
    normalized = _normalize_arm(arm)
    if normalized == "follower":
        server = configs["follower_server"]
        robot = _nested(server, "components", "robot")
        port = _require_text(robot.get("port"), "components.robot.port")
        robot_id = _require_text(_nested(server, "manifest").get("robot_id"), "manifest.robot_id")
        return [
            "--arm=follower",
            f"--port={port}",
            f"--id={robot_id}",
        ]

    server = configs["leader_server"]
    leader = _nested(server, "components", "robot")
    port = _require_text(leader.get("port"), "components.robot.port")
    robot_id = _require_text(_nested(server, "manifest").get("robot_id"), "manifest.robot_id")
    return [
        "--arm=leader",
        f"--port={port}",
        f"--id={robot_id}",
    ]


def _request_arm(payload: Mapping[str, Any]) -> str:
    return _normalize_arm(str(payload.get("arm", "follower")))


def _log_type(line: str) -> str:
    lower = line.lower()
    if "error" in lower or "traceback" in lower or "failed" in lower:
        return "error"
    if "warning" in lower or "warn" in lower:
        return "warning"
    return "info"


def load_yaml_config(path: Path) -> Dict[str, Any]:
    import yaml

    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def save_yaml_config(path: Path, data: Mapping[str, Any]) -> None:
    import yaml

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(dict(data), fh, sort_keys=False, allow_unicode=True)


def load_all_configs() -> Dict[str, Dict[str, Any]]:
    return {name: load_yaml_config(path) for name, path in CONFIG_FILES.items()}


def build_snapshot(configs: Mapping[str, Mapping[str, Any]]) -> Dict[str, Any]:
    follower_server = configs["follower_server"]
    leader_server = configs["leader_server"]
    rynnbot_app = configs["rynnbot_app"]
    rynnbot_cfg = _nested(rynnbot_app, "app")
    manifest = _nested(follower_server, "manifest")

    return {
        "server": {
            "id": manifest.get("robot_id", ""),
        },
        "hardware": {
            "follower_port": _nested(follower_server, "components", "robot").get("port", ""),
            "leader_port": _nested(leader_server, "components", "robot").get("port", ""),
            "front_camera": _nested(follower_server, "components", "front_camera").get("device_id", 0),
            "wrist_camera": _nested(follower_server, "components", "wrist_camera").get("device_id", 1),
        },
        "rynnbot": {
            "product_key": rynnbot_cfg.get("product_key", ""),
            "device_name": rynnbot_cfg.get("device_name", ""),
            "device_secret": rynnbot_cfg.get("device_secret", ""),
            "http_url": rynnbot_cfg.get("http_url", "https://robot-access.damo-academy.com"),
            "image_upload_codec": rynnbot_cfg.get("image_upload_codec", "jpeg"),
        },
    }


def normalize_snapshot(payload: Mapping[str, Any]) -> Dict[str, Any]:
    current = build_snapshot(load_all_configs())
    merged = _deep_merge(current, payload)

    hardware = merged["hardware"]
    rynnbot = merged["rynnbot"]
    server = merged["server"]

    follower_port = str(hardware.get("follower_port", "")).strip()
    leader_port = str(hardware.get("leader_port", "")).strip()
    if not follower_port:
        raise ValueError("hardware.follower_port is required")
    if not leader_port:
        raise ValueError("hardware.leader_port is required")

    return {
        "server": {
            "id": _require_text(server.get("id"), "manifest.robot_id"),
        },
        "hardware": {
            "follower_port": follower_port,
            "leader_port": leader_port,
            "front_camera": _require_non_negative_int(hardware.get("front_camera"), "hardware.front_camera"),
            "wrist_camera": _require_non_negative_int(hardware.get("wrist_camera"), "hardware.wrist_camera"),
        },
        "rynnbot": {
            "product_key": str(rynnbot.get("product_key", "")).strip(),
            "device_name": str(rynnbot.get("device_name", "")).strip(),
            "device_secret": str(rynnbot.get("device_secret", "")).strip(),
            "http_url": _require_text(rynnbot.get("http_url"), "rynnbot.http_url"),
            "image_upload_codec": _image_upload_codec(rynnbot.get("image_upload_codec")),
        },
    }


def apply_snapshot_to_configs(configs: Mapping[str, Dict[str, Any]], snapshot: Mapping[str, Any]) -> None:
    server_snapshot = snapshot["server"]
    hardware = snapshot["hardware"]
    rynnbot = snapshot["rynnbot"]

    follower_server = configs["follower_server"]
    leader_server = configs["leader_server"]
    _set_nested(follower_server, server_snapshot["id"], "manifest", "robot_id")
    _set_nested(follower_server, "SO101 Follower", "manifest", "robot_name")
    _set_nested(follower_server, hardware["follower_port"], "components", "robot", "port")
    _set_nested(follower_server, "follower", "components", "robot", "role")
    _set_nested(follower_server, hardware["front_camera"], "components", "front_camera", "device_id")
    _set_nested(follower_server, hardware["wrist_camera"], "components", "wrist_camera", "device_id")
    _set_nested(
        follower_server,
        "package://rynnrcp_robot_so101/config/robot_integration.yaml",
        "integration",
        "config",
    )
    _clean_server_config(follower_server)

    _set_nested(leader_server, "so101_leader", "manifest", "robot_id")
    _set_nested(leader_server, "SO101 Leader", "manifest", "robot_name")
    _set_nested(leader_server, hardware["leader_port"], "components", "robot", "port")
    _set_nested(leader_server, "leader", "components", "robot", "role")
    _set_nested(
        leader_server,
        "package://rynnrcp_robot_so101/config/robot_integration.yaml",
        "integration",
        "config",
    )
    _clean_server_config(leader_server)

    for key in ("product_key", "device_name", "device_secret", "http_url", "image_upload_codec"):
        _set_nested(configs["rynnbot_app"], rynnbot[key], "app", key)

    _clean_rynnbot_app_config(configs["rynnbot_app"])


def _clean_server_config(config: Dict[str, Any]) -> None:
    server = config.get("server")
    if isinstance(server, dict):
        for key in ("config_name", "capabilities", "interface", "metadata"):
            server.pop(key, None)
        if not server:
            config.pop("server", None)
    config.pop("robot", None)
    config.pop("runtime", None)


def _clean_rynnbot_app_config(config: Dict[str, Any]) -> None:
    app = config.get("app")
    if isinstance(app, dict):
        app.pop("name", None)
    config.pop("target_server", None)
    config.pop("discovery", None)


def _image_upload_codec(value: Any) -> str:
    codec = str(value or "jpeg").strip()
    return codec if codec in ("jpeg", "npy_gzip") else "jpeg"


def scan_serial_ports() -> list[Dict[str, str]]:
    ports: list[Dict[str, str]] = []
    seen: set[str] = set()

    try:
        from serial.tools import list_ports

        for item in list_ports.comports():
            device = str(item.device)
            ports.append(
                {
                    "device": device,
                    "description": str(item.description or ""),
                    "hwid": str(item.hwid or ""),
                }
            )
            seen.add(device)
    except Exception as exc:
        LOGGER.info("pyserial serial scan unavailable: %s", exc)

    for pattern in (
        "/dev/tty.usbmodem*",
        "/dev/cu.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/cu.usbserial*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ):
        for device in sorted(glob.glob(pattern)):
            if device not in seen:
                ports.append({"device": device, "description": "serial device", "hwid": ""})
                seen.add(device)

    return _dedupe_serial_ports(ports)


def _dedupe_serial_ports(ports: list[Dict[str, str]]) -> list[Dict[str, str]]:
    selected: dict[str, Dict[str, str]] = {}
    for port in ports:
        device = str(port.get("device") or "")
        if not device or _is_ignored_serial_device(device):
            continue
        key = _serial_device_key(device)
        current = selected.get(key)
        if current is None or _serial_device_rank(device) < _serial_device_rank(str(current.get("device") or "")):
            selected[key] = dict(port)

    return sorted(selected.values(), key=lambda item: str(item.get("device") or ""))


def _is_ignored_serial_device(device: str) -> bool:
    lower = device.lower()
    return "bluetooth-incoming-port" in lower or "wlan-debug" in lower


def _serial_device_key(device: str) -> str:
    name = device.rsplit("/", 1)[-1]
    for prefix in ("cu.", "tty."):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _serial_device_rank(device: str) -> int:
    name = device.rsplit("/", 1)[-1]
    if name.startswith("cu."):
        return 0
    if name.startswith("tty."):
        return 1
    return 2


def scan_cameras(max_index: int = 20) -> Dict[str, Any]:
    try:
        import cv2
    except Exception as exc:
        return {"ok": False, "error": f"OpenCV is not available: {exc}", "cameras": []}

    cameras: list[Dict[str, Any]] = []
    errors: list[str] = []
    for index in range(max(0, int(max_index)) + 1):
        result = _probe_camera_with_cv2(cv2, index)
        if result.get("ok"):
            cameras.append(result["camera"])
        elif result.get("error"):
            errors.append(f"camera {index}: {result['error']}")

    return {"ok": True, "cameras": cameras, "errors": errors}


def probe_camera(index: int) -> Dict[str, Any]:
    try:
        import cv2
    except Exception as exc:
        return {"ok": False, "index": index, "error": f"OpenCV is not available: {exc}"}
    return _probe_camera_with_cv2(cv2, index)


def _probe_camera_with_cv2(cv2_module: Any, index: int) -> Dict[str, Any]:
    cap = cv2_module.VideoCapture(index)
    try:
        if not cap.isOpened():
            return {"ok": False, "index": index, "opened": False}
        ok, frame = cap.read()
        item: Dict[str, Any] = {"index": index, "opened": True}
        if ok and frame is not None:
            item.update(_encode_frame(cv2_module, frame))
            return {"ok": True, "camera": item}
        return {"ok": False, "index": index, "opened": True, "error": "opened but no frame was returned"}
    except Exception as exc:
        return {"ok": False, "index": index, "error": str(exc)}
    finally:
        cap.release()


def capture_camera_preview(index: int) -> Dict[str, Any]:
    try:
        import cv2
    except Exception as exc:
        return {"ok": False, "error": f"OpenCV is not available: {exc}"}

    cap = cv2.VideoCapture(index)
    try:
        if not cap.isOpened():
            return {"ok": False, "error": f"camera {index} could not be opened"}
        ok, frame = cap.read()
        if not ok or frame is None:
            return {"ok": False, "error": f"camera {index} returned no frame"}
        encoded = _encode_frame(cv2, frame)
        encoded["ok"] = True
        encoded["index"] = index
        return encoded
    except Exception as exc:
        LOGGER.exception("Failed to preview SO101 camera %s", index)
        return {"ok": False, "error": str(exc)}
    finally:
        cap.release()


def validate_configs() -> Dict[str, Any]:
    for path in (str(REPO_ROOT), str(SO101_PROJECT_ROOT)):
        if path not in sys.path:
            sys.path.insert(0, path)

    results: list[Dict[str, Any]] = []
    try:
        from rynnrcp.config.loader import load_config
        from rynnrcp.config.validator import ConfigValidator
    except Exception as exc:
        return {"ok": False, "error": f"RynnRCP config validator is not available: {exc}", "results": []}

    for name, path in CONFIG_FILES.items():
        try:
            source_config = load_yaml_config(path)
            ConfigValidator.validate_source(source_config)
            expanded_config = load_config(str(path))
            ConfigValidator.validate(expanded_config)
            results.append({"name": name, "ok": True, "path": str(path)})
        except Exception as exc:
            LOGGER.exception("SO101 config validation failed for %s", path)
            results.append({"name": name, "ok": False, "path": str(path), "error": str(exc)})

    return {"ok": all(item["ok"] for item in results), "results": results}


def _encode_frame(cv2_module: Any, frame: Any) -> Dict[str, Any]:
    height, width = frame.shape[:2]
    scale = min(1.0, 360.0 / float(max(width, height)))
    if scale < 1.0:
        frame = cv2_module.resize(frame, (int(width * scale), int(height * scale)))
    ok, data = cv2_module.imencode(".jpg", frame, [int(cv2_module.IMWRITE_JPEG_QUALITY), 75])
    if not ok:
        raise RuntimeError("failed to encode camera frame as jpg")
    return {
        "width": int(width),
        "height": int(height),
        "image": "data:image/jpeg;base64," + base64.b64encode(data.tobytes()).decode("ascii"),
    }


def _path_snapshot() -> Dict[str, str]:
    paths = {name: str(path) for name, path in CONFIG_FILES.items()}
    paths["config_dir"] = str(CONFIG_DIR)
    return paths


def _load_html_template() -> str:
    return HTML_TEMPLATE


def _nested(root: Mapping[str, Any], *keys: str) -> Mapping[str, Any]:
    current: Any = root
    for key in keys:
        if not isinstance(current, Mapping):
            return {}
        current = current.get(key, {})
    return current if isinstance(current, Mapping) else {}


def _set_nested(root: Dict[str, Any], value: Any, *keys: str) -> None:
    current = root
    for key in keys[:-1]:
        next_value = current.get(key)
        if not isinstance(next_value, dict):
            next_value = {}
            current[key] = next_value
        current = next_value
    current[keys[-1]] = value


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> Dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _to_int(value: Any, fallback: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback


def _require_non_negative_int(value: Any, field: str) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be an integer") from exc
    if parsed < 0:
        raise ValueError(f"{field} must be >= 0")
    return parsed


def _require_port(value: Any, field: str) -> int:
    parsed = _require_non_negative_int(value, field)
    if parsed == 0 or parsed > 65535:
        raise ValueError(f"{field} must be in 1..65535")
    return parsed


def _require_text(value: Any, field: str) -> str:
    text = str(value or "").strip()
    if not text:
        raise ValueError(f"{field} is required")
    return text


def _get_local_ip() -> str:
    """Return the LAN IP of this machine, fallback to 127.0.0.1."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def _is_port_available(host: str, port: int) -> bool:
    bind_host = "127.0.0.1" if host == "0.0.0.0" else host
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((bind_host, int(port)))
        except OSError:
            return False
    return True


def _select_available_port(host: str, preferred_port: int) -> int:
    for port in range(int(preferred_port), int(preferred_port) + 100):
        if _is_port_available(host, port):
            return port
    raise RuntimeError(f"No available port found in {preferred_port}..{preferred_port + 99}")


# ---------------------------------------------------------------------------
#  Calibration runner (invoked as a subprocess via --calibrate)
# ---------------------------------------------------------------------------

_CALIB_EVENT_PREFIX = "RCP_CALIB_JSON "
_CALIB_POLL_INTERVAL_S = 0.15
_CALIB_ENTER_EVENT = threading.Event()
_CALIB_STDIN_STARTED = False


def _run_calibration_cli(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Run SO101 calibration with structured progress output.")
    parser.add_argument("--calibrate", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--arm", choices=["follower", "leader"], required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", required=True)
    args = parser.parse_args(argv)

    device: Any | None = None
    try:
        device = _calib_create_device(args.arm, args.port, args.id)
        _calib_emit("phase", phase="connecting", message=f"Connecting {args.arm} on {args.port}")
        device.connect(calibrate=False)
        _calib_emit("log", level="info", message=f"{args.arm} connected")

        bus = device.bus
        bus.disable_torque()
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, _calib_operating_mode_position_value())

        _calib_emit(
            "phase",
            phase="middle_position",
            message="Move all joints to the middle of their range, then click Record Middle Position.",
        )
        _calib_wait_for_enter_with_positions(bus, phase="middle_position")

        homing_offsets = bus.set_half_turn_homings()
        _calib_emit("log", level="info", message=f"Homing offsets captured: {homing_offsets}")

        _calib_emit(
            "phase",
            phase="recording_range",
            message="Move every joint through its full range, then click Finish And Save.",
        )
        range_mins, range_maxes = _calib_record_ranges_with_progress(bus)

        device.calibration = _calib_build_calibration(device, homing_offsets, range_mins, range_maxes)
        bus.write_calibration(device.calibration)
        device._save_calibration()
        _calib_emit("phase", phase="saved", message=f"Calibration saved to {device.calibration_fpath}")
        _calib_emit("log", level="info", message=f"Calibration saved to {device.calibration_fpath}")
        return 0
    except Exception as exc:
        _calib_emit("log", level="error", message=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        if device is not None:
            try:
                device.disconnect()
                _calib_emit("log", level="info", message="Disconnected")
            except Exception as exc:
                _calib_emit("log", level="warning", message=f"Disconnect ignored: {exc}")


def _calib_create_device(arm: str, port: str, device_id: str) -> Any:
    if arm == "follower":
        from lerobot_so101 import SO101Follower, SO101FollowerConfig

        return SO101Follower(SO101FollowerConfig(port=port, id=device_id, use_degrees=False))

    from lerobot_so101 import SO101Leader, SO101LeaderConfig

    return SO101Leader(SO101LeaderConfig(port=port, id=device_id, use_degrees=False))


def _calib_operating_mode_position_value() -> int:
    from lerobot_so101 import OperatingMode

    return OperatingMode.POSITION.value


def _calib_build_calibration(
    device: Any,
    homing_offsets: dict[str, int],
    range_mins: dict[str, int],
    range_maxes: dict[str, int],
) -> dict[str, Any]:
    from lerobot_so101 import MotorCalibration

    calibration = {}
    for motor, motor_config in device.bus.motors.items():
        calibration[motor] = MotorCalibration(
            id=motor_config.id,
            drive_mode=0,
            homing_offset=homing_offsets[motor],
            range_min=range_mins[motor],
            range_max=range_maxes[motor],
        )
    return calibration


def _calib_wait_for_enter_with_positions(bus: Any, phase: str) -> None:
    while True:
        positions = bus.sync_read("Present_Position", normalize=False)
        _calib_emit_ranges(phase, positions, positions, positions)
        if _calib_enter_pressed():
            return
        time.sleep(_CALIB_POLL_INTERVAL_S)


def _calib_record_ranges_with_progress(bus: Any) -> tuple[dict[str, int], dict[str, int]]:
    motors = list(bus.motors)
    start_positions = bus.sync_read("Present_Position", motors, normalize=False)
    mins = start_positions.copy()
    maxes = start_positions.copy()

    while True:
        positions = bus.sync_read("Present_Position", motors, normalize=False)
        mins = {motor: min(positions[motor], mins[motor]) for motor in motors}
        maxes = {motor: max(positions[motor], maxes[motor]) for motor in motors}
        _calib_emit_ranges("recording_range", positions, mins, maxes)
        if _calib_enter_pressed():
            break
        time.sleep(_CALIB_POLL_INTERVAL_S)

    same_min_max = [motor for motor in motors if mins[motor] == maxes[motor]]
    if same_min_max:
        raise ValueError(f"Some motors have the same min and max values: {same_min_max}")
    return mins, maxes


def _calib_enter_pressed() -> bool:
    global _CALIB_STDIN_STARTED
    if not _CALIB_STDIN_STARTED:
        _CALIB_STDIN_STARTED = True
        threading.Thread(target=_calib_stdin_loop, daemon=True).start()
    if not _CALIB_ENTER_EVENT.is_set():
        return False
    _CALIB_ENTER_EVENT.clear()
    return True


def _calib_stdin_loop() -> None:
    for line in sys.stdin:
        if line.strip() == "":
            _CALIB_ENTER_EVENT.set()


def _calib_emit_ranges(
    phase: str,
    positions: dict[str, Any],
    mins: dict[str, Any],
    maxes: dict[str, Any],
) -> None:
    ranges = {
        motor: {
            "min": int(mins[motor]),
            "position": int(positions[motor]),
            "max": int(maxes[motor]),
        }
        for motor in positions
    }
    _calib_emit("ranges", phase=phase, ranges=ranges)


def _calib_emit(event: str, **payload: Any) -> None:
    print(_CALIB_EVENT_PREFIX + json.dumps({"event": event, **payload}, ensure_ascii=False), flush=True)


def main(argv: Iterable[str] | None = None) -> int:
    argv_list = list(argv) if argv is not None else sys.argv[1:]
    if "--calibrate" in argv_list:
        return _run_calibration_cli(argv_list)

    parser = argparse.ArgumentParser(description="Configure SO101 RynnRCP config files in a browser.")
    parser.add_argument("--host", default="127.0.0.1", help="Host for the configuration web server.")
    parser.add_argument("--port", default=28401, type=int, help="Port for the configuration web server.")
    parser.add_argument("--debug", action="store_true", help="Enable Flask debug mode.")
    parser.add_argument("--no-open", action="store_true", help="Do not open a browser automatically.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[logging.StreamHandler()],
    )

    try:
        app = create_app()
    except RuntimeError as exc:
        LOGGER.error("%s", exc)
        return 2

    try:
        port = _select_available_port(args.host, args.port)
    except RuntimeError as exc:
        LOGGER.error("%s", exc)
        return 2
    if port != args.port:
        LOGGER.warning("Port %s is in use; using %s instead", args.port, port)

    url = f"http://{args.host}:{port}"
    browser_host = _get_local_ip() if args.host == "0.0.0.0" else args.host
    browser_url = f"http://{browser_host}:{port}"
    LOGGER.info("SO101 configuration UI: %s", url)
    LOGGER.info("Config directory: %s", CONFIG_DIR)
    if not args.no_open:
        threading.Timer(0.6, lambda: webbrowser.open(browser_url)).start()
    app.run(host=args.host, port=port, debug=args.debug, use_reloader=args.debug)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
