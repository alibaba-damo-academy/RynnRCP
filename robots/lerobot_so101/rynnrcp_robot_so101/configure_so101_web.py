#!/usr/bin/env python3
"""Browser-based SO101 configuration helper for RynnRCP."""

from __future__ import annotations

import argparse
import base64
import glob
import json
import logging
import math
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

from rynnrcp.utils.device_identity import machine_mac_suffix, with_machine_suffix
from rynnrcp.utils.web_urls import browser_urls, primary_browser_url


LOGGER = logging.getLogger("rynnrcp.so101.configure_web")

PACKAGE_DIR = Path(__file__).resolve().parent
SO101_PROJECT_ROOT = PACKAGE_DIR.parent
REPO_ROOT = PACKAGE_DIR.parents[2]
CONFIG_DIR = PACKAGE_DIR / "config"
PROFILE_CONFIG_FILES = {
    "single": {
        "follower_server": CONFIG_DIR / "so101_follower_server.yaml",
        "leader_server": CONFIG_DIR / "so101_leader_server.yaml",
        "rynnbot_app": CONFIG_DIR / "so101_rynnbot_app.yaml",
        "master_rynnbot_app": CONFIG_DIR / "so101_master_rynnbot_app.yaml",
    },
    "dual": {
        "follower_server": CONFIG_DIR / "so101_bimanual_follower_server.yaml",
        "leader_server": CONFIG_DIR / "so101_bimanual_leader_server.yaml",
        "rynnbot_app": CONFIG_DIR / "so101_bimanual_rynnbot_app.yaml",
        "master_rynnbot_app": CONFIG_DIR / "so101_bimanual_master_rynnbot_app.yaml",
    },
}
# Kept for callers and tests that import the original single-arm mapping.
CONFIG_FILES = PROFILE_CONFIG_FILES["single"]

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
    .profile-hidden { display: none !important; }
    .subpanel { background: #202033; border: 1px solid #38384d; border-radius: 10px; padding: 16px; margin: 14px 0; }
    .subpanel h3 { color: #bae6fd; margin-bottom: 10px; }
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
    .motion-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(210px, 1fr)); gap: 12px; margin-top: 18px; }
    .motion-card {
      background: #2a2a3e;
      border: 1px solid #38384d;
      border-radius: 8px;
      padding: 14px;
    }
    .motion-card-title { color: #f2f4f7; font-weight: 800; margin-bottom: 5px; }
    .motion-card-desc { color: #9aa4b2; font-size: 13px; min-height: 42px; }
    .joint-control { display: grid; grid-template-columns: 130px minmax(180px, 1fr) 76px; gap: 12px; align-items: center; margin: 10px 0; }
    .joint-control input[type="range"] { width: 100%; accent-color: #00d4ff; }
    .joint-value { color: #00ff88; font-family: "SFMono-Regular", Consolas, monospace; text-align: right; }
    .debug-action-groups { display: grid; grid-template-columns: repeat(3, minmax(190px, 1fr)); gap: 12px; margin-top: 18px; }
    .debug-action-group { background: #202033; border: 1px solid #38384d; border-radius: 8px; padding: 12px; }
    .debug-action-title { color: #98a2b3; font-size: 12px; font-weight: 700; margin-bottom: 9px; }
    .debug-action-buttons { display: flex; gap: 8px; flex-wrap: wrap; }
    .debug-action-buttons .btn { flex: 1 1 120px; margin: 0; }
    @media (max-width: 900px) { .debug-action-groups { grid-template-columns: repeat(2, minmax(190px, 1fr)); } }
    @media (max-width: 520px) { .debug-action-groups { grid-template-columns: 1fr; } }
    .motion-status-line { color: #cbd5e1; font-weight: 700; }
    .tracking-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 12px; margin-top: 14px; }
    .tracking-card {
      background: #202033;
      border: 1px solid #38384d;
      border-radius: 8px;
      padding: 10px;
    }
    .tracking-title { display: flex; justify-content: space-between; gap: 10px; color: #dbeafe; font-size: 12px; font-weight: 800; }
    .tracking-title span:last-child { color: #ffd166; font-family: "SFMono-Regular", Consolas, monospace; }
    .tracking-card svg { width: 100%; height: 112px; display: block; margin-top: 8px; background: #111827; border-radius: 6px; }
    .tracking-legend { display: flex; gap: 14px; color: #98a2b3; font-size: 12px; margin-top: 8px; }
    .tracking-legend span::before { content: ""; display: inline-block; width: 18px; height: 3px; margin-right: 6px; vertical-align: middle; border-radius: 999px; }
    .tracking-legend .action::before { background: #00d4ff; }
    .tracking-legend .state::before { background: #00ff88; }
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
      <button class="nav-tab" id="tab-motion-test" onclick="showSection('motion-test')">7. 动作测试</button>
    </div>

    <section class="config-section active" id="section-device">
      <h2 class="section-title">机器人身份与云端连接</h2>
      <div class="tip-box">
        <h4>选择机器人构型</h4>
        <p>单臂保持原有 6 DoF 页面；双臂使用四条机械臂串口，并对外暴露一个 left 6 + right 6 的 12 DoF robot 通道。</p>
      </div>
      <div class="form-group" style="max-width:560px;">
        <label class="form-label">要配置哪种机器人</label>
        <select class="form-input" id="robot-profile" onchange="changeProfile(this.value)">
          <option value="single">LeRobot SO101 Single (6 DoF)</option>
          <option value="dual">LeRobot SO101 Dual (12 DoF)</option>
        </select>
      </div>
      <div class="tip-box">
        <h4>先判断哪个本体要接入 RynnBot</h4>
        <p>下面两组字段分别服务于“从臂本体接入云端”和“主臂控制仿真”两种用途。请填写这次运行所使用的一组。</p>
        <p><strong>本地动作测试、本地 Teleop、MCP：</strong>0 套。保留 Robot ID，然后继续下一步。</p>
        <p><strong>从臂本体接入 RynnBot：</strong>1 套，只填写“从臂执行端”。</p>
        <p><strong>真实主臂通过 RynnBot 控制仿真从臂：</strong>1 套，填写“主臂控制端”。</p>
      </div>
      <div class="grid">
        <div class="form-group">
          <label class="form-label">机器人 ID / Robot ID（必填，默认值可直接使用）</label>
          <input class="form-input" id="robot-id" placeholder="so101_follower">
          <div style="margin-top:7px;color:#98a2b3;font-size:12px;">这是本地 RCP Server 身份。保存时会自动附加本机后缀，供 Teleop、MCP 和设备发现使用。</div>
        </div>
      </div>
      <h3 style="margin:18px 0 12px;color:#00d4ff;">从臂执行端凭据（从臂本体接入 RynnBot 时必填）</h3>
      <div class="tip-box">
        <h4>这套设备代表什么</h4>
        <p>它代表真实 SO101 从臂，负责向云端上传状态和相机，并接收动作。从臂本体接入 RynnBot 时填写；本地 Teleop、动作测试和 MCP 可直接继续下一步。</p>
      </div>
      <div class="grid">
        <div class="form-group">
          <label class="form-label">从臂 App ID（自动生成）</label>
          <input class="form-input" id="device-app-id" readonly>
          <div style="margin-top:7px;color:#98a2b3;font-size:12px;">配置工具自动生成的 RynnBot App 标识。</div>
        </div>
        <div class="form-group">
          <label class="form-label">从臂 Product Key</label>
          <input class="form-input" id="device-product-key" placeholder="仅从臂本体接入 RynnBot 时填写">
        </div>
        <div class="form-group">
          <label class="form-label">从臂 Device Name</label>
          <input class="form-input" id="device-name" placeholder="仅从臂本体接入 RynnBot 时填写">
        </div>
        <div class="form-group">
          <label class="form-label">从臂 Device Secret</label>
          <input class="form-input" id="device-secret" placeholder="仅从臂本体接入 RynnBot 时填写">
        </div>
        <div class="form-group">
          <label class="form-label">从臂 HTTP URL（通常保持默认）</label>
          <input class="form-input" id="device-http-url" placeholder="https://robot-access.damo-academy.com">
        </div>
        <div class="form-group">
          <label class="form-label">云端图片上传编码 / Image Upload Codec</label>
          <select class="form-input" id="device-image-codec">
            <option value="jpeg">jpeg</option>
            <option value="npy_gzip">npy_gzip</option>
          </select>
        </div>
      </div>
      <h3 style="margin:22px 0 12px;color:#00d4ff;">主臂控制端凭据（主臂接入 RynnBot 时必填）</h3>
      <div class="tip-box">
        <h4>主臂控制端的用途</h4>
        <p>它把真实主臂 Server 注册为 RynnBot controller，用于控制仿真从臂。主臂串口在第 5 步选择，标定在第 6 步完成。</p>
        <p>主臂控制仿真从臂时填写这一组；本地 Teleop 直接使用主臂 Robot ID。</p>
      </div>
      <div class="grid">
        <div class="form-group">
          <label class="form-label">主臂 App ID（自动生成）</label>
          <input class="form-input" id="master-device-app-id" readonly>
          <div style="margin-top:7px;color:#98a2b3;font-size:12px;">配置工具自动生成的主臂 RynnBot App 标识。</div>
        </div>
        <div class="form-group">
          <label class="form-label">主臂 Product Key</label>
          <input class="form-input" id="master-device-product-key" placeholder="主臂接入 RynnBot 时填写">
        </div>
        <div class="form-group">
          <label class="form-label">主臂 Device Name</label>
          <input class="form-input" id="master-device-name" placeholder="主臂接入 RynnBot 时填写">
        </div>
        <div class="form-group">
          <label class="form-label">主臂 Device Secret</label>
          <input class="form-input" id="master-device-secret" placeholder="主臂接入 RynnBot 时填写">
        </div>
        <div class="form-group">
          <label class="form-label">主臂 HTTP URL（通常保持默认）</label>
          <input class="form-input" id="master-device-http-url" placeholder="https://robot-access.damo-academy.com">
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
        <p>单臂绑定 front + wrist；双臂绑定共享 front + left_wrist + right_wrist。保存后会写入当前构型配置。</p>
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
        <div class="mapping-item" id="single-wrist-mapping"><span class="mapping-key">observation.images.wrist</span><span class="mapping-value unset" id="mapping-wrist">(未绑定)</span></div>
        <div class="mapping-item profile-hidden" id="left-wrist-mapping"><span class="mapping-key">observation.images.left_wrist</span><span class="mapping-value unset" id="mapping-left-wrist">(未绑定)</span></div>
        <div class="mapping-item profile-hidden" id="right-wrist-mapping"><span class="mapping-key">observation.images.right_wrist</span><span class="mapping-value unset" id="mapping-right-wrist">(未绑定)</span></div>
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
      <div id="follower-single-serial">
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
      </div>
      <div id="follower-dual-serial" class="profile-hidden"></div>
    </section>

    <section class="config-section" id="section-follower-calib">
      <h2 class="section-title">从臂标定</h2>
      <div id="follower-single-calib">
      <div class="tip-box">
        <h4>SO101 标定流程</h4>
        <p>点击“启动标定”后，后端会启动 RynnRCP SO101 标定进程，并在同一个串口连接内读取各关节 Present_Position。先把所有关节移动到中间位置并点击“记录中间位置”，再点击“开始记录范围”，然后缓慢推动每个关节走完整范围，观察 Min / 当前 / Max 是否持续变化，最后点击“结束并保存”。标定期间请保持 MCP、Teleop 和 RynnBot 处于停止状态。</p>
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
        <button class="btn btn-success" id="follower-calib-range-btn" onclick="calibBeginRange('follower')" disabled>开始记录范围</button>
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
      </div>
      <div id="follower-dual-calib" class="profile-hidden"></div>
    </section>

    <section class="config-section" id="section-leader-serial">
      <h2 class="section-title">主臂串口配置</h2>
      <div id="leader-single-serial">
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
      </div>
      <div id="leader-dual-serial" class="profile-hidden"></div>
    </section>

    <section class="config-section" id="section-leader-calib">
      <h2 class="section-title">主臂标定</h2>
      <div id="leader-single-calib">
      <div class="tip-box">
        <h4>SO101 标定流程</h4>
        <p>点击“启动标定”后，后端会启动 RynnRCP SO101 标定进程，并在同一个串口连接内读取各关节 Present_Position。leader 标定先确认中间位置，再点击“开始记录范围”，推动完整关节范围并点击“结束并保存”；标定时请观察 Min / 当前 / Max 是否覆盖每个关节的真实运动范围。标定期间请保持 Teleop leader 处于停止状态。</p>
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
        <button class="btn btn-success" id="leader-calib-range-btn" onclick="calibBeginRange('leader')" disabled>开始记录范围</button>
        <button class="btn btn-success" id="leader-calib-finish-btn" onclick="calibFinishRange('leader')" disabled>结束并保存</button>
        <button class="btn btn-danger" id="leader-calib-stop-btn" onclick="calibStop('leader')" disabled>停止</button>
        <button class="btn btn-primary" onclick="showSection('motion-test')">进入动作测试</button>
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
      </div>
      <div id="leader-dual-calib" class="profile-hidden"></div>
    </section>

    <section class="config-section" id="section-motion-test">
      <h2 class="section-title">从臂动作测试 / 校准验证</h2>
      <div class="tip-box">
        <h4>真机安全提示</h4>
        <p>进入本页会连接 follower 串口；滑块、单关节正弦和固定轨迹会直接驱动机械臂。操作前请清空周围空间、托住机械臂，并确认 MCP/Teleop/RynnBot 未占用串口；若方向或幅度异常，立即点击“停止动作”。退出配置程序时连接会安全释放。</p>
      </div>
      <div class="form-group profile-hidden" id="motion-target-group" style="max-width:420px;">
        <label class="form-label">双臂预设动作 / Home 操作目标</label>
        <select class="form-input" id="motion-target">
          <option value="both">左右双臂同步</option>
          <option value="left">仅左臂</option>
          <option value="right">仅右臂</option>
        </select>
      </div>
      <div class="status-panel">
        <div class="motion-status-line">实时关节控制（rad，夹爪为 0–1）</div>
        <div id="joint-controls" style="margin-top:12px;color:#98a2b3;">正在连接并读取机械臂状态...</div>
      </div>
      <div class="status-panel">
        <div class="motion-status-line">单关节正弦运动</div>
        <div class="button-group" style="align-items:end;">
          <label>关节<select class="form-input" id="sine-joint"></select></label>
          <label>振幅<input class="form-input" id="sine-amplitude" type="number" value="0.2" min="0.01" max="1" step="0.01"></label>
          <label>频率 Hz<input class="form-input" id="sine-frequency" type="number" value="0.2" min="0.05" max="2" step="0.05"></label>
          <button class="btn btn-primary motion-run-btn" onclick="runSineMotion()">开始正弦</button>
        </div>
      </div>
      <div class="motion-grid">
        <div class="motion-card">
          <div class="motion-card-title">Motion 1</div>
          <div class="motion-card-desc">基础姿态后多关节小幅摆动，夹爪同步开合。</div>
          <button class="btn btn-primary motion-run-btn" onclick="runPresetMotion(1)">运行</button>
        </div>
        <div class="motion-card">
          <div class="motion-card-title">Motion 2</div>
          <div class="motion-card-desc">底座与肘部圆滑联动，用于发现关节方向错误。</div>
          <button class="btn btn-primary motion-run-btn" onclick="runPresetMotion(2)">运行</button>
        </div>
        <div class="motion-card">
          <div class="motion-card-title">Motion 3</div>
          <div class="motion-card-desc">肩、肘、腕组合动作，用于检查多轴协同。</div>
          <button class="btn btn-primary motion-run-btn" onclick="runPresetMotion(3)">运行</button>
        </div>
        <div class="motion-card">
          <div class="motion-card-title">Motion 4</div>
          <div class="motion-card-desc">侧向姿态下腕部摆动，覆盖腕部和夹爪。</div>
          <button class="btn btn-primary motion-run-btn" onclick="runPresetMotion(4)">运行</button>
        </div>
        <div class="motion-card">
          <div class="motion-card-title">Motion 5</div>
          <div class="motion-card-desc">单次移动到标准检查姿态。</div>
          <button class="btn btn-primary motion-run-btn" onclick="runPresetMotion(5)">运行</button>
        </div>
      </div>
      <div class="debug-action-groups">
        <div class="debug-action-group">
          <div class="debug-action-title">遥操控制</div>
          <div class="debug-action-buttons">
            <button class="btn btn-primary" id="teleop-start-btn" onclick="startDebugTeleop()" disabled>开始遥操</button>
            <button class="btn btn-danger" id="teleop-stop-btn" onclick="stopDebugTeleop()" disabled>停止遥操</button>
          </div>
        </div>
        <div class="debug-action-group">
          <div class="debug-action-title">动作控制</div>
          <div class="debug-action-buttons">
            <button class="btn btn-danger" id="motion-stop-btn" onclick="stopPresetMotion()" disabled>停止动作</button>
            <button class="btn btn-primary" id="motion-home-btn" onclick="returnDebugHome()" disabled>回到 Home</button>
          </div>
        </div>
        <div class="debug-action-group">
          <div class="debug-action-title">Home 设置</div>
          <div class="debug-action-buttons">
            <button class="btn btn-primary" id="set-home-btn" onclick="editDebugHome()" disabled>重新设置 Home</button>
            <button class="btn btn-success" id="save-home-btn" onclick="saveDebugHome()" disabled>保存为 Home</button>
          </div>
        </div>
      </div>
      <div class="status-panel">
        <div class="motion-status-line" id="motion-status">当前状态: 未运行</div>
      </div>
      <div class="status-panel">
        <div class="motion-status-line" style="display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap;">
          <span>Action / State 跟随图</span>
          <button class="btn btn-primary" id="download-tracking-btn" style="padding:8px 14px;font-size:12px;" onclick="downloadTrackingImage()" disabled>⇩ 导出跟随图</button>
        </div>
        <div class="tracking-legend"><span class="action">action</span><span class="state">state</span></div>
        <div class="tracking-grid" id="motion-tracking-grid">
          <div style="color:#98a2b3;padding:16px;">操作滑块或运行动作后显示跟随曲线</div>
        </div>
      </div>
      <div class="log-container" id="motion-test-log"></div>
    </section>
  </div>

  <div id="toast" class="toast"></div>

  <script>
    let currentConfig = null;
    let currentProfile = "single";
    let cameraMapping = { front: null, wrist: null, left_wrist: null, right_wrist: null };
    let serialBaseline = { follower: [], leader: [], left_follower: [], right_follower: [], left_leader: [], right_leader: [] };
    let selectedPorts = { follower: null, leader: null, left_follower: null, right_follower: null, left_leader: null, right_leader: null };
    let calibrationPhases = { follower: "idle", leader: "idle", left_follower: "idle", right_follower: "idle", left_leader: "idle", right_leader: "idle" };
    let presetMotionPollTimer = null;
    let latestPresetMotionTrace = [];
    let latestPresetMotionId = null;
    let debugSessionStarted = false;
    const jointSendTimers = {};
    const pendingJointValues = {};
    const calibrationJoints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"];
    let motionJointNames = [...calibrationJoints];

    const ids = {
      robotId: "robot-id",
      appId: "device-app-id",
      productKey: "device-product-key",
      deviceName: "device-name",
      deviceSecret: "device-secret",
      httpUrl: "device-http-url",
      imageCodec: "device-image-codec",
      masterProductKey: "master-device-product-key",
      masterAppId: "master-device-app-id",
      masterDeviceName: "master-device-name",
      masterDeviceSecret: "master-device-secret",
      masterHttpUrl: "master-device-http-url"
    };

    function $(id) { return document.getElementById(id); }

    function activeSerialDevices() {
      return currentProfile === "dual"
        ? ["left_follower", "right_follower", "left_leader", "right_leader"]
        : ["follower", "leader"];
    }

    function deviceLabel(device) {
      return ({
        follower: "从臂", leader: "主臂",
        left_follower: "左从臂", right_follower: "右从臂",
        left_leader: "左主臂", right_leader: "右主臂"
      })[device] || device;
    }

    async function changeProfile(profile) {
      currentProfile = profile === "dual" ? "dual" : "single";
      debugSessionStarted = false;
      try {
        await loadConfig(currentProfile);
        showToast("已切换到" + (currentProfile === "dual" ? "双臂 12 DoF" : "单臂 6 DoF") + "配置", "success");
      } catch (error) {
        showToast("切换构型失败: " + error.message, "error", 6000);
      }
    }

    function updateProfileUi() {
      const dual = currentProfile === "dual";
      ["follower", "leader"].forEach(role => {
        $(role + "-single-serial").classList.toggle("profile-hidden", dual);
        $(role + "-dual-serial").classList.toggle("profile-hidden", !dual);
        $(role + "-single-calib").classList.toggle("profile-hidden", dual);
        $(role + "-dual-calib").classList.toggle("profile-hidden", !dual);
      });
      $("single-wrist-mapping").classList.toggle("profile-hidden", dual);
      $("left-wrist-mapping").classList.toggle("profile-hidden", !dual);
      $("right-wrist-mapping").classList.toggle("profile-hidden", !dual);
      $("motion-target-group").classList.toggle("profile-hidden", !dual);
      $("tab-follower-serial").textContent = dual ? "3. 左右从臂串口" : "3. 从臂串口";
      $("tab-follower-calib").textContent = dual ? "4. 左右从臂标定" : "4. 从臂标定";
      $("tab-leader-serial").textContent = dual ? "5. 左右主臂串口" : "5. 主臂串口";
      $("tab-leader-calib").textContent = dual ? "6. 左右主臂标定" : "6. 主臂标定";
      if (dual && !$("left_follower-serial-list")) renderDualHardwarePanels();
    }

    function renderDualHardwarePanels() {
      $("follower-dual-serial").innerHTML = ["left_follower", "right_follower"].map(serialPanelHtml).join("")
        + '<div class="button-group"><button class="btn btn-primary" onclick="showSection(\'follower-calib\')">下一步</button></div>';
      $("leader-dual-serial").innerHTML = ["left_leader", "right_leader"].map(serialPanelHtml).join("")
        + '<div class="button-group"><button class="btn btn-primary" onclick="showSection(\'leader-calib\')">下一步</button></div>';
      $("follower-dual-calib").innerHTML = [
        calibrationPanelHtml("left_follower"), calibrationPanelHtml("right_follower")
      ].join("") + '<div class="button-group"><button class="btn btn-primary" onclick="showSection(\'leader-serial\')">下一步</button></div>';
      $("leader-dual-calib").innerHTML = [
        calibrationPanelHtml("left_leader"), calibrationPanelHtml("right_leader")
      ].join("") + '<div class="button-group"><button class="btn btn-primary" onclick="showSection(\'motion-test\')">进入动作测试</button></div>';
    }

    function serialPanelHtml(device) {
      const label = deviceLabel(device);
      return `<div class="subpanel">
        <h3>${label}串口</h3>
        <div class="button-group">
          <button class="btn btn-warning" onclick="recordSerialBaseline('${device}')">记录基线</button>
          <button class="btn btn-primary" onclick="detectNewSerial('${device}')">检测新增</button>
          <button class="btn btn-plain" onclick="scanSerialOnly('${device}')">刷新列表</button>
        </div>
        <div id="${device}-serial-status" class="status-panel">当前状态: 等待操作...</div>
        <div class="serial-list" id="${device}-serial-list"><div style="color:#98a2b3;text-align:center;padding:20px;">串口列表将在检测后显示</div></div>
        <div class="mapping-display"><div class="mapping-item"><span class="mapping-key">已选择的${label}串口</span><span class="mapping-value unset" id="${device}-selected-port">(未选择)</span></div></div>
        <div class="button-group"><button class="btn btn-success" onclick="saveSerial('${device}', '${device.includes("follower") ? "follower-serial" : "leader-serial"}')" id="btn-save-${device}-serial" disabled>保存${label}串口</button></div>
      </div>`;
    }

    function calibrationPanelHtml(device) {
      const label = deviceLabel(device);
      const steps = ["启动标定进程", "记录中间位置", "记录完整范围", "保存完成"];
      return `<div class="subpanel">
        <h3>${label}标定</h3>
        <div class="tip-box"><p>${label}独立使用自己的串口和 calibration ID；标定期间请保持动作测试、MCP、Teleop 和 RynnBot 处于停止状态。</p></div>
        <div class="calib-steps">${steps.map((title, index) => `<div class="calib-step" id="${device}-calib-step-${index + 1}"><div class="calib-step-number">${index + 1}</div><div><div class="calib-step-title">${title}</div></div></div>`).join("")}</div>
        <div class="button-group">
          <button class="btn btn-primary" id="${device}-calib-start-btn" onclick="calibStart('${device}')">启动标定</button>
          <button class="btn btn-warning" id="${device}-calib-middle-btn" onclick="calibRecordMiddle('${device}')" disabled>记录中间位置</button>
          <button class="btn btn-success" id="${device}-calib-range-btn" onclick="calibBeginRange('${device}')" disabled>开始记录范围</button>
          <button class="btn btn-success" id="${device}-calib-finish-btn" onclick="calibFinishRange('${device}')" disabled>结束并保存</button>
          <button class="btn btn-danger" id="${device}-calib-stop-btn" onclick="calibStop('${device}')" disabled>停止</button>
        </div>
        <div class="status-panel"><span style="color:#98a2b3;">当前阶段:</span><span id="${device}-calib-phase" style="color:#00d4ff;font-weight:700;margin-left:8px;">未启动</span><div class="calib-hint" id="${device}-calib-hint">启动后先记录中间位置，再推动全部关节完成 Min / Max 记录。</div></div>
        <div class="status-panel" id="${device}-range-panel" style="display:none;"><div class="mapping-key">关节 Min / 当前 / Max</div><div id="${device}-range-table"></div></div>
        <div class="log-container" id="${device}-calib-log"></div>
      </div>`;
    }

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
            <p>配置完成后可以直接关闭这个标签页。</p>
          </div>
        </main>`;
      setTimeout(() => window.close(), 150);
    }

    function showSection(name) {
      document.querySelectorAll(".config-section").forEach(s => s.classList.remove("active"));
      document.querySelectorAll(".nav-tab").forEach(t => t.classList.remove("active"));
      $("section-" + name).classList.add("active");
      $("tab-" + name).classList.add("active");
      if (name === "motion-test") startDebugSession();
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

    async function loadConfig(profile = currentProfile) {
      currentProfile = profile === "dual" ? "dual" : "single";
      const data = await api("/api/config?profile=" + encodeURIComponent(currentProfile));
      currentConfig = data.config;
      fillConfig(currentConfig);
      showToast("配置已加载", "success", 1600);
    }

    function fillConfig(config) {
      currentProfile = config.profile === "dual" ? "dual" : "single";
      $("robot-profile").value = currentProfile;
      updateProfileUi();
      $(ids.robotId).value = config.server.id || "";
      $(ids.appId).value = config.rynnbot.app_id || "";
      $(ids.productKey).value = config.rynnbot.product_key || "";
      $(ids.deviceName).value = config.rynnbot.device_name || "";
      $(ids.deviceSecret).value = config.rynnbot.device_secret || "";
      $(ids.httpUrl).value = config.rynnbot.http_url || "";
      $(ids.imageCodec).value = config.rynnbot.image_upload_codec || "jpeg";
      const masterRynnbot = config.master_rynnbot || {};
      $(ids.masterAppId).value = masterRynnbot.app_id || "";
      $(ids.masterProductKey).value = masterRynnbot.product_key || "";
      $(ids.masterDeviceName).value = masterRynnbot.device_name || "";
      $(ids.masterDeviceSecret).value = masterRynnbot.device_secret || "";
      $(ids.masterHttpUrl).value = masterRynnbot.http_url || "";
      cameraMapping.front = Number(config.hardware.front_camera ?? 0);
      if (currentProfile === "dual") {
        cameraMapping.left_wrist = Number(config.hardware.left_wrist_camera ?? 1);
        cameraMapping.right_wrist = Number(config.hardware.right_wrist_camera ?? 2);
        selectedPorts.left_follower = config.hardware.left_follower_port || null;
        selectedPorts.right_follower = config.hardware.right_follower_port || null;
        selectedPorts.left_leader = config.hardware.left_leader_port || null;
        selectedPorts.right_leader = config.hardware.right_leader_port || null;
      } else {
        cameraMapping.wrist = Number(config.hardware.wrist_camera ?? 1);
        selectedPorts.follower = config.hardware.follower_port || null;
        selectedPorts.leader = config.hardware.leader_port || null;
      }
      updateCameraMappingDisplay();
      activeSerialDevices().forEach(updateSelectedPort);
    }

    async function postConfig(partial) {
      const data = await api("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ...partial, profile: currentProfile })
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
            app_id: $(ids.appId).value.trim(),
            product_key: $(ids.productKey).value.trim(),
            device_name: $(ids.deviceName).value.trim(),
            device_secret: $(ids.deviceSecret).value.trim(),
            http_url: $(ids.httpUrl).value.trim(),
            image_upload_codec: $(ids.imageCodec).value
          },
          master_rynnbot: {
            app_id: $(ids.masterAppId).value.trim(),
            product_key: $(ids.masterProductKey).value.trim(),
            device_name: $(ids.masterDeviceName).value.trim(),
            device_secret: $(ids.masterDeviceSecret).value.trim(),
            http_url: $(ids.masterHttpUrl).value.trim()
          }
        });
        markTabCompleted("device");
        showToast("设置已保存；本地模式可直接使用 Robot ID", "success", 3500);
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
              ${currentProfile === "dual"
                ? `<button class="camera-btn wrist" onclick="bindCamera(${cam.index}, 'left_wrist')">绑定左腕</button><button class="camera-btn wrist" onclick="bindCamera(${cam.index}, 'right_wrist')">绑定右腕</button>`
                : `<button class="camera-btn wrist" onclick="bindCamera(${cam.index}, 'wrist')">绑定腕部</button>`}
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
      setMappingValue("mapping-left-wrist", cameraMapping.left_wrist, "Camera ");
      setMappingValue("mapping-right-wrist", cameraMapping.right_wrist, "Camera ");
      $("btn-save-camera").disabled = currentProfile === "dual"
        ? cameraMapping.front === null || cameraMapping.left_wrist === null || cameraMapping.right_wrist === null
        : cameraMapping.front === null || cameraMapping.wrist === null;
    }

    function updateCameraCardHighlight() {
      document.querySelectorAll(".camera-card").forEach(c => c.classList.remove("selected"));
      const selected = currentProfile === "dual"
        ? [cameraMapping.front, cameraMapping.left_wrist, cameraMapping.right_wrist]
        : [cameraMapping.front, cameraMapping.wrist];
      selected.forEach(index => {
        const card = $("cam-card-" + index);
        if (card) card.classList.add("selected");
      });
    }

    function clearCameraMapping() {
      cameraMapping = { front: null, wrist: null, left_wrist: null, right_wrist: null };
      updateCameraMappingDisplay();
      updateCameraCardHighlight();
      showToast("已清除相机绑定", "info");
    }

    async function saveCameraSettings() {
      try {
        const hardware = currentProfile === "dual"
          ? { front_camera: Number(cameraMapping.front), left_wrist_camera: Number(cameraMapping.left_wrist), right_wrist_camera: Number(cameraMapping.right_wrist) }
          : { front_camera: Number(cameraMapping.front), wrist_camera: Number(cameraMapping.wrist) };
        await postConfig({ hardware });
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
      const button = $("btn-save-" + arm + "-serial");
      if (button) button.disabled = !value;
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
        if (currentProfile === "dual") {
          activeSerialDevices().forEach(device => {
            if (selectedPorts[device]) hardware[device + "_port"] = selectedPorts[device];
          });
        } else {
          hardware[arm + "_port"] = selectedPorts[arm];
        }
        await postConfig({ hardware });
        markTabCompleted(tabName);
        showToast(deviceLabel(arm) + "串口已保存", "success");
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
          body: JSON.stringify({ arm, profile: currentProfile })
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

    async function calibBeginRange(arm) {
      if (calibrationPhases[arm] !== "range_prompt") {
        showToast("请先记录中间位置，并等待进入“开始记录范围”阶段。", "info", 5000);
        return;
      }
      await calibSendEnter(arm, "range recording started", 3);
    }

    async function calibFinishRange(arm) {
      if (calibrationPhases[arm] !== "recording_range") {
        showToast("请先点击“开始记录范围”，完成关节范围移动后再保存。", "info", 5000);
        return;
      }
      await calibSendEnter(arm, "range motion captured", 4);
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
          markTabCompleted(arm.includes("follower") ? "follower-calib" : "leader-calib");
          showToast(deviceLabel(arm) + "标定完成", "success", 5000);
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
        range_prompt: "等待开始记录范围",
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
      if (phase === "range_prompt") completeCalibSteps(arm, 2);
      if (phase === "recording_range") completeCalibSteps(arm, 3);
      if (phase === "saving" || phase === "saved") completeCalibSteps(arm, 3);
      if (!running && returncode === 0) completeCalibSteps(arm, 4);
    }

    function updateCalibrationControls(arm, phase, running, returncode) {
      const startBtn = $(arm + "-calib-start-btn");
      const middleBtn = $(arm + "-calib-middle-btn");
      const rangeBtn = $(arm + "-calib-range-btn");
      const finishBtn = $(arm + "-calib-finish-btn");
      const stopBtn = $(arm + "-calib-stop-btn");
      const hint = $(arm + "-calib-hint");
      const readyForMiddle = phase === "middle_position";
      const readyForRange = phase === "range_prompt";
      const readyForFinish = phase === "recording_range";
      if (startBtn) startBtn.disabled = !!running;
      if (middleBtn) middleBtn.disabled = !readyForMiddle;
      if (rangeBtn) rangeBtn.disabled = !readyForRange;
      if (finishBtn) finishBtn.disabled = !readyForFinish;
      if (stopBtn) stopBtn.disabled = !running;
      if (!hint) return;
      hint.classList.toggle("ready", readyForMiddle || readyForRange || readyForFinish || phase === "saved");
      if (phase === "starting" || phase === "connecting" || phase === "existing_cache_prompt") {
        hint.textContent = "标定进程正在启动并读取第一帧关节数据，CPU 较慢时可能需要几秒。请等待按钮变亮后再记录中间位置。";
      } else if (readyForMiddle) {
        hint.textContent = "已准备好。请把所有关节放在运动范围中间，然后点击“记录中间位置”。";
      } else if (readyForRange) {
        hint.textContent = "中间位置已记录。请点击“开始记录范围”，再缓慢推动每个关节走完整范围。";
      } else if (readyForFinish) {
        hint.textContent = "正在记录关节范围。请缓慢推动每个关节走完整范围，完成后点击“结束并保存”。";
      } else if (phase === "saving") {
        hint.textContent = "正在保存标定文件，请等待完成。";
      } else if (phase === "saved" || (!running && returncode === 0)) {
        hint.textContent = "标定已保存。";
      } else if (phase === "failed") {
        hint.textContent = "标定失败，请查看下方日志并确认串口未被其他程序占用。";
      } else {
        hint.textContent = "点击“启动标定”后，按“记录中间位置”“开始记录范围”“结束并保存”的顺序完成标定。";
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
          等待第一帧关节数据。如果数值长时间保持空白，请确认 ${deviceLabel(arm)}串口正确，并保持 Teleop、MCP 和 RynnBot 处于停止状态，然后等待日志进入中间位置阶段。
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

    async function runPresetMotion(motion) {
      clearLog("motion-test-log");
      addLog("motion-test-log", "准备运行 Motion " + motion + "。", "info");
      setPresetMotionBusy(true);
      try {
        const data = await api("/api/preset_motion/start", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ motion, profile: currentProfile, target: currentProfile === "dual" ? $("motion-target").value : "both" })
        });
        renderPresetMotionStatus(data.status);
        pollPresetMotion();
      } catch (error) {
        setPresetMotionBusy(false);
        addLog("motion-test-log", "启动失败: " + error.message, "error");
        showToast("动作测试启动失败: " + error.message, "error", 6000);
      }
    }

    async function startDebugSession() {
      if (debugSessionStarted) return;
      debugSessionStarted = true;
      try {
        const data = await api("/api/preset_motion/connect", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ profile: currentProfile })
        });
        renderPresetMotionStatus(data.status);
        pollPresetMotion();
      } catch (error) {
        debugSessionStarted = false;
        $("joint-controls").textContent = "连接失败: " + error.message;
      }
    }

    function queueDebugJoint(index, value) {
      pendingJointValues[index] = value;
      if (jointSendTimers[index]) return;
      jointSendTimers[index] = setTimeout(() => {
        jointSendTimers[index] = null;
        setDebugJoint(index, pendingJointValues[index]);
      }, 1000 / 30);
    }

    async function setDebugJoint(index, value) {
      try {
        const data = await api("/api/preset_motion/joint", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ index, value: Number(value), profile: currentProfile })
        });
        renderPresetMotionStatus(data.status);
      } catch (error) { showToast("关节控制失败: " + error.message, "error", 6000); }
    }

    async function runSineMotion() {
      try {
        latestPresetMotionId = null;
        const data = await api("/api/preset_motion/sine", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ joint: Number($("sine-joint").value), amplitude: Number($("sine-amplitude").value), frequency: Number($("sine-frequency").value), profile: currentProfile })
        });
        renderPresetMotionStatus(data.status);
        pollPresetMotion();
      } catch (error) { showToast("正弦运动启动失败: " + error.message, "error", 6000); }
    }

    function renderJointControls(status) {
      const target = $("joint-controls");
      if (!target || !status.connected || !Array.isArray(status.positions)) return;
      if (target.contains(document.activeElement)) return;
      const limits = status.limits || [];
      motionJointNames = Array.isArray(status.joint_names) && status.joint_names.length ? status.joint_names : [...calibrationJoints];
      target.innerHTML = motionJointNames.map((name, index) => {
        const value = Number(status.positions[index]);
        const limit = limits[index] || [-3.1416, 3.1416];
        return `<label class="joint-control"><span>${escapeHtml(name)}</span><input type="range" min="${limit[0]}" max="${limit[1]}" step="0.01" value="${value}" ${status.home_editing || status.teleop_running ? "disabled" : ""} oninput="this.nextElementSibling.textContent=Number(this.value).toFixed(3);queueDebugJoint(${index},this.value)"><span class="joint-value">${value.toFixed(3)}</span></label>`;
      }).join("");
      const sine = $("sine-joint");
      if (sine && sine.options.length !== motionJointNames.length) sine.innerHTML = motionJointNames.map((name, i) => `<option value="${i}">${escapeHtml(name)}</option>`).join("");
    }

    async function stopPresetMotion() {
      try {
        const data = await api("/api/preset_motion/stop", { method: "POST" });
        renderPresetMotionStatus(data.status);
        showToast("动作已停止，请确认周围安全后点击“回到 Home”", "info", 8000);
      } catch (error) {
        showToast("停止失败: " + error.message, "error", 6000);
      }
    }

    async function returnDebugHome() {
      const button = $("motion-home-btn");
      button.disabled = true;
      try {
        const data = await api("/api/preset_motion/home", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ profile: currentProfile, target: currentProfile === "dual" ? $("motion-target").value : "both" })
        });
        renderPresetMotionStatus(data.status);
        showToast("机械臂已回到 Home", "success");
      } catch (error) {
        button.disabled = false;
        showToast("回 Home 失败: " + error.message, "error", 6000);
      }
    }

    async function startDebugTeleop() {
      try {
        const data = await api("/api/preset_motion/teleop/start", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ profile: currentProfile })
        });
        renderPresetMotionStatus(data.status);
        showToast("遥操已启动：主臂正在控制从臂", "success");
      } catch (error) { showToast("遥操启动失败: " + error.message, "error", 6000); }
    }

    async function stopDebugTeleop() {
      try {
        const data = await api("/api/preset_motion/teleop/stop", { method: "POST" });
        renderPresetMotionStatus(data.status);
        showToast("遥操已停止", "info");
      } catch (error) { showToast("遥操停止失败: " + error.message, "error", 6000); }
    }

    async function editDebugHome() {
      try {
        const data = await api("/api/preset_motion/home/edit", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ profile: currentProfile, target: currentProfile === "dual" ? $("motion-target").value : "both" })
        });
        renderPresetMotionStatus(data.status);
        showToast("关节已松开，请移动到新的 Home 后点击“保存为 Home”", "info", 8000);
      } catch (error) { showToast("设置 Home 失败: " + error.message, "error", 6000); }
    }

    async function saveDebugHome() {
      try {
        const data = await api("/api/preset_motion/home/save", { method: "POST" });
        renderPresetMotionStatus(data.status);
        showToast("当前位置已保存为新的 Home", "success");
      } catch (error) { showToast("保存 Home 失败: " + error.message, "error", 6000); }
    }

    async function pollPresetMotion() {
      if (presetMotionPollTimer) {
        clearTimeout(presetMotionPollTimer);
        presetMotionPollTimer = null;
      }
      try {
        const data = await api("/api/preset_motion/status");
        renderPresetMotionStatus(data.status);
        if (data.status && (data.status.running || data.status.connected)) presetMotionPollTimer = setTimeout(pollPresetMotion, 500);
      } catch (error) {
        addLog("motion-test-log", "状态刷新失败: " + error.message, "error");
        setPresetMotionBusy(false);
      }
    }

    function renderPresetMotionStatus(status) {
      if (!status) return;
      if (status.motion) latestPresetMotionId = status.motion;
      const motionText = status.motion ? " Motion " + status.motion : "";
      let text = "当前状态: ";
      if (status.returning) {
        text += "正在安全回 home" + motionText;
      } else if (status.teleop_running) {
        text += currentProfile === "dual" ? "遥操中（左主臂→左从臂，右主臂→右从臂）" : "遥操中（主臂 → 从臂）";
      } else if (status.running) {
        text += "正在运行" + motionText;
      } else if (status.error) {
        text += "失败" + motionText + " - " + status.error;
      } else if (status.completed) {
        text += "已完成" + motionText;
      } else if (status.stopped) {
        text += "已停止" + motionText;
      } else {
        text += "未运行";
      }
      $("motion-status").textContent = text;
      setPresetMotionBusy(!!status.running);
      if (status.home_editing) {
        document.querySelectorAll(".motion-run-btn").forEach(btn => { btn.disabled = true; });
        $("motion-stop-btn").disabled = true;
      }
      $("motion-home-btn").disabled = !status.connected || !!status.returning || !!status.home_editing;
      $("set-home-btn").disabled = !status.connected || !!status.running || !!status.teleop_running || !!status.home_editing;
      $("save-home-btn").disabled = !status.home_editing;
      $("teleop-start-btn").disabled = !status.connected || !!status.running || !!status.teleop_running || !!status.home_editing;
      $("teleop-stop-btn").disabled = !status.teleop_running;
      if (status.teleop_running) document.querySelectorAll(".motion-run-btn").forEach(btn => { btn.disabled = true; });
      renderTrackingChart(status.trace || []);
      renderJointControls(status);

      const log = $("motion-test-log");
      log.innerHTML = "";
      (status.logs || []).forEach(item => {
        const line = document.createElement("div");
        line.className = "log-line " + (item.type || "info");
        line.textContent = "[" + (item.time || "--:--:--") + "] " + item.text;
        log.appendChild(line);
      });
      log.scrollTop = log.scrollHeight;
    }

    function renderTrackingChart(trace) {
      const grid = $("motion-tracking-grid");
      if (!grid) return;
      const samples = Array.isArray(trace) ? trace.filter(item => item && Array.isArray(item.action) && Array.isArray(item.state)) : [];
      latestPresetMotionTrace = samples;
      const downloadBtn = $("download-tracking-btn");
      if (downloadBtn) downloadBtn.disabled = samples.length === 0;
      if (!samples.length) {
        grid.innerHTML = '<div style="color:#98a2b3;padding:16px;">等待 action / state 采样...</div>';
        return;
      }
      grid.innerHTML = motionJointNames.map((joint, jointIndex) => {
        const points = samples.filter(item => item.action.length > jointIndex && item.state.length > jointIndex);
        if (!points.length) {
          return `<div class="tracking-card"><div class="tracking-title"><span>${escapeHtml(joint)}</span><span>--</span></div></div>`;
        }
        const actionValues = points.map(item => Number(item.action[jointIndex]));
        const stateValues = points.map(item => Number(item.state[jointIndex]));
        let minValue = Math.min(...actionValues, ...stateValues);
        let maxValue = Math.max(...actionValues, ...stateValues);
        if (!Number.isFinite(minValue) || !Number.isFinite(maxValue)) {
          minValue = 0;
          maxValue = 1;
        }
        [minValue, maxValue] = trackingYRange(minValue, maxValue, jointIndex);
        const latest = points[points.length - 1];
        const latestAction = Number(latest.action[jointIndex]);
        const latestState = Number(latest.state[jointIndex]);
        const latestError = Math.abs(latestAction - latestState);
        const actionPolyline = trackingPolyline(actionValues, minValue, maxValue);
        const statePolyline = trackingPolyline(stateValues, minValue, maxValue);
        const duration = Math.max(0, Number(points[points.length - 1].t || 0) - Number(points[0].t || 0));
        return `<div class="tracking-card">
          <div class="tracking-title"><span>${escapeHtml(joint)}</span><span>误差 ${latestError.toFixed(3)}${jointIndex % 6 === 5 ? "" : " rad"}</span></div>
          <svg viewBox="0 0 220 112" preserveAspectRatio="none" aria-label="${escapeHtml(joint)} tracking">
            <line x1="36" y1="8" x2="36" y2="88" stroke="#64748b" stroke-width="1"/>
            <line x1="36" y1="88" x2="212" y2="88" stroke="#64748b" stroke-width="1"/>
            <text x="2" y="12" fill="#94a3b8" font-size="9">${maxValue.toFixed(2)}</text>
            <text x="2" y="90" fill="#94a3b8" font-size="9">${minValue.toFixed(2)}</text>
            <text x="36" y="104" fill="#94a3b8" font-size="9">0s</text>
            <text x="188" y="104" fill="#94a3b8" font-size="9">${duration.toFixed(1)}s</text>
            <polyline points="${actionPolyline}" fill="none" stroke="#00d4ff" stroke-width="2"/>
            <polyline points="${statePolyline}" fill="none" stroke="#00ff88" stroke-width="2"/>
          </svg>
        </div>`;
      }).join("");
    }

    function downloadTrackingImage() {
      const samples = latestPresetMotionTrace || [];
      if (!samples.length) {
        showToast("暂无跟随数据可下载", "info");
        return;
      }
      const svg = buildTrackingSvg(samples);
      const svgBlob = new Blob([svg], { type: "image/svg+xml;charset=utf-8" });
      const url = URL.createObjectURL(svgBlob);
      const image = new Image();
      image.onload = () => {
        const canvas = document.createElement("canvas");
        canvas.width = image.naturalWidth || 1200;
        canvas.height = image.naturalHeight || 900;
        const ctx = canvas.getContext("2d");
        ctx.fillStyle = "#111827";
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.drawImage(image, 0, 0);
        canvas.toBlob(blob => {
          URL.revokeObjectURL(url);
          if (blob) {
            downloadBlob(blob, trackingFilename("png"));
          } else {
            downloadBlob(svgBlob, trackingFilename("svg"));
          }
        }, "image/png");
      };
      image.onerror = () => {
        URL.revokeObjectURL(url);
        downloadBlob(svgBlob, trackingFilename("svg"));
      };
      image.src = url;
    }

    function buildTrackingSvg(samples) {
      const width = 1200;
      const cardW = 548;
      const cardH = 220;
      const margin = 32;
      const gap = 24;
      const height = 120 + Math.ceil(motionJointNames.length / 2) * (cardH + gap) + margin;
      const chartW = cardW - 74;
      const chartH = 132;
      const title = "SO101 Action / State Tracking" + (latestPresetMotionId ? " - Motion " + latestPresetMotionId : "");
      const parts = [
        `<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}">`,
        `<rect width="${width}" height="${height}" fill="#111827"/>`,
        `<text x="${margin}" y="48" fill="#dbeafe" font-size="30" font-family="Arial, sans-serif" font-weight="700">${escapeSvg(title)}</text>`,
        `<text x="${margin}" y="78" fill="#98a2b3" font-size="15" font-family="Arial, sans-serif">blue: action target, green: state feedback</text>`,
        `<line x1="${margin}" y1="96" x2="${width - margin}" y2="96" stroke="#334155" stroke-width="1"/>`
      ];
      motionJointNames.forEach((joint, jointIndex) => {
        const col = jointIndex % 2;
        const row = Math.floor(jointIndex / 2);
        const x = margin + col * (cardW + gap);
        const y = 120 + row * (cardH + gap);
        const points = samples.filter(item => item.action.length > jointIndex && item.state.length > jointIndex);
        const actionValues = points.map(item => Number(item.action[jointIndex]));
        const stateValues = points.map(item => Number(item.state[jointIndex]));
        let minValue = Math.min(...actionValues, ...stateValues);
        let maxValue = Math.max(...actionValues, ...stateValues);
        if (!Number.isFinite(minValue) || !Number.isFinite(maxValue)) {
          minValue = 0;
          maxValue = 1;
        }
        [minValue, maxValue] = trackingYRange(minValue, maxValue, jointIndex);
        const latest = points[points.length - 1] || { action: [], state: [] };
        const latestError = Math.abs(Number(latest.action[jointIndex] || 0) - Number(latest.state[jointIndex] || 0));
        const duration = points.length ? Math.max(0, Number(points[points.length - 1].t || 0) - Number(points[0].t || 0)) : 0;
        parts.push(`<rect x="${x}" y="${y}" width="${cardW}" height="${cardH}" rx="8" fill="#202033" stroke="#38384d"/>`);
        parts.push(`<text x="${x + 18}" y="${y + 30}" fill="#dbeafe" font-size="17" font-family="Arial, sans-serif" font-weight="700">${escapeSvg(joint)}</text>`);
        parts.push(`<text x="${x + cardW - 170}" y="${y + 30}" fill="#ffd166" font-size="15" font-family="monospace">误差 ${latestError.toFixed(3)}${jointIndex % 6 === 5 ? "" : " rad"}</text>`);
        parts.push(`<rect x="${x + 56}" y="${y + 52}" width="${chartW}" height="${chartH}" rx="6" fill="#111827"/>`);
        parts.push(`<line x1="${x + 56}" y1="${y + 52}" x2="${x + 56}" y2="${y + 52 + chartH}" stroke="#64748b"/>`);
        parts.push(`<line x1="${x + 56}" y1="${y + 52 + chartH}" x2="${x + 56 + chartW}" y2="${y + 52 + chartH}" stroke="#64748b"/>`);
        parts.push(`<text x="${x + 8}" y="${y + 64}" fill="#94a3b8" font-size="12" font-family="monospace">${maxValue.toFixed(2)}</text>`);
        parts.push(`<text x="${x + 8}" y="${y + 52 + chartH}" fill="#94a3b8" font-size="12" font-family="monospace">${minValue.toFixed(2)}</text>`);
        parts.push(`<text x="${x + 56}" y="${y + 52 + chartH + 16}" fill="#94a3b8" font-size="11">0s</text>`);
        parts.push(`<text x="${x + 56 + chartW - 36}" y="${y + 52 + chartH + 16}" fill="#94a3b8" font-size="11">${duration.toFixed(1)}s</text>`);
        parts.push(`<polyline points="${trackingPolylineSvg(actionValues, minValue, maxValue, x + 56, y + 52, chartW, chartH)}" fill="none" stroke="#00d4ff" stroke-width="3"/>`);
        parts.push(`<polyline points="${trackingPolylineSvg(stateValues, minValue, maxValue, x + 56, y + 52, chartW, chartH)}" fill="none" stroke="#00ff88" stroke-width="3"/>`);
      });
      parts.push("</svg>");
      return parts.join("");
    }

    function trackingPolylineSvg(values, minValue, maxValue, x, y, width, height) {
      if (!values.length) return "";
      const pad = 10;
      const span = Math.max(1e-6, maxValue - minValue);
      const count = Math.max(1, values.length - 1);
      return values.map((value, index) => {
        const px = x + pad + (index / count) * (width - pad * 2);
        const py = y + height - pad - ((Number(value) - minValue) / span) * (height - pad * 2);
        return px.toFixed(1) + "," + py.toFixed(1);
      }).join(" ");
    }

    function trackingYRange(minValue, maxValue, jointIndex) {
      if (!Number.isFinite(minValue) || !Number.isFinite(maxValue)) return [0, jointIndex % 6 === 5 ? 1 : 0.2];
      const minimumSpan = jointIndex % 6 === 5 ? 0.1 : 0.2;
      const span = maxValue - minValue;
      if (span >= minimumSpan) return [minValue, maxValue];
      const center = (minValue + maxValue) / 2;
      return [center - minimumSpan / 2, center + minimumSpan / 2];
    }

    function trackingFilename(ext) {
      const stamp = new Date().toISOString().replace(/[:.]/g, "-");
      const motion = latestPresetMotionId ? "-motion-" + latestPresetMotionId : "";
      return "so101-action-state-tracking" + motion + "-" + stamp + "." + ext;
    }

    function downloadBlob(blob, filename) {
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = filename;
      document.body.appendChild(link);
      link.click();
      link.remove();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    }

    function escapeSvg(value) {
      return String(value).replace(/[&<>"']/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&apos;" }[c]));
    }

    function trackingPolyline(values, minValue, maxValue) {
      const width = 220;
      const left = 36;
      const right = 8;
      const top = 8;
      const bottom = 24;
      const height = 112;
      const span = Math.max(1e-6, maxValue - minValue);
      const count = Math.max(1, values.length - 1);
      return values.map((value, index) => {
        const x = left + (index / count) * (width - left - right);
        const y = height - bottom - ((Number(value) - minValue) / span) * (height - top - bottom);
        return x.toFixed(1) + "," + y.toFixed(1);
      }).join(" ");
    }

    function setPresetMotionBusy(running) {
      document.querySelectorAll(".motion-run-btn").forEach(btn => {
        btn.disabled = !!running;
      });
      $("motion-stop-btn").disabled = !running;
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


CALIBRATION_DEVICES = {
    "follower",
    "leader",
    "left_follower",
    "right_follower",
    "left_leader",
    "right_leader",
}


def _normalize_calibration_device(device: str) -> str:
    normalized = str(device).strip().lower().replace("-", "_")
    if normalized not in CALIBRATION_DEVICES:
        raise ValueError(f"unknown SO101 calibration device: {device}")
    return normalized


def _device_role_side(device: str) -> tuple[str, str | None]:
    normalized = _normalize_calibration_device(device)
    if normalized in {"follower", "leader"}:
        return normalized, None
    side, role = normalized.split("_", 1)
    return role, side


class CalibrationJob:
    """Run one SO101 calibration process and expose browser-friendly controls."""

    def __init__(self, arm: str) -> None:
        self.arm = _normalize_calibration_device(arm)
        self._process: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._logs: list[Dict[str, str]] = []
        self._command: list[str] = []
        self._phase = "idle"
        self._ranges: dict[str, Dict[str, int]] = {}
        self._forced_recalibration = False
        self._lock = threading.RLock()

    def start(self, profile: str = "single") -> Dict[str, Any]:
        with self._lock:
            if self.is_running():
                raise RuntimeError(f"{self.arm} calibration is already running")
            profile = _validate_profile(profile)
            configs = load_all_configs(profile)
            self._command = build_calibration_command(self.arm, configs, profile)
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
        elif "start recording" in lower and "range" in lower:
            self._phase = "range_prompt"
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


def _runtime_controller(
    profile: str,
    role: str,
    configs: Mapping[str, Mapping[str, Any]],
) -> Any:
    profile = _validate_profile(profile)
    role = _normalize_arm(role)
    server = configs[f"{role}_server"]
    robot = _nested(server, "components", "robot")
    if profile == "dual":
        from rynnrcp_robot_so101.controller import SO101BimanualController

        return SO101BimanualController(
            left_port=_require_text(robot.get("left_port"), "components.robot.left_port"),
            right_port=_require_text(robot.get("right_port"), "components.robot.right_port"),
            left_robot_id=_require_text(robot.get("left_robot_id"), "components.robot.left_robot_id"),
            right_robot_id=_require_text(robot.get("right_robot_id"), "components.robot.right_robot_id"),
            role=role,
        )

    from rynnrcp_robot_so101.controller import SO101Controller

    return SO101Controller(
        port=_require_text(robot.get("port"), "components.robot.port"),
        robot_id=_require_text(_nested(server, "manifest").get("robot_id"), "manifest.robot_id"),
        role=role,
    )


def _motion_joint_names(profile: str) -> list[str]:
    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    if _validate_profile(profile) == "single":
        return names
    return [f"{side}_{name}" for side in ("left", "right") for name in names]


class PresetMotionJob:
    """Run one finite SO101 follower preset motion for post-calibration validation."""

    def __init__(self) -> None:
        self._controller: Any = None
        self._thread: threading.Thread | None = None
        self._logs: list[Dict[str, str]] = []
        self._motion: int | None = None
        self._result: Dict[str, Any] | None = None
        self._error: str | None = None
        self._completed = False
        self._stopped = False
        self._stop_requested = False
        self._returning = False
        self._trace: list[Dict[str, Any]] = []
        self._debug_targets: list[float] | None = None
        self._home_positions: list[float] | None = None
        self._home_editing = False
        self._sample_stop = threading.Event()
        self._sample_thread: threading.Thread | None = None
        self._teleop_stop = threading.Event()
        self._teleop_thread: threading.Thread | None = None
        self._leader_controller: Any = None
        self._profile = "single"
        self._motion_target = "both"
        self._home_edit_target = "both"
        self._lock = threading.RLock()

    def connect(self, profile: str = "single") -> Dict[str, Any]:
        profile = _validate_profile(profile)
        with self._lock:
            if self._controller is not None and self._profile == profile:
                return self.status()
            if self._controller is not None:
                if self.is_running() or self.is_teleop_running() or self._home_editing:
                    raise RuntimeError("stop the current debug session before changing profile")
                old_controller, self._controller = self._controller, None
            else:
                old_controller = None
        if old_controller is not None:
            self._sample_stop.set()
            old_sample_thread, self._sample_thread = self._sample_thread, None
            if old_sample_thread is not None and old_sample_thread is not threading.current_thread():
                old_sample_thread.join(timeout=1.0)
            old_controller.shutdown()
        configs = load_all_configs(profile)
        server = configs["follower_server"]
        robot = _nested(server, "components", "robot")
        controller = _runtime_controller(profile, "follower", configs)
        ports = (
            f"{robot.get('left_port')} / {robot.get('right_port')}"
            if profile == "dual"
            else str(robot.get("port"))
        )
        self._append("info", f"Connecting {profile} follower on {ports}")
        controller.start()
        with self._lock:
            self._profile = profile
            self._controller = controller
            self._debug_targets = controller.get_joint_positions()["joint_positions"]
            configured_home = robot.get("home_positions")
            expected_dof = 12 if profile == "dual" else 6
            self._home_positions = [float(v) for v in configured_home] if isinstance(configured_home, list) and len(configured_home) == expected_dof else None
            self._append("info", f"Follower connected; {expected_dof}-DoF live joint control is ready")
            self._sample_stop.clear()
            self._sample_thread = threading.Thread(target=self._sample_session, daemon=True)
            self._sample_thread.start()
        return self.status()

    def _sample_session(self) -> None:
        started = time.monotonic()
        frame = 0
        while not self._sample_stop.wait(1 / 30):
            with self._lock:
                controller = self._controller
                action = list(self._debug_targets or [])
                editing = self._home_editing
            if controller is None or not action:
                continue
            try:
                if editing and self._profile == "dual":
                    state = controller.read_joint_positions_now(target=self._home_edit_target)
                else:
                    state = controller.read_joint_positions_now() if editing else controller.get_joint_positions()["joint_positions"]
                self._record_trace_sample({
                    "t": time.monotonic() - started,
                    "frame": frame,
                    "action": action,
                    "state": state,
                })
                frame += 1
            except Exception:
                LOGGER.debug("SO101 debug session sampling failed", exc_info=True)

    def set_joint(self, index: int, value: float, profile: str = "single") -> Dict[str, Any]:
        self.connect(profile)
        if index not in range(self._dof()):
            raise ValueError(f"joint index must be between 0 and {self._dof() - 1}")
        low, high = self._limits()[index]
        value = max(low, min(high, float(value)))
        with self._lock:
            if self.is_running() or self.is_teleop_running():
                raise RuntimeError("stop the current motion before using a joint slider")
            positions = list(self._debug_targets or self._controller.get_joint_positions()["joint_positions"])
            positions[index] = value
            self._debug_targets = positions
            self._controller.set_joint_positions({"joint_positions": positions})
        return self.status()

    def start_sine(self, joint: int, amplitude: float, frequency: float, duration_s: float = 10.0, profile: str = "single") -> Dict[str, Any]:
        self.connect(profile)
        if joint not in range(self._dof()):
            raise ValueError(f"joint index must be between 0 and {self._dof() - 1}")
        amplitude = max(0.01, min(0.5 if joint % 6 == 5 else 1.0, float(amplitude)))
        frequency = max(0.05, min(2.0, float(frequency)))
        with self._lock:
            if self.is_running() or self.is_teleop_running():
                raise RuntimeError("a motion is already running")
            self._motion = None
            self._completed = self._stopped = self._stop_requested = False
            self._error = None
            self._thread = threading.Thread(target=self._run_sine, args=(joint, amplitude, frequency, duration_s), daemon=True)
            self._thread.start()
        return self.status()

    def go_home(self, profile: str = "single", target: str = "both") -> Dict[str, Any]:
        self.connect(profile)
        self.stop()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            self._returning = True
            self._append("info", "Returning to Home")
        try:
            if self._profile == "dual":
                self._controller.go_home(home_positions=self._home_positions, target=target)
            else:
                self._controller.go_home(home_positions=self._home_positions)
            with self._lock:
                self._debug_targets = self._controller.get_joint_positions()["joint_positions"]
                self._stopped = False
                self._completed = True
                self._append("info", "Home position reached")
        finally:
            with self._lock:
                self._returning = False
        return self.status()

    def start_teleop(self, profile: str = "single") -> Dict[str, Any]:
        self.connect(profile)
        with self._lock:
            if self.is_running() or self.is_teleop_running() or self._home_editing:
                raise RuntimeError("stop the current operation before starting teleop")
        configs = load_all_configs(self._profile)
        robot = _nested(configs["leader_server"], "components", "robot")
        leader = _runtime_controller(self._profile, "leader", configs)
        ports = (
            f"{robot.get('left_port')} / {robot.get('right_port')}"
            if self._profile == "dual"
            else str(robot.get("port"))
        )
        self._append("info", f"Connecting {self._profile} leader on {ports}")
        leader.start()
        with self._lock:
            self._leader_controller = leader
            self._teleop_stop.clear()
            self._error = None
            self._stopped = False
            self._teleop_thread = threading.Thread(target=self._run_teleop, daemon=True)
            self._teleop_thread.start()
            self._append("info", "Teleop started: left/right leaders control matching followers at 30 Hz")
        return self.status()

    def _run_teleop(self) -> None:
        try:
            while not self._teleop_stop.wait(1 / 30):
                positions = self._leader_controller.get_joint_positions()["joint_positions"]
                self._controller.set_joint_positions({"joint_positions": positions})
                self._track_action_target({"action": positions})
        except Exception as exc:
            LOGGER.exception("SO101 debug teleop failed")
            with self._lock:
                self._error = str(exc)
            self._append("error", str(exc))
        finally:
            with self._lock:
                leader, self._leader_controller = self._leader_controller, None
            if leader is not None:
                leader.shutdown()

    def stop_teleop(self) -> Dict[str, Any]:
        self._teleop_stop.set()
        thread = self._teleop_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._lock:
            if thread is not None:
                self._append("info", "Teleop stopped")
            self._teleop_thread = None
        return self.status()

    def is_teleop_running(self) -> bool:
        return self._teleop_thread is not None and self._teleop_thread.is_alive()

    def edit_home(self, profile: str = "single", target: str = "both") -> Dict[str, Any]:
        self.connect(profile)
        self.stop()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        if self._profile == "dual":
            self._controller.disable_torque(target=target)
        else:
            self._controller.disable_torque()
        with self._lock:
            self._home_editing = True
            self._home_edit_target = target
            self._append("info", "Joints released; move the arm and save the current position")
        return self.status()

    def save_home(self) -> Dict[str, Any]:
        if not self._home_editing or self._controller is None:
            raise RuntimeError("click Set Home before saving")
        positions = (
            self._controller.read_joint_positions_now(target=self._home_edit_target)
            if self._profile == "dual"
            else self._controller.read_joint_positions_now()
        )
        configs = load_all_configs(self._profile)
        _nested(configs["follower_server"], "components", "robot")["home_positions"] = positions
        save_yaml_config(_profile_files(self._profile)["follower_server"], configs["follower_server"])
        with self._lock:
            self._home_positions = list(positions)
            self._debug_targets = list(positions)
        if self._profile == "dual":
            self._controller.enable_torque_at(positions, target=self._home_edit_target)
        else:
            self._controller.enable_torque_at(positions)
        with self._lock:
            self._home_editing = False
            self._append("info", "Current position saved as Home")
        return self.status()

    def _dof(self) -> int:
        return 12 if self._profile == "dual" else 6

    def _limits(self) -> list[list[float]]:
        return ([[ -math.pi, math.pi] for _ in range(5)] + [[0.0, 1.0]]) * (2 if self._profile == "dual" else 1)

    def start(self, motion: int, profile: str = "single", target: str = "both") -> Dict[str, Any]:
        self.connect(profile)
        motion = int(motion)
        if motion not in {1, 2, 3, 4, 5}:
            raise ValueError("motion must be one of 1, 2, 3, 4, 5")
        with self._lock:
            if self.is_running() or self.is_teleop_running():
                raise RuntimeError("preset motion is already running")
            self._logs = []
            self._motion = motion
            self._motion_target = target
            self._result = None
            self._error = None
            self._completed = False
            self._stopped = False
            self._stop_requested = False
            self._returning = False
            self._append("info", f"Starting SO101 follower preset Motion {motion} on {target}")
            self._thread = threading.Thread(target=self._run, args=(motion,), daemon=True)
            self._thread.start()
            return self.status()

    def stop(self) -> Dict[str, Any]:
        controller = None
        with self._lock:
            self._stop_requested = True
            self._stopped = True
            controller = self._controller
            self._append("warning", "Stop requested; cancelling the current motion")
        if controller is not None:
            try:
                controller.stop_motion()
            except Exception as exc:
                with self._lock:
                    self._append("error", f"Failed to stop preset motion: {exc}")
        if self.is_teleop_running():
            self.stop_teleop()
        return self.status()

    def status(self) -> Dict[str, Any]:
        with self._lock:
            positions = self._controller.get_joint_positions()["joint_positions"] if self._controller is not None else None
            return {
                "running": self.is_running(),
                "motion": self._motion,
                "completed": self._completed,
                "stopped": self._stopped,
                "returning": self._returning,
                "error": self._error,
                "result": dict(self._result or {}),
                "trace": list(self._trace),
                "logs": list(self._logs),
                "connected": self._controller is not None,
                "positions": positions,
                "limits": self._limits(),
                "profile": self._profile,
                "target": self._motion_target,
                "joint_names": _motion_joint_names(self._profile),
                "home_editing": self._home_editing,
                "teleop_running": self.is_teleop_running(),
            }

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self, motion: int) -> None:
        controller = None
        try:
            self.connect(self._profile)
            controller = self._controller
            if self._stop_requested:
                with self._lock:
                    self._result = {"cancelled": True, "frames_sent": 0}
                    self._stopped = True
                self._append("warning", "Stop was requested before motion playback; skipping preset")
                return
            self._append("info", f"Running Motion {motion}; keep clear and watch joint directions")
            request = {"motion": motion}
            if self._profile == "dual":
                request["target"] = self._motion_target
            result = controller.preset_motion(request, progress_callback=self._track_action_target)
            with self._lock:
                self._debug_targets = controller.get_joint_positions()["joint_positions"]
                self._result = result
                self._completed = bool(result.get("completed")) and not self._stop_requested
                self._stopped = self._stopped or bool(result.get("cancelled"))
            if self._completed:
                self._append("info", f"Motion {motion} completed")
            else:
                self._append("warning", f"Motion {motion} stopped before completion")
        except Exception as exc:
            LOGGER.exception("SO101 preset motion failed")
            with self._lock:
                self._error = str(exc)
            self._append("error", str(exc))
    def _run_sine(self, joint: int, amplitude: float, frequency: float, duration_s: float) -> None:
        try:
            controller = self._controller
            center = controller.get_joint_positions()["joint_positions"]
            low, high = self._limits()[joint]
            started = time.monotonic()
            frame = 0
            while not self._stop_requested and time.monotonic() - started < duration_s:
                elapsed = time.monotonic() - started
                action = list(center)
                action[joint] = max(low, min(high, center[joint] + amplitude * math.sin(2 * math.pi * frequency * elapsed)))
                controller.set_joint_positions({"joint_positions": action})
                self._track_action_target({"action": action})
                frame += 1
                time.sleep(1 / 30)
            with self._lock:
                self._debug_targets = controller.get_joint_positions()["joint_positions"]
                self._completed = not self._stop_requested
        except Exception as exc:
            LOGGER.exception("SO101 sine motion failed")
            with self._lock:
                self._error = str(exc)
            self._append("error", str(exc))

    def shutdown(self) -> None:
        self.stop()
        self._sample_stop.set()
        sample_thread = self._sample_thread
        if sample_thread is not None and sample_thread is not threading.current_thread():
            sample_thread.join(timeout=1.0)
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            controller, self._controller = self._controller, None
            self._debug_targets = None
        if controller is not None:
            if self._home_editing:
                if self._profile == "dual":
                    for arm in controller._arms.values():
                        arm.go_home_on_disconnect = False
                else:
                    controller.go_home_on_disconnect = False
            controller.shutdown()

    def _append(self, kind: str, text: str) -> None:
        with self._lock:
            self._logs.append({"time": time.strftime("%H:%M:%S"), "type": kind, "text": text})
            if len(self._logs) > CALIBRATION_LOG_LIMIT:
                self._logs = self._logs[-CALIBRATION_LOG_LIMIT:]

    def _record_trace_sample(self, sample: Mapping[str, Any]) -> None:
        action = [float(v) for v in sample.get("action") or []]
        state_value = sample.get("state")
        state = [float(v) for v in state_value] if isinstance(state_value, list) else None
        errors = [abs(a - s) for a, s in zip(action, state)] if state is not None else []
        item = {
            "t": float(sample.get("t") or 0.0),
            "frame": int(sample.get("frame") or 0),
            "action": action,
            "state": state,
            "error": (sum(errors) / len(errors)) if errors else None,
            "joint_errors": errors,
        }
        with self._lock:
            if action:
                self._debug_targets = list(action)
            self._trace.append(item)
            if len(self._trace) > 9000:
                self._trace = self._trace[-9000:]

    def _track_action_target(self, sample: Mapping[str, Any]) -> None:
        action = [float(v) for v in sample.get("action") or []]
        if action:
            with self._lock:
                self._debug_targets = action


_CALIBRATION_JOBS = {device: CalibrationJob(device) for device in sorted(CALIBRATION_DEVICES)}
_PRESET_MOTION_JOB = PresetMotionJob()


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
            "dependencies, for example: uv pip install -e robots/lerobot_so101"
        ) from exc

    app = Flask(__name__)

    @app.get("/")
    def index():
        return render_template_string(_load_html_template())

    @app.get("/api/config")
    def api_get_config():
        profile = _validate_profile(request.args.get("profile", "single"))
        return jsonify(
            {
                "ok": True,
                "config": build_snapshot(load_all_configs(profile), profile),
                "paths": _path_snapshot(profile),
            }
        )

    @app.post("/api/config")
    def api_save_config():
        payload = request.get_json(silent=True) or {}
        if not isinstance(payload, dict):
            return jsonify({"ok": False, "error": "request body must be a JSON object"}), 400

        try:
            snapshot = normalize_snapshot(payload)
            profile = snapshot["profile"]
            configs = load_all_configs(profile)
            apply_snapshot_to_configs(configs, snapshot)
            for name, config in configs.items():
                save_yaml_config(_profile_files(profile)[name], config)
        except Exception as exc:
            LOGGER.exception("Failed to save SO101 web configuration")
            return jsonify({"ok": False, "error": str(exc)}), 400

        return jsonify({"ok": True, "config": build_snapshot(load_all_configs(profile), profile)})

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
        return jsonify(validate_configs(request.args.get("profile", "single")))

    @app.get("/api/commands")
    def api_commands():
        return jsonify({"ok": True, "commands": COMMANDS, "cwd": str(REPO_ROOT)})

    @app.post("/api/shutdown")
    def api_shutdown():
        try:
            _PRESET_MOTION_JOB.shutdown()
        except Exception:
            LOGGER.exception("Failed to stop SO101 preset motion during shutdown")
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
            payload = request.get_json(silent=True) or {}
            arm = _request_arm(payload)
            return jsonify(
                {
                    "ok": True,
                    "status": _CALIBRATION_JOBS[arm].start(_validate_profile(payload.get("profile") or "single")),
                }
            )
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
            arm = _normalize_calibration_device(request.args.get("arm", "follower"))
            return jsonify({"ok": True, "status": _CALIBRATION_JOBS[arm].status()})
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/start")
    def api_preset_motion_start():
        try:
            payload = request.get_json(silent=True) or {}
            return jsonify(
                {
                    "ok": True,
                    "status": _PRESET_MOTION_JOB.start(
                        int(payload.get("motion", 0)),
                        _validate_profile(payload.get("profile") or "single"),
                        str(payload.get("target") or "both"),
                    ),
                }
            )
        except Exception as exc:
            LOGGER.exception("Failed to start SO101 preset motion")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/connect")
    def api_preset_motion_connect():
        try:
            payload = request.get_json(silent=True) or {}
            return jsonify(
                {
                    "ok": True,
                    "status": _PRESET_MOTION_JOB.connect(_validate_profile(payload.get("profile") or "single")),
                }
            )
        except Exception as exc:
            LOGGER.exception("Failed to connect SO101 debug session")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/joint")
    def api_preset_motion_joint():
        try:
            payload = request.get_json(silent=True) or {}
            return jsonify(
                {
                    "ok": True,
                    "status": _PRESET_MOTION_JOB.set_joint(
                        int(payload.get("index", -1)),
                        float(payload.get("value")),
                        _validate_profile(payload.get("profile") or "single"),
                    ),
                }
            )
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/sine")
    def api_preset_motion_sine():
        try:
            payload = request.get_json(silent=True) or {}
            status = _PRESET_MOTION_JOB.start_sine(
                int(payload.get("joint", -1)),
                float(payload.get("amplitude", 0.2)),
                float(payload.get("frequency", 0.2)),
                profile=_validate_profile(payload.get("profile") or "single"),
            )
            return jsonify({"ok": True, "status": status})
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/home")
    def api_preset_motion_home():
        try:
            payload = request.get_json(silent=True) or {}
            return jsonify(
                {
                    "ok": True,
                    "status": _PRESET_MOTION_JOB.go_home(
                        _validate_profile(payload.get("profile") or "single"),
                        str(payload.get("target") or "both"),
                    ),
                }
            )
        except Exception as exc:
            LOGGER.exception("Failed to return SO101 to Home")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/home/edit")
    def api_preset_motion_home_edit():
        try:
            payload = request.get_json(silent=True) or {}
            return jsonify(
                {
                    "ok": True,
                    "status": _PRESET_MOTION_JOB.edit_home(
                        _validate_profile(payload.get("profile") or "single"),
                        str(payload.get("target") or "both"),
                    ),
                }
            )
        except Exception as exc:
            LOGGER.exception("Failed to release SO101 for Home setup")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/home/save")
    def api_preset_motion_home_save():
        try:
            return jsonify({"ok": True, "status": _PRESET_MOTION_JOB.save_home()})
        except Exception as exc:
            LOGGER.exception("Failed to save SO101 Home")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/teleop/start")
    def api_preset_motion_teleop_start():
        try:
            payload = request.get_json(silent=True) or {}
            return jsonify(
                {
                    "ok": True,
                    "status": _PRESET_MOTION_JOB.start_teleop(
                        _validate_profile(payload.get("profile") or "single")
                    ),
                }
            )
        except Exception as exc:
            LOGGER.exception("Failed to start SO101 debug teleop")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/teleop/stop")
    def api_preset_motion_teleop_stop():
        try:
            return jsonify({"ok": True, "status": _PRESET_MOTION_JOB.stop_teleop()})
        except Exception as exc:
            LOGGER.exception("Failed to stop SO101 debug teleop")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/preset_motion/stop")
    def api_preset_motion_stop():
        try:
            return jsonify({"ok": True, "status": _PRESET_MOTION_JOB.stop()})
        except Exception as exc:
            LOGGER.exception("Failed to stop SO101 preset motion")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.get("/api/preset_motion/status")
    def api_preset_motion_status():
        return jsonify({"ok": True, "status": _PRESET_MOTION_JOB.status()})

    return app


def build_calibration_command(
    arm: str,
    configs: Mapping[str, Mapping[str, Any]],
    profile: str = "single",
) -> list[str]:
    return [sys.executable, "-m", "rynnrcp_robot_so101.configure_so101_web", "--calibrate"] + calibration_args(
        arm, configs, profile
    )


def calibration_args(
    arm: str,
    configs: Mapping[str, Mapping[str, Any]],
    profile: str = "single",
) -> list[str]:
    device = _normalize_calibration_device(arm)
    role, side = _device_role_side(device)
    profile = _validate_profile(profile)
    if profile == "single" and side is not None:
        raise ValueError("left/right calibration devices require the dual profile")
    if profile == "dual" and side is None:
        raise ValueError("dual profile calibration requires a left/right device")
    server = configs[f"{role}_server"]
    robot = _nested(server, "components", "robot")
    if side is None:
        port = _require_text(robot.get("port"), "components.robot.port")
        robot_id = _require_text(_nested(server, "manifest").get("robot_id"), "manifest.robot_id")
    else:
        port = _require_text(robot.get(f"{side}_port"), f"components.robot.{side}_port")
        robot_id = _require_text(robot.get(f"{side}_robot_id"), f"components.robot.{side}_robot_id")
    return [
        f"--arm={role}",
        f"--port={port}",
        f"--id={robot_id}",
    ]


def _request_arm(payload: Mapping[str, Any]) -> str:
    return _normalize_calibration_device(str(payload.get("arm", "follower")))


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


def _validate_profile(profile: Any) -> str:
    normalized = str(profile or "single").strip().lower()
    if normalized not in PROFILE_CONFIG_FILES:
        raise ValueError("profile must be 'single' or 'dual'")
    return normalized


def _profile_files(profile: Any) -> Mapping[str, Path]:
    return PROFILE_CONFIG_FILES[_validate_profile(profile)]


def _infer_profile(configs: Mapping[str, Mapping[str, Any]]) -> str:
    robot = _nested(configs.get("follower_server", {}), "components", "robot")
    integration = str(_nested(configs.get("follower_server", {}), "integration").get("config") or "")
    return "dual" if "left_port" in robot or "bimanual" in integration else "single"


def load_all_configs(profile: str = "single") -> Dict[str, Dict[str, Any]]:
    return {name: load_yaml_config(path) for name, path in _profile_files(profile).items()}


def build_snapshot(
    configs: Mapping[str, Mapping[str, Any]],
    profile: str | None = None,
) -> Dict[str, Any]:
    profile = _validate_profile(profile or _infer_profile(configs))
    follower_server = configs["follower_server"]
    leader_server = configs["leader_server"]
    rynnbot_app = configs["rynnbot_app"]
    master_rynnbot_app = configs["master_rynnbot_app"]
    rynnbot_cfg = _nested(rynnbot_app, "app")
    master_rynnbot_cfg = _nested(master_rynnbot_app, "app")
    manifest = _nested(follower_server, "manifest")
    suffix = machine_mac_suffix()

    robot = _nested(follower_server, "components", "robot")
    leader_robot = _nested(leader_server, "components", "robot")
    hardware = (
        {
            "left_follower_port": robot.get("left_port", ""),
            "right_follower_port": robot.get("right_port", ""),
            "left_leader_port": leader_robot.get("left_port", ""),
            "right_leader_port": leader_robot.get("right_port", ""),
            "front_camera": _nested(follower_server, "components", "front_camera").get("device_id", 0),
            "left_wrist_camera": _nested(follower_server, "components", "left_wrist_camera").get("device_id", 1),
            "right_wrist_camera": _nested(follower_server, "components", "right_wrist_camera").get("device_id", 2),
        }
        if profile == "dual"
        else {
            "follower_port": robot.get("port", ""),
            "leader_port": leader_robot.get("port", ""),
            "front_camera": _nested(follower_server, "components", "front_camera").get("device_id", 0),
            "wrist_camera": _nested(follower_server, "components", "wrist_camera").get("device_id", 1),
        }
    )
    server_base = "so101_bimanual_follower" if profile == "dual" else "so101_follower"
    app_base = "so101_bimanual_rynnbot_app" if profile == "dual" else "so101_rynnbot_app"
    master_app_base = (
        "so101_bimanual_master_rynnbot_app" if profile == "dual" else "so101_master_rynnbot_app"
    )

    return {
        "profile": profile,
        "server": {
            "id": with_machine_suffix(manifest.get("robot_id"), server_base, suffix),
        },
        "hardware": hardware,
        "rynnbot": {
            "app_id": with_machine_suffix(rynnbot_cfg.get("app_id"), app_base, suffix),
            "product_key": rynnbot_cfg.get("product_key", ""),
            "device_name": rynnbot_cfg.get("device_name", ""),
            "device_secret": rynnbot_cfg.get("device_secret", ""),
            "http_url": rynnbot_cfg.get("http_url", "https://robot-access.damo-academy.com"),
            "image_upload_codec": rynnbot_cfg.get("image_upload_codec", "jpeg"),
        },
        "master_rynnbot": {
            "app_id": with_machine_suffix(master_rynnbot_cfg.get("app_id"), master_app_base, suffix),
            "product_key": master_rynnbot_cfg.get("product_key", ""),
            "device_name": master_rynnbot_cfg.get("device_name", ""),
            "device_secret": master_rynnbot_cfg.get("device_secret", ""),
            "http_url": master_rynnbot_cfg.get("http_url", "https://robot-access.damo-academy.com"),
        },
    }


def normalize_snapshot(payload: Mapping[str, Any]) -> Dict[str, Any]:
    profile = _validate_profile(payload.get("profile") or "single")
    current = build_snapshot(load_all_configs(profile), profile)
    merged = _deep_merge(current, payload)

    hardware = merged["hardware"]
    rynnbot = merged["rynnbot"]
    master_rynnbot = merged["master_rynnbot"]
    server = merged["server"]
    suffix = machine_mac_suffix()

    server_base = "so101_bimanual_follower" if profile == "dual" else "so101_follower"
    app_base = "so101_bimanual_rynnbot_app" if profile == "dual" else "so101_rynnbot_app"
    master_app_base = (
        "so101_bimanual_master_rynnbot_app" if profile == "dual" else "so101_master_rynnbot_app"
    )
    if profile == "dual":
        normalized_hardware = {
            key: _require_text(hardware.get(key), f"hardware.{key}")
            for key in (
                "left_follower_port",
                "right_follower_port",
                "left_leader_port",
                "right_leader_port",
            )
        }
        normalized_hardware.update(
            {
                key: _require_non_negative_int(hardware.get(key), f"hardware.{key}")
                for key in ("front_camera", "left_wrist_camera", "right_wrist_camera")
            }
        )
    else:
        normalized_hardware = {
            "follower_port": _require_text(hardware.get("follower_port"), "hardware.follower_port"),
            "leader_port": _require_text(hardware.get("leader_port"), "hardware.leader_port"),
            "front_camera": _require_non_negative_int(hardware.get("front_camera"), "hardware.front_camera"),
            "wrist_camera": _require_non_negative_int(hardware.get("wrist_camera"), "hardware.wrist_camera"),
        }

    return {
        "profile": profile,
        "server": {
            "id": with_machine_suffix(server.get("id"), server_base, suffix),
        },
        "hardware": normalized_hardware,
        "rynnbot": {
            "app_id": with_machine_suffix(rynnbot.get("app_id"), app_base, suffix),
            "product_key": str(rynnbot.get("product_key", "")).strip(),
            "device_name": str(rynnbot.get("device_name", "")).strip(),
            "device_secret": str(rynnbot.get("device_secret", "")).strip(),
            "http_url": _require_text(rynnbot.get("http_url"), "rynnbot.http_url"),
            "image_upload_codec": _image_upload_codec(rynnbot.get("image_upload_codec")),
        },
        "master_rynnbot": {
            "app_id": with_machine_suffix(master_rynnbot.get("app_id"), master_app_base, suffix),
            "product_key": str(master_rynnbot.get("product_key", "")).strip(),
            "device_name": str(master_rynnbot.get("device_name", "")).strip(),
            "device_secret": str(master_rynnbot.get("device_secret", "")).strip(),
            "http_url": _require_text(master_rynnbot.get("http_url"), "master_rynnbot.http_url"),
        },
    }


def apply_snapshot_to_configs(configs: Mapping[str, Dict[str, Any]], snapshot: Mapping[str, Any]) -> None:
    profile = _validate_profile(snapshot.get("profile") or _infer_profile(configs))
    server_snapshot = snapshot["server"]
    hardware = snapshot["hardware"]
    rynnbot = snapshot["rynnbot"]
    master_rynnbot = snapshot["master_rynnbot"]
    suffix = machine_mac_suffix()

    follower_server = configs["follower_server"]
    leader_server = configs["leader_server"]
    if profile == "dual":
        follower_id = with_machine_suffix(server_snapshot["id"], "so101_bimanual_follower", suffix)
        leader_id = with_machine_suffix(
            _nested(leader_server, "manifest").get("robot_id"), "so101_bimanual_leader", suffix
        )
        _set_nested(follower_server, follower_id, "manifest", "robot_id")
        _set_nested(follower_server, "SO101 Bimanual Follower", "manifest", "robot_name")
        _set_nested(follower_server, hardware["left_follower_port"], "components", "robot", "left_port")
        _set_nested(follower_server, hardware["right_follower_port"], "components", "robot", "right_port")
        _set_nested(follower_server, f"{follower_id}_left", "components", "robot", "left_robot_id")
        _set_nested(follower_server, f"{follower_id}_right", "components", "robot", "right_robot_id")
        _set_nested(follower_server, "follower", "components", "robot", "role")
        _set_nested(follower_server, hardware["front_camera"], "components", "front_camera", "device_id")
        _set_nested(follower_server, hardware["left_wrist_camera"], "components", "left_wrist_camera", "device_id")
        _set_nested(follower_server, hardware["right_wrist_camera"], "components", "right_wrist_camera", "device_id")
        _set_nested(
            follower_server,
            "package://rynnrcp_robot_so101/config/robot_integration_bimanual.yaml",
            "integration",
            "config",
        )
        _set_nested(leader_server, leader_id, "manifest", "robot_id")
        _set_nested(leader_server, "SO101 Bimanual Leader", "manifest", "robot_name")
        _set_nested(leader_server, hardware["left_leader_port"], "components", "robot", "left_port")
        _set_nested(leader_server, hardware["right_leader_port"], "components", "robot", "right_port")
        _set_nested(leader_server, f"{leader_id}_left", "components", "robot", "left_robot_id")
        _set_nested(leader_server, f"{leader_id}_right", "components", "robot", "right_robot_id")
        _set_nested(leader_server, "leader", "components", "robot", "role")
        _set_nested(
            leader_server,
            "package://rynnrcp_robot_so101/config/robot_integration_bimanual.yaml",
            "integration",
            "config",
        )
        app_base = "so101_bimanual_rynnbot_app"
        master_app_base = "so101_bimanual_master_rynnbot_app"
    else:
        _set_nested(
            follower_server,
            with_machine_suffix(server_snapshot["id"], "so101_follower", suffix),
            "manifest",
            "robot_id",
        )
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
        _set_nested(
            leader_server,
            with_machine_suffix(_nested(leader_server, "manifest").get("robot_id"), "so101_leader", suffix),
            "manifest",
            "robot_id",
        )
        _set_nested(leader_server, "SO101 Leader", "manifest", "robot_name")
        _set_nested(leader_server, hardware["leader_port"], "components", "robot", "port")
        _set_nested(leader_server, "leader", "components", "robot", "role")
        _set_nested(
            leader_server,
            "package://rynnrcp_robot_so101/config/robot_integration.yaml",
            "integration",
            "config",
        )
        app_base = "so101_rynnbot_app"
        master_app_base = "so101_master_rynnbot_app"
    _clean_server_config(follower_server)
    _clean_server_config(leader_server)

    if rynnbot.get("app_id"):
        _set_nested(configs["rynnbot_app"], rynnbot["app_id"], "app", "app_id")
    for key in ("product_key", "device_name", "device_secret", "http_url", "image_upload_codec"):
        _set_nested(configs["rynnbot_app"], rynnbot[key], "app", key)

    if master_rynnbot.get("app_id"):
        _set_nested(configs["master_rynnbot_app"], master_rynnbot["app_id"], "app", "app_id")
    for key in ("product_key", "device_name", "device_secret", "http_url"):
        _set_nested(configs["master_rynnbot_app"], master_rynnbot[key], "app", key)

    _ensure_machine_app_id(configs["rynnbot_app"], app_base, suffix)
    _ensure_machine_app_id(configs["master_rynnbot_app"], master_app_base, suffix)

    _clean_rynnbot_app_config(configs["rynnbot_app"])
    _clean_master_rynnbot_app_config(configs["master_rynnbot_app"])


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


def _clean_master_rynnbot_app_config(config: Dict[str, Any]) -> None:
    _clean_rynnbot_app_config(config)
    app = config.get("app")
    if isinstance(app, dict):
        app.pop("image_upload_codec", None)


def _ensure_machine_app_id(config: Dict[str, Any], base_app_id: str, suffix: str) -> str:
    app = config.setdefault("app", {})
    app_id = with_machine_suffix(app.get("app_id"), base_app_id, suffix)
    app["app_id"] = app_id
    return app_id


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

    for device in _fallback_serial_devices():
        if device not in seen:
            ports.append({"device": device, "description": "serial device", "hwid": ""})
            seen.add(device)

    return _dedupe_serial_ports(ports)


def _fallback_serial_devices() -> list[str]:
    devices: list[str] = []
    for pattern in (
        "/dev/tty.usbmodem*",
        "/dev/cu.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/cu.usbserial*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "/dev/ttyCH343USB*",
        "/dev/ttyCH34x*",
        "/dev/ttyWCHUSB*",
    ):
        devices.extend(sorted(glob.glob(pattern)))
    if os.name == "nt":
        # Do not manufacture COM1..COM256 here.  The serial detection UI uses
        # the current list as a hot-plug baseline, so imaginary COM ports make
        # every real Windows port look as if it was already connected.
        devices.extend(_windows_serial_devices())
    return devices


def _windows_serial_devices() -> list[str]:
    """Return COM ports currently mapped by Windows.

    pyserial/SetupAPI remains the primary scanner.  This registry lookup is a
    small native fallback for environments where pyserial cannot enumerate
    ports; unlike probing a fixed COM range, it only reports mapped devices.
    """
    try:
        import winreg
    except ImportError:
        return []

    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
    except OSError:
        return []

    devices: list[str] = []
    try:
        index = 0
        while True:
            try:
                _name, value, _value_type = winreg.EnumValue(key, index)
            except OSError:
                break
            device = str(value).strip()
            if re.fullmatch(r"COM\d+", device, flags=re.IGNORECASE):
                devices.append(device.upper())
            index += 1
    finally:
        winreg.CloseKey(key)

    return sorted(set(devices), key=_serial_device_sort_key)


def _dedupe_serial_ports(ports: list[Dict[str, str]]) -> list[Dict[str, str]]:
    selected: dict[str, Dict[str, str]] = {}
    for port in ports:
        device = str(port.get("device") or "")
        if not device or _is_ignored_serial_device(device):
            continue
        if re.fullmatch(r"COM\d+", device, flags=re.IGNORECASE):
            device = device.upper()
        key = _serial_device_key(device)
        current = selected.get(key)
        if current is None or _serial_device_rank(device) < _serial_device_rank(str(current.get("device") or "")):
            selected[key] = {**port, "device": device}

    return sorted(
        selected.values(),
        key=lambda item: _serial_device_sort_key(str(item.get("device") or "")),
    )


def _is_ignored_serial_device(device: str) -> bool:
    lower = device.lower()
    return "bluetooth-incoming-port" in lower or "wlan-debug" in lower


def _serial_device_key(device: str) -> str:
    name = device.rsplit("/", 1)[-1]
    if re.fullmatch(r"COM\d+", name, flags=re.IGNORECASE):
        return name.upper()
    for prefix in ("cu.", "tty."):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _serial_device_sort_key(device: str) -> tuple[int, int | str]:
    name = device.rsplit("/", 1)[-1]
    match = re.fullmatch(r"COM(\d+)", name, flags=re.IGNORECASE)
    if match:
        return (0, int(match.group(1)))
    return (1, device.casefold())


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


def _open_camera_preferred(cv2_module: Any, index: int):
    """Open *index* camera, preferring 640×360; fall back to default if unsupported."""
    cap = cv2_module.VideoCapture(index)
    if not cap.isOpened():
        return cap
    cap.set(cv2_module.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2_module.CAP_PROP_FRAME_HEIGHT, 360)
    return cap


def _probe_camera_with_cv2(cv2_module: Any, index: int) -> Dict[str, Any]:
    cap = _open_camera_preferred(cv2_module, index)
    try:
        if not cap.isOpened():
            return {"ok": False, "index": index, "opened": False}
        ok, frame = cap.read()
        if ok and frame is not None and frame.shape[1] == 640 and frame.shape[0] == 360:
            item: Dict[str, Any] = {"index": index, "opened": True}
            item.update(_encode_frame(cv2_module, frame))
            return {"ok": True, "camera": item}
        # 640×360 not supported — reopen with default resolution
        cap.release()
        cap = cv2_module.VideoCapture(index)
        if not cap.isOpened():
            return {"ok": False, "index": index, "opened": False}
        ok, frame = cap.read()
        item = {"index": index, "opened": True}
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

    cap = _open_camera_preferred(cv2, index)
    try:
        if not cap.isOpened():
            return {"ok": False, "error": f"camera {index} could not be opened"}
        ok, frame = cap.read()
        if ok and frame is not None and frame.shape[1] == 640 and frame.shape[0] == 360:
            encoded = _encode_frame(cv2, frame)
            encoded["ok"] = True
            encoded["index"] = index
            return encoded
        # 640×360 not supported — reopen with default resolution
        cap.release()
        cap = cv2.VideoCapture(index)
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


def validate_configs(profile: str = "single") -> Dict[str, Any]:
    for path in (str(REPO_ROOT), str(SO101_PROJECT_ROOT)):
        if path not in sys.path:
            sys.path.insert(0, path)

    results: list[Dict[str, Any]] = []
    try:
        from rynnrcp.config.loader import load_config
        from rynnrcp.config.validator import ConfigValidator
    except Exception as exc:
        return {"ok": False, "error": f"RynnRCP config validator is not available: {exc}", "results": []}

    for name, path in _profile_files(profile).items():
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


def _path_snapshot(profile: str = "single") -> Dict[str, str]:
    paths = {name: str(path) for name, path in _profile_files(profile).items()}
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


def _is_port_available(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, int(port)))
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
            phase="range_prompt",
            message="Click Start Recording Range, then move every joint through its full range.",
        )
        _calib_wait_for_enter_with_positions(bus, phase="range_prompt")

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
    parser.add_argument("--host", default="0.0.0.0", help="Host for the configuration web server.")
    parser.add_argument("--port", default=28401, type=int, help="Port for the configuration web server.")
    parser.add_argument("--debug", action="store_true", help="Enable Flask debug mode.")
    parser.add_argument("--no-open", action="store_true", help="Skip automatic browser launch.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    from rynnrcp.utils.logging import configure_logging

    configure_logging(level=logging.INFO, sinks=["stderr"])

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

    urls = browser_urls(args.host, port)
    LOGGER.info("SO101 configuration UI Local: %s", urls[0])
    for url in urls[1:]:
        LOGGER.info("SO101 configuration UI LAN:   %s", url)
    LOGGER.info("Config directory: %s", CONFIG_DIR)
    if not args.no_open:
        threading.Timer(0.6, lambda: webbrowser.open(primary_browser_url(args.host, port))).start()
    app.run(host=args.host, port=port, debug=args.debug, use_reloader=args.debug)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
