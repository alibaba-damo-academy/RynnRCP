#!/usr/bin/env python3
"""Browser-based Aero Hand configuration helper for RynnRCP."""

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
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import yaml

from rynnrcp.utils.web_urls import browser_urls, primary_browser_url


LOGGER = logging.getLogger("rynnrcp.aero_hand.configure_web")
PACKAGE_DIR = Path(__file__).resolve().parent
CONFIG_DIR = PACKAGE_DIR / "config"
CONFIG_PATHS = {
    "single": CONFIG_DIR / "aero_hand_single_server.yaml",
    "dual": CONFIG_DIR / "aero_hand_dual_server.yaml",
}
RYNNBOT_CONFIG_PATHS = {
    "single": CONFIG_DIR / "aero_hand_single_rynnbot_app.yaml",
    "dual": CONFIG_DIR / "aero_hand_dual_rynnbot_app.yaml",
}
CAMERA_MASTER_CONFIG_PATHS = {
    "single": CONFIG_DIR / "aero_hand_single_hand_master_server.yaml",
    "dual": CONFIG_DIR / "aero_hand_dual_hand_master_server.yaml",
}
CAMERA_MASTER_RYNNBOT_CONFIG_PATHS = {
    "single": CONFIG_DIR / "aero_hand_single_hand_master_rynnbot_app.yaml",
    "dual": CONFIG_DIR / "aero_hand_dual_hand_master_rynnbot_app.yaml",
}
JOINT_NAMES = [
    "thumb_cmc_abd",
    "thumb_cmc_flex",
    "thumb_mcp_ip",
    "index_finger",
    "middle_finger",
    "ring_finger",
    "pinky_finger",
]
OPEN_POSE_DEG = [0, 0, 0, 0, 0, 0, 0]
RELAXED_POSE_DEG = [10, 10, 10, 15, 15, 15, 15]
DEBUG_SERIAL_TIMEOUT_S = 0.05
DEBUG_SERIAL_WRITE_TIMEOUT_S = 0.2
DEBUG_POSE_REPEATS = 3
DEBUG_POSE_REPEAT_INTERVAL_S = 0.08
DEBUG_POSE_SETTLE_S = 0.15
HARDWARE_LOCK = threading.Lock()


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Aero Hand 配置向导 - RynnRCP</title>
  <style>
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #101418; color: #e8edf2; font: 14px/1.55 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    main { max-width: 980px; margin: 0 auto; padding: 24px; }
    header { display: flex; align-items: flex-start; justify-content: space-between; gap: 18px; margin-bottom: 20px; border-bottom: 1px solid #26313b; padding-bottom: 18px; }
    h1 { margin: 0; font-size: 28px; letter-spacing: 0; color: #7dd3fc; }
    h2 { margin: 0 0 14px; font-size: 18px; color: #bae6fd; }
    p { margin: 6px 0; color: #aab6c3; }
    section { background: #161d24; border: 1px solid #26313b; border-radius: 8px; padding: 18px; margin-bottom: 16px; }
    label { display: block; color: #b6c2cf; font-weight: 650; margin-bottom: 6px; }
    input, select { width: 100%; border: 1px solid #33414f; background: #0f151b; color: #eef5fb; border-radius: 7px; padding: 11px 12px; font: inherit; }
    button { border: 0; border-radius: 7px; padding: 11px 16px; font-weight: 750; cursor: pointer; color: #081018; background: #7dd3fc; }
    button.secondary { background: #94a3b8; }
    button.good { background: #86efac; }
    button.warn { background: #facc15; }
    button.danger { background: #fb7185; color: white; }
    button:disabled { opacity: .5; cursor: not-allowed; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(260px, 1fr)); gap: 16px; }
    .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
    .wizard { display: grid; gap: 12px; margin-top: 16px; }
    .step { display: grid; grid-template-columns: 34px 1fr auto; gap: 12px; align-items: center; padding: 12px; border: 1px solid #293744; border-radius: 8px; background: #111820; }
    .step-number { display: grid; place-items: center; width: 30px; height: 30px; border-radius: 50%; background: #26313b; color: #bae6fd; font-weight: 800; }
    .step strong { color: #e5f4ff; }
    .step small { display: block; color: #93a4b5; margin-top: 3px; }
    .ports { display: grid; gap: 8px; margin-top: 12px; }
    .port { display: grid; grid-template-columns: 1fr auto auto; gap: 10px; align-items: center; padding: 12px; border: 1px solid #293744; border-radius: 8px; background: #111820; }
    .port strong { color: #e5f4ff; word-break: break-all; }
    .port small { display: block; color: #93a4b5; margin-top: 4px; word-break: break-all; }
    .hint { border-left: 4px solid #7dd3fc; background: #10202b; padding: 12px 14px; border-radius: 0 8px 8px 0; margin-bottom: 14px; }
    .usage-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; margin-bottom: 16px; }
    .usage-card { border: 1px solid #33414f; border-radius: 8px; padding: 12px; background: #111820; color: #cbd5e1; }
    .usage-card strong { display: block; color: #e5f4ff; margin-bottom: 5px; }
    .usage-card b { color: #86efac; }
    .field-note { margin-top: 6px; color: #93a4b5; font-size: 12px; }
    .status { margin-top: 12px; padding: 11px 12px; border: 1px solid #2b3b48; border-radius: 8px; background: #101820; color: #cbd5e1; }
    .status.good { border-color: #14532d; background: #102017; color: #86efac; }
    .status.warn { border-color: #713f12; background: #211b0c; color: #fde68a; }
    .status.bad { border-color: #7f1d1d; background: #241012; color: #fecdd3; }
    .value { color: #86efac; font-weight: 750; }
    .summary { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 12px; }
    .pill { border: 1px solid #33414f; border-radius: 999px; padding: 7px 10px; color: #cbd5e1; background: #0f151b; }
    .pill.done { border-color: #14532d; color: #86efac; }
    .camera-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 14px; margin-top: 14px; }
    .camera-card { overflow: hidden; border: 1px solid #293744; border-radius: 8px; background: #0f151b; }
    .camera-card.selected { border-color: #86efac; box-shadow: 0 0 0 1px #86efac; }
    .camera-card img { width: 100%; aspect-ratio: 16/9; object-fit: cover; display: block; background: #0b1015; }
    .camera-card-body { padding: 12px; }
    .camera-card-body strong { display: block; margin-bottom: 8px; }
    .teleop-preview { display: none; margin-top: 14px; overflow: hidden; border: 1px solid #293744; border-radius: 8px; background: #0b1015; }
    .teleop-preview img { display: none; width: 100%; max-height: 540px; object-fit: contain; }
    .teleop-preview-placeholder { padding: 48px 16px; text-align: center; color: #93a4b5; }
    pre { margin: 0; min-height: 180px; max-height: 360px; overflow: auto; white-space: pre-wrap; word-break: break-word; background: #0b1015; border: 1px solid #26313b; border-radius: 8px; padding: 14px; color: #a7f3d0; }
    @media (max-width: 760px) { main { padding: 14px; } header, .grid, .usage-grid, .port, .step { grid-template-columns: 1fr; display: grid; } }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>Aero Hand 配置向导</h1>
      <p>单手和双手是两个云端构型；RCP 内部都只暴露一个 robot 向量通道。</p>
    </div>
    <button class="danger" onclick="shutdown()">退出</button>
  </header>

  <section>
    <h2>1. 选择配置类型</h2>
    <div class="hint">单手构型输出画面中的一只手，共 7 维；双手构型输出 14 维，顺序为 left 7 维 + right 7 维。Baudrate 固定为 921600；Server 启动后由用户手动执行 homing。</div>
    <div class="grid">
      <div>
        <label>要配置哪种机器人</label>
        <select id="profile" onchange="loadConfig().then(updatePortFields).then(updateDetectTarget).then(updateSetupCopy)">
          <option value="single">Aero Hand Single (7 DoF)</option>
          <option value="dual">Aero Hand Dual (14 DoF)</option>
        </select>
      </div>
    </div>
  </section>

  <section>
    <h2>2. Server 配置</h2>
    <div class="hint">这里配置当前机器人构型的本地 Server。请使用下面的插拔识别选择并写入端口。</div>
    <div class="grid">
      <div><label>Robot ID</label><input id="robot-id"></div>
      <div><label>Robot Name</label><input id="robot-name"></div>
      <div id="single-port-field"><label>Hand Port</label><input id="port" readonly placeholder="扫描后自动识别"></div>
      <div id="left-port-field"><label>Left Port</label><input id="left-port" readonly placeholder="扫描后选择左手"></div>
      <div id="right-port-field"><label>Right Port</label><input id="right-port" readonly placeholder="扫描后选择右手"></div>
    </div>

    <div class="summary" id="port-summary"></div>

    <div class="wizard">
      <div class="step">
        <div class="step-number">1</div>
        <div>
          <strong>先拔掉要配置的 Aero Hand</strong>
          <small>记录电脑当前已有串口，后面只识别新插入的设备。</small>
        </div>
        <button onclick="recordBaselinePorts()">记录当前端口</button>
      </div>
      <div class="step">
        <div class="step-number">2</div>
        <div>
          <strong id="detect-title">插入 Aero Hand 并识别</strong>
          <small id="detect-copy">插入单手后点击识别，端口会自动写入 Hand Port。</small>
        </div>
        <div class="row">
          <select id="detect-target" style="width:auto; min-width:150px;"></select>
          <button onclick="detectAddedPorts()">识别新插入的手</button>
        </div>
      </div>
      <div class="step">
        <div class="step-number">3</div>
        <div>
          <strong>保存构型文件</strong>
          <small>确认端口识别正确后保存。单手写 single server，双手写 dual server。</small>
        </div>
        <div class="row">
          <button class="good" onclick="saveConfig()">保存当前构型</button>
          <button class="secondary" onclick="loadConfig()">重新加载</button>
        </div>
      </div>
    </div>

    <div id="config-status" class="status">等待加载配置...</div>
    <div id="port-status" class="status">先拔掉要配置的手，点击“记录当前端口”。</div>
    <div class="ports" id="ports"></div>
  </section>

  <section>
    <h2>3. 云端连接（RynnBot）</h2>
    <div class="hint"><strong>两组凭据对应两种使用方式。</strong>真机本体接入 RynnBot 时填写真机执行端；摄像手势控制仿真时填写摄像手势控制端。本地“开始手势遥操”、本地 Teleop 和本地数采可直接进入下一步。</div>
    <div class="usage-grid">
      <div class="usage-card"><strong>本地手势遥操 / Teleop / 数采</strong><b>填写 0 套凭据</b><br>直接进入下一步选择摄像头。</div>
      <div class="usage-card"><strong>真机本体接入 RynnBot</strong><b>填写 1 套凭据</b><br>填写“真机执行端”。</div>
      <div class="usage-card"><strong>手势控制端控制仿真目标</strong><b>填写 1 套凭据</b><br>填写“摄像手势控制端”。</div>
    </div>
    <h3 style="margin:0 0 12px;color:#7dd3fc;">真机执行端凭据（接入 RynnBot 时必填）</h3>
    <div class="hint">这套设备代表实际 Aero Hand，负责向云端上传状态并接收动作。真机接入 RynnBot 时填写；本地遥操可直接进入下一步。</div>
    <div class="grid">
      <div><label>真机执行端 App ID（自动生成）</label><input id="rynnbot-app-id" readonly></div>
      <div><label>真机执行端 Product Key</label><input id="product-key" autocomplete="off" placeholder="真机接入 RynnBot 时填写"></div>
      <div><label>真机执行端 Device Name</label><input id="device-name" autocomplete="off" placeholder="真机接入 RynnBot 时填写"></div>
      <div><label>真机执行端 Device Secret</label><input id="device-secret" autocomplete="new-password" placeholder="真机接入 RynnBot 时填写"></div>
      <div><label>真机执行端 HTTP URL（通常保持默认）</label><input id="http-url"></div>
    </div>
    <h3 style="margin:22px 0 12px;color:#7dd3fc;">摄像手势控制端凭据（控制端接入 RynnBot 时必填）</h3>
    <div class="hint"><strong>这套凭据用于摄像手势控制服务。</strong>该服务读取摄像头、识别手势、产生控制动作，并作为 RynnBot controller 控制仿真目标。使用云端仿真控制时填写；摄像头编号在下一节选择。</div>
    <div class="grid">
      <div><label>手势控制端 App ID（自动生成）</label><input id="master-app-id" readonly></div>
      <div><label>手势控制端 Product Key</label><input id="master-product-key" autocomplete="off" placeholder="控制端接入 RynnBot 时填写"></div>
      <div><label>手势控制端 Device Name</label><input id="master-device-name" autocomplete="off" placeholder="控制端接入 RynnBot 时填写"></div>
      <div><label>手势控制端 Device Secret</label><input id="master-device-secret" autocomplete="new-password" placeholder="控制端接入 RynnBot 时填写"></div>
      <div><label>手势控制端 HTTP URL（通常保持默认）</label><input id="master-http-url"></div>
    </div>
    <p class="field-note">请根据本次使用方式填写其中一组。Product Key、Device Name 和 Device Secret 使用云端平台为该设备签发的值。</p>
    <div class="row" style="margin-top:16px;">
      <button class="good" onclick="saveRynnBotConfig()">保存 RynnBot 设置</button>
    </div>
    <div id="rynnbot-status" class="status">等待加载设备设置...</div>
  </section>

  <section>
    <h2>4. 手势摄像头与采集画面</h2>
    <div class="hint">在这里选择手势识别摄像头。同一个摄像头实例同时提供手势识别画面和本地数据集的 observation.images.front；RynnBot 凭据在上一节配置。</div>
    <input id="master-camera-index" type="hidden" value="0">
    <div class="row" style="margin-top:14px;">
      <button onclick="scanMasterCameras()">扫描可用摄像头</button>
      <span id="master-camera-selection" class="pill">当前：Camera 0</span>
    </div>
    <div id="master-camera-scan-status" class="status">点击“扫描可用摄像头”后选择画面。</div>
    <div id="master-camera-grid" class="camera-grid"></div>
    <div class="row" style="margin-top:16px;">
      <button class="good" onclick="saveCameraMasterConfig()">保存摄像头选择</button>
    </div>
    <div id="camera-master-status" class="status">正在读取摄像头选择...</div>
  </section>

  <section>
    <h2>5. 真机调试</h2>
    <div class="hint">调试动作按当前选择的云端构型执行。单手连接一个端口；双手会同时连接左右手端口。</div>
    <div class="row">
      <button class="debug-action" onclick="testConnect()">只测试连接</button>
      <button class="debug-action" onclick="readState()">读取状态</button>
      <button class="warn debug-action" onclick="runHoming()">执行 homing</button>
      <button class="good debug-action" onclick="sendPose('open')">张开手</button>
      <button class="warn debug-action" onclick="sendPose('relaxed')">轻微握合</button>
    </div>
    <div id="debug-status" class="status">等待调试操作。</div>
    <h3 style="margin:22px 0 12px;color:#7dd3fc;">摄像头手势遥操</h3>
    <div class="hint">使用已选择的摄像头通过本地调试链路直接控制当前真机。启动后请让手保持在画面内，并随时准备点击停止。</div>
    <div class="row">
      <button class="good" id="camera-teleop-start" onclick="startCameraTeleop()">开始手势遥操</button>
      <button class="danger" id="camera-teleop-stop" onclick="stopCameraTeleop()" disabled>停止手势遥操</button>
    </div>
    <div id="camera-teleop-status" class="status">未启动。</div>
    <div id="camera-teleop-preview" class="teleop-preview">
      <img id="camera-teleop-preview-image" alt="摄像头手势识别画面">
      <div id="camera-teleop-preview-placeholder" class="teleop-preview-placeholder">等待摄像头画面...</div>
    </div>
  </section>

  <section>
    <h2>日志</h2>
    <pre id="log"></pre>
  </section>
</main>

<script>
function $(id) { return document.getElementById(id); }
let baselinePorts = null;
let debugBusy = false;
let masterCameras = [];
let cameraTeleopActive = false;
let cameraTeleopPollTimer = null;
let cameraTeleopPreviewTimer = null;
let cameraTeleopPreviewLoading = false;
const CAMERA_TELEOP_PREVIEW_INTERVAL_MS = 33;
function currentProfile() { return $("profile").value; }
function currentDof() { return currentProfile() === "dual" ? 14 : 7; }
function updatePortFields() {
  const dual = currentProfile() === "dual";
  $("single-port-field").style.display = dual ? "none" : "block";
  $("left-port-field").style.display = dual ? "block" : "none";
  $("right-port-field").style.display = dual ? "block" : "none";
  updatePortSummary();
}
function updateDetectTarget() {
  const target = $("detect-target");
  target.innerHTML = "";
  const options = currentProfile() === "dual" ? [["left", "写入 Left Port"], ["right", "写入 Right Port"]] : [["single", "写入 Hand Port"]];
  for (const [value, label] of options) {
    const option = document.createElement("option");
    option.value = value;
    option.textContent = label;
    target.appendChild(option);
  }
  target.style.display = currentProfile() === "dual" ? "block" : "none";
}
function updateSetupCopy() {
  if (currentProfile() === "dual") {
    $("detect-title").textContent = "逐个插入左右手并识别";
    $("detect-copy").textContent = "先在右侧选择 Left 或 Right，再插入对应手并点击识别。识别成功后再插下一只手。";
  } else {
    $("detect-title").textContent = "插入 Aero Hand 并识别";
    $("detect-copy").textContent = "插入单手后点击识别，端口会自动写入 Hand Port。";
  }
}
function updatePortSummary() {
  const items = currentProfile() === "dual"
    ? [["Left", $("left-port").value], ["Right", $("right-port").value]]
    : [["Hand", $("port").value]];
  $("port-summary").innerHTML = items.map(([label, value]) => {
    const ok = value ? " done" : "";
    return `<span class="pill${ok}">${label}: ${value || "未识别"}</span>`;
  }).join("");
}
function log(message) {
  const now = new Date().toLocaleTimeString();
  $("log").textContent += `[${now}] ${message}\n`;
  $("log").scrollTop = $("log").scrollHeight;
}
function setStatus(id, message, type = "") {
  const el = $(id);
  el.textContent = message;
  el.className = "status" + (type ? " " + type : "");
}
function setDebugBusy(busy) {
  debugBusy = busy;
  for (const button of document.querySelectorAll(".debug-action")) {
    button.disabled = busy || cameraTeleopActive;
  }
  $("camera-teleop-start").disabled = busy || cameraTeleopActive;
}
async function api(path, options = {}) {
  const res = await fetch(path, options);
  const data = await res.json();
  if (!res.ok || data.ok === false) throw new Error(data.error || "request failed");
  return data;
}
async function scanMasterCameras() {
  const maxIndex = 20;
  masterCameras = [];
  $("master-camera-grid").innerHTML = "";
  setStatus("master-camera-scan-status", "正在扫描 Camera 0-" + maxIndex + "...", "warn");
  for (let index = 0; index <= maxIndex; index += 1) {
    setStatus("master-camera-scan-status", "正在扫描 Camera " + index + " / " + maxIndex + "，已发现 " + masterCameras.length + " 个", "warn");
    try {
      const data = await api("/api/camera/" + index + "/probe");
      if (data.camera) {
        masterCameras.push(data.camera);
        renderMasterCameras();
      }
    } catch {}
  }
  setStatus("master-camera-scan-status", masterCameras.length ? "扫描完成，请选择用于手势识别和数据采集的画面。" : "未检测到可用摄像头，请检查系统摄像头权限。", masterCameras.length ? "good" : "warn");
}
function renderMasterCameras() {
  const selected = Number($("master-camera-index").value);
  $("master-camera-grid").innerHTML = masterCameras.map(camera => `
    <div class="camera-card${camera.index === selected ? " selected" : ""}">
      <img src="${camera.image || ""}" alt="Camera ${camera.index}">
      <div class="camera-card-body">
        <strong>Camera ${camera.index}${camera.width ? " · " + camera.width + "×" + camera.height : ""}</strong>
        <button class="${camera.index === selected ? "good" : "secondary"}" onclick="selectMasterCamera(${camera.index})">${camera.index === selected ? "已选择" : "选择这个摄像头"}</button>
      </div>
    </div>`).join("");
}
function selectMasterCamera(index) {
  $("master-camera-index").value = String(index);
  $("master-camera-selection").textContent = "当前：Camera " + index;
  renderMasterCameras();
}
async function loadConfig() {
  const data = await api("/api/config?profile=" + currentProfile());
  const c = data.config;
  const robot = c.components.robot || {};
  $("robot-id").value = c.manifest.robot_id || "";
  $("robot-name").value = c.manifest.robot_name || "";
  $("port").value = robot.port || "";
  $("left-port").value = robot.left_port || "";
  $("right-port").value = robot.right_port || "";
  updatePortFields();
  updateDetectTarget();
  updateSetupCopy();
  const rb = data.rynnbot.app || {};
  $("rynnbot-app-id").value = rb.app_id || "";
  $("product-key").value = displayCredential(rb.product_key);
  $("device-name").value = displayCredential(rb.device_name);
  $("device-secret").value = displayCredential(rb.device_secret);
  $("http-url").value = rb.http_url || "https://robot-access.damo-academy.com";
  const masterRobot = ((data.camera_master || {}).components || {}).robot || {};
  $("master-camera-index").value = masterRobot.camera_index ?? 0;
  $("master-camera-selection").textContent = "当前：Camera " + $("master-camera-index").value;
  const masterRb = (data.camera_master_rynnbot || {}).app || {};
  $("master-app-id").value = masterRb.app_id || "";
  $("master-product-key").value = displayCredential(masterRb.product_key);
  $("master-device-name").value = displayCredential(masterRb.device_name);
  $("master-device-secret").value = displayCredential(masterRb.device_secret);
  $("master-http-url").value = masterRb.http_url || "https://robot-access.damo-academy.com";
  const profileText = currentProfile() === "dual" ? "双手 14 维" : "单手 7 维";
  setStatus("config-status", profileText + " 配置已加载：" + data.path, "good");
  setStatus("port-status", portSummary(), portSummary().includes("未设置") ? "warn" : "good");
  const cloudReady = hasCloudCreds(rb);
  const masterReady = hasCloudCreds(masterRb);
  const deviceStatus = "本地遥操可直接使用；RynnBot 云端：真机执行端" + (cloudReady ? "已配置" : "待配置") + "，摄像手势控制端" + (masterReady ? "已配置" : "待配置");
  setStatus("rynnbot-status", deviceStatus, "good");
  setStatus("camera-master-status", "已选择 Camera " + $("master-camera-index").value + "；识别与数采共用", "good");
  log("配置已加载: " + data.path);
}
function displayCredential(value) {
  const text = String(value || "");
  return text.startsWith("YOUR_") ? "" : text;
}
function hasCloudCreds(app) {
  return ["product_key", "device_name", "device_secret"].every(key => app[key] && !String(app[key]).startsWith("YOUR_"));
}
function portSummary() {
  if (currentProfile() === "dual") {
    return "双手端口：left=" + ($("left-port").value || "未设置") + "，right=" + ($("right-port").value || "未设置");
  }
  return "单手端口：" + ($("port").value || "未设置");
}
async function saveConfig() {
  try {
    const body = configPayload();
    const data = await api("/api/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
    setStatus("config-status", currentDof() + " 维构型已保存：" + data.path, "good");
    setStatus("port-status", portSummary(), portSummary().includes("未设置") ? "warn" : "good");
    log("配置已保存: " + data.path);
  } catch (err) {
    setStatus("config-status", "保存失败：" + err.message, "bad");
  }
}
async function saveRynnBotConfig() {
  try {
    const body = configPayload();
    body.rynnbot = {
      product_key: $("product-key").value.trim(),
      device_name: $("device-name").value.trim(),
      device_secret: $("device-secret").value.trim(),
      http_url: $("http-url").value.trim() || "https://robot-access.damo-academy.com"
    };
    body.camera_master_rynnbot = {
      product_key: $("master-product-key").value.trim(),
      device_name: $("master-device-name").value.trim(),
      device_secret: $("master-device-secret").value.trim(),
      http_url: $("master-http-url").value.trim() || "https://robot-access.damo-academy.com"
    };
    const data = await api("/api/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
    const targetReady = ["product-key", "device-name", "device-secret"].every(id => $(id).value.trim());
    const controllerReady = ["master-product-key", "master-device-name", "master-device-secret"].every(id => $(id).value.trim());
    const message = "已保存。本地遥操可直接使用；RynnBot 云端：真机执行端" + (targetReady ? "已配置" : "待配置") + "，摄像手势控制端" + (controllerReady ? "已配置" : "待配置");
    setStatus("rynnbot-status", message, "good");
    log("RynnBot 设备设置已保存: " + data.rynnbot_path);
  } catch (err) {
    setStatus("rynnbot-status", "保存失败：" + err.message, "bad");
  }
}
async function saveCameraMasterConfig() {
  try {
    const body = configPayload();
    body.front_camera = {
      enabled: true,
      device_id: Number($("master-camera-index").value)
    };
    body.camera_master = {
      camera_index: Number($("master-camera-index").value),
      side: currentProfile() === "single" ? "auto" : "right"
    };
    const data = await api("/api/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
    setStatus("camera-master-status", "已保存 Camera " + $("master-camera-index").value + "；识别与数采共用", "good");
    log("共用摄像头选择已保存: " + data.camera_master_path);
  } catch (err) {
    setStatus("camera-master-status", "保存失败：" + err.message, "bad");
  }
}
function configPayload() {
  return {
    profile: currentProfile(),
    robot_id: $("robot-id").value.trim(),
    robot_name: $("robot-name").value.trim(),
    port: $("port").value.trim(),
    left_port: $("left-port").value.trim(),
    right_port: $("right-port").value.trim()
  };
}
async function recordBaselinePorts() {
  try {
    const data = await api("/api/ports");
    baselinePorts = data.ports.map(p => p.device);
    $("ports").innerHTML = "";
    setStatus("port-status", "已记录 " + baselinePorts.length + " 个当前端口。现在插入要配置的手，再点“识别新插入的手”。", "good");
    log("基线端口: " + JSON.stringify(baselinePorts));
  } catch (err) {
    setStatus("port-status", "记录基线失败：" + err.message, "bad");
  }
}
async function detectAddedPorts() {
  if (!baselinePorts) {
    setStatus("port-status", "请先点击“记录基线端口”。", "warn");
    return;
  }
  try {
    const data = await api("/api/ports");
    const currentPorts = data.ports.map(p => p.device);
    const added = data.ports.filter(p => !baselinePorts.includes(p.device));
    renderDetectedPorts(added, currentPorts);
    if (added.length === 1) {
      setPort($("detect-target").value, added[0].device);
      baselinePorts = currentPorts;
      setStatus("port-status", "已识别并写入：" + added[0].device + "。双手构型可继续插入另一只手；完成后保存。", "good");
    } else if (added.length === 0) {
      setStatus("port-status", "请在记录基线后插入这只手，再点击识别新增端口。", "warn");
    } else {
      setStatus("port-status", "检测到 " + added.length + " 个新增端口，请选择属于这只手的端口。", "warn");
    }
    log("新增端口: " + JSON.stringify(added.map(p => p.device)));
  } catch (err) {
    setStatus("port-status", "检测失败：" + err.message, "bad");
  }
}
function renderDetectedPorts(ports, currentPorts) {
  const list = $("ports");
  list.innerHTML = "";
  for (const p of ports) {
    const item = document.createElement("div");
    item.className = "port";
    item.innerHTML = `<div><strong>${p.device}</strong><small>${p.description || ""}</small><small>${p.hwid || ""}</small></div>`;
    const btn = document.createElement("button");
    btn.textContent = "使用";
    btn.onclick = () => {
      setPort($("detect-target").value, p.device);
      baselinePorts = currentPorts;
      setStatus("port-status", "已写入：" + p.device + "。双手构型可继续插入另一只手；完成后保存。", "good");
    };
    item.appendChild(btn);
    list.appendChild(item);
  }
}
function setPort(side, port) {
  const id = side === "single" ? "port" : side + "-port";
  $(id).value = port;
  const label = side === "single" ? "单手" : (side === "left" ? "左手" : "右手");
  updatePortSummary();
  setStatus("port-status", "已设置" + label + "串口：" + port + "。完成后点击“保存当前构型”。", "good");
  log("已设置 " + side + " port: " + port);
}
async function testConnect() {
  try {
    setStatus("debug-status", "正在测试 " + currentDof() + " 维构型连接...", "warn");
    const data = await action("connect");
    setStatus("debug-status", "连接成功：" + JSON.stringify(data.result.ports), "good");
    log("连接成功: " + JSON.stringify(data.result));
  } catch (err) {
    setStatus("debug-status", "连接失败：" + err.message, "bad");
  }
}
async function readState() {
  try {
    setStatus("debug-status", "正在读取 " + currentDof() + " 维状态...", "warn");
    const data = await action("read");
    setStatus("debug-status", "读取成功，" + data.result.joint_positions_deg.length + " 维 deg。", "good");
    log("当前关节 rad: " + JSON.stringify(data.result.joint_positions_rad));
    log("当前关节 deg: " + JSON.stringify(data.result.joint_positions_deg));
  } catch (err) {
    setStatus("debug-status", "读取失败：" + err.message, "bad");
  }
}
async function runHoming() {
  if (!confirm("确认执行当前构型 homing？请确保所有已连接手周围安全。")) return;
  try {
    setStatus("debug-status", "正在执行 homing，手会移动...", "warn");
    const data = await action("homing");
    setStatus("debug-status", "homing 指令已发送到 " + data.result.homed.length + " 只手，已等待 " + data.result.settle_s + " 秒。", "good");
    log("homing 完成: " + JSON.stringify(data.result));
  } catch (err) {
    setStatus("debug-status", "homing 失败：" + err.message, "bad");
  }
}
async function sendPose(name) {
  const label = name === "open" ? "张开手" : "轻微握合";
  if (!confirm("确认向当前构型发送姿态：" + label + "？")) return;
  try {
    setStatus("debug-status", "正在发送姿态：" + label, "warn");
    const data = await action("pose/" + name);
    setStatus("debug-status", label + " 已发送，向量长度 " + data.result.sent_deg.length + "。", "good");
    log(label + " 已发送: " + JSON.stringify(data.result));
  } catch (err) {
    setStatus("debug-status", label + " 失败：" + err.message, "bad");
  }
}
async function startCameraTeleop() {
  if (debugBusy) {
    setStatus("camera-teleop-status", "请等待当前调试操作完成。", "warn");
    return;
  }
  if (!confirm("启动后，摄像头中的手势会直接控制真机。请确认机械手周围安全，是否继续？")) return;
  try {
    const body = configPayload();
    body.camera_index = Number($("master-camera-index").value);
    const data = await api("/api/camera-teleop/start", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(body)
    });
    renderCameraTeleopStatus(data.status);
    pollCameraTeleopStatus();
  } catch (err) {
    setStatus("camera-teleop-status", "启动失败：" + err.message, "bad");
  }
}
async function stopCameraTeleop() {
  try {
    const data = await api("/api/camera-teleop/stop", {method: "POST"});
    renderCameraTeleopStatus(data.status);
  } catch (err) {
    setStatus("camera-teleop-status", "停止失败：" + err.message, "bad");
  }
}
async function pollCameraTeleopStatus() {
  if (cameraTeleopPollTimer) clearTimeout(cameraTeleopPollTimer);
  try {
    const data = await api("/api/camera-teleop/status");
    renderCameraTeleopStatus(data.status);
    if (data.status.active) cameraTeleopPollTimer = setTimeout(pollCameraTeleopStatus, 500);
  } catch (err) {
    setStatus("camera-teleop-status", "状态读取失败：" + err.message, "bad");
  }
}
function renderCameraTeleopStatus(status) {
  cameraTeleopActive = !!status.active;
  const labels = {
    idle: "未启动",
    starting: "正在连接摄像头和真机",
    waiting_for_hand: "已连接，等待识别到手",
    running: "遥操中",
    stopping: "正在停止",
    stopped: "已停止",
    error: "运行失败"
  };
  let message = labels[status.state] || status.state;
  if (status.frames_sent) message += "，已发送 " + status.frames_sent + " 帧";
  if (status.error) message += "：" + status.error;
  setStatus("camera-teleop-status", message, status.state === "error" ? "bad" : (cameraTeleopActive ? "warn" : "good"));
  $("camera-teleop-start").disabled = cameraTeleopActive;
  $("camera-teleop-stop").disabled = !cameraTeleopActive;
  setDebugBusy(debugBusy);
  if (cameraTeleopActive || status.frames_sent) {
    $("camera-teleop-preview").style.display = "block";
    refreshCameraTeleopPreview();
  }
}
function refreshCameraTeleopPreview() {
  if (cameraTeleopPreviewTimer) clearTimeout(cameraTeleopPreviewTimer);
  if (!cameraTeleopActive || cameraTeleopPreviewLoading) return;
  cameraTeleopPreviewLoading = true;
  const image = $("camera-teleop-preview-image");
  const scheduleNext = () => {
    cameraTeleopPreviewLoading = false;
    if (cameraTeleopActive) cameraTeleopPreviewTimer = setTimeout(refreshCameraTeleopPreview, CAMERA_TELEOP_PREVIEW_INTERVAL_MS);
  };
  image.onload = () => {
    image.style.display = "block";
    $("camera-teleop-preview-placeholder").style.display = "none";
    scheduleNext();
  };
  image.onerror = () => {
    if (!image.src) return;
    image.style.display = "none";
    $("camera-teleop-preview-placeholder").style.display = "block";
    scheduleNext();
  };
  image.src = "/api/camera-teleop/preview?t=" + Date.now();
}
async function action(name) {
  if (debugBusy) throw new Error("已有调试操作正在执行，请稍后再试。");
  setDebugBusy(true);
  try {
    return await api("/api/" + name, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(configPayload())
    });
  } finally {
    setDebugBusy(false);
  }
}
async function shutdown() {
  try { await api("/api/shutdown", {method: "POST"}); } catch {}
  document.body.innerHTML = "<main><h1>Aero Hand 配置程序已退出</h1><p>可以关闭这个标签页。</p></main>";
}
loadConfig().then(updatePortFields).then(updateDetectTarget).then(updateSetupCopy).then(pollCameraTeleopStatus).catch(err => log("初始化失败: " + err.message));
</script>
</body>
</html>
"""


class CameraTeleopJob:
    """Run camera gesture control against the currently configured Aero Hand."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._state = "idle"
        self._error: str | None = None
        self._frames_sent = 0
        self._profile: str | None = None
        self._vision: Any = None
        self._last_preview: bytes | None = None

    def start(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("camera gesture teleoperation is already running")
            self._stop.clear()
            self._state = "starting"
            self._error = None
            self._frames_sent = 0
            self._profile = _profile_from_payload(payload)
            self._last_preview = None
            self._thread = threading.Thread(
                target=self._run,
                args=(dict(payload),),
                name="aero-hand-camera-teleop",
                daemon=True,
            )
            self._thread.start()
            return self.status()

    def stop(self) -> dict[str, Any]:
        self._stop.set()
        with self._lock:
            thread = self._thread
            if thread is not None and thread.is_alive():
                self._state = "stopping"
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        with self._lock:
            if thread is not None and not thread.is_alive():
                self._thread = None
                if self._state != "error":
                    self._state = "stopped"
        return self.status()

    def is_active(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def status(self) -> dict[str, Any]:
        with self._lock:
            active = self._thread is not None and self._thread.is_alive()
            return {
                "active": active,
                "state": self._state,
                "error": self._error,
                "frames_sent": self._frames_sent,
                "profile": self._profile,
            }

    def preview_jpeg(self) -> bytes:
        with self._lock:
            last_preview = self._last_preview
        if last_preview is not None:
            return last_preview
        raise RuntimeError("camera teleoperation has no preview frame")

    def _run(self, payload: dict[str, Any]) -> None:
        vision = None
        controller = None
        try:
            profile = _profile_from_payload(payload)
            vision = _new_camera_teleop_vision(profile, payload)
            with self._lock:
                self._vision = vision
            with HARDWARE_LOCK:
                controller = _new_camera_teleop_controller(profile, payload)
                controller.start()
            vision.start()
            with self._lock:
                self._state = "waiting_for_hand"

            while not self._stop.wait(1.0 / 30.0):
                try:
                    positions = vision.get_joint_positions()["joint_positions"]
                except RuntimeError as exc:
                    if "has not detected" in str(exc):
                        for warning in vision.get_health().get("warnings", []):
                            if warning.get("code") == "aero_hand.vision_error":
                                details = warning.get("details") or {}
                                raise RuntimeError(
                                    str(details.get("error") or warning.get("message"))
                                )
                        continue
                    raise
                controller.set_joint_positions({"joint_positions": positions})
                try:
                    preview = vision.get_preview_jpeg()
                except RuntimeError:
                    preview = None
                with self._lock:
                    if preview is not None:
                        self._last_preview = preview
                    self._state = "running"
                    self._frames_sent += 1
        except Exception as exc:
            LOGGER.exception("Aero Hand camera gesture teleoperation failed")
            with self._lock:
                self._state = "error"
                self._error = str(exc)
        finally:
            if vision is not None:
                try:
                    with self._lock:
                        self._last_preview = vision.get_preview_jpeg()
                except Exception:
                    pass
                try:
                    vision.shutdown()
                except Exception:
                    LOGGER.exception("Failed to stop Aero Hand camera gesture source")
            if controller is not None:
                try:
                    controller.shutdown()
                except Exception:
                    LOGGER.exception("Failed to stop Aero Hand camera teleoperation controller")
            with self._lock:
                self._vision = None
                if self._state != "error":
                    self._state = "stopped"


_CAMERA_TELEOP_JOB = CameraTeleopJob()


class Handler(BaseHTTPRequestHandler):
    server: "ConfigureServer"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send_html(HTML)
        elif parsed.path == "/api/config":
            profile = _profile_from_query(parsed.query)
            self._json(
                {
                    "ok": True,
                    "profile": profile,
                    "path": str(_config_path(profile)),
                    "profiles": {key: str(path) for key, path in CONFIG_PATHS.items()},
                    "rynnbot_path": str(_rynnbot_config_path(profile)),
                    "rynnbot_profiles": {key: str(path) for key, path in RYNNBOT_CONFIG_PATHS.items()},
                    "camera_master_path": str(_camera_master_config_path(profile)),
                    "camera_master_rynnbot_path": str(_camera_master_rynnbot_config_path(profile)),
                    "config": _load_config(profile),
                    "rynnbot": _load_rynnbot_config(profile),
                    "camera_master": _load_camera_master_config(profile),
                    "camera_master_rynnbot": _load_camera_master_rynnbot_config(profile),
                }
            )
        elif parsed.path == "/api/ports":
            ports = _serial_ports()
            self._json({"ok": True, "ports": ports, "recommended": _recommended_port(ports)})
        elif match := re.fullmatch(r"/api/camera/(\d+)/probe", parsed.path):
            self._json(_probe_camera(int(match.group(1))))
        elif parsed.path == "/api/camera-teleop/status":
            self._json({"ok": True, "status": _CAMERA_TELEOP_JOB.status()})
        elif parsed.path == "/api/camera-teleop/preview":
            try:
                self._send_bytes(_CAMERA_TELEOP_JOB.preview_jpeg(), "image/jpeg")
            except RuntimeError:
                self.send_response(204)
                self.end_headers()
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        try:
            if self.path == "/api/camera-teleop/start":
                self._json({"ok": True, "status": _CAMERA_TELEOP_JOB.start(self._payload())})
            elif self.path == "/api/camera-teleop/stop":
                self._json({"ok": True, "status": _CAMERA_TELEOP_JOB.stop()})
            elif self.path == "/api/config":
                payload = self._payload()
                profile = _profile_from_payload(payload)
                config = _load_config(profile)
                manifest = config.setdefault("manifest", {})
                robot = config.setdefault("components", {}).setdefault("robot", {})
                if "robot_id" in payload:
                    manifest["robot_id"] = str(payload["robot_id"] or _default_robot_id(profile))
                if "robot_name" in payload:
                    manifest["robot_name"] = str(payload["robot_name"] or _default_robot_name(profile))
                robot["enabled"] = True
                robot["mode"] = profile
                robot["embodiment_type"] = _embodiment_type(profile)
                robot["dof"] = _dof(profile)
                robot["port"] = str(payload.get("port") or robot.get("port") or "")
                robot["left_port"] = str(payload.get("left_port") or robot.get("left_port") or "")
                robot["right_port"] = str(payload.get("right_port") or robot.get("right_port") or "")
                robot["baudrate"] = 921600
                robot["homing_on_connect"] = False
                robot.setdefault("homing_timeout_s", 175.0)
                robot.setdefault("homing_settle_s", 8.0)
                if "front_camera" in payload and isinstance(payload["front_camera"], dict):
                    front_camera = config.setdefault("components", {}).setdefault("front_camera", {})
                    front_camera["enabled"] = bool(payload["front_camera"].get("enabled", True))
                    if "device_id" in payload["front_camera"]:
                        device_id = int(payload["front_camera"]["device_id"])
                        if device_id < 0:
                            raise ValueError("front_camera.device_id must be >= 0")
                        front_camera["device_id"] = device_id
                if "rynnbot" in payload and isinstance(payload["rynnbot"], dict):
                    rynnbot_config = _load_rynnbot_config(profile)
                    app = rynnbot_config.setdefault("app", {})
                    app["app_id"] = _rynnbot_app_id(profile)
                    for key in ("product_key", "device_name", "device_secret", "http_url"):
                        if key in payload["rynnbot"]:
                            app[key] = payload["rynnbot"][key]
                    _save_rynnbot_config(profile, rynnbot_config)
                if "camera_master" in payload and isinstance(payload["camera_master"], dict):
                    master_config = _load_camera_master_config(profile)
                    master_manifest = master_config.setdefault("manifest", {})
                    master_robot = master_config.setdefault("components", {}).setdefault("robot", {})
                    master_manifest["robot_id"] = _camera_master_robot_id(profile)
                    master_manifest["robot_name"] = _camera_master_robot_name(profile)
                    master_robot["enabled"] = True
                    master_robot["mode"] = profile
                    master_robot["embodiment_type"] = _embodiment_type(profile)
                    master_robot["dof"] = _dof(profile)
                    for key in (
                        "side",
                        "camera_index",
                        "fps",
                        "hand_landmarker_model",
                        "swap_handedness",
                        "camera_width",
                        "camera_height",
                        "min_hand_area",
                        "ema_alpha",
                        "min_cutoff",
                        "beta",
                        "d_cutoff",
                        "visual",
                    ):
                        if key in payload["camera_master"]:
                            master_robot[key] = payload["camera_master"][key]
                    _save_camera_master_config(profile, master_config)
                if "camera_master_rynnbot" in payload and isinstance(payload["camera_master_rynnbot"], dict):
                    master_app_config = _load_camera_master_rynnbot_config(profile)
                    master_app = master_app_config.setdefault("app", {})
                    master_app["app_id"] = _camera_master_rynnbot_app_id(profile)
                    master_app["role"] = "controller"
                    for key in ("product_key", "device_name", "device_secret", "http_url"):
                        if key in payload["camera_master_rynnbot"]:
                            master_app[key] = payload["camera_master_rynnbot"][key]
                    _save_camera_master_rynnbot_config(profile, master_app_config)
                _save_config(profile, config)
                self._json(
                    {
                        "ok": True,
                        "path": str(_config_path(profile)),
                        "rynnbot_path": str(_rynnbot_config_path(profile)),
                        "camera_master_path": str(_camera_master_config_path(profile)),
                        "camera_master_rynnbot_path": str(_camera_master_rynnbot_config_path(profile)),
                        "config": config,
                        "rynnbot": _load_rynnbot_config(profile),
                        "camera_master": _load_camera_master_config(profile),
                        "camera_master_rynnbot": _load_camera_master_rynnbot_config(profile),
                    }
                )
            elif self.path in {"/api/connect", "/api/read", "/api/homing", "/api/pose/open", "/api/pose/relaxed"}:
                if _CAMERA_TELEOP_JOB.is_active():
                    raise RuntimeError("stop camera gesture teleoperation before using another debug action")
                with HARDWARE_LOCK:
                    if self.path == "/api/connect":
                        self._json({"ok": True, "result": _test_connect(self._payload())})
                    elif self.path == "/api/read":
                        self._json({"ok": True, "result": _read_state(self._payload())})
                    elif self.path == "/api/homing":
                        self._json({"ok": True, "result": _homing(self._payload())})
                    elif self.path == "/api/pose/open":
                        self._json({"ok": True, "result": _send_pose(self._payload(), OPEN_POSE_DEG)})
                    else:
                        self._json({"ok": True, "result": _send_pose(self._payload(), RELAXED_POSE_DEG)})
            elif self.path == "/api/shutdown":
                _CAMERA_TELEOP_JOB.stop()
                self._json({"ok": True})
                threading.Thread(target=self.server.shutdown, daemon=True).start()
            else:
                self.send_error(404)
        except Exception as exc:
            self._json({"ok": False, "error": str(exc)}, status=500)

    def _payload(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length") or "0")
        if not length:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def _send_html(self, html: str) -> None:
        data = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _json(self, payload: dict[str, Any], *, status: int = 200) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format: str, *args: Any) -> None:
        return


class ConfigureServer(ThreadingHTTPServer):
    allow_reuse_address = True


def _profile_from_query(query: str) -> str:
    return _validate_profile((parse_qs(query).get("profile") or ["single"])[0])


def _profile_from_payload(payload: dict[str, Any]) -> str:
    return _validate_profile(payload.get("profile") or "single")


def _validate_profile(profile: Any) -> str:
    profile = str(profile or "single").strip().lower()
    if profile not in CONFIG_PATHS:
        raise ValueError("profile must be 'single' or 'dual'")
    return profile


def _config_path(profile: str) -> Path:
    return CONFIG_PATHS[profile]


def _rynnbot_config_path(profile: str) -> Path:
    return RYNNBOT_CONFIG_PATHS[profile]


def _camera_master_config_path(profile: str) -> Path:
    return CAMERA_MASTER_CONFIG_PATHS[profile]


def _camera_master_rynnbot_config_path(profile: str) -> Path:
    return CAMERA_MASTER_RYNNBOT_CONFIG_PATHS[profile]


def _load_config(profile: str) -> dict[str, Any]:
    return yaml.safe_load(_config_path(profile).read_text(encoding="utf-8")) or {}


def _save_config(profile: str, config: dict[str, Any]) -> None:
    text = yaml.safe_dump(config, sort_keys=False, allow_unicode=True)
    _config_path(profile).write_text(text, encoding="utf-8")


def _load_rynnbot_config(profile: str) -> dict[str, Any]:
    return yaml.safe_load(_rynnbot_config_path(profile).read_text(encoding="utf-8")) or {}


def _save_rynnbot_config(profile: str, config: dict[str, Any]) -> None:
    _rynnbot_config_path(profile).write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _load_camera_master_config(profile: str) -> dict[str, Any]:
    return yaml.safe_load(_camera_master_config_path(profile).read_text(encoding="utf-8")) or {}


def _save_camera_master_config(profile: str, config: dict[str, Any]) -> None:
    _camera_master_config_path(profile).write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _load_camera_master_rynnbot_config(profile: str) -> dict[str, Any]:
    return yaml.safe_load(_camera_master_rynnbot_config_path(profile).read_text(encoding="utf-8")) or {}


def _save_camera_master_rynnbot_config(profile: str, config: dict[str, Any]) -> None:
    _camera_master_rynnbot_config_path(profile).write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _new_camera_teleop_vision(profile: str, payload: dict[str, Any]) -> Any:
    from .vision_master import AeroHandVisionMaster

    config = _load_camera_master_config(profile)
    robot = config["components"]["robot"]
    camera_index = int(
        payload["camera_index"] if "camera_index" in payload else robot.get("camera_index", 0)
    )
    if camera_index < 0:
        raise ValueError("camera_index must be >= 0")
    return AeroHandVisionMaster(
        robot_id=str(config["manifest"].get("robot_id") or _camera_master_robot_id(profile)),
        mode=profile,
        side="auto" if profile == "single" else "right",
        camera_index=camera_index,
        fps=float(robot.get("fps", 30.0)),
        hand_landmarker_model=robot.get("hand_landmarker_model") or None,
        swap_handedness=bool(robot.get("swap_handedness", True)),
        camera_width=robot.get("camera_width"),
        camera_height=robot.get("camera_height"),
        min_hand_area=float(robot.get("min_hand_area", 0.005)),
        ema_alpha=float(robot.get("ema_alpha", 0.7)),
        min_cutoff=float(robot.get("min_cutoff", 5.0)),
        beta=float(robot.get("beta", 1.3)),
        d_cutoff=float(robot.get("d_cutoff", 1.4)),
        visual=str(robot.get("visual") or "none"),
    )


def _new_camera_teleop_controller(profile: str, payload: dict[str, Any]) -> Any:
    from .controller import AeroHandController

    config = _load_config(profile)
    robot = config["components"]["robot"]
    return AeroHandController(
        robot_id=str(config["manifest"].get("robot_id") or _default_robot_id(profile)),
        mode=profile,
        port=str(payload.get("port") or robot.get("port") or "") or None,
        left_port=str(payload.get("left_port") or robot.get("left_port") or "") or None,
        right_port=str(payload.get("right_port") or robot.get("right_port") or "") or None,
        baudrate=int(robot.get("baudrate", 921600)),
        homing_on_connect=False,
        homing_timeout_s=float(robot.get("homing_timeout_s", 175.0)),
        homing_settle_s=float(robot.get("homing_settle_s", 8.0)),
    )


def _rynnbot_app_id(profile: str) -> str:
    return "aero_hand_dual_rynnbot_app" if profile == "dual" else "aero_hand_single_rynnbot_app"


def _camera_master_robot_id(profile: str) -> str:
    return f"aero_hand_{profile}_hand_master"


def _camera_master_robot_name(profile: str) -> str:
    label = "Dual" if profile == "dual" else "Single"
    return f"Aero Hand {label}-Hand Vision Leader"


def _camera_master_rynnbot_app_id(profile: str) -> str:
    return f"aero_hand_{profile}_hand_master_rynnbot_app"


def _serial_ports() -> list[dict[str, str]]:
    ports: list[dict[str, str]] = []
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
        devices.extend(_windows_serial_devices())
    return devices


def _windows_serial_devices() -> list[str]:
    """Return COM ports currently mapped by Windows."""
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


def _dedupe_serial_ports(ports: list[dict[str, str]]) -> list[dict[str, str]]:
    selected: dict[str, dict[str, str]] = {}
    for port in ports:
        device = str(port.get("device") or "")
        if not device:
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


def _probe_camera(index: int) -> dict[str, Any]:
    try:
        import cv2
    except Exception as exc:
        return {"ok": False, "index": index, "error": f"OpenCV is not available: {exc}"}
    return _probe_camera_with_cv2(cv2, index)


def _probe_camera_with_cv2(cv2_module: Any, index: int) -> dict[str, Any]:
    capture = cv2_module.VideoCapture(index)
    try:
        if not capture.isOpened():
            return {"ok": False, "index": index, "opened": False}
        ok, frame = capture.read()
        if not ok or frame is None:
            return {
                "ok": False,
                "index": index,
                "opened": True,
                "error": "opened but no frame was returned",
            }
        height, width = frame.shape[:2]
        scale = min(1.0, 360.0 / float(max(width, height)))
        if scale < 1.0:
            frame = cv2_module.resize(frame, (int(width * scale), int(height * scale)))
        encoded, image = cv2_module.imencode(
            ".jpg",
            frame,
            [int(cv2_module.IMWRITE_JPEG_QUALITY), 75],
        )
        if not encoded:
            return {"ok": False, "index": index, "error": "failed to encode camera preview"}
        return {
            "ok": True,
            "camera": {
                "index": index,
                "width": int(width),
                "height": int(height),
                "image": "data:image/jpeg;base64," + base64.b64encode(image.tobytes()).decode("ascii"),
            },
        }
    except Exception as exc:
        return {"ok": False, "index": index, "error": str(exc)}
    finally:
        capture.release()


def _recommended_port(ports: list[dict[str, str]]) -> str | None:
    for port in ports:
        text = f"{port['device']} {port['description']} {port['hwid']}".lower()
        if "303a:1001" in text or "usb jtag/serial debug unit" in text or "espressif" in text:
            return port["device"]
    return None


def _default_robot_id(profile: str) -> str:
    return "aero_hand_dual" if profile == "dual" else "aero_hand_single"


def _default_robot_name(profile: str) -> str:
    return "Aero Hand Dual" if profile == "dual" else "Aero Hand Single"


def _embodiment_type(profile: str) -> str:
    return "dual_dexterous_hand" if profile == "dual" else "single_dexterous_hand"


def _dof(profile: str) -> int:
    return 14 if profile == "dual" else 7


def _runtime(payload: dict[str, Any]) -> tuple[str, dict[str, str], int]:
    profile = _profile_from_payload(payload)
    config = _load_config(profile)
    robot = config["components"]["robot"]
    ports = {
        "hand": str(payload.get("port") or robot.get("port") or "").strip(),
        "left": str(payload.get("left_port") or robot.get("left_port") or "").strip(),
        "right": str(payload.get("right_port") or robot.get("right_port") or "").strip(),
    }
    return profile, ports, 921600


def _active_hands(profile: str) -> tuple[str, ...]:
    return ("left", "right") if profile == "dual" else ("hand",)


def _new_hands(payload: dict[str, Any]) -> tuple[dict[str, Any], int]:
    from .aero_open_sdk.aero_hand import AeroHand

    profile, ports, baudrate = _runtime(payload)
    hands = {}
    try:
        for hand_side in _active_hands(profile):
            port = ports[hand_side] or None
            hand = AeroHand(port=port, baudrate=baudrate)
            _configure_debug_serial(hand)
            hands[hand_side] = hand
        return hands, baudrate
    except Exception:
        for hand in hands.values():
            hand.close()
        raise


def _configure_debug_serial(hand: Any) -> None:
    serial_port = getattr(hand, "ser", None)
    if serial_port is None:
        return
    serial_port.timeout = DEBUG_SERIAL_TIMEOUT_S
    serial_port.write_timeout = DEBUG_SERIAL_WRITE_TIMEOUT_S


def _test_connect(payload: dict[str, Any]) -> dict[str, Any]:
    hands, baudrate = _new_hands(payload)
    try:
        return {
            "baudrate": baudrate,
            "ports": {hand_side: hand.ser.port for hand_side, hand in hands.items()},
        }
    finally:
        _close_hands(hands)


def _read_state(payload: dict[str, Any]) -> dict[str, Any]:
    hands, _ = _new_hands(payload)
    try:
        return _read_state_from_hands(hands)
    finally:
        _close_hands(hands)


def _homing(payload: dict[str, Any]) -> dict[str, Any]:
    hands, _ = _new_hands(payload)
    try:
        for hand in hands.values():
            _send_homing_command(hand)
        robot = _load_config(_profile_from_payload(payload))["components"]["robot"]
        settle_s = float(robot.get("homing_settle_s") or 0)
        if settle_s:
            time.sleep(settle_s)
        return {"homed": list(hands), "settle_s": settle_s}
    finally:
        _close_hands(hands)


def _send_homing_command(hand: Any) -> None:
    from .aero_open_sdk.aero_hand import HOMING_MODE

    try:
        hand.ser.reset_input_buffer()
    except Exception:
        pass
    hand._send_data(HOMING_MODE)


def _send_pose(payload: dict[str, Any], pose_deg: list[float]) -> dict[str, Any]:
    hands, _ = _new_hands(payload)
    try:
        sent: list[float] = []
        for repeat_index in range(DEBUG_POSE_REPEATS):
            for hand in hands.values():
                hand.set_joint_positions(pose_deg)
            if repeat_index < DEBUG_POSE_REPEATS - 1:
                time.sleep(DEBUG_POSE_REPEAT_INTERVAL_S)
        if DEBUG_POSE_SETTLE_S:
            time.sleep(DEBUG_POSE_SETTLE_S)
        for _ in hands.values():
            sent.extend(pose_deg)
        return {"joint_names": _joint_names_for(hands), "sent_deg": sent, "repeat_count": DEBUG_POSE_REPEATS}
    finally:
        _close_hands(hands)


def _read_state_from_hands(hands: dict[str, Any]) -> dict[str, Any]:
    positions_deg: list[float] = []
    per_hand: dict[str, list[float]] = {}
    for hand_side, hand in hands.items():
        hand_positions = _read_state_from_hand(hand)
        per_hand[hand_side] = hand_positions
        positions_deg.extend(hand_positions)
    return {
        "joint_names": _joint_names_for(hands),
        "joint_positions_deg": positions_deg,
        "joint_positions_rad": [math.radians(v) for v in positions_deg],
        "per_hand_deg": per_hand,
    }


def _read_state_from_hand(hand: Any) -> list[float]:
    positions_deg = None
    for _ in range(5):
        positions_deg = hand.get_joint_positions_compact()
        if positions_deg is not None:
            break
        time.sleep(0.05)
    if positions_deg is None:
        raise RuntimeError("Aero Hand returned no joint positions")
    return [float(v) for v in positions_deg]


def _joint_names_for(hands: dict[str, Any]) -> list[str]:
    if len(hands) == 1:
        return list(JOINT_NAMES)
    return [f"{hand_side}_{name}" for hand_side in hands for name in JOINT_NAMES]


def _close_hands(hands: dict[str, Any]) -> None:
    for hand in hands.values():
        hand.close()


def _free_port(host: str, start: int) -> int:
    port = start
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            try:
                sock.bind((host, port))
            except OSError:
                port += 1
                continue
            return port


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Configure Aero Hand for RynnRCP.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=28411)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)

    port = _free_port(args.host, args.port)
    server = ConfigureServer((args.host, port), Handler)
    urls = browser_urls(args.host, port)
    print(f"Aero Hand configure UI Local: {urls[0]}")
    for url in urls[1:]:
        print(f"Aero Hand configure UI LAN:   {url}")
    if not args.no_browser:
        webbrowser.open(primary_browser_url(args.host, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Aero Hand configure UI.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
