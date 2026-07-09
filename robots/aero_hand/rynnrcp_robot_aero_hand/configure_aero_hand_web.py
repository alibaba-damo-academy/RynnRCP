#!/usr/bin/env python3
"""Browser-based Aero Hand configuration helper for RynnRCP."""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import yaml
from serial.tools import list_ports


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
    .status { margin-top: 12px; padding: 11px 12px; border: 1px solid #2b3b48; border-radius: 8px; background: #101820; color: #cbd5e1; }
    .status.good { border-color: #14532d; background: #102017; color: #86efac; }
    .status.warn { border-color: #713f12; background: #211b0c; color: #fde68a; }
    .status.bad { border-color: #7f1d1d; background: #241012; color: #fecdd3; }
    .value { color: #86efac; font-weight: 750; }
    .summary { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 12px; }
    .pill { border: 1px solid #33414f; border-radius: 999px; padding: 7px 10px; color: #cbd5e1; background: #0f151b; }
    .pill.done { border-color: #14532d; color: #86efac; }
    pre { margin: 0; min-height: 180px; max-height: 360px; overflow: auto; white-space: pre-wrap; word-break: break-word; background: #0b1015; border: 1px solid #26313b; border-radius: 8px; padding: 14px; color: #a7f3d0; }
    @media (max-width: 760px) { main { padding: 14px; } header, .grid, .port, .step { grid-template-columns: 1fr; display: grid; } }
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
    <div class="hint">单手构型是 7 维，不区分左右手；双手构型是 14 维，顺序为 left 7 维 + right 7 维。Baudrate 固定 921600，Server 启动不会自动 homing。</div>
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
    <div class="hint">这里配置当前机器人构型的本地 Server。端口通过下面的插拔识别写入，不需要手填。</div>
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
    <h2>3. RynnBot 云端配置</h2>
    <div class="hint">这些字段用于 `rynnrcp-rynnbot-app` 接入云端 RynnBot 工作流；单手和双手是不同云端设备，会分别保存到不同 RynnBot 配置文件。</div>
    <div class="grid">
      <div><label>Product Key</label><input id="product-key"></div>
      <div><label>Device Name</label><input id="device-name"></div>
      <div><label>Device Secret</label><input id="device-secret"></div>
      <div><label>HTTP URL</label><input id="http-url"></div>
    </div>
    <div class="row" style="margin-top:16px;">
      <button class="good" onclick="saveRynnBotConfig()">保存 RynnBot 配置</button>
    </div>
    <div id="rynnbot-status" class="status">等待加载 RynnBot 配置...</div>
  </section>

  <section>
    <h2>4. 真机调试</h2>
    <div class="hint">调试动作按当前选择的云端构型执行。单手连接一个端口；双手会同时连接左右手端口。</div>
    <div class="row">
      <button class="debug-action" onclick="testConnect()">只测试连接</button>
      <button class="debug-action" onclick="readState()">读取状态</button>
      <button class="warn debug-action" onclick="runHoming()">执行 homing</button>
      <button class="good debug-action" onclick="sendPose('open')">张开手</button>
      <button class="warn debug-action" onclick="sendPose('relaxed')">轻微握合</button>
    </div>
    <div id="debug-status" class="status">等待调试操作。</div>
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
    button.disabled = busy;
  }
}
async function api(path, options = {}) {
  const res = await fetch(path, options);
  const data = await res.json();
  if (!res.ok || data.ok === false) throw new Error(data.error || "request failed");
  return data;
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
  $("product-key").value = rb.product_key || "";
  $("device-name").value = rb.device_name || "";
  $("device-secret").value = rb.device_secret || "";
  $("http-url").value = rb.http_url || "https://robot-access.damo-academy.com";
  const profileText = currentProfile() === "dual" ? "双手 14 维" : "单手 7 维";
  setStatus("config-status", profileText + " 配置已加载：" + data.path, "good");
  setStatus("port-status", portSummary(), portSummary().includes("未设置") ? "warn" : "good");
  const cloudName = currentProfile() === "dual" ? "双手云端设备" : "单手云端设备";
  const cloudReady = hasCloudCreds(rb);
  setStatus("rynnbot-status", cloudReady ? cloudName + "配置已填写：" + data.rynnbot_path : cloudName + "配置未完整填写：" + data.rynnbot_path, cloudReady ? "good" : "warn");
  log("配置已加载: " + data.path);
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
    const data = await api("/api/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
    setStatus("rynnbot-status", "RynnBot 配置已保存：" + data.rynnbot_path, "good");
    log("RynnBot 配置已保存: " + data.rynnbot_path);
  } catch (err) {
    setStatus("rynnbot-status", "保存失败：" + err.message, "bad");
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
      setStatus("port-status", "没有检测到新端口。确认这只手是在记录基线之后插入的。", "warn");
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
    setStatus("debug-status", "homing 完成，读取到 " + data.result.joint_positions_deg.length + " 维。", "good");
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
loadConfig().then(updatePortFields).then(updateDetectTarget).then(updateSetupCopy).catch(err => log("初始化失败: " + err.message));
</script>
</body>
</html>
"""


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
                    "config": _load_config(profile),
                    "rynnbot": _load_rynnbot_config(profile),
                }
            )
        elif parsed.path == "/api/ports":
            ports = _serial_ports()
            self._json({"ok": True, "ports": ports, "recommended": _recommended_port(ports)})
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        try:
            if self.path == "/api/config":
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
                if "rynnbot" in payload and isinstance(payload["rynnbot"], dict):
                    rynnbot_config = _load_rynnbot_config(profile)
                    app = rynnbot_config.setdefault("app", {})
                    app["app_id"] = _rynnbot_app_id(profile)
                    for key in ("product_key", "device_name", "device_secret", "http_url"):
                        if key in payload["rynnbot"]:
                            app[key] = payload["rynnbot"][key]
                    _save_rynnbot_config(profile, rynnbot_config)
                _save_config(profile, config)
                self._json(
                    {
                        "ok": True,
                        "path": str(_config_path(profile)),
                        "rynnbot_path": str(_rynnbot_config_path(profile)),
                        "config": config,
                        "rynnbot": _load_rynnbot_config(profile),
                    }
                )
            elif self.path in {"/api/connect", "/api/read", "/api/homing", "/api/pose/open", "/api/pose/relaxed"}:
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


def _rynnbot_app_id(profile: str) -> str:
    return "aero_hand_dual_rynnbot_app" if profile == "dual" else "aero_hand_single_rynnbot_app"


def _serial_ports() -> list[dict[str, str]]:
    return [
        {"device": p.device, "description": p.description or "", "hwid": p.hwid or ""}
        for p in list_ports.comports()
    ]


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
            hand.send_homing()
        time.sleep(float(_load_config(_profile_from_payload(payload))["components"]["robot"].get("homing_settle_s") or 0))
        return _read_state_from_hands(hands)
    finally:
        _close_hands(hands)


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
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=28411)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)

    port = _free_port(args.host, args.port)
    server = ConfigureServer((args.host, port), Handler)
    url = f"http://{args.host}:{port}"
    print(f"Aero Hand configure UI: {url}")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Aero Hand configure UI.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
