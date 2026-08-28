"""Web app for inspecting RCP observations and testing model inference."""

from __future__ import annotations

import argparse
import base64
import gzip
import io
import json
import logging
import math
import sys
import threading
import time
import uuid
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlparse
from urllib.request import Request, urlopen

import numpy as np
from PIL import Image

from rynnrcp.interface import RcpProtocolClient
from rynnrcp.server import RynnRCPServer
from rynnrcp.utils.logging import configure_logging, resolve_log_run_id, set_log_context
from rynnrcp.utils.user_paths import app_root, logs_dir

from .proto_codec import RynnProtoCodec

logger = logging.getLogger(__name__)
HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RCP 本地模型调试</title>
  <style>
    :root { color-scheme: dark; --bg:#0b1014; --card:#131b21; --card2:#18232b; --line:#2b3944; --text:#edf3f7; --muted:#9ba8b3; --blue:#7dd3fc; --green:#86efac; --red:#fb7185; }
    * { box-sizing:border-box; }
    body { margin:0; font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; color:var(--text); background:var(--bg); }
    main { max-width:1440px; margin:0 auto; padding:28px 20px 56px; }
    h1 { margin:0 0 6px; font-size:28px; color:var(--blue); }
    h2 { margin:0 0 14px; font-size:19px; color:#dae5ec; }
    h3 { margin:0; font-size:14px; color:#eef5f9; }
    p { line-height:1.6; }
    .muted { color:var(--muted); }
    .steps { display:grid; gap:16px; margin-top:18px; }
    .card { padding:18px; border:1px solid var(--line); border-radius:10px; background:var(--card); }
    .subcard { margin-top:14px; padding:14px; border:1px solid var(--line); border-radius:9px; background:#0f171d; }
    .subcard > h3 { margin-bottom:12px; }
    .telemetry-grid { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:14px; }
    .row { display:flex; gap:10px; align-items:end; flex-wrap:wrap; }
    label { display:grid; gap:6px; flex:1; min-width:180px; font-size:13px; color:var(--muted); }
    .row label { min-width:min(100%, 180px); }
    input,select,textarea,button { font:inherit; }
    input,select,textarea { width:100%; padding:9px 10px; border:1px solid #394754; border-radius:8px; background:#0c1217; color:var(--text); }
    textarea { min-height:86px; resize:vertical; }
    button { padding:9px 14px; border:0; border-radius:8px; cursor:pointer; background:var(--blue); color:#061018; font-weight:700; }
    button.secondary { background:#a7b2bd; color:#17202a; }
    button.execute { background:var(--green); }
    button.stop { background:var(--red); color:#26060d; }
    button:disabled { cursor:not-allowed; opacity:.5; }
    .status { margin-top:12px; padding:10px 12px; border:1px solid #263745; border-radius:8px; background:#101a22; color:#bae6fd; white-space:pre-wrap; }
    .status.error { background:#2a151b; color:var(--red); }
    pre { margin:0; max-height:360px; overflow:auto; padding:13px; border:1px solid #26323d; border-radius:8px; background:#080d11; color:#a7f3d0; font-size:12px; line-height:1.5; }
    .images { display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:12px; }
    figure { margin:0; padding:10px; border:1px solid var(--line); border-radius:9px; background:var(--card2); }
    figure img { display:block; width:100%; min-height:150px; max-height:300px; object-fit:contain; background:#111; border-radius:6px; }
    figcaption { margin-top:7px; font-size:12px; color:var(--muted); overflow-wrap:anywhere; }
    .chart-box { padding:10px; border:1px solid #26323d; border-radius:8px; background:#080d11; }
    .state-charts { display:grid; grid-template-columns:repeat(auto-fit,minmax(360px,1fr)); gap:10px; }
    .state-chart-card { padding:10px; border:1px solid var(--line); border-radius:9px; background:var(--card2); }
    .state-chart-card > .key-tags { margin-bottom:8px; }
    canvas.chart { display:block; width:100%; height:220px; }
    .legend { display:flex; flex-wrap:wrap; gap:6px 12px; margin-top:8px; color:var(--muted); font-size:11px; }
    .legend-item { display:flex; align-items:center; gap:5px; }
    .legend-dot { width:8px; height:8px; border-radius:999px; }
    details { margin-top:10px; }
    summary { cursor:pointer; color:var(--muted); font-size:12px; }
    details pre { margin-top:8px; }
    .input-picker { margin-bottom:16px; padding:13px; border:1px solid var(--line); border-radius:9px; background:#0f171d; }
    .picker-head { display:flex; align-items:center; justify-content:space-between; gap:12px; margin-bottom:4px; }
    .key-list { display:flex; flex-wrap:wrap; gap:8px; margin-top:10px; }
    .key-choice { display:flex; flex:0 1 auto; min-width:0; align-items:center; gap:7px; padding:7px 9px; border:1px solid #394754; border-radius:8px; background:var(--card2); cursor:pointer; }
    .key-choice:has(input:checked) { border-color:var(--blue); box-shadow:0 0 0 1px var(--blue); }
    .key-choice input { width:auto; margin:0; accent-color:var(--blue); }
    .key-tags { display:flex; flex-wrap:wrap; gap:4px; }
    .key-tag { padding:2px 7px; border:1px solid #3a4652; border-radius:999px; color:#cbd6de; font:12px ui-monospace,SFMono-Regular,Menlo,monospace; }
    .key-tag:first-child { color:var(--blue); border-color:#31596b; }
    .key-tag:last-child { color:var(--green); border-color:#365f4a; }
    .count-badge { border:1px solid #3a4652; border-radius:999px; padding:3px 8px; color:#cbd6de; font-size:12px; white-space:nowrap; }
    @media (max-width:900px) { .telemetry-grid { grid-template-columns:1fr; } }
  </style>
</head>
<body>
<main>
  <h1>RCP 本地模型调试</h1>
  <p class="muted">查看状态和图像，设置总执行步数后开始闭环推理；运行期间可随时停止。</p>

  <div class="steps">
    <section class="card">
      <h2>1. 连接机器人</h2>
      <p class="muted">选择 Server 配置并确认机器人、关节和相机连接成功。</p>
      <div class="row">
        <label>Server 配置文件
          <input id="serverConfig" value="__SERVER_CONFIG__">
        </label>
        <button id="startServer">重新启动 Server</button>
        <button id="stopServer" class="secondary">停止 Server</button>
      </div>
      <div id="serverStatus" class="status">正在读取 Server 状态…</div>
    </section>

    <section class="card">
      <h2>2. 配置推理</h2>
      <p class="muted">选择模型输入，并填写推理服务和任务信息。</p>
      <div class="input-picker">
        <div class="picker-head">
          <h3>推理输入</h3>
          <span id="selectedKeyCount" class="count-badge">已选 0</span>
        </div>
        <div class="muted">点选本次推理需要的状态和图像。</div>
        <div id="inferenceKeys" class="key-list">
          <span class="muted">正在读取 Observation…</span>
        </div>
      </div>
      <div class="row">
        <label>请求协议
          <select id="protocol">
            <option value="protobuf">Protobuf（RynnBot）</option>
            <option value="json">JSON</option>
          </select>
        </label>
        <label>推理 URL
          <input id="modelUrl" placeholder="https://your-model-service.example:PORT/api/predict/model">
        </label>
        <label>API Key
          <input id="apiKey" type="password" autocomplete="off" placeholder="可留空">
        </label>
        <label>鉴权 Header
          <input id="authHeader" value="Authorization">
        </label>
        <label>Key 前缀
          <input id="authPrefix" placeholder="PAI-EAS 留空">
        </label>
      </div>
      <div class="row" style="margin-top:10px">
        <label>Prompt
          <textarea id="prompt" placeholder="输入任务描述"></textarea>
        </label>
        <label style="max-width:180px">超时（秒）
          <input id="timeout" type="number" value="60" min="1">
        </label>
      </div>
    </section>

    <section class="card">
      <h2>3. 开始推理并查看结果</h2>
      <p class="muted">设置运行参数后开始闭环推理；这里持续展示 Observation、执行进度和最新 Action。</p>

      <div class="subcard">
        <h3>运行控制</h3>
        <div class="row">
          <label style="max-width:180px">总执行步数
            <input id="totalSteps" type="number" value="100" min="1" step="1">
          </label>
          <label>目标 Action
            <select id="actionName"></select>
          </label>
          <label style="max-width:180px">默认帧率
            <input id="frameRate" type="number" value="30" min="0.1" step="0.1">
          </label>
          <button id="startInference" class="execute">开始推理并执行</button>
          <button id="stopInference" class="stop" disabled>停止推理</button>
          <button id="goHome" class="secondary" hidden>回到 Home</button>
        </div>
        <div id="inferStatus" class="status">填写配置后开始推理。</div>
        <div id="actionStatus" class="status">已执行 0/0 step。</div>
      </div>

      <div class="telemetry-grid">
        <div class="subcard">
          <h3>Observation 状态 · 最近 120 step</h3>
          <div id="stateCharts" class="state-charts">
            <span class="muted">正在读取状态 Observation…</span>
          </div>
          <details>
            <summary>查看原始状态</summary>
            <pre id="stateView">连接后显示状态。</pre>
          </details>
        </div>

        <div class="subcard">
          <h3>Action 历史 · 最近 120 step</h3>
          <div class="chart-box">
            <canvas id="actionChart" class="chart"></canvas>
            <div id="actionLegend" class="legend"></div>
          </div>
          <details>
            <summary>查看最新原始 Action</summary>
            <pre id="actionView">开始推理后显示最新 Action。</pre>
          </details>
        </div>
      </div>

      <div class="subcard">
        <h3>实时图像</h3>
        <div id="imageView" class="images">
          <span class="muted">启动 Server 后显示图像。</span>
        </div>
      </div>
    </section>
  </div>
</main>
<script>
const $ = id => document.getElementById(id);
let stateTimer = null;
let stateObservationNames = [];
let imageGeneration = 0;
let lastAction = null;
let runActive = false;
let runTimer = null;
let lastActionRound = 0;
let homeActionName = "";
let selectedInferenceKeys = new Set();
const stateHistory = new Map();
const stateChartElements = new Map();
const actionHistory = new Map();
const chartColors = ["#7dd3fc","#86efac","#fde047","#c4b5fd","#fb7185","#fdba74","#67e8f9","#f9a8d4"];

function setStatus(id, text, error=false) {
  const el = $(id);
  el.textContent = text;
  el.classList.toggle("error", error);
}

function errorMessage(error) {
  return error instanceof Error ? error.message : String(error);
}

async function api(path, body={}) {
  const response = await fetch(path, {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body),
  });
  const data = await response.json();
  if (!response.ok || !data.success) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function appendKeyTags(root, name) {
  root.classList.add("key-tags");
  for (const part of String(name).split(".")) {
    const tag = document.createElement("span");
    tag.className = "key-tag";
    tag.textContent = part;
    root.append(tag);
  }
}

function collectNumeric(prefix, value, output) {
  if (Object.keys(output).length >= 40) return;
  if (typeof value === "number" && Number.isFinite(value)) {
    output[prefix] = value;
  } else if (Array.isArray(value)) {
    value.forEach((item, index) => collectNumeric(`${prefix}[${index}]`, item, output));
  } else if (value && typeof value === "object") {
    for (const [key, child] of Object.entries(value)) {
      collectNumeric(prefix ? `${prefix} · ${key}` : key, child, output);
    }
  }
}

function drawChart(canvas, legend, series) {
  const width = Math.max(320, Math.floor(canvas.getBoundingClientRect().width));
  const height = 220;
  const scale = window.devicePixelRatio || 1;
  canvas.width = width * scale;
  canvas.height = height * scale;
  const ctx = canvas.getContext("2d");
  ctx.scale(scale, scale);
  ctx.clearRect(0, 0, width, height);
  const entries = Object.entries(series).filter(([, values]) => values.length);
  legend.replaceChildren();
  if (!entries.length) {
    ctx.fillStyle = "#9ba8b3";
    ctx.font = "13px sans-serif";
    ctx.fillText("等待数值数据…", 14, 24);
    return;
  }
  const allValues = entries.flatMap(([, values]) => values).filter(Number.isFinite);
  let min = Math.min(...allValues);
  let max = Math.max(...allValues);
  if (min === max) { min -= 1; max += 1; }
  const padding = {left:48, right:12, top:12, bottom:24};
  const plotWidth = width - padding.left - padding.right;
  const plotHeight = height - padding.top - padding.bottom;
  ctx.strokeStyle = "#26323d";
  ctx.fillStyle = "#9ba8b3";
  ctx.font = "10px ui-monospace, monospace";
  for (let line = 0; line <= 4; line++) {
    const y = padding.top + plotHeight * line / 4;
    ctx.beginPath();
    ctx.moveTo(padding.left, y);
    ctx.lineTo(width - padding.right, y);
    ctx.stroke();
    const value = max - (max - min) * line / 4;
    ctx.fillText(value.toFixed(2), 4, y + 3);
  }
  const maxLength = Math.max(...entries.map(([, values]) => values.length));
  entries.forEach(([label, values], index) => {
    const color = chartColors[index % chartColors.length];
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    values.forEach((value, point) => {
      const x = padding.left + plotWidth * point / Math.max(1, maxLength - 1);
      const y = padding.top + (max - value) / (max - min) * plotHeight;
      if (point === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    const item = document.createElement("span");
    item.className = "legend-item";
    const dot = document.createElement("span");
    dot.className = "legend-dot";
    dot.style.background = color;
    const text = document.createElement("span");
    text.textContent = label;
    item.append(dot, text);
    legend.append(item);
  });
}

function renderStateObservations(observations) {
  const states = {};
  for (const item of observations) {
    states[item.name] = item.value;
    const name = String(item.name || "");
    const sample = {};
    collectNumeric("", item.value, sample);
    if (!stateHistory.has(name)) stateHistory.set(name, new Map());
    const history = stateHistory.get(name);
    for (const [label, value] of Object.entries(sample)) {
      if (!history.has(label)) history.set(label, []);
      const values = history.get(label);
      values.push(value);
      if (values.length > 120) values.shift();
    }
    const elements = ensureStateChart(name);
    drawChart(elements.canvas, elements.legend, Object.fromEntries(history));
  }
  $("stateView").textContent = JSON.stringify(states, null, 2);
}

function ensureStateChart(name) {
  if (stateChartElements.has(name)) return stateChartElements.get(name);
  const card = document.createElement("div");
  card.className = "state-chart-card";
  const title = document.createElement("div");
  appendKeyTags(title, name);
  const box = document.createElement("div");
  box.className = "chart-box";
  const canvas = document.createElement("canvas");
  canvas.className = "chart";
  const legend = document.createElement("div");
  legend.className = "legend";
  box.append(canvas, legend);
  card.append(title, box);
  $("stateCharts").append(card);
  const elements = {canvas, legend};
  stateChartElements.set(name, elements);
  return elements;
}

function resetStateCharts() {
  stateHistory.clear();
  stateChartElements.clear();
  $("stateCharts").replaceChildren();
}

function renderImageStreams(observations) {
  imageGeneration += 1;
  const generation = imageGeneration;
  const root = $("imageView");
  root.replaceChildren();
  const images = observations.filter(item => item.type === "image");
  if (!images.length) {
    const empty = document.createElement("span");
    empty.className = "muted";
    empty.textContent = "连接提供图像 Observation 的 Server 后在此显示。";
    root.append(empty);
    return;
  }
  for (const observation of images) {
    const figure = document.createElement("figure");
    const image = document.createElement("img");
    image.alt = String(observation.name);
    const caption = document.createElement("figcaption");
    appendKeyTags(caption, observation.name);
    figure.append(image, caption);
    root.append(figure);
    const refresh = () => {
      if (generation !== imageGeneration) return;
      image.onload = () => setTimeout(refresh, 100);
      image.onerror = () => setTimeout(refresh, 500);
      image.src = `/api/image?name=${encodeURIComponent(observation.name)}&_=${Date.now()}`;
    };
    refresh();
  }
}

function renderActionChart(action) {
  for (const frame of action.frames || []) {
    const positions = frame.joint_positions || [];
    positions.forEach((value, index) => {
      const label = `joint_positions[${index}]`;
      if (!actionHistory.has(label)) actionHistory.set(label, []);
      const values = actionHistory.get(label);
      values.push(Number(value));
      if (values.length > 120) values.shift();
    });
  }
  drawChart(
    $("actionChart"),
    $("actionLegend"),
    Object.fromEntries(actionHistory),
  );
}

function resetActionChart() {
  actionHistory.clear();
  drawChart($("actionChart"), $("actionLegend"), {});
  $("actionView").textContent = "开始推理后显示最新 Action。";
}

function applyServerInfo(data) {
  setStatus("serverStatus", `已连接 ${data.robot_name}，Observation ${data.observation_count} 个。`);
  $("actionName").innerHTML = data.actions.map(name => `<option>${name}</option>`).join("");
  homeActionName = String(data.home_action || "");
  $("goHome").hidden = !homeActionName;
  renderInferenceKeys(data.observations || []);
  stateObservationNames = (data.observations || []).filter(item => item.type !== "image").map(item => item.name);
  resetStateCharts();
  resetActionChart();
  renderImageStreams(data.observations || []);
  startStatePolling();
}

function renderInferenceKeys(observations) {
  selectedInferenceKeys = new Set();
  const root = $("inferenceKeys");
  root.replaceChildren();
  for (const observation of observations) {
    const name = String(observation.name || "");
    if (!name) continue;
    const choice = document.createElement("label");
    choice.className = "key-choice";
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.onchange = () => {
      if (checkbox.checked) selectedInferenceKeys.add(name);
      else selectedInferenceKeys.delete(name);
      updateSelectedKeyCount();
    };
    const tags = document.createElement("span");
    appendKeyTags(tags, name);
    choice.append(checkbox, tags);
    root.append(choice);
  }
  if (!root.children.length) {
    const empty = document.createElement("span");
    empty.className = "muted";
    empty.textContent = "连接 Server 后在此选择推理所需的 Observation。";
    root.append(empty);
  }
  updateSelectedKeyCount();
}

function updateSelectedKeyCount() {
  $("selectedKeyCount").textContent = `已选 ${selectedInferenceKeys.size}`;
  $("startInference").disabled = runActive || selectedInferenceKeys.size === 0;
}

function startStatePolling() {
  clearTimeout(stateTimer);
  stateTimer = null;
  if (stateObservationNames.length) refreshStateObservations();
  else {
    const empty = document.createElement("span");
    empty.className = "muted";
    empty.textContent = "连接提供状态 Observation 的 Server 后在此显示。";
    $("stateCharts").append(empty);
  }
}

async function refreshStateObservations() {
  try {
    const data = await api("/api/observations", {names:stateObservationNames});
    renderStateObservations(data.observations);
  } catch (error) {
    setStatus("serverStatus", errorMessage(error), true);
  } finally {
    if (stateObservationNames.length) {
      stateTimer = setTimeout(refreshStateObservations, 100);
    }
  }
}

$("startServer").onclick = async () => {
  try {
    const data = await api("/api/server/start", {config_path:$("serverConfig").value});
    applyServerInfo(data);
  } catch (error) {
    setStatus("serverStatus", errorMessage(error), true);
  }
};

$("stopServer").onclick = async () => {
  clearTimeout(stateTimer);
  stateTimer = null;
  stateObservationNames = [];
  imageGeneration += 1;
  try {
    await api("/api/server/stop");
    setStatus("serverStatus", "Server 已停止。");
    homeActionName = "";
    $("goHome").hidden = true;
    renderInferenceKeys([]);
  } catch (error) {
    setStatus("serverStatus", errorMessage(error), true);
  }
};

function inferenceConfig() {
  return {
    url:$("modelUrl").value,
    protocol:$("protocol").value,
    api_key:$("apiKey").value,
    auth_header:$("authHeader").value,
    auth_prefix:$("authPrefix").value,
    prompt:$("prompt").value,
    timeout_s:Number($("timeout").value),
    total_steps:Number($("totalSteps").value),
    observation_names:Array.from(selectedInferenceKeys),
    action_name:$("actionName").value,
    frame_rate:Number($("frameRate").value),
  };
}

function applyRunStatus(data) {
  runActive = Boolean(data.running);
  $("startInference").disabled = runActive || selectedInferenceKeys.size === 0;
  $("stopInference").disabled = !runActive;
  const elapsed = data.last_elapsed_ms == null ? "" : ` 最近一次推理 ${Number(data.last_elapsed_ms).toFixed(1)} ms。`;
  setStatus("inferStatus", `${data.message}${elapsed}`, Boolean(data.error));
  setStatus("actionStatus", `已执行 ${data.completed_steps}/${data.total_steps} step，第 ${data.round} 轮。`, Boolean(data.error));
  if (data.last_action && data.round !== lastActionRound) {
    lastActionRound = data.round;
    lastAction = data.last_action;
    $("actionView").textContent = JSON.stringify(lastAction, null, 2);
    renderActionChart(lastAction);
  }
  clearTimeout(runTimer);
  runTimer = null;
  if (runActive) runTimer = setTimeout(refreshRunStatus, 200);
}

async function refreshRunStatus() {
  try {
    applyRunStatus(await api("/api/inference/status"));
  } catch (error) {
    setStatus("inferStatus", errorMessage(error), true);
  }
}

$("startInference").onclick = async () => {
  const config = inferenceConfig();
  if (!Number.isInteger(config.total_steps) || config.total_steps <= 0) {
    setStatus("inferStatus", "总执行步数必须是大于 0 的整数。", true);
    return;
  }
  if (!confirm(`将自动推理并向真机执行最多 ${config.total_steps} step Action，是否开始？`)) return;
  $("startInference").disabled = true;
  lastActionRound = 0;
  lastAction = null;
  $("actionView").textContent = "等待第一轮推理结果。";
  setStatus("inferStatus", "正在启动推理…");
  try {
    applyRunStatus(await api("/api/inference/start", config));
  } catch (error) {
    runActive = false;
    updateSelectedKeyCount();
    setStatus("inferStatus", errorMessage(error), true);
  }
};

$("stopInference").onclick = async () => {
  $("stopInference").disabled = true;
  setStatus("inferStatus", "正在停止推理和当前 Action…");
  try {
    applyRunStatus(await api("/api/inference/stop"));
  } catch (error) {
    setStatus("inferStatus", errorMessage(error), true);
  }
};

$("goHome").onclick = async () => {
  if (!homeActionName) return;
  if (!confirm(`将停止当前推理并执行 ${homeActionName}，是否继续？`)) return;
  $("goHome").disabled = true;
  setStatus("inferStatus", "正在停止当前动作并回到 Home…");
  try {
    const data = await api("/api/action/home");
    applyRunStatus(data.inference);
    setStatus("actionStatus", `已执行 ${data.action_name}。`);
  } catch (error) {
    setStatus("actionStatus", errorMessage(error), true);
  } finally {
    $("goHome").disabled = false;
  }
};

api("/api/server/status")
  .then(applyServerInfo)
  .catch(error => setStatus("serverStatus", errorMessage(error), true));
refreshRunStatus();
</script>
</body>
</html>
"""


class ModelDebugApp:
    def __init__(self, server_config: str) -> None:
        self.server_config = str(Path(server_config).expanduser())
        self.server: RynnRCPServer | None = None
        self.client: RcpProtocolClient | None = None
        self.manifest: Any | None = None
        self.observations: list[dict[str, Any]] = []
        self.lock = threading.RLock()
        self.run_lock = threading.RLock()
        self.action_lock = threading.Lock()
        self.run_stop_event = threading.Event()
        self.run_thread: threading.Thread | None = None
        self.run_state = self._new_run_state()

    def start_server(self, config_path: str | None = None) -> dict[str, Any]:
        with self.lock:
            self.stop_server()
            self.server_config = str(
                Path(config_path or self.server_config).expanduser()
            )
            server = RynnRCPServer(self.server_config)
            client: RcpProtocolClient | None = None
            try:
                server.start()
                client = RcpProtocolClient.connect(f"127.0.0.1:{server.bound_port}")
                manifest = client.get_manifest()
            except Exception as exc:
                if client is not None:
                    client.close()
                server.stop()
                raise RuntimeError(
                    f"使用配置 {self.server_config} 启动 RCP Server 失败："
                    f"{_error_detail(exc)}"
                ) from exc
            self.server = server
            self.client = client
            self.manifest = manifest
            return self._server_info(manifest)

    def server_status(self) -> dict[str, Any]:
        with self.lock:
            _, manifest = self._require_connection()
            return self._server_info(manifest)

    @staticmethod
    def _server_info(manifest: Any) -> dict[str, Any]:
        actions = [
            str(item["name"])
            for item in manifest.actions
            if item.get("type") == "joint_position"
        ]
        return {
            "robot_id": manifest.robot_id,
            "robot_name": manifest.robot_name,
            "observation_count": len(manifest.observations),
            "observations": [
                {
                    "name": str(item["name"]),
                    "type": str(item.get("type") or ""),
                }
                for item in manifest.observations
                if item.get("name")
            ],
            "actions": actions,
            "home_action": ModelDebugApp._home_action_name(manifest),
        }

    @staticmethod
    def _home_action_name(manifest: Any) -> str | None:
        for item in manifest.actions:
            name = str(item.get("name") or "")
            short_name = name.rsplit(".", 1)[-1].lower()
            if short_name in {"home", "go_home"}:
                return name
        return None

    def stop_server(self) -> dict[str, Any]:
        self.stop_inference()
        with self.lock:
            if self.client is not None:
                self.client.close()
            if self.server is not None:
                self.server.stop()
            self.client = None
            self.server = None
            self.manifest = None
            self.observations = []
            return {"stopped": True}

    @staticmethod
    def _new_run_state() -> dict[str, Any]:
        return {
            "running": False,
            "stopping": False,
            "phase": "idle",
            "message": "填写配置后开始推理。",
            "total_steps": 0,
            "completed_steps": 0,
            "round": 0,
            "last_elapsed_ms": None,
            "last_action": None,
            "error": None,
        }

    def start_inference(self, config: dict[str, Any]) -> dict[str, Any]:
        total_steps = int(config.get("total_steps") or 0)
        if total_steps <= 0:
            raise ValueError("总步数必须大于 0")
        names = config.get("observation_names")
        if not isinstance(names, list) or not names or not all(
            isinstance(name, str) and name for name in names
        ):
            raise ValueError("请先选择推理输入")
        if not str(config.get("action_name") or "").strip():
            raise ValueError("请选择目标 Action")
        with self.lock:
            self._require_connection()
        with self.run_lock:
            if self.run_thread is not None and self.run_thread.is_alive():
                raise RuntimeError("推理任务正在运行")
            self.run_stop_event.clear()
            self.run_state = {
                **self._new_run_state(),
                "running": True,
                "phase": "starting",
                "message": f"准备执行 0/{total_steps} step。",
                "total_steps": total_steps,
            }
            run_config = dict(config)
            run_config["total_steps"] = total_steps
            thread = threading.Thread(
                target=self._inference_loop,
                args=(run_config,),
                name="model-debug-inference",
                daemon=True,
            )
            self.run_thread = thread
            thread.start()
            return dict(self.run_state)

    def stop_inference(self) -> dict[str, Any]:
        self.run_stop_event.set()
        with self.run_lock:
            running = bool(
                self.run_thread is not None and self.run_thread.is_alive()
            )
            if running:
                self.run_state["stopping"] = True
                self.run_state["phase"] = "stopping"
                self.run_state["message"] = "正在停止推理和当前 Action…"
        with self.lock:
            client = self.client
        if running and client is not None:
            try:
                response = client.stop_action(
                    reason="model_debug_stop",
                    timeout_ms=1000,
                )
                if not response.ok:
                    raise RuntimeError(
                        response.message or "Server 未返回具体原因"
                    )
            except Exception as exc:
                logger.warning(
                    "[ModelDebug][STOP_ACTION_FAILED] error=%s; "
                    "use the robot emergency stop and inspect the server logs",
                    exc,
                    exc_info=True,
                )
                with self.run_lock:
                    detail = (
                        f"停止当前 Action 失败：{_error_detail(exc)}。"
                        "请使用急停并检查 Server 日志。"
                    )
                    self.run_state["error"] = detail
                    self.run_state["phase"] = "stop_error"
                    self.run_state["message"] = detail
        return self.inference_status()

    def inference_status(self) -> dict[str, Any]:
        with self.run_lock:
            return dict(self.run_state)

    def _inference_loop(self, config: dict[str, Any]) -> None:
        total_steps = int(config["total_steps"])
        stopped = False
        try:
            while True:
                with self.run_lock:
                    completed = int(self.run_state["completed_steps"])
                if completed >= total_steps:
                    break
                if self.run_stop_event.is_set():
                    stopped = True
                    break

                with self.run_lock:
                    self.run_state["phase"] = "inferring"
                    self.run_state["message"] = (
                        f"正在推理，已执行 {completed}/{total_steps} step。"
                    )
                    round_number = int(self.run_state["round"]) + 1
                try:
                    result = self.infer(config)
                except Exception as exc:
                    raise RuntimeError(
                        f"第 {round_number} 轮推理失败"
                        f"（已执行 {completed}/{total_steps} step）："
                        f"{_error_detail(exc)}"
                    ) from exc
                action = dict(result["action"])
                remaining = total_steps - completed
                frames = list(action["frames"][:remaining])
                action["frames"] = frames
                with self.run_lock:
                    self.run_state["round"] = int(self.run_state["round"]) + 1
                    self.run_state["last_elapsed_ms"] = result["elapsed_ms"]
                    self.run_state["last_action"] = action
                if self.run_stop_event.is_set():
                    stopped = True
                    break
                if not frames:
                    raise RuntimeError("模型返回的 Action frames 为空")

                with self.run_lock:
                    self.run_state["phase"] = "executing"
                    self.run_state["message"] = (
                        f"正在执行 {len(frames)} step，"
                        f"当前进度 {completed}/{total_steps}。"
                    )
                try:
                    execution = self.execute_action(
                        str(action["name"]),
                        frames,
                        float(action["frame_rate"]),
                    )
                except Exception as exc:
                    raise RuntimeError(
                        f"第 {round_number} 轮 Action 执行失败"
                        f"（目标 {action['name']}，{len(frames)} step，"
                        f"已累计 {completed}/{total_steps} step）："
                        f"{_error_detail(exc)}"
                    ) from exc
                accepted = int(execution["accepted_frames"])
                if accepted <= 0:
                    raise RuntimeError("Server 接收的 Action step 数为 0")
                with self.run_lock:
                    self.run_state["completed_steps"] = min(
                        total_steps,
                        completed + accepted,
                    )
                if self.run_stop_event.is_set():
                    stopped = True
                    break
        except Exception as exc:
            if self.run_stop_event.is_set():
                stopped = True
            else:
                logger.exception(
                    "[ModelDebug][INFERENCE_FAILED] completed_steps=%d total_steps=%d "
                    "error=%s; inspect the model response and ActionService logs",
                    int(self.run_state["completed_steps"]),
                    total_steps,
                    exc,
                )
                with self.run_lock:
                    self.run_state["error"] = str(exc)
                    self.run_state["phase"] = "error"
                    self.run_state["message"] = f"运行失败：{_error_detail(exc)}"
        finally:
            with self.run_lock:
                completed = int(self.run_state["completed_steps"])
                if self.run_state["error"] is None:
                    if stopped:
                        self.run_state["phase"] = "stopped"
                        self.run_state["message"] = (
                            f"已停止，共执行 {completed}/{total_steps} step。"
                        )
                    else:
                        self.run_state["phase"] = "completed"
                        self.run_state["message"] = (
                            f"推理完成，共执行 {completed}/{total_steps} step。"
                        )
                self.run_state["running"] = False
                self.run_state["stopping"] = False

    def read_observations(self, names: list[str] | None = None) -> list[dict[str, Any]]:
        with self.lock:
            client, manifest = self._require_connection()
            if names is not None and (
                not isinstance(names, list)
                or not all(isinstance(name, str) and name for name in names)
            ):
                raise ValueError("Observation names 必须是字符串数组")
            available_names = [
                str(item["name"]) for item in manifest.observations if item.get("name")
            ]
            available = set(available_names)
            selected = available_names if names is None else list(dict.fromkeys(names))
            unknown = [name for name in selected if name not in available]
            if unknown:
                raise ValueError(f"Observation 不存在：{', '.join(unknown)}")
            if not selected:
                raise ValueError("请先选择推理输入")
            response = client.get_observations(selected, timeout_ms=3000)
            if not response.ok or not isinstance(response.payload, dict):
                raise RuntimeError(
                    f"Server 读取 {', '.join(selected)} 失败："
                    f"{response.message or '返回数据不是对象'}"
                )
            observations = response.payload.get("observations")
            if not isinstance(observations, list):
                raise RuntimeError(
                    "Server 响应缺少 observations 数组，"
                    f"实际字段：{', '.join(map(str, response.payload)) or '(空)'}"
                )
            selected_set = set(selected)
            self.observations = [
                dict(item)
                for item in observations
                if isinstance(item, dict) and item.get("name") in selected_set
            ]
            return self.observations

    def read_image(self, name: str) -> tuple[bytes, str]:
        with self.lock:
            _, manifest = self._require_connection()
            image_names = {
                str(item["name"])
                for item in manifest.observations
                if item.get("type") == "image" and item.get("name")
            }
            if name not in image_names:
                raise ValueError(f"图像 Observation 不存在：{name}")
            observations = self.read_observations([name])
            if not observations:
                raise RuntimeError(f"图像 Observation 暂无数据：{name}")
            value = observations[0].get("value")
            if not isinstance(value, dict):
                raise RuntimeError("图像 Observation 格式不正确")
            image = value.get("image")
            if isinstance(image, bytearray):
                image = bytes(image)
            elif isinstance(image, memoryview):
                image = image.tobytes()
            if not isinstance(image, bytes):
                raise RuntimeError("图像 Observation 缺少二进制数据")
            encoding = str(value.get("encoding") or "jpeg").lower()
            content_type = {
                "jpg": "image/jpeg",
                "jpeg": "image/jpeg",
                "png": "image/png",
                "webp": "image/webp",
            }.get(encoding, "application/octet-stream")
            return image, content_type

    def infer(self, config: dict[str, Any]) -> dict[str, Any]:
        names = config.get("observation_names")
        if not isinstance(names, list) or not names or not all(
            isinstance(name, str) and name for name in names
        ):
            raise ValueError("请先选择推理输入")
        try:
            with self.lock:
                self._require_connection()
                observations = self.read_observations(names)
                inference_observations = {
                    str(item["name"]): item.get("value")
                    for item in observations
                    if item.get("name")
                }
                if str(config.get("protocol") or "json").lower() != "protobuf":
                    inference_observations = _inference_value(inference_observations)
                payload = {
                    "prompt": str(config.get("prompt") or ""),
                    "observations": inference_observations,
                }
        except Exception as exc:
            raise RuntimeError(
                f"读取推理 Observation 失败：{_error_detail(exc)}"
            ) from exc
        started = time.perf_counter()
        try:
            result = _request_model(config, payload)
        except Exception as exc:
            raise RuntimeError(
                f"请求模型服务失败：{_error_detail(exc)}"
            ) from exc
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        try:
            action = _normalize_action(
                result,
                action_name=str(config.get("action_name") or ""),
                frame_rate=float(config.get("frame_rate") or 30),
            )
        except Exception as exc:
            raise RuntimeError(
                f"解析模型 Action 失败：{_error_detail(exc)}"
            ) from exc
        return {"action": action, "elapsed_ms": elapsed_ms}

    def execute_action(
        self,
        name: str,
        frames: Any,
        frame_rate: float,
    ) -> dict[str, Any]:
        with self.lock:
            client, manifest = self._require_connection()
            if not math.isfinite(frame_rate) or frame_rate <= 0:
                raise ValueError("帧率必须大于 0")
            allowed = {
                str(item["name"])
                for item in manifest.actions
                if item.get("type") == "joint_position"
            }
            if name not in allowed:
                raise ValueError(f"目标 Action 不存在：{name}")
            normalized_frames = _validate_frames(frames)
        with self.action_lock:
            response = client.run_action_chunk(
                name,
                normalized_frames,
                frame_rate=float(frame_rate),
                timeout_ms=max(
                    3000, int(len(normalized_frames) / frame_rate * 1000) + 3000
                ),
            )
        if not response.ok:
            raise RuntimeError(
                f"Server 拒绝 Action {name}："
                f"{response.message or '未返回具体原因'}"
            )
        payload = response.payload if isinstance(response.payload, dict) else {}
        return {
            "accepted_frames": int(
                payload.get("accepted_frames") or len(normalized_frames)
            )
        }

    def go_home(self) -> dict[str, Any]:
        inference = self.stop_inference()
        with self.lock:
            client, manifest = self._require_connection()
            action_name = self._home_action_name(manifest)
        if action_name is None:
            raise RuntimeError("当前机器人未提供 Home Action")
        try:
            stop_response = client.stop_action(
                reason="model_debug_go_home",
                timeout_ms=1000,
            )
            if not stop_response.ok:
                raise RuntimeError(
                    stop_response.message or "Server 未返回具体原因"
                )
        except Exception as exc:
            raise RuntimeError(
                f"执行 Home 前停止当前 Action 失败：{_error_detail(exc)}"
            ) from exc
        with self.action_lock:
            response = client.run_action_chunk(
                action_name,
                [{}],
                frame_rate=1,
                timeout_ms=30000,
            )
        if not response.ok:
            raise RuntimeError(
                f"Server 执行 Home Action {action_name} 失败："
                f"{response.message or '未返回具体原因'}"
            )
        return {
            "action_name": action_name,
            "inference": inference,
        }

    def _require_connection(self) -> tuple[RcpProtocolClient, Any]:
        if self.client is None or self.manifest is None:
            raise RuntimeError("请先启动 Server")
        return self.client, self.manifest


def _request_model(config: dict[str, Any], payload: dict[str, Any]) -> Any:
    protocol = str(config.get("protocol") or "json").strip().lower()
    if protocol == "protobuf":
        return _request_protobuf_model(config, payload)
    if protocol != "json":
        raise ValueError(f"不支持的请求协议：{protocol}")
    return _request_json_model(config, payload)


def _error_detail(exc: Any) -> str:
    message = str(exc).strip()
    return message or exc.__class__.__name__


def _response_text(value: bytes, limit: int = 2000) -> str:
    text = value[:limit].decode("utf-8", errors="replace").strip()
    return text or "响应体为空"


def _request_json_model(config: dict[str, Any], payload: dict[str, Any]) -> Any:
    url = str(config.get("url") or "").strip()
    if not url:
        raise ValueError("请填写推理 URL")
    timeout_s = float(config.get("timeout_s") or 60)
    if timeout_s <= 0:
        raise ValueError("超时必须大于 0")
    headers = {"Content-Type": "application/json", "Accept": "application/json"}
    api_key = str(config.get("api_key") or "").strip()
    if api_key:
        header = str(config.get("auth_header") or "Authorization").strip()
        prefix = str(config.get("auth_prefix") or "").strip()
        headers[header] = f"{prefix} {api_key}".strip()
    request = Request(
        url,
        data=json.dumps(_inference_value(payload), ensure_ascii=False).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout_s) as response:
            body = response.read()
    except HTTPError as exc:
        detail = _response_text(exc.read(2000))
        raise RuntimeError(
            f"POST {url} 返回 HTTP {exc.code}：{detail}"
        ) from exc
    except URLError as exc:
        raise RuntimeError(
            f"无法连接 {url}：{_error_detail(exc.reason)}"
        ) from exc
    except TimeoutError as exc:
        raise RuntimeError(
            f"请求 {url} 超过 {timeout_s:g} 秒，已超时"
        ) from exc
    try:
        return json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            f"推理服务返回的内容不是有效 UTF-8 JSON："
            f"{_response_text(body, limit=500)}"
        ) from exc


def _request_protobuf_model(config: dict[str, Any], payload: dict[str, Any]) -> Any:
    url = _protobuf_url(str(config.get("url") or ""))
    timeout_s = float(config.get("timeout_s") or 60)
    if timeout_s <= 0:
        raise ValueError("超时必须大于 0")
    proto = RynnProtoCodec()
    try:
        packet = _build_protobuf_observation(proto, payload)
    except Exception as exc:
        raise RuntimeError(
            f"Protobuf Observation 编码失败：{_error_detail(exc)}"
        ) from exc
    headers = {
        "Content-Type": "application/octet-stream",
        "Accept": "application/octet-stream",
        "X-Request-Id": str(uuid.uuid4()),
    }
    api_key = str(config.get("api_key") or "").strip()
    if api_key:
        header = str(config.get("auth_header") or "Authorization").strip()
        prefix = str(config.get("auth_prefix") or "").strip()
        headers[header] = f"{prefix} {api_key}".strip()
    request = Request(
        url,
        data=packet.SerializeToString(),
        headers=headers,
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout_s) as response:
            body = response.read()
    except HTTPError as exc:
        detail = _response_text(exc.read(2000))
        raise RuntimeError(
            f"POST {url} 返回 HTTP {exc.code}：{detail}"
        ) from exc
    except URLError as exc:
        raise RuntimeError(
            f"无法连接 {url}：{_error_detail(exc.reason)}"
        ) from exc
    except TimeoutError as exc:
        raise RuntimeError(
            f"请求 {url} 超过 {timeout_s:g} 秒，已超时"
        ) from exc

    response_packet = proto.DataPacket()
    try:
        response_packet.ParseFromString(body)
        actions = proto.MultiAction()
        actions.ParseFromString(response_packet.data)
    except Exception as exc:
        raise RuntimeError(
            f"Protobuf Action 解码失败"
            f"（响应 {len(body)} bytes）：{_error_detail(exc)}"
        ) from exc
    if not actions.action_list:
        raise RuntimeError("Protobuf 响应缺少 Action")
    action = actions.action_list[0]
    values = _decode_protobuf_array(action.action_data)
    if values.ndim == 1:
        values = values.reshape(1, -1)
    if values.ndim != 2:
        raise RuntimeError(f"Protobuf Action 维度不正确：{values.shape}")
    frame_rate = float(action.action_rate or config.get("frame_rate") or 30)
    return {
        "frames": [
            {"joint_positions": [float(number) for number in row]}
            for row in values.tolist()
        ],
        "frame_rate": frame_rate,
    }


def _protobuf_url(value: str) -> str:
    url = str(value).strip().rstrip("/")
    if not url:
        raise ValueError("请填写推理 URL")
    parsed = urlparse(url)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError("Protobuf 推理 URL 必须以 http:// 或 https:// 开头")
    return url if url.endswith("/protobuf") else f"{url}/protobuf"


def _build_protobuf_observation(
    proto: RynnProtoCodec,
    payload: dict[str, Any],
) -> Any:
    observations = payload.get("observations")
    if not isinstance(observations, dict):
        raise ValueError("请在推理输入中选择 Observation")
    state_parts = []
    images: list[tuple[str, Any]] = []
    for name, value in observations.items():
        if isinstance(value, dict) and "image" in value:
            component = _observation_component(str(name))
            images.append(
                (
                    f"observation.images.{component}",
                    _decode_image_value(value),
                )
            )
            continue
        vector = _numeric_observation(value)
        if vector is not None:
            state_parts.append(vector)
    if not state_parts:
        raise ValueError("Protobuf 推理至少选择一个状态 Observation")
    state = np.concatenate(state_parts).astype(np.float32)
    packet = proto.CombineDataPacket()
    packet.prompt = str(payload.get("prompt") or "")
    multi_state = proto.MultiState()
    state_item = multi_state.state_list.add()
    state_item.name = "observation.state"
    state_item.state_data.data = state.tobytes()
    state_item.state_data.shape.extend(list(state.shape))
    state_item.state_data.dtype = proto.DataType.FLOAT32
    packet.state = multi_state.SerializeToString()

    multi_image = proto.MultiImage()
    for name, image in images:
        item = multi_image.image_list.add()
        item.name = name
        item.format = "jpeg"
        item.image_data.data = _encode_jpeg(image)
        item.image_data.shape.extend(list(image.shape))
        item.image_data.dtype = proto.DataType.UINT8
    packet.image = multi_image.SerializeToString()
    return packet


def _numeric_observation(value: Any) -> Any | None:
    candidate = value
    if isinstance(value, dict):
        for key in ("joint_positions", "positions", "state"):
            if key in value:
                candidate = value[key]
                break
        else:
            return None
    if not isinstance(candidate, (list, tuple)):
        return None
    try:
        array = np.asarray(candidate, dtype=np.float32).reshape(-1)
    except (TypeError, ValueError):
        return None
    return array if array.size else None


def _observation_component(name: str) -> str:
    parts = [part for part in name.split(".") if part]
    if len(parts) >= 3 and parts[0] == "observation":
        if parts[1] == "images":
            return parts[2]
        return parts[1]
    raise ValueError(f"无法识别图像 Observation：{name}")


def _decode_image_value(value: dict[str, Any]) -> Any:
    image = value.get("image")
    if isinstance(image, bytearray):
        image = bytes(image)
    elif isinstance(image, memoryview):
        image = image.tobytes()
    if not isinstance(image, bytes):
        raise ValueError("图像 Observation 缺少二进制数据")
    encoding = str(value.get("encoding") or "").lower()
    width = int(value.get("width") or 0)
    height = int(value.get("height") or 0)
    if encoding in {"raw", "rgb888", "bgr888"} and width > 0 and height > 0:
        array = np.frombuffer(image, dtype=np.uint8).reshape(height, width, 3)
        return array[:, :, ::-1] if encoding == "bgr888" else array
    try:
        with Image.open(io.BytesIO(image)) as decoded:
            return np.asarray(decoded.convert("RGB"), dtype=np.uint8)
    except Exception as exc:
        raise ValueError("无法解码图像 Observation") from exc


def _encode_jpeg(value: Any) -> bytes:
    image = np.asarray(value, dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"JPEG 图像需要 H×W×3，当前 shape 为 {image.shape}")
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="JPEG", quality=95)
    return buffer.getvalue()


def _decode_protobuf_array(value: Any) -> Any:
    shape = tuple(int(size) for size in value.shape)
    dtype = {
        RynnProtoCodec().DataType.UINT8: np.uint8,
        RynnProtoCodec().DataType.INT8: np.int8,
        RynnProtoCodec().DataType.UINT16: np.uint16,
        RynnProtoCodec().DataType.INT16: np.int16,
        RynnProtoCodec().DataType.INT32: np.int32,
        RynnProtoCodec().DataType.FLOAT32: np.float32,
        RynnProtoCodec().DataType.FLOAT64: np.float64,
    }.get(value.dtype, np.float32)
    try:
        with gzip.GzipFile(fileobj=io.BytesIO(value.data), mode="rb") as stream:
            result = np.load(stream, allow_pickle=False)
    except (OSError, ValueError):
        result = np.frombuffer(value.data, dtype=dtype)
    return result.reshape(shape) if shape else result


def _normalize_action(
    value: Any,
    *,
    action_name: str,
    frame_rate: float,
) -> dict[str, Any]:
    if not math.isfinite(frame_rate) or frame_rate <= 0:
        raise ValueError("帧率必须大于 0")
    candidate = value
    for _ in range(4):
        if not isinstance(candidate, dict):
            break
        if isinstance(candidate.get("frames"), list):
            frames = candidate["frames"]
            return {
                "name": str(candidate.get("name") or action_name),
                "frames": _validate_frames(frames),
                "frame_rate": float(candidate.get("frame_rate") or frame_rate),
            }
        next_candidate = None
        for key in ("action", "actions", "action_chunk", "output", "result"):
            if key in candidate:
                next_candidate = candidate[key]
                break
        if next_candidate is None:
            break
        candidate = next_candidate
    if isinstance(candidate, list):
        frames = [
            frame if isinstance(frame, dict) else {"joint_positions": list(frame)}
            for frame in candidate
        ]
        return {
            "name": action_name,
            "frames": _validate_frames(frames),
            "frame_rate": frame_rate,
        }
    raise ValueError("请检查推理响应，需包含 frames、action 或 actions")


def _validate_frames(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list) or not value:
        raise ValueError("Action frames 必须是非空数组")
    result: list[dict[str, Any]] = []
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise ValueError(f"Action frame {index} 必须是对象")
        positions = item.get("joint_positions")
        if not isinstance(positions, list) or not positions:
            raise ValueError(f"Action frame {index} 缺少 joint_positions")
        numbers = [float(number) for number in positions]
        if not all(math.isfinite(number) for number in numbers):
            raise ValueError(f"Action frame {index} 包含无效数值")
        result.append({**item, "joint_positions": numbers})
    return result


def _inference_value(value: Any) -> Any:
    if isinstance(value, bytes):
        return {"encoding": "base64", "data": base64.b64encode(value).decode("ascii")}
    if isinstance(value, bytearray):
        return _inference_value(bytes(value))
    if isinstance(value, memoryview):
        return _inference_value(value.tobytes())
    if isinstance(value, dict):
        return {str(key): _inference_value(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_inference_value(child) for child in value]
    return value


def _web_value(value: Any) -> Any:
    if isinstance(value, bytes):
        return {"__bytes_base64": base64.b64encode(value).decode("ascii")}
    if isinstance(value, bytearray):
        return _web_value(bytes(value))
    if isinstance(value, memoryview):
        return _web_value(value.tobytes())
    if isinstance(value, dict):
        return {str(key): _web_value(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_web_value(child) for child in value]
    return value


class Handler(BaseHTTPRequestHandler):
    app: ModelDebugApp

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            html = HTML.replace(
                "__SERVER_CONFIG__",
                _html_attribute(self.app.server_config),
            )
            self._send(200, html.encode("utf-8"), "text/html; charset=utf-8")
            return
        if parsed.path == "/api/image":
            name = ""
            try:
                name = str(parse_qs(parsed.query).get("name", [""])[0])
                data, content_type = self.app.read_image(name)
                self._send(200, data, content_type)
            except Exception as exc:
                logger.warning(
                    "[ModelDebug][IMAGE_READ_FAILED] name=%s error=%s; "
                    "inspect the Observation publisher and image codec",
                    name or "<missing>",
                    exc,
                    exc_info=True,
                )
                self._json(
                    500,
                    {
                        "success": False,
                        "error": f"读取图像 {name or '(未指定)'} 失败："
                        f"{_error_detail(exc)}",
                        "error_type": exc.__class__.__name__,
                    },
                )
            return
        self._json(404, {"success": False, "error": "页面不存在"})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            body = self._read_json()
            if path == "/api/server/start":
                self._ok(self.app.start_server(body.get("config_path")))
            elif path == "/api/server/status":
                self._ok(self.app.server_status())
            elif path == "/api/server/stop":
                self._ok(self.app.stop_server())
            elif path == "/api/observations":
                self._ok(
                    {
                        "observations": _web_value(
                            self.app.read_observations(body.get("names"))
                        )
                    }
                )
            elif path == "/api/inference/start":
                self._ok(self.app.start_inference(body))
            elif path == "/api/inference/stop":
                self._ok(self.app.stop_inference())
            elif path == "/api/inference/status":
                self._ok(self.app.inference_status())
            elif path == "/api/infer":
                self._ok(self.app.infer(body))
            elif path == "/api/action/execute":
                self._ok(
                    self.app.execute_action(
                        str(body.get("name") or ""),
                        body.get("frames"),
                        float(body.get("frame_rate") or 30),
                    )
                )
            elif path == "/api/action/home":
                self._ok(self.app.go_home())
            else:
                self._json(404, {"success": False, "error": "接口不存在"})
        except Exception as exc:
            operation = {
                "/api/server/start": "启动 RCP Server",
                "/api/server/status": "读取 RCP Server 状态",
                "/api/server/stop": "停止 RCP Server",
                "/api/observations": "读取 Observation",
                "/api/inference/start": "启动推理",
                "/api/inference/stop": "停止推理",
                "/api/inference/status": "读取推理状态",
                "/api/infer": "请求单轮推理",
                "/api/action/execute": "执行 Action",
                "/api/action/home": "执行 Home Action",
            }.get(path, f"处理接口 {path}")
            logger.exception(
                "[ModelDebug][REQUEST_FAILED] path=%s operation=%s error=%s; "
                "inspect the model-debug and server logs for the same run_id",
                path,
                operation,
                exc,
            )
            self._json(
                500,
                {
                    "success": False,
                    "error": f"{operation}失败：{_error_detail(exc)}",
                    "error_type": exc.__class__.__name__,
                },
            )

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("content-length") or 0)
        if length <= 0:
            return {}
        value = json.loads(self.rfile.read(length).decode("utf-8"))
        return value if isinstance(value, dict) else {}

    def _ok(self, value: dict[str, Any]) -> None:
        self._json(200, {"success": True, **_web_value(value)})

    def _json(self, status: int, value: Any) -> None:
        data = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self._send(status, data, "application/json; charset=utf-8")

    def _send(self, status: int, data: bytes, content_type: str) -> None:
        try:
            self.send_response(status)
            self.send_header("content-type", content_type)
            self.send_header("content-length", str(len(data)))
            self.send_header("cache-control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return


def _html_attribute(value: str) -> str:
    return (
        str(value)
        .replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="启动 RCP 模型调试 App")
    parser.add_argument("--server-config", required=True, help="RCP Server 配置文件")
    parser.add_argument("--host", default="127.0.0.1", help="Web 监听地址")
    parser.add_argument("--port", type=int, default=8093, help="Web 端口")
    parser.add_argument("--no-open", action="store_true", help="启动后保持浏览器关闭")
    args = parser.parse_args(argv)

    app_id = "model_debug"
    set_log_context(
        app_id=app_id,
        run_id=resolve_log_run_id(),
        process="model_debug_app",
    )
    configure_logging(
        sinks=["stderr", "file"],
        file_path=str(logs_dir(app_root(app_id)) / "model_debug.log"),
    )

    app = ModelDebugApp(args.server_config)
    try:
        info = app.start_server()
    except Exception as exc:
        logger.exception(
            "[ModelDebug][SERVER_START_FAILED] config=%s error=%s; "
            "verify the server config and device dependencies",
            args.server_config,
            exc,
        )
        print(f"RCP Server 启动失败：{exc}", file=sys.stderr)
        return 1

    Handler.app = app
    try:
        server = ThreadingHTTPServer((args.host, args.port), Handler)
    except Exception:
        app.stop_server()
        raise
    url = f"http://{args.host}:{server.server_port}/"
    print(
        f"RCP Server 已连接：{info['robot_name']}，"
        f"Observation {info['observation_count']} 个"
    )
    print(f"Model Debug UI: {url}")
    if not args.no_open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        return 130
    finally:
        Handler.app.stop_server()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
