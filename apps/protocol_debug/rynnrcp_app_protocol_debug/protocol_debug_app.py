"""Browser debug app for one RynnRCP server config."""

from __future__ import annotations

import argparse
import base64
import json
import logging
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

try:
    import yaml
except ModuleNotFoundError:  # keep the debug page usable before optional deps are installed
    yaml = None

from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.discovery import LocalRegistry
from rynnrcp.interface.protocol_client import RcpProtocolClient
from rynnrcp.utils.logging import configure_logging, resolve_log_run_id, set_log_context
from rynnrcp.utils.user_paths import app_root, logs_dir, new_log_session_id
from rynnrcp.utils.web_urls import browser_urls, primary_browser_url

logger = logging.getLogger(__name__)

HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RCP 协议调试台</title>
  <style>
    * { box-sizing: border-box; }
    :root {
      color-scheme: dark;
      --bg:#0f1418; --panel:#171d22; --panel2:#111820; --line:#2b3640;
      --text:#edf3f7; --muted:#9ba8b3; --blue:#7dd3fc; --green:#86efac;
      --yellow:#fde047; --red:#fb7185; --purple:#c4b5fd;
    }
    body { margin:0; min-height:100vh; background:var(--bg); color:var(--text); font:14px/1.48 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
    main { max-width:1540px; margin:0 auto; padding:18px; }
    header { display:flex; justify-content:space-between; gap:16px; align-items:flex-start; border-bottom:1px solid var(--line); padding-bottom:14px; margin-bottom:14px; }
    h1 { margin:0; color:var(--blue); font-size:25px; letter-spacing:0; }
    h2 { margin:0 0 10px; font-size:16px; color:#dae5ec; }
    h3 { margin:0 0 6px; font-size:14px; color:#eef5f9; }
    p { margin:4px 0 0; color:var(--muted); }
    code { color:#bae6fd; }
    button { border:0; border-radius:7px; padding:9px 12px; font-weight:750; cursor:pointer; color:#061018; background:var(--blue); }
    button.secondary { background:#a7b2bd; }
    button.good { background:var(--green); }
    button.warn { background:var(--yellow); }
    button.danger { background:var(--red); color:white; }
    button.ghost { background:#23303a; color:#d9e3ea; border:1px solid #3a4652; }
    button.small { padding:6px 9px; font-size:12px; }
    input, textarea, select { width:100%; border:1px solid #394754; background:#0c1217; color:var(--text); border-radius:7px; padding:8px 9px; font:13px/1.4 ui-monospace,SFMono-Regular,Menlo,monospace; }
    textarea { min-height:92px; resize:vertical; }
    label { display:block; color:#c5d1da; margin-bottom:5px; font-weight:650; }
    section { border:1px solid var(--line); background:var(--panel); border-radius:8px; padding:14px; margin-bottom:12px; }
    .topbar { display:grid; grid-template-columns:repeat(6,minmax(0,1fr)); gap:8px; margin-bottom:14px; }
    .metric { background:var(--panel2); border:1px solid var(--line); border-radius:7px; padding:9px; min-height:56px; }
    .metric span { display:block; color:var(--muted); font-size:12px; }
    .metric strong { display:block; margin-top:3px; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
    .layout { display:grid; grid-template-columns:360px 1fr; gap:14px; align-items:start; }
    .row { display:flex; flex-wrap:wrap; gap:8px; align-items:center; }
    .stack { display:grid; gap:10px; }
    .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(310px,1fr)); gap:10px; }
    .group { margin-top:12px; }
    .group-title { display:flex; align-items:center; gap:8px; margin:0 0 8px; color:#bae6fd; font-size:14px; }
    .group-title::after { content:""; height:1px; background:#2b3640; flex:1; }
    .card { background:var(--panel2); border:1px solid var(--line); border-radius:8px; padding:12px; }
    .head { display:flex; justify-content:space-between; gap:10px; align-items:flex-start; margin-bottom:9px; }
    .badge { border:1px solid #3a4652; border-radius:999px; padding:3px 8px; color:#cbd6de; font-size:12px; white-space:nowrap; }
    .muted { color:var(--muted); }
    .hidden { display:none; }
    .form-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(96px,1fr)); gap:8px; }
    .joint-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(88px,1fr)); gap:6px; max-height:260px; overflow:auto; padding-right:3px; }
    .joint-grid label { font-size:11px; color:#9fb0bd; }
    .obs-card.selected { outline:2px solid var(--blue); }
    .obs-value { margin-top:10px; background:#081015; border:1px solid #26323d; border-radius:7px; padding:9px; max-height:380px; overflow:auto; }
    .bars { display:grid; gap:3px; }
    .bar { display:grid; grid-template-columns:92px 1fr 62px; gap:6px; align-items:center; font:12px ui-monospace,SFMono-Regular,Menlo,monospace; }
    .track { height:8px; background:#22303a; border-radius:999px; overflow:hidden; }
    .fill { height:100%; background:var(--green); width:50%; }
    img.obs-image { max-width:100%; border-radius:7px; border:1px solid var(--line); background:#05080a; }
    pre { margin:0; min-height:220px; max-height:560px; overflow:auto; white-space:pre-wrap; word-break:break-word; color:#a7f3d0; background:#080d11; border:1px solid var(--line); border-radius:8px; padding:12px; }
    @media (max-width:980px) { main{padding:12px;} header,.layout,.topbar{grid-template-columns:1fr; display:grid;} }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>RCP 协议调试台</h1>
      <p>只调试启动参数 <code>--config</code> 指定的 RCP server。</p>
    </div>
    <button class="ghost" onclick="refreshAll()">刷新全部</button>
  </header>

  <div class="topbar" id="statusBar"></div>

  <div class="layout">
    <aside>
      <section>
        <h2>Server</h2>
        <div class="stack">
          <div><label>Server config</label><input id="configPath" readonly></div>
          <div><label>Endpoint</label><input id="endpoint" readonly placeholder="由 config / local registry 自动确定"></div>
          <div class="row">
            <button class="good" onclick="startServer()">启动指定 config</button>
            <button class="warn" onclick="connectServer()">连接并扫描</button>
            <button class="danger" onclick="stopServer()">停止</button>
          </div>
        </div>
      </section>

      <section>
        <h2>Raw Request</h2>
        <div class="stack">
          <div><label>method</label><input id="rawMethod" value="get_manifest"></div>
          <div><label>payload JSON</label><textarea id="rawPayload">{}</textarea></div>
          <button onclick="rawRequest()">发送</button>
        </div>
      </section>
    </aside>

    <div>
      <section id="panel-manifest">
        <h2>Manifest / Response</h2>
        <pre id="manifestBox"></pre>
      </section>

      <section id="panel-tools">
        <div class="row" style="justify-content:space-between; margin-bottom:12px;">
          <h2 style="margin:0;">注册工具</h2>
          <button class="secondary" onclick="loadTools()">list_tools</button>
        </div>
        <div class="grid" id="tools"></div>
      </section>

      <section id="panel-obs">
        <div class="row" style="justify-content:space-between; margin-bottom:12px;">
          <h2 style="margin:0;">Observations</h2>
          <div class="row">
            <button class="secondary" onclick="loadObservations()">扫描 obs</button>
            <button class="good" onclick="startObsPolling()">开始轮询</button>
            <button class="warn" onclick="stopObsPolling()">停止轮询</button>
            <button onclick="sampleObsStream()">订阅采样</button>
          </div>
        </div>
        <p class="muted">勾选多个 obs 后，统一按 10Hz 调用 <code>get_observations</code> 并渲染返回值。</p>
        <div class="grid" id="observations" style="margin-top:10px;"></div>
      </section>

      <section id="panel-streams">
        <div class="row" style="justify-content:space-between; margin-bottom:12px;">
          <h2 style="margin:0;">Streaming</h2>
          <div class="row">
            <button onclick="sampleHealthStream()">subscribe_health 采样</button>
          </div>
        </div>
        <p class="muted">流式接口会采样有限帧数后返回，方便确认接口可用。</p>
      </section>

      <section id="panel-actions">
        <div class="row" style="justify-content:space-between; margin-bottom:12px;">
          <h2 style="margin:0;">Actions</h2>
          <button class="secondary" onclick="loadActions()">list_actions</button>
        </div>
        <div class="grid" id="actions"></div>
      </section>

      <section id="panel-policies">
        <div class="row" style="justify-content:space-between; margin-bottom:12px;">
          <h2 style="margin:0;">Policies</h2>
          <button class="secondary" onclick="loadPolicies()">list_policies</button>
        </div>
        <div class="grid" id="policies"></div>
      </section>

      <section id="panel-log">
        <h2>调用日志</h2>
        <pre id="log"></pre>
      </section>
    </div>
  </div>
</main>

<script>
const DEFAULT_CONFIG = __DEFAULT_CONFIG__;
let manifest = null;
let tools = {};
let actions = [];
let observations = [];
let policies = [];
let selectedObs = new Set();
let obsTimer = null;
let pollBusy = false;

function $(id) { return document.getElementById(id); }
function shortName(name) { return String(name).replace(/^action\.[^.]+\./, "").replace(/^observation\.[^.]+\./, ""); }
function log(title, obj) {
  $("log").textContent = `[${new Date().toLocaleTimeString()}] ${title}\n${JSON.stringify(obj, null, 2)}\n\n` + $("log").textContent;
}
async function api(path, options = {}) {
  const res = await fetch(path, options);
  const data = await res.json();
  if (!res.ok || data.success === false) throw data;
  return data;
}
async function post(path, body = {}) {
  return api(path, {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body)});
}
function showError(title, err) {
  log("ERR " + title, err);
  $("manifestBox").textContent = JSON.stringify(err, null, 2);
}
function payload(data) { return data?.response?.payload ?? data?.payload ?? data; }
function resultOf(data) {
  const p = payload(data);
  return p?.result || p || {};
}
function metric(k,v) { return `<div class="metric"><span>${k}</span><strong title="${v ?? "-"}">${v ?? "-"}</strong></div>`; }
async function refreshState() {
  const data = await api("/api/state");
  $("configPath").value = data.default_config;
  $("endpoint").value = data.address || "";
  $("statusBar").innerHTML = [
    metric("server", data.running ? "running" : "stopped"),
    metric("endpoint", data.address || "-"),
    metric("robot", manifest?.robot_id || "-"),
    metric("name", manifest?.robot_name || "-"),
    metric("obs/actions", `${observations.length}/${actions.length}`),
    metric("last_error", data.last_error || "-"),
  ].join("");
  return data;
}
async function startServer() {
  try { log("server/start", await post("/api/server/start", {})); await refreshAll(); }
  catch (err) { showError("server/start", err); await refreshState(); }
}
async function connectServer() {
  try { log("server/connect", await post("/api/server/connect", {})); await refreshAll(); }
  catch (err) { showError("server/connect", err); await refreshState(); }
}
async function stopServer() {
  stopObsPolling();
  try { log("server/stop", await post("/api/server/stop", {})); manifest=null; actions=[]; observations=[]; policies=[]; renderAll(); await refreshState(); }
  catch (err) { showError("server/stop", err); }
}
async function request(method, body = {}) {
  const data = await post("/api/protocol", {method, payload:body});
  log(method, data.response);
  $("manifestBox").textContent = JSON.stringify(data.response, null, 2);
  return data;
}
async function refreshAll() {
  await refreshState();
  await loadManifest();
  await loadTools();
  await loadObservations();
  await loadActions();
  await loadPolicies();
  await request("get_health").catch(() => {});
  await refreshState();
}
async function loadManifest() {
  try {
    const data = await request("get_manifest");
    manifest = payload(data);
    if (manifest?.actions) actions = manifest.actions;
    if (manifest?.observations) observations = manifest.observations;
    renderAll();
  } catch (err) { showError("get_manifest", err); }
}
async function loadTools() {
  try {
    const data = await request("list_tools");
    tools = payload(data)?.tools || payload(data) || {};
    renderTools();
  } catch (err) { showError("list_tools", err); }
}
async function loadObservations() {
  try {
    const data = await request("list_observations");
    observations = resultOf(data).observations || [];
    renderObservations();
  } catch (err) { showError("list_observations", err); }
}
async function loadActions() {
  try {
    const data = await request("list_actions");
    actions = resultOf(data).actions || [];
    renderActions();
  } catch (err) { showError("list_actions", err); }
}
async function loadPolicies() {
  try {
    const data = await request("list_policies");
    policies = resultOf(data).policies || [];
    renderPolicies(resultOf(data).active_policy_id, resultOf(data).last_error);
  } catch (err) {
    policies = [];
    renderPolicies(null, err.error || err.message || "policy service unavailable");
  }
}
async function rawRequest() {
  let body;
  try { body = JSON.parse($("rawPayload").value || "{}"); }
  catch (err) { showError("raw JSON", {success:false, error:String(err)}); return; }
  await request($("rawMethod").value, body).catch(err => showError("raw request", err));
}
function renderTools() {
  const entries = Array.isArray(tools) ? tools.map(x => [x.name || "", x]) : Object.entries(tools || {});
  if (!entries.length) {
    $("tools").innerHTML = `<p class="muted">连接 server 后点击 list_tools。</p>`;
    return;
  }
  const groups = {};
  for (const [name, spec] of entries) {
    const group = toolGroup(name);
    (groups[group] ||= []).push([name, spec]);
  }
  const order = ["Manifest / Health", "Observations", "Actions", "Policies", "Resources", "Collections", "Other"];
  $("tools").innerHTML = order.filter(g => groups[g]?.length).map(group => {
    const cards = groups[group].map(([name, spec]) => toolCard(name, spec)).join("");
    return `<div class="group"><h3 class="group-title">${group}</h3><div class="grid">${cards}</div></div>`;
  }).join("");
}
function toolGroup(name) {
  if (name.includes("observation")) return "Observations";
  if (name.includes("action")) return "Actions";
  if (name.includes("policy")) return "Policies";
  if (name.includes("resource")) return "Resources";
  if (name.includes("collection")) return "Collections";
  if (name.includes("manifest") || name.includes("health") || name === "ping") return "Manifest / Health";
  return "Other";
}
function toolCard(name, spec) {
  const id = "tool_" + String(name).replace(/[^a-zA-Z0-9_]/g, "_");
  const sample = samplePayloadForTool(name, spec);
  return `<div class="card">
    <div class="head"><div><h3>${name}</h3><div class="muted">${spec.description || ""}</div></div><span class="badge">tool</span></div>
    <label>payload JSON</label><textarea id="${id}">${JSON.stringify(sample, null, 2)}</textarea>
    <div class="row" style="margin-top:8px;"><button onclick="runTool('${name}', '${id}')">调用工具</button></div>
  </div>`;
}
function samplePayloadForTool(name, spec) {
  if (name === "get_observations") return {names: observations.map(o => o.name).slice(0,1), sync:false};
  if (name === "run_action_chunk") return {name: actions[0]?.name || "", frames:[{}], frame_rate:1};
  if (name === "start_policy") return {policy_id: policies[0]?.policy_id || "", runtime_inputs:{}};
  if (name === "stop_policy") return {};
  return {};
}
async function runTool(name, id) {
  let body;
  try { body = JSON.parse($(id).value || "{}"); }
  catch (err) { showError("tool JSON", {success:false, error:String(err)}); return; }
  await request(name, body).catch(err => showError(name, err));
}
function componentDof(action) {
  const comp = (manifest?.components || []).find(item => item.name === action.component_name);
  return Number(comp?.dof || 0);
}
function renderActions() {
  $("actions").innerHTML = actions.map(actionCard).join("") || `<p class="muted">连接 server 后点击 list_actions。</p>`;
}
function actionCard(action) {
  const id = "act_" + action.name.replace(/[^a-zA-Z0-9_]/g, "_");
  const rate = Number(action.frame_rate || 1);
  return `<div class="card">
    <div class="head"><div><h3>${shortName(action.name)}</h3><div class="muted">${action.description || ""}</div></div><span class="badge">${action.type}</span></div>
    <div id="${id}">${actionInputs(action, id)}</div>
    <div class="row" style="margin-top:9px;"><button class="good" onclick="runActionFromForm('${action.name}', '${action.type}', '${id}', ${rate})">发送 action</button><button onclick="runActionAsyncFromForm('${action.name}', '${action.type}', '${id}', ${rate})">async stream</button><span class="muted">${rate} Hz</span></div>
  </div>`;
}
function actionInputs(action, id) {
  if (action.type === "prearranged") return `<p class="muted">无需参数。</p>`;
  if (action.type === "base_velocity") {
    return `<div class="form-grid" data-kind="base_velocity">
      ${numInput("linear x", "linear_0", 0)}${numInput("linear y", "linear_1", 0)}${numInput("linear z", "linear_2", 0)}
      ${numInput("angular x", "angular_0", 0)}${numInput("angular y", "angular_1", 0)}${numInput("angular z", "angular_2", 0)}
    </div>`;
  }
  if (action.type === "joint_position") {
    const n = componentDof(action) || 0;
    const cells = Array.from({length:n}, (_, i) => `<label>j${i}<input data-joint="${i}" type="number" step="0.001" value="0"></label>`).join("");
    return `<div class="row" style="margin-bottom:8px;"><button class="ghost small" onclick="fillJointZeros('${id}')">全 0</button></div><div class="joint-grid" data-kind="joint_position">${cells}</div>`;
  }
  return `<label>value JSON</label><textarea data-kind="json">{}</textarea>`;
}
function numInput(label, key, value) { return `<label>${label}<input data-key="${key}" type="number" step="0.01" value="${value}"></label>`; }
function fillJointZeros(id) { document.querySelectorAll(`#${id} [data-joint]`).forEach(input => input.value = "0"); }
function readActionValue(type, id) {
  const root = $(id);
  if (type === "prearranged") return {};
  if (type === "base_velocity") {
    const get = key => Number(root.querySelector(`[data-key="${key}"]`)?.value || 0);
    return {linear_vel:[get("linear_0"), get("linear_1"), get("linear_2")], angular_vel:[get("angular_0"), get("angular_1"), get("angular_2")]};
  }
  if (type === "joint_position") {
    return {joint_positions:Array.from(root.querySelectorAll("[data-joint]")).map(input => Number(input.value || 0))};
  }
  return JSON.parse(root.querySelector("textarea")?.value || "{}");
}
async function runActionFromForm(name, type, id, frameRate) {
  let value;
  try { value = readActionValue(type, id); }
  catch (err) { showError("action input", {success:false, error:String(err)}); return; }
  try {
    const data = await post("/api/action", {name, value, frame_rate:frameRate || 1});
    log("run_action_chunk " + name, data.response);
    $("manifestBox").textContent = JSON.stringify(data.response, null, 2);
  } catch (err) { showError("action " + name, err); }
}
async function runActionAsyncFromForm(name, type, id, frameRate) {
  let value;
  try { value = readActionValue(type, id); }
  catch (err) { showError("action input", {success:false, error:String(err)}); return; }
  try {
    const data = await post("/api/action/async", {name, value, frame_rate:frameRate || 1, frames:3});
    log("run_action_chunk_async " + name, data);
    $("manifestBox").textContent = JSON.stringify(data, null, 2);
  } catch (err) { showError("action async " + name, err); }
}
function renderObservations() {
  if (!observations.length) {
    $("observations").innerHTML = `<p class="muted">连接 server 后点击 list_observations。</p>`;
    return;
  }
  const groups = {};
  for (const obs of observations) (groups[obs.type || "other"] ||= []).push(obs);
  $("observations").innerHTML = Object.keys(groups).sort().map(type => {
    const cards = groups[type].map(obsCard).join("");
    return `<div class="group"><h3 class="group-title">${type}</h3><div class="grid">${cards}</div></div>`;
  }).join("");
}
function obsCard(obs) {
  const selected = selectedObs.has(obs.name);
  return `<div class="card obs-card ${selected ? "selected" : ""}" id="obs_${obs.name.replace(/[^a-zA-Z0-9_]/g, "_")}">
    <div class="head"><div><h3>${shortName(obs.name)}</h3><div class="muted">${obs.description || ""}</div></div><span class="badge">${obs.type}</span></div>
    <label><input type="checkbox" ${selected ? "checked" : ""} onchange="toggleObs('${obs.name}', this.checked)" style="width:auto;"> 10Hz 轮询</label>
    <div class="row" style="margin-top:8px;"><button class="small" onclick="fetchObs(['${obs.name}'])">读取一次</button></div>
    <div class="obs-value" data-obs="${obs.name}"><span class="muted">暂无数据</span></div>
  </div>`;
}
function toggleObs(name, checked) {
  if (checked) selectedObs.add(name); else selectedObs.delete(name);
  renderObservations();
  if (selectedObs.size) startObsPolling(); else stopObsPolling();
}
function startObsPolling() {
  stopObsPolling();
  pollObs();
  obsTimer = setInterval(() => pollObs(), 100);
}
function stopObsPolling() {
  if (obsTimer) clearInterval(obsTimer);
  obsTimer = null;
}
async function pollObs() {
  if (pollBusy) return;
  const names = observations.filter(o => selectedObs.has(o.name)).map(o => o.name);
  if (!names.length) return;
  pollBusy = true;
  try { await fetchObs(names, false); } finally { pollBusy = false; }
}
async function fetchObs(names, writeLog = true) {
  try {
    const data = await post("/api/observation", {names, sync:false, timeout_ms:1000});
    if (writeLog) log("get_observations", data.response);
    const items = resultOf(data).observations || [];
    for (const item of items) renderObsValue(item.name, item.value, item.timestamp);
  } catch (err) { if (writeLog) showError("get_observations", err); }
}
async function sampleObsStream() {
  const names = observations.filter(o => selectedObs.has(o.name)).map(o => o.name);
  const selected = names.length ? names : observations.slice(0, 1).map(o => o.name);
  if (!selected.length) {
    showError("subscribe_observations", {success:false, error:"no observation selected"});
    return;
  }
  try {
    const data = await post("/api/observation/subscribe", {names:selected, stream_hz:10, frames:5});
    log("subscribe_observations", data);
    const last = data.frames?.[data.frames.length - 1]?.payload?.observations || [];
    for (const item of last) renderObsValue(item.name, item.value, item.timestamp);
    $("manifestBox").textContent = JSON.stringify(data, null, 2);
  } catch (err) { showError("subscribe_observations", err); }
}
async function sampleHealthStream() {
  try {
    const data = await post("/api/health/subscribe", {stream_hz:2, frames:3});
    log("subscribe_health", data);
    $("manifestBox").textContent = JSON.stringify(data, null, 2);
  } catch (err) { showError("subscribe_health", err); }
}
function renderObsValue(name, value, ts) {
  const box = document.querySelector(`[data-obs="${name}"]`);
  if (!box) return;
  const obs = observations.find(o => o.name === name);
  if (obs?.type === "image") box.innerHTML = renderImage(value, ts);
  else if (obs?.type === "joint_state") box.innerHTML = renderJointState(value, ts);
  else box.innerHTML = `<div class="muted">${new Date((ts || 0) * 1000).toLocaleTimeString()}</div><pre>${JSON.stringify(value, null, 2)}</pre>`;
}
function bytesToDataUrl(bytes, encoding) {
  const b64 = bytes?.__bytes_base64;
  if (!b64) return "";
  const mime = encoding === "png" ? "image/png" : encoding === "jpg" || encoding === "jpeg" ? "image/jpeg" : "application/octet-stream";
  return `data:${mime};base64,${b64}`;
}
function renderImage(value, ts) {
  const src = typeof value?.image === "string" && value.image.startsWith("data:") ? value.image : bytesToDataUrl(value?.image, value?.encoding);
  return `<div class="muted">${value?.width || "?"}x${value?.height || "?"} ${value?.encoding || ""} ${new Date((ts || 0) * 1000).toLocaleTimeString()}</div>${src ? `<img class="obs-image" src="${src}">` : `<pre>${JSON.stringify(value, null, 2)}</pre>`}`;
}
function renderJointState(value, ts) {
  const arr = value?.joint_positions || [];
  const bars = arr.slice(0, 40).map((v, i) => {
    const pct = Math.max(0, Math.min(100, 50 + Number(v) / 3.14 * 50));
    return `<div class="bar"><span>j${i}</span><div class="track"><div class="fill" style="width:${pct}%"></div></div><span>${Number(v).toFixed(3)}</span></div>`;
  }).join("");
  return `<div class="muted">${arr.length} joints ${new Date((ts || 0) * 1000).toLocaleTimeString()}</div><div class="bars">${bars}</div>`;
}
function renderPolicies(activeId = null, lastError = "") {
  $("policies").innerHTML = policies.map(policy => {
    const id = "policy_" + policy.policy_id.replace(/[^a-zA-Z0-9_]/g, "_");
    return `<div class="card">
      <div class="head"><div><h3>${policy.name || policy.policy_id}</h3><div class="muted">${policy.description || ""}</div></div><span class="badge">${policy.policy_id === activeId ? "active" : "policy"}</span></div>
      <div id="${id}">${policyInputs(policy)}</div>
      <div class="row" style="margin-top:9px;"><button class="good" onclick="startPolicy('${policy.policy_id}', '${id}')">启动 policy</button><button onclick="updatePolicy('${policy.policy_id}', '${id}')">更新输入</button><button class="warn" onclick="stopPolicy('${policy.policy_id}')">停止 policy</button></div>
    </div>`;
  }).join("") || `<p class="muted">${lastError || "没有注册 policy_service 或暂无 policy。"}</p>`;
}
function policyInputs(policy) {
  const sample = {};
  const inputs = policy.inputs?.runtime_inputs || policy.inputs || {};
  for (const [key, spec] of Object.entries(inputs)) sample[key] = spec && typeof spec === "object" && "default" in spec ? spec.default : "";
  return `<label>runtime_inputs JSON</label><textarea data-policy-inputs>${JSON.stringify(sample, null, 2)}</textarea>`;
}
function readPolicyInputs(id) {
  return JSON.parse(document.querySelector(`#${id} [data-policy-inputs]`)?.value || "{}");
}
async function startPolicy(policyId, id) {
  let runtime_inputs;
  try { runtime_inputs = readPolicyInputs(id); }
  catch (err) { showError("policy JSON", {success:false, error:String(err)}); return; }
  await request("start_policy", {policy_id:policyId, runtime_inputs}).catch(err => showError("start_policy", err));
  await loadPolicies();
}
async function updatePolicy(policyId, id) {
  let runtime_inputs;
  try { runtime_inputs = readPolicyInputs(id); }
  catch (err) { showError("policy JSON", {success:false, error:String(err)}); return; }
  await request("update_policy_inputs", {policy_id:policyId, runtime_inputs}).catch(err => showError("update_policy_inputs", err));
  await loadPolicies();
}
async function stopPolicy(policyId) {
  await request("stop_policy", {policy_id:policyId, reason:"debug app"}).catch(err => showError("stop_policy", err));
  await loadPolicies();
}
function renderAll() { renderTools(); renderObservations(); renderActions(); renderPolicies(); refreshState(); }
$("configPath").value = DEFAULT_CONFIG;
renderAll();
connectServer().catch(() => refreshState());
</script>
</body>
</html>
"""


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[3]
    / "robots"
    / "roboparty_atom01"
    / "rynnrcp_robot_atom01"
    / "config"
    / "atom01_server.yaml"
)


class DebugApp:
    def __init__(self, default_config: str):
        self.default_config = str(Path(default_config).expanduser())
        self.server: Any | None = None
        self.client: RcpProtocolClient | None = None
        self.address = ""
        self.last_error = ""
        self.lock = threading.RLock()

    def state(self) -> dict[str, Any]:
        with self.lock:
            return {
                "running": self.server is not None,
                "connected": self.client is not None,
                "address": self.address,
                "default_config": self.default_config,
                "last_error": self.last_error,
            }

    def start_server(self, config_path: str | None = None) -> dict[str, Any]:
        with self.lock:
            self.stop_server(close_only=True)
            path = str(Path(config_path or self.default_config).expanduser())
            from rynnrcp.server import RynnRCPServer

            server = RynnRCPServer(path, log_session_id=new_log_session_id())
            server.start()
            self.server = server
            self.address = f"127.0.0.1:{server.bound_port}"
            self.client = RcpProtocolClient.connect(self.address)
            self.default_config = path
            self.last_error = ""
            return {"address": self.address, "config_path": path}

    def connect(self, address: str | None = None, config_path: str | None = None) -> dict[str, Any]:
        with self.lock:
            if self.client is not None:
                self.client.close()
                self.client = None
            path = str(Path(config_path or self.default_config).expanduser())
            target = (address or "").strip() or self._address_from_config(path)
            self.client = RcpProtocolClient.connect(target)
            self.address = target
            self.default_config = path
            self.last_error = ""
            response = self.client.ping({"source": "rynnrcp-protocol-debug"})
            return {"address": target, "ping": _response_dict(response)}

    def stop_server(self, *, close_only: bool = False) -> dict[str, Any]:
        if self.client is not None:
            self.client.close()
            self.client = None
        if not close_only and self.server is not None:
            self.server.stop()
            self.server = None
        if not close_only:
            self.address = ""
        return {"stopped": True}

    def request(self, method: str, payload: Any, timeout_ms: int | None = 1000) -> dict[str, Any]:
        with self.lock:
            if self.client is None:
                self.connect()
            assert self.client is not None
            response = self.client.request(method, payload, timeout_ms=timeout_ms)
            result = _response_dict(response)
            if not response.ok:
                self.last_error = response.message
            return result

    def run_action(self, name: str, value: Any, frame_rate: float = 1.0) -> dict[str, Any]:
        return self.request(
            "run_action_chunk",
            {"name": name, "frames": [value], "frame_rate": float(frame_rate)},
        )

    def run_action_async(
        self,
        name: str,
        value: Any,
        frame_rate: float = 1.0,
        frames: int = 3,
    ) -> dict[str, Any]:
        with self.lock:
            if self.client is None:
                self.connect()
            assert self.client is not None
            stream = self.client.run_action_chunk_async(
                str(name),
                [value],
                frame_rate=float(frame_rate),
            )
            return {
                "success": True,
                "method": "run_action_chunk_async",
                "frames": _collect_stream(stream, frames),
            }

    def subscribe_observations(
        self,
        names: list[Any],
        *,
        stream_hz: float = 10.0,
        frames: int = 5,
        sync: bool = False,
    ) -> dict[str, Any]:
        with self.lock:
            if self.client is None:
                self.connect()
            assert self.client is not None
            stream = self.client.subscribe_observations(
                [str(name) for name in names],
                sync=bool(sync),
                stream_hz=float(stream_hz),
            )
            return {
                "success": True,
                "method": "subscribe_observations",
                "frames": _collect_stream(stream, frames),
            }

    def subscribe_health(self, *, stream_hz: float = 2.0, frames: int = 3) -> dict[str, Any]:
        with self.lock:
            if self.client is None:
                self.connect()
            assert self.client is not None
            stream = self.client.subscribe_health(stream_hz=float(stream_hz))
            return {
                "success": True,
                "method": "subscribe_health",
                "frames": _collect_stream(stream, frames),
            }

    def _address_from_config(self, config_path: str) -> str:
        cfg = _load_yaml(config_path)
        interface = (((cfg.get("server") or {}).get("interface")) or {})
        host = str(interface.get("host") or "127.0.0.1")
        port = int(interface.get("port") or 0)
        if port:
            return f"{host}:{port}"
        robot_id = str((cfg.get("manifest") or {}).get("robot_id") or "")
        for endpoint in LocalRegistry().discover(timeout_s=0.5):
            meta = endpoint.metadata or {}
            if endpoint.endpoint_id == robot_id or meta.get("robot_id") == robot_id or meta.get("server_id") == robot_id:
                return endpoint.address
        client = ClientInterface.with_defaults(local_registry=True, mdns=bool(interface.get("mdns", False)))
        endpoints = client.discover(timeout_s=0.8)
        if robot_id:
            for endpoint in endpoints:
                meta = endpoint.metadata or {}
                if endpoint.endpoint_id == robot_id or meta.get("robot_id") == robot_id or meta.get("server_id") == robot_id:
                    return endpoint.address
        if endpoints:
            return endpoints[0].address
        raise RuntimeError(f"no endpoint found for config: {config_path}")


class Handler(BaseHTTPRequestHandler):
    app: DebugApp

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            html = HTML.replace("__DEFAULT_CONFIG__", json.dumps(self.app.default_config))
            self._send(200, html.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/api/state":
            self._json(200, self.app.state())
            return
        self._json(404, {"success": False, "error": "not found"})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        body = self._read_json()
        try:
            if path == "/api/server/start":
                self._ok(self.app.start_server(body.get("config_path")))
            elif path == "/api/server/connect":
                self._ok(self.app.connect(body.get("address"), body.get("config_path")))
            elif path == "/api/server/stop":
                self._ok(self.app.stop_server())
            elif path == "/api/protocol":
                timeout_ms = body.get("timeout_ms", 1000)
                self._response(self.app.request(str(body["method"]), body.get("payload"), timeout_ms=timeout_ms))
            elif path == "/api/action":
                self._response(self.app.run_action(str(body["name"]), body.get("value") or {}, float(body.get("frame_rate") or 1)))
            elif path == "/api/action/async":
                self._ok(self.app.run_action_async(
                    str(body["name"]),
                    body.get("value") or {},
                    float(body.get("frame_rate") or 1),
                    int(body.get("frames") or 3),
                ))
            elif path == "/api/observation":
                timeout_ms = body.get("timeout_ms", 1000)
                self._response(self.app.request("get_observations", {"names": body.get("names") or [], "sync": bool(body.get("sync", False))}, timeout_ms=timeout_ms))
            elif path == "/api/observation/subscribe":
                self._ok(self.app.subscribe_observations(
                    body.get("names") or [],
                    stream_hz=float(body.get("stream_hz") or 10),
                    frames=int(body.get("frames") or 5),
                    sync=bool(body.get("sync", False)),
                ))
            elif path == "/api/health/subscribe":
                self._ok(self.app.subscribe_health(
                    stream_hz=float(body.get("stream_hz") or 2),
                    frames=int(body.get("frames") or 3),
                ))
            else:
                self._json(404, {"success": False, "error": "not found"})
        except Exception as exc:
            self.app.last_error = str(exc)
            logger.exception(
                "[ProtocolDebug][REQUEST_FAILED] path=%s error=%s; "
                "inspect the selected server config and server logs for the same run_id",
                path,
                exc,
            )
            self._json(500, {"success": False, "error": str(exc)})

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("content-length") or 0)
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def _ok(self, data: Any) -> None:
        self._json(200, {"success": True, **(_jsonable(data) if isinstance(data, dict) else {"result": _jsonable(data)})})

    def _response(self, response: dict[str, Any]) -> None:
        code = 200 if int(response.get("status", 1)) == 0 else 500
        self._json(code, {"success": code == 200, "response": response})

    def _json(self, status: int, data: Any) -> None:
        self._send(status, json.dumps(_jsonable(data), ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def _send(self, status: int, data: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("content-type", content_type)
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def _load_yaml(path: str) -> dict[str, Any]:
    text = Path(path).expanduser().read_text(encoding="utf-8")
    if yaml is None:
        return _load_config_subset(text)
    data = yaml.safe_load(text) or {}
    if not isinstance(data, dict):
        raise ValueError(f"config is not a mapping: {path}")
    return data


def _load_config_subset(text: str) -> dict[str, Any]:
    result: dict[str, Any] = {"manifest": {}, "server": {"interface": {}}}
    section: list[str] = []
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or ":" not in line:
            continue
        indent = len(line) - len(line.lstrip(" "))
        key, value = line.strip().split(":", 1)
        value = value.strip()
        if indent == 0:
            section = [key]
            continue
        if indent == 2:
            section = [section[0], key] if section else [key]
            if value:
                _set_subset(result, section, value)
            continue
        if indent == 4 and len(section) >= 2:
            _set_subset(result, [section[0], section[1], key], value)
    return result


def _set_subset(root: dict[str, Any], path: list[str], value: str) -> None:
    target = root
    for key in path[:-1]:
        target = target.setdefault(key, {})
    if value in {"true", "false"}:
        parsed: Any = value == "true"
    elif value in {"null", "~", ""}:
        parsed = None
    else:
        try:
            parsed = int(value)
        except ValueError:
            parsed = value.strip('"\'')
    target[path[-1]] = parsed


def _response_dict(response: Any) -> dict[str, Any]:
    return _jsonable(response.to_dict())


def _collect_stream(stream: Any, frames: int) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    limit = max(1, int(frames))
    try:
        for _ in range(limit):
            try:
                result.append(_response_dict(next(stream)))
            except StopIteration:
                break
    finally:
        cancel = getattr(stream, "cancel", None)
        if cancel is not None:
            cancel()
    return result


def _jsonable(value: Any) -> Any:
    if isinstance(value, bytes):
        return {"__bytes_base64": base64.b64encode(value).decode("ascii")}
    if isinstance(value, bytearray):
        return {"__bytes_base64": base64.b64encode(bytes(value)).decode("ascii")}
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    try:
        import numpy as np

        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
    except Exception:
        pass
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the RynnRCP protocol debug web app.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Default server config path.")
    parser.add_argument("--host", default="0.0.0.0", help="Web app host.")
    parser.add_argument("--port", type=int, default=8091, help="Web app port.")
    parser.add_argument("--no-open", action="store_true", help="Do not open the browser.")
    args = parser.parse_args(argv)

    app_id = "protocol_debug"
    set_log_context(
        app_id=app_id,
        run_id=resolve_log_run_id(),
        process="protocol_debug_app",
    )
    configure_logging(
        sinks=["stderr", "file"],
        file_path=str(logs_dir(app_root(app_id)) / "protocol_debug.log"),
    )
    logger.info(
        "[ProtocolDebug][START] host=%s port=%d default_config=%s",
        args.host,
        args.port,
        args.config,
    )

    Handler.app = DebugApp(args.config)
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    urls = browser_urls(args.host, httpd.server_port)
    print(f"rynnrcp-protocol-debug Local: {urls[0]}")
    for url in urls[1:]:
        print(f"rynnrcp-protocol-debug LAN:   {url}")
    print(f"default config: {Handler.app.default_config}")
    if not args.no_open:
        threading.Timer(0.4, lambda: webbrowser.open(primary_browser_url(args.host, httpd.server_port))).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        return 130
    finally:
        Handler.app.stop_server()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
