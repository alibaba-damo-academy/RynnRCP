#!/usr/bin/env python3
"""Browser configuration and supervised hardware test tool for Astribot S1."""

from __future__ import annotations

import argparse
import logging
import math
import socket
import threading
import webbrowser
from collections.abc import Callable, Iterable, Mapping
from pathlib import Path
from typing import Any

import yaml

from rynnrcp.utils.web_urls import browser_urls, primary_browser_url

from .controller import AstribotS1Controller


LOGGER = logging.getLogger("rynnrcp.astribot_s1.configure_web")
PACKAGE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = PACKAGE_DIR / "config" / "astribot_s1_server.yaml"
RYNNBOT_CONFIG_PATH = PACKAGE_DIR / "config" / "astribot_s1_rynnbot_app.yaml"
GRIPPER_INDICES = {11, 19}
CONTROL_CONFIRMATION = "S1_CONTROL_CONFIRMED"
RYNNBOT_APP_ID = "astribot_s1_rynnbot_app"
RYNNBOT_FIELDS = ("product_key", "device_name", "device_secret", "http_url")
JOINT_NAMES = tuple(
    [
        *(f"torso_{index}" for index in range(4)),
        *(f"left_arm_{index}" for index in range(7)),
        "left_gripper",
        *(f"right_arm_{index}" for index in range(7)),
        "right_gripper",
        *(f"head_{index}" for index in range(2)),
    ]
)

HTML_TEMPLATE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Astribot S1 配置 - RynnRCP</title>
  <style>
    * { box-sizing: border-box; }
    body { margin:0; min-height:100vh; color:#e8e8e8; background:linear-gradient(135deg,#111827,#172554); font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
    main { max-width:1100px; margin:0 auto; padding:24px; }
    header { text-align:center; margin-bottom:20px; }
    h1,h2 { color:#67e8f9; }
    h1 { margin:0; font-size:28px; }
    h2 { margin:0 0 14px; font-size:19px; }
    .card { margin:16px 0; padding:20px; border:1px solid #334155; border-radius:12px; background:#111827dd; box-shadow:0 4px 20px #0005; }
    .grid { display:grid; grid-template-columns:1fr 1fr; gap:14px; }
    label { display:block; color:#cbd5e1; }
    input[type="text"],input[type="number"],input[type="password"] { width:100%; margin-top:6px; padding:9px 11px; border:1px solid #475569; border-radius:7px; color:#fff; background:#0f172a; }
    button { padding:9px 14px; border:0; border-radius:7px; color:#fff; background:#087ea4; cursor:pointer; font-weight:650; }
    button.secondary { background:#475569; }
    button.danger { background:#b91c1c; }
    button:disabled,input:disabled { opacity:.45; cursor:not-allowed; }
    .buttons { display:flex; gap:10px; flex-wrap:wrap; margin-top:14px; }
    .notice { padding:12px 14px; border-left:4px solid #fbbf24; border-radius:5px; color:#fde68a; background:#422006; }
    .status { color:#cbd5e1; }
    .ok { color:#34d399; }
    .error { color:#fb7185; }
    table { width:100%; border-collapse:collapse; margin-top:12px; }
    th,td { padding:8px; border-bottom:1px solid #334155; text-align:right; }
    th:first-child,td:first-child { text-align:left; }
    .value { color:#67e8f9; font-family:"SFMono-Regular",Consolas,monospace; }
    .slider { width:min(460px,48vw); accent-color:#22d3ee; }
    .raw { color:#94a3b8; }
    @media(max-width:720px){main{padding:12px}.grid{grid-template-columns:1fr}.slider{width:42vw}th,td{padding:6px 3px}}
  </style>
</head>
<body>
<main>
  <header>
    <h1>Astribot S1 配置</h1>
    <p>保存 SDK 路径，连接机器人读取状态，并在现场监护下进行单关节与夹爪测试。</p>
  </header>

  <section class="card">
    <h2>1. 连接配置</h2>
    <div class="grid">
      <label>SDK 目录
        <input id="sdk-root" type="text" autocomplete="off">
      </label>
      <label style="padding-top:30px">
        <input id="high-control-rights" type="checkbox"> RynnRCP Server 启动时自动接管控制权
      </label>
    </div>
    <div class="buttons">
      <button onclick="saveConfig()">保存配置</button>
      <button onclick="connectRobot()">连接并读取状态（不抢占）</button>
      <button class="secondary" onclick="disconnectRobot()">断开连接</button>
      <button class="danger" id="stop-button" onclick="stopRobot()" disabled>停止运动</button>
    </div>
    <p id="connection-status" class="status">正在读取配置...</p>
  </section>

  <section class="card">
    <h2>2. RynnBot 云端配置</h2>
    <div class="grid">
      <label>Product Key
        <input id="product-key" type="text" autocomplete="off">
      </label>
      <label>Device Name
        <input id="device-name" type="text" autocomplete="off">
      </label>
      <label>Device Secret
        <input id="device-secret" type="password" autocomplete="off">
      </label>
      <label>HTTP URL
        <input id="http-url" type="text" autocomplete="off">
      </label>
    </div>
    <div class="buttons">
      <button onclick="saveConfig()">保存 RynnBot 配置</button>
    </div>
    <p id="rynnbot-status" class="status">正在读取 RynnBot 配置...</p>
  </section>

  <section class="card">
    <h2>3. 实时状态</h2>
    <p>底盘：<span id="chassis-state" class="value">连接后显示</span></p>
    <p>健康状态：<span id="health-state" class="value">连接后显示</span></p>
    <table>
      <thead><tr><th>关节</th><th>当前位置</th><th>单关节目标</th></tr></thead>
      <tbody id="joint-rows"><tr><td colspan="3">连接后显示 22 维状态</td></tr></tbody>
    </table>
  </section>

  <section class="card">
    <h2>4. 解锁运动测试</h2>
    <p class="notice">先完成上电、启动机器人驱动并长按进入初始姿势。确认工作空间已清空、急停可触达且现场有人监护，再解锁控制。若其他客户端持有控制权，接管会立即停止其当前运动。拖动滑块时目标会由 100 Hz 后台控制平滑跟随，每次只测试一个关节或夹爪。</p>
    <label><input id="safety-confirmed" type="checkbox" onchange="updateUnlockButton()"> 我已确认机器人周围安全，并准备好使用急停</label>
    <div class="buttons">
      <button id="unlock-button" onclick="setMotionEnabled(true)" disabled>解锁运动控制</button>
      <button class="secondary" id="lock-button" onclick="setMotionEnabled(false)" disabled>锁定运动控制</button>
    </div>
    <div class="grid" style="margin-top:16px">
      <label>左夹爪标准化目标（0 打开，1 闭合）
        <input id="left-gripper" type="range" min="0" max="1" step="0.01" value="0" disabled oninput="queueGripper('left',this.value)">
      </label>
      <label>右夹爪标准化目标（0 打开，1 闭合）
        <input id="right-gripper" type="range" min="0" max="1" step="0.01" value="0" disabled oninput="queueGripper('right',this.value)">
      </label>
    </div>
    <p>夹爪反馈（标准化值与 SDK 原始值）：左 <span id="left-gripper-raw" class="value">--</span>，右 <span id="right-gripper-raw" class="value">--</span></p>
  </section>
</main>
<script>
  let connected = false;
  let hasControlRights = false;
  let motionEnabled = false;
  let pollTimer = null;
  let requestInFlight = false;
  let pendingCommand = null;
  let commandTimer = null;
  const COMMAND_INTERVAL_MS = 33;

  async function api(path, options = {}) {
    const response = await fetch(path, {headers:{"Content-Type":"application/json"}, ...options});
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || "请求失败");
    return data;
  }

  function message(text, kind = "") {
    const target = document.getElementById("connection-status");
    target.textContent = text;
    target.className = "status " + kind;
  }

  async function loadConfig() {
    try {
      const data = await api("/api/config");
      document.getElementById("sdk-root").value = data.config.sdk_root;
      document.getElementById("high-control-rights").checked = data.config.high_control_rights;
      const rb = data.config.rynnbot || {};
      document.getElementById("product-key").value = rb.product_key || "";
      document.getElementById("device-name").value = rb.device_name || "";
      document.getElementById("device-secret").value = rb.device_secret || "";
      document.getElementById("http-url").value = rb.http_url || "https://robot-access.damo-academy.com";
      updateRynnBotStatus(rb);
      render(data.status);
    } catch (error) { message(error.message, "error"); }
  }

  async function saveConfig() {
    try {
      const payload = {
        sdk_root:document.getElementById("sdk-root").value.trim(),
        high_control_rights:document.getElementById("high-control-rights").checked,
        rynnbot:{
          product_key:document.getElementById("product-key").value.trim(),
          device_name:document.getElementById("device-name").value.trim(),
          device_secret:document.getElementById("device-secret").value.trim(),
          http_url:document.getElementById("http-url").value.trim() || "https://robot-access.damo-academy.com"
        }
      };
      const data = await api("/api/config", {method:"POST", body:JSON.stringify(payload)});
      render(data.status);
      updateRynnBotStatus(payload.rynnbot);
      message("配置已保存。", "ok");
    } catch (error) { message(error.message, "error"); }
  }

  async function connectRobot() {
    message("正在连接机器人并读取状态...");
    try {
      const data = await api("/api/connect", {method:"POST", body:"{}"});
      render(data.status);
      startPolling();
      message(data.status.has_control_rights
        ? "机器人已连接并持有控制权；运动控制保持锁定。"
        : "机器人已连接；控制权由其他客户端持有，当前只读取状态。", "ok");
    } catch (error) { message(error.message, "error"); }
  }

  async function disconnectRobot() {
    clearPendingCommand();
    try {
      const data = await api("/api/disconnect", {method:"POST", body:"{}"});
      stopPolling();
      render(data.status);
      message("机器人连接已断开。");
    } catch (error) { message(error.message, "error"); }
  }

  async function setMotionEnabled(enabled) {
    if (!enabled) clearPendingCommand();
    try {
      const data = await api("/api/control", {method:"POST", body:JSON.stringify({
        enabled,
        confirmation:enabled ? "S1_CONTROL_CONFIRMED" : ""
      })});
      render(data.status);
      message(enabled
        ? "控制权已取得，运动控制已解锁；请从单个小步目标开始。"
        : "运动控制已锁定。", enabled ? "ok" : "");
    } catch (error) {
      clearPendingCommand();
      motionEnabled = false;
      message(error.message, "error");
    }
  }

  function queueJoint(index, value) {
    if (!motionEnabled) return;
    pendingCommand = {kind:"joint", index, value:Number(value)};
    scheduleCommand();
  }

  function queueGripper(side, value) {
    if (!motionEnabled) return;
    pendingCommand = {kind:"gripper", side, value:Number(value)};
    scheduleCommand();
  }

  function scheduleCommand() {
    if (commandTimer !== null || requestInFlight || pendingCommand === null || !motionEnabled) return;
    commandTimer = setTimeout(flushCommand, COMMAND_INTERVAL_MS);
  }

  function clearPendingCommand() {
    pendingCommand = null;
    if (commandTimer !== null) clearTimeout(commandTimer);
    commandTimer = null;
  }

  async function flushCommand() {
    commandTimer = null;
    if (!motionEnabled || pendingCommand === null) return;
    const command = pendingCommand;
    pendingCommand = null;
    requestInFlight = true;
    try {
      const data = command.kind === "joint"
        ? await api("/api/joint", {method:"POST", body:JSON.stringify({index:command.index, value:command.value})})
        : await api("/api/gripper", {method:"POST", body:JSON.stringify({side:command.side, position:command.value})});
      if (command.kind === "joint") {
        const applied = Number(data.command.target);
        if (!Number.isFinite(applied)) throw new Error("关节目标响应无效");
      }
    } catch (error) {
      clearPendingCommand();
      message(error.message, "error");
    } finally {
      requestInFlight = false;
      scheduleCommand();
    }
  }

  async function stopRobot() {
    clearPendingCommand();
    try {
      const data = await api("/api/stop", {method:"POST", body:"{}"});
      render(data.status);
      message("停止命令已发送，运动控制已锁定。");
    } catch (error) { message(error.message, "error"); }
  }

  async function poll() {
    if (!connected || requestInFlight) return;
    try { render((await api("/api/status")).status); }
    catch (error) { message(error.message, "error"); }
  }

  function startPolling() {
    stopPolling();
    pollTimer = setInterval(poll, 500);
  }

  function stopPolling() {
    if (pollTimer !== null) clearInterval(pollTimer);
    pollTimer = null;
  }

  function updateUnlockButton() {
    document.getElementById("unlock-button").disabled =
      !connected || motionEnabled || !document.getElementById("safety-confirmed").checked;
    document.getElementById("unlock-button").textContent =
      hasControlRights ? "解锁运动控制" : "接管并解锁控制";
  }

  function updateRynnBotStatus(app) {
    const configured = ["product_key","device_name","device_secret"].every(
      key => app[key] && !String(app[key]).startsWith("YOUR_")
    );
    const target = document.getElementById("rynnbot-status");
    target.textContent = configured
      ? "RynnBot 云端凭据已填写。"
      : "填写 Product Key、Device Name 和 Device Secret 后保存。";
    target.className = "status " + (configured ? "ok" : "");
  }

  function render(status) {
    connected = !!status.connected;
    hasControlRights = !!status.has_control_rights;
    motionEnabled = !!status.motion_enabled && hasControlRights;
    if (!motionEnabled) clearPendingCommand();
    document.getElementById("stop-button").disabled = !connected || !hasControlRights;
    document.getElementById("lock-button").disabled = !connected || !motionEnabled;
    document.getElementById("left-gripper").disabled = !motionEnabled;
    document.getElementById("right-gripper").disabled = !motionEnabled;
    updateUnlockButton();
    if (connected) {
      document.getElementById("connection-status").textContent =
        `机器人已连接；控制权：${hasControlRights ? "已持有" : "由其他客户端持有"}；运动控制：${motionEnabled ? "已解锁" : "已锁定"}`;
      document.getElementById("connection-status").className = "status " + (hasControlRights ? "ok" : "");
    }
    if (!connected) {
      document.getElementById("joint-rows").innerHTML = '<tr><td colspan="3">连接后显示 22 维状态</td></tr>';
      document.getElementById("chassis-state").textContent = "连接后显示";
      document.getElementById("health-state").textContent = "连接后显示";
      return;
    }
    const positions = status.positions || [];
    const limits = status.limits || [];
    const targets = status.targets || positions;
    const rows = document.getElementById("joint-rows");
    if (!document.getElementById("joint-position-0")) {
      rows.innerHTML = status.joint_names.map((name,index) => {
        const value = Number(positions[index] || 0);
        if (status.gripper_indices.includes(index)) {
          return `<tr><td>${name}</td><td id="joint-position-${index}" class="value">${value.toFixed(5)}</td><td class="raw">使用下方独立夹爪控制</td></tr>`;
        }
        const limit = limits[index] || [-3.14,3.14];
        return `<tr><td>${name}</td><td id="joint-position-${index}" class="value">${value.toFixed(5)}</td><td><input id="joint-slider-${index}" class="slider" type="range" min="${limit[0]}" max="${limit[1]}" step="0.005" value="${targets[index]}" ${motionEnabled ? "" : "disabled"} oninput="queueJoint(${index},this.value)"></td></tr>`;
      }).join("");
    }
    status.joint_names.forEach((_,index) => {
      const value = Number(positions[index] || 0);
      document.getElementById(`joint-position-${index}`).textContent = value.toFixed(5);
      if (status.gripper_indices.includes(index)) {
        return;
      }
      const slider = document.getElementById(`joint-slider-${index}`);
      const limit = limits[index] || [-3.14,3.14];
      slider.min = limit[0];
      slider.max = limit[1];
      slider.disabled = !motionEnabled;
      if (document.activeElement !== slider) slider.value = targets[index];
    });
    const chassis = status.chassis || {};
    document.getElementById("chassis-state").textContent = JSON.stringify(chassis);
    const health = status.health || {errors:[],warnings:[]};
    document.getElementById("health-state").textContent = `错误 ${health.errors.length}，警告 ${health.warnings.length}`;
    document.getElementById("left-gripper-raw").textContent = JSON.stringify(status.left_gripper || {});
    document.getElementById("right-gripper-raw").textContent = JSON.stringify(status.right_gripper || {});
  }

  window.addEventListener("beforeunload", () => {
    clearPendingCommand();
    stopPolling();
  });
  loadConfig();
</script>
</body>
</html>
"""


def load_server_config(path: Path = CONFIG_PATH) -> dict[str, Any]:
    return _load_yaml_object(path, "Server 配置")


def save_server_config(config: Mapping[str, Any], path: Path = CONFIG_PATH) -> None:
    _save_yaml_object(path, config)


def load_rynnbot_config(path: Path = RYNNBOT_CONFIG_PATH) -> dict[str, Any]:
    return _load_yaml_object(path, "RynnBot 配置")


def save_rynnbot_config(
    config: Mapping[str, Any], path: Path = RYNNBOT_CONFIG_PATH
) -> None:
    _save_yaml_object(path, config)


def _load_yaml_object(path: Path, label: str) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{label}文件必须是 YAML 对象：{path}")
    return data


def _save_yaml_object(path: Path, config: Mapping[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(dict(config), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(path)


def _rynnbot_app(config: Mapping[str, Any]) -> Mapping[str, Any]:
    app = config.get("app")
    if not isinstance(app, Mapping):
        raise ValueError("RynnBot 配置中的 app 必须是 YAML 对象")
    return app


def _updated_rynnbot_config(
    config: dict[str, Any], values: Mapping[str, Any]
) -> dict[str, Any]:
    app = dict(_rynnbot_app(config))
    app["app_id"] = RYNNBOT_APP_ID
    for key in RYNNBOT_FIELDS:
        if key in values:
            app[key] = str(values[key]).strip()
    config["app"] = app
    return config


def _robot_config(config: Mapping[str, Any]) -> Mapping[str, Any]:
    components = config.get("components")
    if not isinstance(components, Mapping):
        raise ValueError("components 必须是 YAML 对象")
    robot = components.get("robot")
    if not isinstance(robot, Mapping):
        raise ValueError("components.robot 必须是 YAML 对象")
    return robot


def _controller_kwargs(config: Mapping[str, Any]) -> dict[str, Any]:
    robot = _robot_config(config)
    manifest = config.get("manifest")
    if not isinstance(manifest, Mapping):
        raise ValueError("manifest 必须是 YAML 对象")
    return {
        "robot_id": str(manifest["robot_id"]),
        "sdk_root": str(robot["sdk_root"]),
        "frequency_hz": float(robot["frequency_hz"]),
        "high_control_rights": bool(robot["high_control_rights"]),
        "node_name": str(robot["node_name"]),
        "control_way": str(robot["control_way"]),
        "use_wbc": bool(robot["use_wbc"]),
        "activate_cameras": bool(robot["activate_cameras"]),
        "base_command_dt_s": float(robot["base_command_dt_s"]),
        "max_base_x_m_s": float(robot["max_base_x_m_s"]),
        "max_base_y_m_s": float(robot["max_base_y_m_s"]),
        "max_base_yaw_rad_s": float(robot["max_base_yaw_rad_s"]),
    }


def _gripper_state_from_joint_state(
    state: Mapping[str, list[float]], index: int
) -> dict[str, float]:
    position = float(state["joint_positions"][index])
    velocity = float(state["joint_velocities"][index])
    return {
        "position": position,
        "velocity": velocity,
        "sdk_joint_position": position * 100.0,
        "sdk_joint_velocity": velocity * 100.0,
    }


class ConfigureSession:
    """Own one supervised Astribot connection used by the configuration page."""

    def __init__(
        self,
        config_path: Path = CONFIG_PATH,
        controller_factory: Callable[..., AstribotS1Controller] = AstribotS1Controller,
        rynnbot_config_path: Path | None = None,
    ) -> None:
        self.config_path = config_path
        self.rynnbot_config_path = (
            rynnbot_config_path
            or config_path.with_name("astribot_s1_rynnbot_app.yaml")
        )
        self.controller_factory = controller_factory
        self._controller: AstribotS1Controller | None = None
        self._targets: list[float] | None = None
        self._limits: list[list[float]] | None = None
        self._motion_enabled = False
        self._lock = threading.RLock()

    def config_snapshot(self) -> dict[str, Any]:
        robot = _robot_config(load_server_config(self.config_path))
        app = _rynnbot_app(load_rynnbot_config(self.rynnbot_config_path))
        return {
            "sdk_root": str(robot["sdk_root"]),
            "high_control_rights": bool(robot["high_control_rights"]),
            "rynnbot": dict(app),
        }

    def save_config(
        self,
        sdk_root: str,
        high_control_rights: bool,
        rynnbot: Mapping[str, Any] | None = None,
    ) -> None:
        sdk_path = Path(str(sdk_root).strip()).expanduser()
        if not sdk_path.is_absolute():
            raise ValueError("SDK 目录必须是绝对路径")
        with self._lock:
            if self._controller is not None:
                raise RuntimeError("请先断开机器人，再修改连接配置")
            server_config = load_server_config(self.config_path)
            robot = _robot_config(server_config)
            robot["sdk_root"] = str(sdk_path)
            robot["high_control_rights"] = bool(high_control_rights)
            rynnbot_config = None
            if rynnbot is not None:
                rynnbot_config = _updated_rynnbot_config(
                    load_rynnbot_config(self.rynnbot_config_path),
                    rynnbot,
                )
            save_server_config(server_config, self.config_path)
            if rynnbot_config is not None:
                save_rynnbot_config(rynnbot_config, self.rynnbot_config_path)

    def connect(self) -> dict[str, Any]:
        with self._lock:
            if self._controller is not None:
                return self.status()
            self._connect_locked(force_control=False)
            self._motion_enabled = False
            return self.status()

    def disconnect(self) -> dict[str, Any]:
        with self._lock:
            controller, self._controller = self._controller, None
            self._targets = None
            self._limits = None
            self._motion_enabled = False
        if controller is not None:
            controller.shutdown()
        return self.status()

    def set_motion_enabled(self, enabled: bool, confirmation: str = "") -> dict[str, Any]:
        with self._lock:
            controller = self._require_controller()
            if enabled and confirmation != CONTROL_CONFIRMATION:
                raise ValueError("请先确认现场安全，再解锁运动控制")
            if enabled:
                if not controller.has_control_rights():
                    self._connect_locked(force_control=True)
                    controller = self._require_controller()
                if not controller.has_control_rights():
                    raise RuntimeError("接管失败，机器人仍由其他客户端控制")
                self._targets = list(
                    controller.get_joint_positions()["joint_positions"]
                )
            else:
                controller.clear_command()
            self._motion_enabled = bool(enabled)
            return self.status()

    def set_joint(self, index: int, value: float) -> dict[str, Any]:
        if index not in range(len(JOINT_NAMES)) or index in GRIPPER_INDICES:
            raise ValueError("请选择躯干、手臂或头部关节")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("关节目标必须是有限数值")
        with self._lock:
            controller = self._require_motion_enabled()
            targets = list(
                self._targets or controller.get_joint_positions()["joint_positions"]
            )
            low, high = (self._limits or [])[index]
            target = max(low, min(high, value))
            controller.set_single_joint_position(index, target)
            targets[index] = target
            self._targets = targets
            return {"index": index, "target": target}

    def set_gripper(self, side: str, position: float) -> dict[str, Any]:
        position = float(position)
        if not math.isfinite(position) or not 0.0 <= position <= 1.0:
            raise ValueError("夹爪目标必须在 0 到 1 之间")
        with self._lock:
            controller = self._require_motion_enabled()
            if side == "left":
                controller.set_left_gripper({"position": position})
            elif side == "right":
                controller.set_right_gripper({"position": position})
            else:
                raise ValueError("夹爪侧必须是 left 或 right")
            return {"side": side, "position": position}

    def stop(self) -> dict[str, Any]:
        with self._lock:
            controller = self._require_controller()
            if not controller.has_control_rights():
                self._motion_enabled = False
                raise RuntimeError("当前未持有机器人控制权，无法发送停止命令")
            controller.stop({})
            self._motion_enabled = False
            return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            controller = self._controller
            result: dict[str, Any] = {
                "connected": controller is not None,
                "has_control_rights": False,
                "motion_enabled": self._motion_enabled,
                "joint_names": list(JOINT_NAMES),
                "gripper_indices": sorted(GRIPPER_INDICES),
                "positions": None,
                "velocities": None,
                "targets": list(self._targets) if self._targets is not None else None,
                "limits": list(self._limits) if self._limits is not None else [],
                "left_gripper": None,
                "right_gripper": None,
                "chassis": None,
                "health": {"errors": [], "warnings": []},
            }
            if controller is not None:
                has_control_rights = controller.has_control_rights()
                result["has_control_rights"] = has_control_rights
                if not has_control_rights:
                    controller.clear_command()
                    self._motion_enabled = False
                    result["motion_enabled"] = False
                state = controller.get_joint_positions()
                result["positions"] = state["joint_positions"]
                result["velocities"] = state["joint_velocities"]
                result["left_gripper"] = _gripper_state_from_joint_state(state, 11)
                result["right_gripper"] = _gripper_state_from_joint_state(state, 19)
                result["chassis"] = controller.get_chassis_state()
                result["health"] = controller.get_health()
            return result

    def _require_controller(self) -> AstribotS1Controller:
        if self._controller is None:
            raise RuntimeError("请先连接机器人")
        return self._controller

    def _require_motion_enabled(self) -> AstribotS1Controller:
        controller = self._require_controller()
        if not self._motion_enabled:
            raise RuntimeError("请先确认现场安全并解锁运动控制")
        if not controller.has_control_rights():
            controller.clear_command()
            self._motion_enabled = False
            raise RuntimeError("机器人控制权已转移，请重新接管并解锁控制")
        return controller

    def _connect_locked(self, force_control: bool) -> None:
        previous, self._controller = self._controller, None
        self._targets = None
        self._limits = None
        self._motion_enabled = False
        if previous is not None:
            previous.shutdown()
        kwargs = _controller_kwargs(load_server_config(self.config_path))
        kwargs["high_control_rights"] = force_control
        kwargs["control_rights_mode"] = "force" if force_control else "read_only"
        controller = self.controller_factory(**kwargs)
        try:
            controller.start()
            state = controller.get_joint_positions()
            limits = controller.get_joint_limits()
        except Exception:
            controller.shutdown()
            raise
        self._controller = controller
        self._targets = list(state["joint_positions"])
        self._limits = [
            [float(low), float(high)]
            for low, high in zip(limits["lower"], limits["upper"])
        ]


SESSION = ConfigureSession()


def create_app(session: ConfigureSession | None = None):
    try:
        from flask import Flask, jsonify, render_template_string, request
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "Astribot S1 配置页面需要 Flask，请重新运行 setup_astribot_s1.sh"
        ) from exc

    active_session = session or SESSION
    app = Flask(__name__)

    @app.get("/")
    def index():
        return render_template_string(HTML_TEMPLATE)

    @app.get("/api/config")
    def api_config():
        return jsonify(
            {
                "ok": True,
                "config": active_session.config_snapshot(),
                "status": active_session.status(),
            }
        )

    @app.post("/api/config")
    def api_save_config():
        payload = request.get_json(silent=True) or {}
        return _api_call(
            lambda: active_session.save_config(
                payload.get("sdk_root", ""),
                bool(payload.get("high_control_rights", False)),
                payload.get("rynnbot")
                if isinstance(payload.get("rynnbot"), Mapping)
                else None,
            ),
            active_session,
        )

    @app.post("/api/connect")
    def api_connect():
        return _api_call(active_session.connect, active_session)

    @app.post("/api/disconnect")
    def api_disconnect():
        return _api_call(active_session.disconnect, active_session)

    @app.get("/api/status")
    def api_status():
        return _api_call(active_session.status, active_session)

    @app.post("/api/control")
    def api_control():
        payload = request.get_json(silent=True) or {}
        return _api_call(
            lambda: active_session.set_motion_enabled(
                bool(payload.get("enabled", False)),
                str(payload.get("confirmation", "")),
            ),
            active_session,
        )

    @app.post("/api/joint")
    def api_joint():
        payload = request.get_json(silent=True) or {}
        return _api_command_call(
            lambda: active_session.set_joint(
                int(payload.get("index", -1)),
                float(payload.get("value")),
            ),
        )

    @app.post("/api/gripper")
    def api_gripper():
        payload = request.get_json(silent=True) or {}
        return _api_command_call(
            lambda: active_session.set_gripper(
                str(payload.get("side", "")),
                float(payload.get("position")),
            ),
        )

    @app.post("/api/stop")
    def api_stop():
        return _api_call(active_session.stop, active_session)

    return app


def _api_call(
    operation: Callable[[], Any], session: ConfigureSession
) -> tuple[Any, int] | Any:
    from flask import jsonify

    try:
        result = operation()
        status = (
            result
            if isinstance(result, dict) and "connected" in result
            else session.status()
        )
        return jsonify({"ok": True, "status": status})
    except Exception as exc:
        LOGGER.exception("Astribot S1 配置请求失败")
        return jsonify({"ok": False, "error": str(exc)}), 400


def _api_command_call(operation: Callable[[], Any]) -> tuple[Any, int] | Any:
    from flask import jsonify

    try:
        return jsonify({"ok": True, "command": operation()})
    except Exception as exc:
        LOGGER.exception("Astribot S1 运动目标请求失败")
        return jsonify({"ok": False, "error": str(exc)}), 400


def _select_available_port(host: str, preferred_port: int) -> int:
    for port in range(preferred_port, preferred_port + 20):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            try:
                probe.bind((host, port))
            except OSError:
                continue
            return port
    raise RuntimeError(f"端口 {preferred_port}–{preferred_port + 19} 均被占用")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="在浏览器中配置和测试 Astribot S1。")
    parser.add_argument("--host", default="0.0.0.0", help="配置服务监听地址。")
    parser.add_argument("--port", default=28411, type=int, help="配置服务端口。")
    parser.add_argument("--debug", action="store_true", help="启用 Flask 调试模式。")
    parser.add_argument("--no-open", action="store_true", help="跳过自动打开浏览器。")
    args = parser.parse_args(list(argv) if argv is not None else None)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    port = _select_available_port(args.host, args.port)
    urls = browser_urls(args.host, port)
    LOGGER.info("Astribot S1 配置页面 Local: %s", urls[0])
    for url in urls[1:]:
        LOGGER.info("Astribot S1 配置页面 LAN:   %s", url)
    LOGGER.info("配置文件: %s", CONFIG_PATH)
    if not args.no_open:
        threading.Timer(
            0.6, lambda: webbrowser.open(primary_browser_url(args.host, port))
        ).start()
    try:
        create_app().run(
            host=args.host,
            port=port,
            debug=args.debug,
            use_reloader=args.debug,
        )
    finally:
        SESSION.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
