#!/usr/bin/env python3
"""Browser configuration, camera binding, and joint test tool for Franka FR3."""

from __future__ import annotations

import argparse
import logging
import math
import re
import socket
import threading
import webbrowser
from collections.abc import Callable, Iterable, Mapping
from pathlib import Path
from typing import Any

import yaml

from rynnrcp.utils.web_urls import browser_urls, primary_browser_url

from .controller import FrankaController


LOGGER = logging.getLogger("rynnrcp.franka_fr3.configure_web")
PACKAGE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = PACKAGE_DIR / "config" / "franka_fr3_server.yaml"
JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
    "gripper",
)
CAMERA_NAMES = ("cam_arm", "cam_main", "cam_side")

HTML_TEMPLATE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Franka FR3 配置 - RynnRCP</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      color: #e8e8e8;
      background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
      font: 14px/1.5 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main { max-width: 980px; margin: 0 auto; padding: 24px; }
    header { text-align: center; margin-bottom: 20px; }
    h1 { margin: 0; color: #00d4ff; font-size: 28px; }
    header p { color: #aab3c2; }
    .card {
      margin: 16px 0;
      padding: 20px;
      border: 1px solid #303044;
      border-radius: 12px;
      background: #1e1e2f;
      box-shadow: 0 4px 20px rgba(0,0,0,.25);
    }
    h2 { margin: 0 0 14px; color: #00d4ff; font-size: 19px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
    label { display: block; color: #b8c0cc; }
    input[type="text"] {
      width: 100%;
      margin-top: 6px;
      padding: 10px 12px;
      border: 1px solid #454a5e;
      border-radius: 7px;
      color: #fff;
      background: #171725;
    }
    select {
      width: 100%;
      margin-top: 8px;
      padding: 10px 12px;
      border: 1px solid #454a5e;
      border-radius: 7px;
      color: #fff;
      background: #171725;
    }
    button {
      padding: 9px 14px;
      border: 0;
      border-radius: 7px;
      color: #fff;
      background: #087ea4;
      cursor: pointer;
      font-weight: 650;
    }
    button.secondary { background: #3d4355; }
    button.danger { background: #a33a45; }
    button:disabled { opacity: .45; cursor: not-allowed; }
    .buttons { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 14px; }
    .notice {
      padding: 12px 14px;
      border-left: 4px solid #ffd166;
      border-radius: 5px;
      color: #f4dfae;
      background: #3a321f;
    }
    .status { color: #aab3c2; }
    .status.ok { color: #00e68a; }
    .status.error { color: #ff6b75; }
    table { width: 100%; border-collapse: collapse; margin-top: 12px; }
    th, td { padding: 10px 8px; border-bottom: 1px solid #343448; text-align: right; }
    th:first-child, td:first-child { text-align: left; }
    td.value { font-family: "SFMono-Regular", Consolas, monospace; color: #7de3ff; }
    .joint-slider { width: min(430px, 52vw); accent-color: #00d4ff; }
    .camera-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
      gap: 14px;
      margin-top: 14px;
    }
    .camera-card {
      overflow: hidden;
      border: 1px solid #3b4053;
      border-radius: 10px;
      background: #171725;
    }
    .camera-preview {
      display: grid;
      place-items: center;
      width: 100%;
      aspect-ratio: 4 / 3;
      color: #7d8797;
      background: #0e1018;
    }
    .camera-preview img { width: 100%; height: 100%; object-fit: contain; }
    .camera-details { padding: 12px; }
    .camera-name { font-weight: 700; color: #e8edf5; }
    .camera-serial {
      margin-top: 2px;
      color: #8f9bad;
      font: 12px "SFMono-Regular", Consolas, monospace;
      overflow-wrap: anywhere;
    }
    .camera-state { min-height: 21px; margin: 7px 0 0; color: #aab3c2; }
    .camera-state.ok { color: #00e68a; }
    .camera-state.error { color: #ff6b75; }
    code { color: #7de3ff; }
    @media (max-width: 680px) {
      main { padding: 14px; }
      .grid { grid-template-columns: 1fr; }
      th, td { padding: 8px 4px; }
    }
  </style>
</head>
<body>
<main>
  <header>
    <h1>Franka FR3 配置</h1>
    <p>配置机械臂与三路 RealSense，查看状态，并通过单关节滑块进行小步运动测试。</p>
  </header>

  <section class="card">
    <h2>1. 连接配置</h2>
    <div class="grid">
      <label>机械臂 IP
        <input id="robot-ip" type="text" autocomplete="off">
      </label>
      <label style="padding-top:30px">
        <input id="realtime-enforce" type="checkbox"> 运动控制时强制检查实时调度
      </label>
    </div>
    <div class="buttons">
      <button onclick="saveConfig()">保存配置</button>
      <button onclick="connectRobot()">连接并读取状态</button>
      <button class="secondary" onclick="disconnectRobot()">断开连接</button>
    </div>
    <p id="connection-status" class="status">正在读取配置...</p>
  </section>

  <section class="card">
    <h2>2. RealSense 对应关系</h2>
    <p class="notice">页面会自动显示所有已连接 RealSense 的实时画面。请在画面下选择绑定为 cam_arm、cam_main 或 cam_side；无需使用的设备保持“未绑定”。保存配置后，Server 只会启用已绑定的相机。</p>
    <div class="buttons">
      <button onclick="saveCameraBindings()">保存相机对应关系</button>
      <button class="secondary" onclick="refreshCameras()">重新扫描相机</button>
      <span id="camera-scan-status" class="status">正在扫描并打开相机画面...</span>
    </div>
    <div id="camera-grid" class="camera-grid"></div>
  </section>

  <section class="card">
    <h2>3. 状态与单关节测试</h2>
    <p class="notice">连接后先确认工作空间安全并点击“回到 Home”，再进行单关节测试。运动期间保持急停可用并安排现场人员监护。</p>
    <table>
      <thead><tr><th>轴</th><th>当前状态</th><th>Home</th><th>单关节目标</th></tr></thead>
      <tbody id="joint-rows"><tr><td colspan="4">连接后显示状态</td></tr></tbody>
    </table>
    <div class="buttons">
      <button id="go-home" onclick="goHome()" disabled>回到 Home</button>
      <button id="save-home" onclick="saveHome()" disabled>将当前状态保存为 Home</button>
    </div>
  </section>

</main>
<script>
  let connected = false;
  let controlTimer = null;
  let requestInFlight = false;
  let pendingControlCommand = null;
  let activeControlCommand = null;
  let configuredCameras = {};
  let cameraDevices = [];
  let cameraFrameTimer = null;
  const CONTROL_INTERVAL_MS = 1000 / 60;
  const CAMERA_NAMES = ["cam_arm", "cam_main", "cam_side"];

  async function api(path, options = {}) {
    const response = await fetch(path, {headers: {"Content-Type": "application/json"}, ...options});
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || "请求失败");
    return data;
  }

  function setMessage(message, kind = "") {
    const target = document.getElementById("connection-status");
    target.textContent = message;
    target.className = "status " + kind;
  }

  async function loadConfig() {
    try {
      const data = await api("/api/config");
      document.getElementById("robot-ip").value = data.config.robot_ip;
      document.getElementById("realtime-enforce").checked = data.config.realtime_enforce;
      configuredCameras = data.config.cameras;
      renderStatus(data.status);
      await refreshCameras();
    } catch (error) {
      setMessage(error.message, "error");
    }
  }

  function cameraRoleForSerial(serial) {
    for (const name of CAMERA_NAMES) {
      const camera = configuredCameras[name] || {};
      if (camera.enabled && camera.serial === serial) return name;
    }
    return "";
  }

  function escapeHtml(value) {
    const span = document.createElement("span");
    span.textContent = String(value);
    return span.innerHTML;
  }

  function cameraSelectOptions(selected) {
    return [
      '<option value="">未绑定</option>',
      ...CAMERA_NAMES.map(name =>
        `<option value="${name}" ${name === selected ? "selected" : ""}>${name}</option>`
      )
    ].join("");
  }

  function renderCameras() {
    const grid = document.getElementById("camera-grid");
    if (cameraDevices.length === 0) {
      grid.innerHTML = '<div class="camera-preview">未发现 RealSense。请检查 USB 连接后重新扫描。</div>';
      return;
    }
    grid.innerHTML = cameraDevices.map(device => {
      const serial = String(device.serial);
      const encodedSerial = encodeURIComponent(serial);
      const role = cameraRoleForSerial(serial);
      const stateClass = device.online ? "" : "error";
      const state = device.online
        ? (device.error || "正在打开预览...")
        : "当前未连接，保留已保存的绑定";
      const displayName = escapeHtml(device.name || "Intel RealSense");
      const displaySerial = escapeHtml(serial);
      return `
        <article class="camera-card" data-serial="${encodedSerial}">
          <div class="camera-preview">
            ${device.online
              ? `<img alt="${displayName} ${displaySerial} 实时画面">`
              : "相机离线"}
          </div>
          <div class="camera-details">
            <div class="camera-name">${displayName}</div>
            <div class="camera-serial">SN: ${displaySerial}</div>
            <p class="camera-state ${stateClass}">${escapeHtml(state)}</p>
            <label>绑定到
              <select data-camera-role="${encodedSerial}" onchange="changeCameraRole(this)">
                ${cameraSelectOptions(role)}
              </select>
            </label>
          </div>
        </article>`;
    }).join("");
    updateCameraFrames();
  }

  function changeCameraRole(changed) {
    const role = changed.value;
    if (role) {
      for (const select of document.querySelectorAll("[data-camera-role]")) {
        if (select !== changed && select.value === role) select.value = "";
      }
    }
    configuredCameras = selectedCameraConfig();
  }

  function selectedCameraConfig() {
    const selects = document.querySelectorAll("[data-camera-role]");
    if (selects.length === 0) return structuredClone(configuredCameras);
    const cameras = Object.fromEntries(
      CAMERA_NAMES.map(name => [name, {enabled: false, serial: null}])
    );
    for (const select of selects) {
      if (!select.value) continue;
      cameras[select.value] = {
        enabled: true,
        serial: decodeURIComponent(select.dataset.cameraRole)
      };
    }
    return cameras;
  }

  async function refreshCameras() {
    const status = document.getElementById("camera-scan-status");
    status.textContent = "正在扫描并打开相机画面...";
    status.className = "status";
    try {
      const data = await api("/api/cameras/refresh", {method: "POST", body: "{}"});
      const present = new Set(data.cameras.map(device => String(device.serial)));
      const offline = [];
      for (const name of CAMERA_NAMES) {
        const camera = configuredCameras[name] || {};
        if (camera.enabled && camera.serial && !present.has(String(camera.serial))) {
          offline.push({
            serial: String(camera.serial),
            name: "已保存的 RealSense",
            online: false,
            error: ""
          });
        }
      }
      cameraDevices = [...data.cameras, ...offline];
      renderCameras();
      status.textContent = data.cameras.length
        ? `已发现 ${data.cameras.length} 台 RealSense，请根据画面选择对应关系。`
        : "未发现 RealSense，请检查 USB 连接后重新扫描。";
      status.className = "status " + (data.cameras.length ? "ok" : "error");
      if (cameraFrameTimer !== null) clearInterval(cameraFrameTimer);
      cameraFrameTimer = setInterval(updateCameraFrames, 250);
    } catch (error) {
      status.textContent = error.message;
      status.className = "status error";
    }
  }

  function updateCameraFrames() {
    const stamp = Date.now();
    for (const card of document.querySelectorAll(".camera-card")) {
      const image = card.querySelector("img");
      if (!image) continue;
      const serial = decodeURIComponent(card.dataset.serial);
      image.onload = () => {
        const state = card.querySelector(".camera-state");
        state.textContent = "实时预览";
        state.className = "camera-state ok";
      };
      image.onerror = () => {
        const state = card.querySelector(".camera-state");
        state.textContent = "正在等待相机画面...";
        state.className = "camera-state";
      };
      image.src = `/api/cameras/${encodeURIComponent(serial)}/frame?t=${stamp}`;
    }
  }

  async function saveConfig() {
    try {
      const cameras = selectedCameraConfig();
      const data = await api("/api/config", {
        method: "POST",
        body: JSON.stringify({
          robot_ip: document.getElementById("robot-ip").value.trim(),
          realtime_enforce: document.getElementById("realtime-enforce").checked,
          cameras
        })
      });
      configuredCameras = cameras;
      renderStatus(data.status);
      const boundCount = Object.values(cameras).filter(camera => camera.enabled).length;
      setMessage(`配置已保存，Server 将启用 ${boundCount} 路 RealSense。`, "ok");
      return true;
    } catch (error) {
      setMessage(error.message, "error");
      return false;
    }
  }

  async function saveCameraBindings() {
    if (!(await saveConfig())) return;
    const status = document.getElementById("camera-scan-status");
    status.textContent = "相机对应关系已保存。停止配置页后即可启动 Server。";
    status.className = "status ok";
  }

  async function connectRobot() {
    try {
      if (!(await saveConfig())) return;
      setMessage("正在连接...");
      const data = await api("/api/connect", {method: "POST", body: "{}"});
      renderStatus(data.status);
      startControlLoop();
    } catch (error) {
      setMessage(error.message, "error");
    }
  }

  async function disconnectRobot() {
    stopControlLoop();
    try {
      const data = await api("/api/disconnect", {method: "POST", body: "{}"});
      renderStatus(data.status);
    } catch (error) {
      setMessage(error.message, "error");
    }
  }

  function queueJoint(index, value) {
    pendingControlCommand = {type: "joint", index, value: Number(value)};
  }

  function goHome() {
    if (!connected) return;
    pendingControlCommand = {type: "home"};
    setMessage("正在返回 Home，请保持工作空间清空并准备急停。");
  }

  async function controlTick() {
    if (!connected || requestInFlight) return;
    requestInFlight = true;
    if (pendingControlCommand !== null) {
      activeControlCommand = pendingControlCommand;
      pendingControlCommand = null;
    }
    const command = activeControlCommand;
    try {
      let data;
      if (!command) {
        data = await api("/api/status");
      } else if (command.type === "home") {
        data = await api("/api/home/go", {method: "POST", body: "{}"});
      } else {
        data = await api("/api/joint", {
            method: "POST",
            body: JSON.stringify({index: command.index, value: command.value})
          });
      }
      renderStatus(data.status);
    } catch (error) {
      stopControlLoop();
      setMessage(error.message, "error");
    } finally {
      requestInFlight = false;
    }
  }

  function startControlLoop() {
    stopControlLoop();
    controlTimer = setInterval(controlTick, CONTROL_INTERVAL_MS);
    controlTick();
  }

  function stopControlLoop() {
    if (controlTimer !== null) clearInterval(controlTimer);
    controlTimer = null;
    pendingControlCommand = null;
    activeControlCommand = null;
  }

  async function saveHome() {
    try {
      const data = await api("/api/home/save", {method: "POST", body: "{}"});
      renderStatus(data.status);
      setMessage("当前 7 个关节位置已保存为 Home。", "ok");
    } catch (error) {
      setMessage(error.message, "error");
    }
  }

  function renderStatus(status) {
    connected = !!status.connected;
    document.getElementById("go-home").disabled = !connected;
    document.getElementById("save-home").disabled = !connected;
    if (connected) {
      setMessage("已连接 " + status.robot_ip + "，建议确认安全后先回到 Home。控制与状态目标频率 60 Hz。", "ok");
    } else if (!document.getElementById("connection-status").classList.contains("error")) {
      setMessage("尚未连接。保存配置后连接机械臂。");
    }
    const positions = status.positions || [];
    const home = status.home_joint_positions || [];
    const names = status.joint_names || [];
    const limits = status.limits || [];
    const rows = document.getElementById("joint-rows");
    if (!connected) {
      rows.innerHTML = '<tr><td colspan="4">连接后显示状态</td></tr>';
      return;
    }
    if (rows.contains(document.activeElement)) return;
    rows.innerHTML = names.map((name, index) => `
      <tr>
        <td>${name}</td>
        <td class="value">${Number(positions[index]).toFixed(4)}</td>
        <td class="value">${index < 7 ? Number(home[index]).toFixed(4) : "—"}</td>
        <td>
          <input class="joint-slider" type="range"
            min="${limits[index][0]}" max="${limits[index][1]}" step="0.01"
            value="${Number(positions[index])}"
            oninput="this.nextElementSibling.textContent=Number(this.value).toFixed(3);queueJoint(${index},this.value)">
          <span class="value">${Number(positions[index]).toFixed(3)}</span>
        </td>
      </tr>`).join("");
  }

  loadConfig();
</script>
</body>
</html>
"""


def load_server_config(path: Path = CONFIG_PATH) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"configuration must be a YAML object: {path}")
    return data


def save_server_config(config: Mapping[str, Any], path: Path = CONFIG_PATH) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(dict(config), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(path)


def _robot_config(config: Mapping[str, Any]) -> Mapping[str, Any]:
    components = config.get("components")
    if not isinstance(components, Mapping):
        raise ValueError("components must be a YAML object")
    robot = components.get("robot")
    if not isinstance(robot, Mapping):
        raise ValueError("components.robot must be a YAML object")
    return robot


def _controller_kwargs(config: Mapping[str, Any]) -> dict[str, Any]:
    robot = _robot_config(config)
    manifest = config.get("manifest")
    if not isinstance(manifest, Mapping):
        raise ValueError("manifest must be a YAML object")
    return {
        "robot_ip": str(robot["robot_ip"]),
        "robot_id": str(manifest["robot_id"]),
        "with_gripper": bool(robot["with_gripper"]),
        "realtime_enforce": bool(robot["realtime_enforce"]),
        "max_joint_velocity_rad_s": float(robot["max_joint_velocity_rad_s"]),
        "max_joint_acceleration_rad_s2": float(robot["max_joint_acceleration_rad_s2"]),
        "max_joint_jerk_rad_s3": float(robot["max_joint_jerk_rad_s3"]),
        "max_cartesian_velocity_m_s": float(robot["max_cartesian_velocity_m_s"]),
        "max_cartesian_rotation_rad_s": float(
            robot["max_cartesian_rotation_rad_s"]
        ),
        "target_timeout_s": float(robot["target_timeout_s"]),
        "gripper_max_width_m": float(robot["gripper_max_width_m"]),
        "gripper_speed_m_s": float(robot["gripper_speed_m_s"]),
        "gripper_command_deadband": float(robot["gripper_command_deadband"]),
        "gripper_grasp_threshold": float(robot["gripper_grasp_threshold"]),
        "gripper_grasp_force_n": float(robot["gripper_grasp_force_n"]),
        "home_joint_positions": list(robot["home_joint_positions"]),
        "joint_lower_limits": list(robot["joint_lower_limits"]),
        "joint_upper_limits": list(robot["joint_upper_limits"]),
        "collision_torque_thresholds": list(robot["collision_torque_thresholds"]),
        "collision_force_thresholds": list(robot["collision_force_thresholds"]),
    }


def scan_realsense_devices() -> list[dict[str, str]]:
    """Return the serial number and product name of every connected RealSense."""
    try:
        import pyrealsense2 as rs
    except ImportError as exc:  # pragma: no cover - depends on target hardware
        raise RuntimeError(
            "未安装 RealSense 驱动，请重新运行 setup_franka_fr3.sh"
        ) from exc

    devices: list[dict[str, str]] = []
    for device in rs.context().query_devices():
        serial = str(device.get_info(rs.camera_info.serial_number)).strip()
        if not serial:
            continue
        try:
            name = str(device.get_info(rs.camera_info.name)).strip()
        except Exception:
            name = "Intel RealSense"
        devices.append({"serial": serial, "name": name or "Intel RealSense"})
    return sorted(devices, key=lambda item: item["serial"])


def _create_realsense_preview(serial: str):
    from rynnkit.cameras.realsense_camera import RealSenseCamera

    return RealSenseCamera(
        name=f"configure_preview_{serial}",
        serial=serial,
        width=640,
        height=480,
        encoding="jpg",
        fps=15,
        warmup_frames=5,
        jpeg_quality=72,
    )


class _RealSensePreview:
    def __init__(
        self,
        serial: str,
        name: str,
        camera_factory: Callable[[str], Any],
    ) -> None:
        self.serial = serial
        self.name = name
        self._camera_factory = camera_factory
        self._camera: Any = None
        self._frame: bytes | None = None
        self._error = ""
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run,
            name=f"realsense-preview-{serial}",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "serial": self.serial,
                "name": self.name,
                "online": True,
                "ready": self._frame is not None,
                "error": self._error,
            }

    def frame(self) -> bytes:
        with self._lock:
            if self._frame is not None:
                return self._frame
            if self._error:
                raise RuntimeError(self._error)
        raise RuntimeError("相机正在打开，请稍后重试")

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def should_restart(self) -> bool:
        return not self._thread.is_alive() and bool(self.snapshot()["error"])

    def _run(self) -> None:
        camera = None
        try:
            camera = self._camera_factory(self.serial)
            self._camera = camera
            camera.start()
            while not self._stop.is_set():
                ok, _width, _height, encoding, frame = camera.read_frame()
                if not ok or frame is None:
                    continue
                if str(encoding).lower() not in ("jpg", "jpeg"):
                    raise RuntimeError("RealSense 预览必须输出 JPEG")
                with self._lock:
                    self._frame = bytes(frame)
                    self._error = ""
        except Exception as exc:
            LOGGER.warning("RealSense preview failed for %s: %s", self.serial, exc)
            with self._lock:
                self._error = f"预览不可用：{exc}"
        finally:
            if camera is not None:
                try:
                    camera.stop()
                except Exception:
                    LOGGER.debug(
                        "Unable to stop RealSense preview %s",
                        self.serial,
                        exc_info=True,
                    )


class RealSensePreviewManager:
    """Discover connected devices and keep one low-bandwidth preview per camera."""

    def __init__(
        self,
        scanner: Callable[[], list[dict[str, str]]] = scan_realsense_devices,
        camera_factory: Callable[[str], Any] = _create_realsense_preview,
    ) -> None:
        self._scanner = scanner
        self._camera_factory = camera_factory
        self._previews: dict[str, _RealSensePreview] = {}
        self._lock = threading.RLock()

    def refresh(self) -> list[dict[str, Any]]:
        devices = self._scanner()
        found = {str(item["serial"]): item for item in devices}
        with self._lock:
            removed = [
                self._previews.pop(serial)
                for serial in set(self._previews) - set(found)
            ]
            for serial, device in found.items():
                existing = self._previews.get(serial)
                if existing is not None and not existing.should_restart():
                    continue
                if existing is not None:
                    removed.append(self._previews.pop(serial))
                preview = _RealSensePreview(
                    serial,
                    str(device.get("name") or "Intel RealSense"),
                    self._camera_factory,
                )
                self._previews[serial] = preview
                preview.start()
            snapshots = [
                self._previews[serial].snapshot() for serial in sorted(found)
            ]
        for preview in removed:
            preview.close()
        return snapshots

    def frame(self, serial: str) -> bytes:
        with self._lock:
            preview = self._previews.get(str(serial))
        if preview is None:
            raise KeyError(f"未发现序列号为 {serial} 的 RealSense")
        return preview.frame()

    def close(self) -> None:
        with self._lock:
            previews = list(self._previews.values())
            self._previews.clear()
        for preview in previews:
            preview.close()


class ConfigureSession:
    """Own one read/test connection used by the configuration page."""

    def __init__(
        self,
        config_path: Path = CONFIG_PATH,
        controller_factory: Callable[..., FrankaController] = FrankaController,
    ) -> None:
        self.config_path = config_path
        self.controller_factory = controller_factory
        self._controller: FrankaController | None = None
        self._targets: list[float] | None = None
        self._lock = threading.RLock()

    def config_snapshot(self) -> dict[str, Any]:
        config = load_server_config(self.config_path)
        robot = _robot_config(config)
        return {
            "robot_ip": str(robot["robot_ip"]),
            "realtime_enforce": bool(robot["realtime_enforce"]),
            "cameras": {
                name: {
                    "enabled": bool(config["components"][name]["enabled"]),
                    "serial": config["components"][name].get("serial"),
                }
                for name in CAMERA_NAMES
            },
        }

    def save_config(
        self,
        robot_ip: str,
        realtime_enforce: bool,
        cameras: Mapping[str, Any] | None = None,
    ) -> None:
        robot_ip = str(robot_ip).strip()
        if not robot_ip or not re.fullmatch(r"[A-Za-z0-9._-]+", robot_ip):
            raise ValueError("机械臂 IP 或主机名格式不正确")
        with self._lock:
            if self._controller is not None:
                raise RuntimeError("请先断开机械臂，再修改连接配置")
            config = load_server_config(self.config_path)
            robot = _robot_config(config)
            robot["robot_ip"] = robot_ip
            robot["realtime_enforce"] = bool(realtime_enforce)
            if cameras is not None:
                for name in CAMERA_NAMES:
                    if name not in cameras:
                        continue
                    camera_value = cameras.get(name, {})
                    if not isinstance(camera_value, Mapping):
                        raise ValueError(f"{name} 配置必须是对象")
                    serial = str(camera_value.get("serial") or "").strip() or None
                    enabled = bool(camera_value.get("enabled", False))
                    if enabled and serial is None:
                        raise ValueError(f"启用 {name} 前请填写 RealSense 序列号")
                    camera_config = config["components"][name]
                    camera_config["enabled"] = enabled
                    camera_config["serial"] = serial
                enabled_serials = [
                    str(config["components"][name]["serial"])
                    for name in CAMERA_NAMES
                    if config["components"][name]["enabled"]
                ]
                if len(enabled_serials) != len(set(enabled_serials)):
                    raise ValueError("每路已启用摄像头必须使用不同的 RealSense 序列号")
            save_server_config(config, self.config_path)

    def connect(self) -> dict[str, Any]:
        with self._lock:
            if self._controller is not None:
                return self.status()
            config = load_server_config(self.config_path)
            controller = self.controller_factory(**_controller_kwargs(config))
            try:
                controller.start()
                positions = self._read_positions(controller)
            except Exception:
                controller.shutdown()
                raise
            self._controller = controller
            self._targets = positions
            return self.status()

    def disconnect(self) -> dict[str, Any]:
        with self._lock:
            controller, self._controller = self._controller, None
            self._targets = None
        if controller is not None:
            controller.shutdown()
        return self.status()

    def set_joint(self, index: int, value: float) -> dict[str, Any]:
        if index not in range(len(JOINT_NAMES)):
            raise ValueError("关节编号必须在 0 到 7 之间")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("关节目标必须是有限数值")
        with self._lock:
            controller = self._require_controller()
            target = list(
                self._targets or controller.get_joint_positions()["joint_positions"]
            )
            low, high = (
                (0.0, 1.0)
                if index == 7
                else (
                    controller.joint_lower_limits[index],
                    controller.joint_upper_limits[index],
                )
            )
            target[index] = max(low, min(high, value))
            if index == 7:
                controller.set_gripper({"position": target[index]})
            else:
                controller.set_joint_positions({"joint_positions": target[:7]})
            self._targets = target
            return self.status()

    def go_home(self) -> dict[str, Any]:
        with self._lock:
            controller = self._require_controller()
            controller.home({})
            gripper = self._read_positions(controller)[7]
            self._targets = [*controller.home_joint_positions, gripper]
            return self.status()

    def save_current_home(self) -> dict[str, Any]:
        with self._lock:
            controller = self._require_controller()
            positions = list(controller.get_joint_positions()["joint_positions"])
            config = load_server_config(self.config_path)
            robot = _robot_config(config)
            robot["home_joint_positions"] = positions
            save_server_config(config, self.config_path)
            controller.home_joint_positions = positions
            return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            controller = self._controller
            config = load_server_config(self.config_path)
            robot = _robot_config(config)
            result = {
                "connected": controller is not None,
                "robot_ip": str(robot["robot_ip"]),
                "joint_names": list(JOINT_NAMES),
                "positions": None,
                "limits": [
                    *[
                        [low, high]
                        for low, high in zip(
                            robot["joint_lower_limits"],
                            robot["joint_upper_limits"],
                        )
                    ],
                    [0.0, 1.0],
                ],
                "home_joint_positions": list(robot["home_joint_positions"]),
                "health": {"errors": [], "warnings": []},
            }
            if controller is not None:
                result["positions"] = self._read_positions(controller)
                result["health"] = controller.get_health()
            return result

    @staticmethod
    def _read_positions(controller: FrankaController) -> list[float]:
        joints = list(controller.get_joint_positions()["joint_positions"])
        gripper = float(controller.get_gripper_state()["position"])
        return [*joints, gripper]

    def _require_controller(self) -> FrankaController:
        if self._controller is None:
            raise RuntimeError("请先连接机械臂")
        return self._controller


SESSION = ConfigureSession()
CAMERA_PREVIEWS = RealSensePreviewManager()


def create_app(
    session: ConfigureSession | None = None,
    camera_previews: RealSensePreviewManager | None = None,
):
    try:
        from flask import Flask, Response, jsonify, render_template_string, request
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "Franka FR3 配置页面需要 Flask，请重新运行 setup_franka_fr3.sh"
        ) from exc

    active_session = session or SESSION
    active_camera_previews = camera_previews or CAMERA_PREVIEWS
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
                payload.get("robot_ip", ""),
                bool(payload.get("realtime_enforce", False)),
                payload.get("cameras"),
            ),
            active_session,
        )

    @app.post("/api/cameras/refresh")
    def api_refresh_cameras():
        try:
            return jsonify(
                {"ok": True, "cameras": active_camera_previews.refresh()}
            )
        except Exception as exc:
            LOGGER.exception("RealSense scan failed")
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.get("/api/cameras/<serial>/frame")
    def api_camera_frame(serial: str):
        try:
            return Response(
                active_camera_previews.frame(serial),
                mimetype="image/jpeg",
                headers={"Cache-Control": "no-store, max-age=0"},
            )
        except KeyError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 404
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 503

    @app.post("/api/connect")
    def api_connect():
        return _api_call(active_session.connect, active_session)

    @app.post("/api/disconnect")
    def api_disconnect():
        return _api_call(active_session.disconnect, active_session)

    @app.get("/api/status")
    def api_status():
        return _api_call(active_session.status, active_session)

    @app.post("/api/joint")
    def api_joint():
        payload = request.get_json(silent=True) or {}
        return _api_call(
            lambda: active_session.set_joint(
                int(payload.get("index", -1)),
                float(payload.get("value")),
            ),
            active_session,
        )

    @app.post("/api/home/save")
    def api_home_save():
        return _api_call(active_session.save_current_home, active_session)

    @app.post("/api/home/go")
    def api_home_go():
        return _api_call(active_session.go_home, active_session)

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
        LOGGER.exception("Franka FR3 configuration request failed")
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
    parser = argparse.ArgumentParser(description="在浏览器中配置和测试 Franka FR3。")
    parser.add_argument("--host", default="0.0.0.0", help="配置服务监听地址。")
    parser.add_argument("--port", default=28403, type=int, help="配置服务端口。")
    parser.add_argument("--debug", action="store_true", help="启用 Flask 调试模式。")
    parser.add_argument("--no-open", action="store_true", help="跳过自动打开浏览器。")
    args = parser.parse_args(list(argv) if argv is not None else None)

    from rynnrcp.utils.logging import configure_logging

    configure_logging(level=logging.INFO, sinks=["stderr"])
    port = _select_available_port(args.host, args.port)
    urls = browser_urls(args.host, port)
    LOGGER.info("Franka FR3 配置页面 Local: %s", urls[0])
    for url in urls[1:]:
        LOGGER.info("Franka FR3 配置页面 LAN:   %s", url)
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
        CAMERA_PREVIEWS.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
