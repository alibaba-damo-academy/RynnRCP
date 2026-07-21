#!/usr/bin/env python3
"""Browser-based Atom01 configuration and calibration helper."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import yaml

from rynnrcp.utils.web_urls import browser_urls, primary_browser_url


PACKAGE_DIR = Path(__file__).resolve().parent
CONFIG_DIR = PACKAGE_DIR / "config"
ATOM_CONTROL_DIR = PACKAGE_DIR / "atom_control"
SERVER_CONFIG_PATH = CONFIG_DIR / "atom01_server.yaml"
RYNNBOT_CONFIG_PATH = CONFIG_DIR / "atom01_rynnbot_app.yaml"
ROBOT_CONFIG_PATH = ATOM_CONTROL_DIR / "config" / "robot.yaml"
ZERO_POSE_IMAGE = ATOM_CONTROL_DIR / "roboparty_rpo_zero_pose.png"

CALIBRATION_ZERO_POSE_RAD = [0.0] * 23
POLICY_HOME_POSITIONS_RAD = [
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0, 0.0,
    0.18, 0.06, 0.0, 0.78, 0.0,
    0.18, -0.06, 0.0, 0.78, 0.0,
]
JOINT_NAMES = [
    "left_thigh_yaw_joint",
    "left_thigh_roll_joint",
    "left_thigh_pitch_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_thigh_yaw_joint",
    "right_thigh_roll_joint",
    "right_thigh_pitch_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "torso_joint",
    "left_arm_pitch_joint",
    "left_arm_roll_joint",
    "left_arm_yaw_joint",
    "left_elbow_pitch_joint",
    "left_elbow_yaw_joint",
    "right_arm_pitch_joint",
    "right_arm_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
]


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Atom01 配置与标定</title>
  <style>
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #101418; color: #e8edf2; font: 14px/1.55 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    main { max-width: 1080px; margin: 0 auto; padding: 24px; }
    header { display: flex; justify-content: space-between; gap: 18px; align-items: flex-start; padding-bottom: 18px; margin-bottom: 18px; border-bottom: 1px solid #26313b; }
    h1 { margin: 0; color: #7dd3fc; font-size: 28px; letter-spacing: 0; }
    h2 { margin: 0 0 14px; color: #bae6fd; font-size: 18px; }
    p { margin: 6px 0; color: #aab6c3; }
    section { background: #161d24; border: 1px solid #26313b; border-radius: 8px; padding: 18px; margin-bottom: 16px; }
    label { display: block; color: #b6c2cf; font-weight: 650; margin-bottom: 6px; }
    input { width: 100%; border: 1px solid #33414f; background: #0f151b; color: #eef5fb; border-radius: 7px; padding: 11px 12px; font: inherit; }
    button { border: 0; border-radius: 7px; padding: 11px 16px; font-weight: 750; cursor: pointer; color: #081018; background: #7dd3fc; }
    button.good { background: #86efac; }
    button.warn { background: #facc15; }
    button.danger { background: #fb7185; color: white; }
    button.secondary { background: #94a3b8; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(260px, 1fr)); gap: 16px; }
    .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
    .hint { border-left: 4px solid #7dd3fc; background: #10202b; padding: 12px 14px; border-radius: 0 8px 8px 0; margin-bottom: 14px; }
    .status { margin-top: 12px; padding: 11px 12px; border: 1px solid #2b3b48; border-radius: 8px; background: #101820; color: #cbd5e1; }
    .status.good { border-color: #14532d; background: #102017; color: #86efac; }
    .status.warn { border-color: #713f12; background: #211b0c; color: #fde68a; }
    .status.bad { border-color: #7f1d1d; background: #241012; color: #fecdd3; }
    .pose { display: grid; grid-template-columns: minmax(300px, 1fr) minmax(280px, .8fr); gap: 18px; align-items: start; }
    .pose img { width: 100%; border-radius: 8px; border: 1px solid #26313b; background: white; }
    .checks { margin: 0; padding-left: 20px; color: #cbd5e1; }
    .checks li { margin: 8px 0; }
    pre { margin: 0; max-height: 280px; overflow: auto; white-space: pre-wrap; word-break: break-word; background: #0b1015; border: 1px solid #26313b; border-radius: 8px; padding: 14px; color: #a7f3d0; }
    @media (max-width: 760px) { main { padding: 14px; } header, .grid, .pose { grid-template-columns: 1fr; display: grid; } }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>Atom01 配置与标定</h1>
      <p>配置 RCP Server、RynnBot App 和零位标定。标定动作只在这个页面手动触发。</p>
    </div>
    <button class="danger" onclick="shutdown()">退出</button>
  </header>

  <section>
    <h2>1. Server 配置</h2>
    <div class="hint">正常只需要配置 Robot ID。RCP 地址、包内 atom_control 路径和 robot.yaml 都使用默认值。</div>
    <div class="grid">
      <div><label>Robot ID</label><input id="robot-id"></div>
    </div>
    <div class="row" style="margin-top:16px;">
      <button class="good" onclick="saveConfig()">保存 Server 配置</button>
    </div>
    <div id="server-status" class="status">等待加载配置...</div>
  </section>

  <section>
    <h2>2. RynnBot 云端配置</h2>
    <div class="grid">
      <div><label>Product Key</label><input id="product-key"></div>
      <div><label>Device Name</label><input id="device-name"></div>
      <div><label>Device Secret</label><input id="device-secret"></div>
      <div><label>HTTP URL</label><input id="http-url"></div>
    </div>
    <div class="row" style="margin-top:16px;">
      <button class="good" onclick="saveConfig()">保存 RynnBot 配置</button>
    </div>
    <div id="rynnbot-status" class="status">等待加载 RynnBot 配置...</div>
  </section>

  <section>
    <h2>3. 零位标定</h2>
    <div class="hint">标定零位是 URDF 零位，不是 policy 初始站姿。摆好后执行标定，底层会向所有电机发送 set zero。</div>
    <div class="pose">
      <img src="/zero_pose.png" alt="Atom01 zero pose">
      <div>
        <p><strong>标准姿势</strong></p>
        <ul class="checks">
          <li>机器人直立，躯干/骨盆保持正中，不前俯后仰。</li>
          <li>双脚平放、平行、左右对称，髋/膝/踝按 URDF 零位摆正。</li>
          <li>双腿伸直，膝关节不要摆成 policy 站立时的弯曲姿态。</li>
          <li>双臂自然下垂并左右对称，肩/肘按图中 URDF frame 对齐。</li>
          <li>标定后读取 `joint_q` 应接近 23 维全 0。</li>
        </ul>
        <p style="margin-top:12px;">policy HOME_POSITIONS 只是站立控制初始姿态，不用于电机零位标定。</p>
        <div class="row" style="margin-top:16px;">
          <button class="secondary" onclick="readState()">读取当前状态</button>
          <button class="danger" onclick="calibrateZeros()">标定全部电机零位</button>
        </div>
        <div id="calibration-status" class="status warn">标定前请先断开 policy/server 控制，确认机器人被可靠支撑。</div>
      </div>
    </div>
  </section>

  <section>
    <h2>4. 实时数据</h2>
    <div id="state-panel" style="display:none;">
      <div style="margin-bottom:12px;">
        <span id="state-badge" style="display:inline-block;padding:4px 10px;border-radius:4px;font-weight:700;font-size:13px;"></span>
        <span id="motors-badge" style="display:inline-block;padding:4px 10px;border-radius:4px;font-weight:700;font-size:13px;margin-left:8px;"></span>
        <span id="damping-badge" style="display:inline-block;padding:4px 10px;border-radius:4px;font-weight:700;font-size:13px;margin-left:8px;"></span>
      </div>
      <div style="margin-bottom:10px;"><strong>Joint Positions (rad)</strong></div>
      <table id="joint-table" style="width:100%;border-collapse:collapse;font-size:12px;font-family:monospace;">
        <thead><tr style="border-bottom:1px solid #26313b;color:#7dd3fc;"><th style="text-align:left;padding:4px;">#</th><th style="text-align:left;padding:4px;">关节名</th><th style="text-align:right;padding:4px;">pos(rad)</th><th style="text-align:right;padding:4px;">vel</th><th style="text-align:right;padding:4px;">tau</th></tr></thead>
        <tbody id="joint-tbody"></tbody>
      </table>
      <div style="margin-top:14px;"><strong>IMU</strong></div>
      <div id="imu-data" style="font-family:monospace;font-size:13px;margin-top:6px;padding:8px;background:#0b1015;border-radius:6px;border:1px solid #26313b;"></div>
      <div style="margin-top:14px;"><strong>CAN 诊断</strong></div>
      <div id="can-diag" style="font-family:monospace;font-size:13px;margin-top:6px;padding:8px;background:#0b1015;border-radius:6px;border:1px solid #26313b;"></div>
    </div>
    <div id="state-empty" class="status">点击上方「读取当前状态」查看数据</div>
  </section>

  <section>
    <h2>命令</h2>
    <pre id="commands"></pre>
  </section>

  <section>
    <h2>日志</h2>
    <pre id="log"></pre>
  </section>
</main>

<script>
function $(id) { return document.getElementById(id); }
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
async function api(path, options = {}) {
  const res = await fetch(path, options);
  const data = await res.json();
  if (!res.ok || data.ok === false) throw new Error(data.error || "request failed");
  return data;
}
async function loadConfig() {
  const data = await api("/api/config");
  const server = data.server;
  const robot = data.robot;
  const rb = data.rynnbot.app || {};
  const calibration = robot.calibration || {};
  $("robot-id").value = server.manifest.robot_id || "";
  $("product-key").value = rb.product_key || "";
  $("device-name").value = rb.device_name || "";
  $("device-secret").value = rb.device_secret || "";
  $("http-url").value = rb.http_url || "https://robot-access.damo-academy.com";
  $("commands").textContent = data.commands.join("\n");
  setStatus("server-status", "Server 配置已加载：" + data.paths.server, "good");
  setStatus("rynnbot-status", hasCloudCreds(rb) ? "RynnBot 配置已填写：" + data.paths.rynnbot : "RynnBot 配置未完整填写：" + data.paths.rynnbot, hasCloudCreds(rb) ? "good" : "warn");
  setStatus("calibration-status", calibration.confirmed ? "零位已确认：" + (calibration.confirmed_at || "已标定") : "零位未确认。启动时会提示警告，请先确认机器人零位。", calibration.confirmed ? "good" : "warn");
  log("配置已加载");
}
function hasCloudCreds(app) {
  return ["product_key", "device_name", "device_secret"].every(key => app[key] && !String(app[key]).startsWith("YOUR_"));
}
function payload() {
  return {
    robot_id: $("robot-id").value.trim(),
    rynnbot: {
      product_key: $("product-key").value.trim(),
      device_name: $("device-name").value.trim(),
      device_secret: $("device-secret").value.trim(),
      http_url: $("http-url").value.trim() || "https://robot-access.damo-academy.com"
    }
  };
}
async function saveConfig() {
  try {
    const data = await api("/api/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(payload())});
    $("commands").textContent = data.commands.join("\n");
    setStatus("server-status", "配置已保存：" + data.paths.server, "good");
    setStatus("rynnbot-status", "RynnBot 配置已保存：" + data.paths.rynnbot, "good");
    log("配置已保存");
  } catch (err) {
    setStatus("server-status", "保存失败：" + err.message, "bad");
  }
}
const JOINT_NAMES = [
  "left_thigh_yaw","left_thigh_roll","left_thigh_pitch","left_knee","left_ankle_pitch","left_ankle_roll",
  "right_thigh_yaw","right_thigh_roll","right_thigh_pitch","right_knee","right_ankle_pitch","right_ankle_roll","torso",
  "left_arm_pitch","left_arm_roll","left_arm_yaw","left_elbow_pitch","left_elbow_yaw",
  "right_arm_pitch","right_arm_roll","right_arm_yaw","right_elbow_pitch","right_elbow_yaw"
];
function renderState(r) {
  $("state-panel").style.display = "block";
  $("state-empty").style.display = "none";
  // badges
  const stateColors = {idle:"#94a3b8",damping:"#facc15",manual:"#86efac",inference:"#7dd3fc"};
  $("state-badge").textContent = "状态: " + r.state;
  $("state-badge").style.background = stateColors[r.state] || "#94a3b8";
  $("state-badge").style.color = "#081018";
  $("motors-badge").textContent = r.motors_init ? "电机已使能" : "电机未使能";
  $("motors-badge").style.background = r.motors_init ? "#86efac" : "#fb7185";
  $("motors-badge").style.color = r.motors_init ? "#081018" : "#fff";
  $("damping-badge").textContent = r.damping ? "阻尼模式" : "非阻尼";
  $("damping-badge").style.background = r.damping ? "#facc15" : "#334155";
  $("damping-badge").style.color = r.damping ? "#081018" : "#cbd5e1";
  // joint table
  let html = "";
  const allZero = r.joint_q.every(v => v === 0);
  for (let i = 0; i < r.joint_q.length; i++) {
    const pos = r.joint_q[i].toFixed(4);
    const vel = r.joint_vel[i].toFixed(4);
    const tau = r.joint_tau[i].toFixed(4);
    const color = (pos === "0.0000" && allZero) ? "#fb7185" : "#a7f3d0";
    html += `<tr style="border-bottom:1px solid #1e2a35;"><td style="padding:3px 4px;">${i}</td><td style="padding:3px 4px;color:#bae6fd;">${JOINT_NAMES[i]||""}</td><td style="text-align:right;padding:3px 4px;color:${color};">${pos}</td><td style="text-align:right;padding:3px 4px;color:#cbd5e1;">${vel}</td><td style="text-align:right;padding:3px 4px;color:#cbd5e1;">${tau}</td></tr>`;
  }
  $("joint-tbody").innerHTML = html;
  // imu
  const q = r.imu_quat || [0,0,0,0];
  const w = r.imu_ang_vel || [0,0,0];
  const a = r.imu_accel || [0,0,0];
  $("imu-data").innerHTML = `quat: [${q.map(v=>v.toFixed(4)).join(", ")}]<br>ang_vel: [${w.map(v=>v.toFixed(4)).join(", ")}]<br>accel: [${a.map(v=>v.toFixed(4)).join(", ")}]`;
  // CAN diag
  if (r.can_diag) {
    $("can-diag").innerHTML = r.can_diag.map(d => `<span style="color:${d.state==='ERROR-ACTIVE'?'#86efac':d.state==='ERROR-PASSIVE'?'#facc15':'#fb7185'}">${d.interface}: ${d.state}</span>  TX:${d.tx_packets} RX:${d.rx_packets} ERR:${d.errors}`).join("<br>");
  } else {
    $("can-diag").innerHTML = "<span style='color:#94a3b8;'>无诊断数据</span>";
  }
}
async function readState() {
  try {
    setStatus("calibration-status", "正在读取状态...", "warn");
    const data = await api("/api/read_state", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(payload())});
    const r = data.result;
    const allZero = r.joint_q.every(v => v === 0);
    if (allZero) {
      setStatus("calibration-status", "读取完成，但 joint_q 全为 0 — CAN 通信可能异常！", "bad");
    } else {
      setStatus("calibration-status", "读取成功，joint_q 长度 " + r.joint_q.length, "good");
    }
    renderState(r);
    log("joint_q: [" + r.joint_q.map(v=>v.toFixed(4)).join(", ") + "]");
  } catch (err) {
    setStatus("calibration-status", "读取失败：" + err.message, "bad");
  }
}
async function calibrateZeros() {
  const msg = "确认已经把 Atom01 摆到图中的 URDF 零位，并且机器人被可靠支撑？这会写入全部电机零位。";
  if (!confirm(msg)) return;
  try {
    setStatus("calibration-status", "正在标定全部电机零位...", "warn");
    const data = await api("/api/calibrate_zeros", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(payload())});
    setStatus("calibration-status", data.message, "good");
    log("标定完成: " + JSON.stringify(data.result));
  } catch (err) {
    setStatus("calibration-status", "标定失败：" + err.message, "bad");
  }
}
async function shutdown() {
  try { await api("/api/shutdown", {method: "POST"}); } catch {}
  document.body.innerHTML = "<main><h1>Atom01 配置程序已退出</h1><p>可以关闭这个标签页。</p></main>";
}
loadConfig().catch(err => log("初始化失败: " + err.message));
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    server: "ConfigureServer"

    def do_GET(self) -> None:
        if self.path == "/":
            self._send_html(HTML)
        elif self.path == "/zero_pose.png":
            self._send_file(ZERO_POSE_IMAGE, "image/png")
        elif self.path == "/api/config":
            self._json(_snapshot())
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        try:
            if self.path == "/api/config":
                payload = self._payload()
                _apply_config(payload)
                self._json(_snapshot())
            elif self.path == "/api/read_state":
                self._json({"ok": True, "result": _read_state(self._payload())})
            elif self.path == "/api/calibrate_zeros":
                self._json({"ok": True, "message": "全部电机零位标定完成，控制器已进入阻尼/退出。", "result": _calibrate_zeros(self._payload())})
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

    def _send_file(self, path: Path, content_type: str) -> None:
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
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


def _snapshot() -> dict[str, Any]:
    return {
        "ok": True,
        "paths": {
            "server": str(SERVER_CONFIG_PATH),
            "rynnbot": str(RYNNBOT_CONFIG_PATH),
            "robot": str(ROBOT_CONFIG_PATH),
        },
        "server": _load_yaml(SERVER_CONFIG_PATH),
        "rynnbot": _load_yaml(RYNNBOT_CONFIG_PATH),
        "robot": _load_yaml(ROBOT_CONFIG_PATH),
        "calibration": {
            "zero_pose_rad": CALIBRATION_ZERO_POSE_RAD,
            "policy_home_positions_rad": POLICY_HOME_POSITIONS_RAD,
            "joint_names": JOINT_NAMES,
        },
        "commands": _commands(),
    }


def _apply_config(payload: dict[str, Any]) -> None:
    server = _load_yaml(SERVER_CONFIG_PATH)
    manifest = server.setdefault("manifest", {})
    manifest["robot_id"] = str(payload.get("robot_id") or manifest.get("robot_id") or "atom01")
    manifest["robot_name"] = str(manifest.get("robot_name") or "Atom01")
    interface = server.setdefault("server", {}).setdefault("interface", {})
    interface["host"] = str(interface.get("host") or "127.0.0.1")
    interface["port"] = int(interface.get("port") or 0)
    interface["local_registry"] = True
    interface["mdns"] = False
    robot_component = server.setdefault("components", {}).setdefault("robot", {})
    robot_component["enabled"] = True
    robot_component["atom_root"] = None
    robot_component["config_path"] = None
    _save_yaml(SERVER_CONFIG_PATH, server)

    robot = _load_yaml(ROBOT_CONFIG_PATH)
    robot.setdefault("calibration", {})["standard_pose"] = {
        "description": "URDF zero pose. All 23 joint readings should be close to 0 rad after set_zeros.",
        "joint_names": JOINT_NAMES,
        "joint_positions_rad": CALIBRATION_ZERO_POSE_RAD,
        "reference_image": "roboparty_rpo_zero_pose.png",
    }
    _save_yaml(ROBOT_CONFIG_PATH, robot)

    if isinstance(payload.get("rynnbot"), dict):
        rynnbot = _load_yaml(RYNNBOT_CONFIG_PATH)
        app = rynnbot.setdefault("app", {})
        app["app_id"] = "atom01_rynnbot_app"
        for key in ("product_key", "device_name", "device_secret", "http_url"):
            if key in payload["rynnbot"]:
                app[key] = str(payload["rynnbot"][key])
        _save_yaml(RYNNBOT_CONFIG_PATH, rynnbot)


def _read_state(payload: dict[str, Any]) -> dict[str, Any]:
    ctrl = _new_controller(payload)
    try:
        return ctrl.get_state()
    finally:
        ctrl.shutdown()


def _calibrate_zeros(payload: dict[str, Any]) -> dict[str, Any]:
    ctrl = _new_controller(payload)
    try:
        before = ctrl.get_state()
        ok, message = ctrl.set_zeros()
        if not ok:
            raise RuntimeError(message)
        time.sleep(0.5)
        after = ctrl.get_state()
        _mark_calibration_confirmed()
        return {"message": message, "before_joint_q": before.get("joint_q"), "after_joint_q": after.get("joint_q")}
    finally:
        ctrl.shutdown()


def _new_controller(payload: dict[str, Any]) -> Any:
    atom_root = Path(payload.get("atom_root") or ATOM_CONTROL_DIR).expanduser().resolve()
    config_path = Path(payload.get("config_path") or ROBOT_CONFIG_PATH).expanduser().resolve()
    for item in (atom_root / "python", atom_root / "build"):
        text = str(item)
        if text not in sys.path:
            sys.path.insert(0, text)
    import importlib.util

    controller_path = atom_root / "python" / "controller.py"
    spec = importlib.util.spec_from_file_location("_atom01_config_controller", controller_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load Atom01 controller: {controller_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RobotController(str(config_path), require_calibration=False)


def _mark_calibration_confirmed() -> None:
    robot = _load_yaml(ROBOT_CONFIG_PATH)
    calibration = robot.setdefault("calibration", {})
    calibration["confirmed"] = True
    calibration["confirmed_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    calibration["standard_pose"] = {
        "description": "URDF zero pose. All 23 joint readings should be close to 0 rad after set_zeros.",
        "joint_names": JOINT_NAMES,
        "joint_positions_rad": CALIBRATION_ZERO_POSE_RAD,
        "reference_image": "roboparty_rpo_zero_pose.png",
    }
    _save_yaml(ROBOT_CONFIG_PATH, robot)


def _commands() -> list[str]:
    return [
        "rynnrcp-atom01-configure",
        "rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml",
        "rynnrcp-rynnbot-app --config rynnrcp_robot_atom01/config/atom01_rynnbot_app.yaml --server-config rynnrcp_robot_atom01/config/atom01_server.yaml",
    ]


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _save_yaml(path: Path, data: dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")


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
    parser = argparse.ArgumentParser(description="Configure Atom01 for RynnRCP.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=28421)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)

    port = _free_port(args.host, args.port)
    server = ConfigureServer((args.host, port), Handler)
    urls = browser_urls(args.host, port)
    print(f"Atom01 configure UI Local: {urls[0]}")
    for url in urls[1:]:
        print(f"Atom01 configure UI LAN:   {url}")
    print(f"Zero pose reference: {ZERO_POSE_IMAGE}")
    if not args.no_browser:
        webbrowser.open(primary_browser_url(args.host, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Atom01 configure UI.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
