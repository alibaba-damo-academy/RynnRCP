#!/usr/bin/env python3
"""
SO101 Robot Configuration Tool (Bilingual, minimal-field changes)

Added:
- Camera visual mapper (DearPyGui) preferred; fallback to hotplug on CLI/unavailable GUI.
- Writes mapping directly into robots/so101/config/so101_config.yaml (NOT temp json).

Only updates in so101_config.yaml:
- sensor_server.inputs[*].params.init_args.device_id for out_key:
    - observation.images.front
    - observation.images.wrist
"""

from __future__ import annotations

import os
import io
import sys
import time
import yaml
import cv2
import logging
import subprocess
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional
from contextlib import contextmanager


# Optional GUI deps (best-effort)
try:
    import numpy as np  # type: ignore
    import dearpygui.dearpygui as dpg  # type: ignore

    DPG_AVAILABLE = True
except Exception:
    DPG_AVAILABLE = False

# Optional serial deps (cross-platform)
try:
    from serial.tools import list_ports  # type: ignore

    PYSERIAL_AVAILABLE = True
except Exception:
    PYSERIAL_AVAILABLE = False

# ----------------------------
# Logging
# ----------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    stream=sys.stdout,
)

# ----------------------------
# Paths (SO101)
# ----------------------------
SCRIPT_DIR = Path(__file__).parent.absolute()
RYNNBOT_CONFIG_PATH = SCRIPT_DIR / "config" / "rynnbot_config.yaml"
SO101_RCP_CONFIG_PATH = SCRIPT_DIR / "config" / "so101_config.yaml"
SO101_LOWLEVEL_CONFIG_PATH = (
    SCRIPT_DIR.parent.parent
    / "rcp_motion"
    / "robots"
    / "so101"
    / "configs"
    / "so101.yaml"
)
SO101_ROBOT_DIR = SCRIPT_DIR.parent.parent / "rcp_motion" / "robots" / "so101"

# ----------------------------
# i18n
# ----------------------------
LANG = "zh"

I18N: Dict[str, Dict[str, str]] = {
    "choose_lang": {
        "zh": "请选择语言 / Choose language:\n  1) 中文\n  2) English\n请输入(1/2): ",
        "en": "Choose language:\n  1) 中文\n  2) English\nEnter (1/2): ",
    },
    "invalid_lang_default": {
        "zh": "输入无效，默认使用中文。",
        "en": "Invalid input, defaulting to Chinese.",
    },
    "cancelled": {"zh": "用户取消操作。", "en": "Cancelled by user."},
    "tool_title": {"zh": "SO101 配置工具", "en": "SO101 CONFIGURATION TOOL"},
    "welcome": {
        "zh": "欢迎使用 SO101 配置工具！请按提示完成配置。\n日志会输出到控制台，便于排查问题。\n",
        "en": "Welcome! Follow the prompts to configure SO101.\nLogs are printed to console for troubleshooting.\n",
    },
    "menu_title": {"zh": "主菜单", "en": "MAIN MENU"},
    "menu_desc": {
        "zh": "请选择需要配置的项目：",
        "en": "Select configuration to modify:",
    },
    "menu_1": {"zh": "  1. 设备设置 (RynnBot)", "en": "  1. Device settings (RynnBot)"},
    "menu_2": {"zh": "  2. 相机设置", "en": "  2. Camera settings"},
    "menu_3": {"zh": "  3. 机器人串口设置", "en": "  3. Robot serial settings"},
    "menu_4": {"zh": "  4. 机械臂标定", "en": "  4. Robot arm calibration"},
    "menu_5": {"zh": "  5. 一键配置全部", "en": "  5. Configure all"},
    "menu_q": {"zh": "  q. 退出", "en": "  q. Quit"},
    "menu_prompt": {"zh": "请输入选项(1-5/q): ", "en": "Enter your choice (1-5/q): "},
    "invalid_choice": {"zh": "无效选项：{choice}", "en": "Invalid choice: {choice}"},
    "exit": {"zh": "退出配置工具。", "en": "Exiting configuration tool."},
    "load_cfg": {"zh": "加载配置文件：{path}", "en": "Loading configuration: {path}"},
    "save_cfg": {"zh": "保存配置文件：{path}", "en": "Saving configuration: {path}"},
    "saved": {"zh": "✅ 已保存：{path}", "en": "✅ Saved: {path}"},
    "file_not_found": {
        "zh": "配置文件不存在：{path}",
        "en": "Configuration file not found: {path}",
    },
    "device_title": {"zh": "设备设置 (RYNNBOT)", "en": "DEVICE SETTINGS (RYNNBOT)"},
    "prompt_product_key": {"zh": "请输入 product_key", "en": "Enter product_key"},
    "prompt_device_name": {"zh": "请输入 device_name", "en": "Enter device_name"},
    "prompt_device_secret": {"zh": "请输入 device_secret", "en": "Enter device_secret"},
    "prompt_http_url": {"zh": "请输入 http_url", "en": "Enter http_url"},
    "device_try_gui": {
        "zh": "尝试使用可视化方式配置设备参数（DearPyGui）...",
        "en": "Trying to configure device settings using GUI (DearPyGui)...",
    },
    "device_gui_unavailable": {
        "zh": "可视化不可用或失败，回退到命令行输入方式。",
        "en": "GUI unavailable/failed, falling back to CLI input.",
    },
    "device_gui_done": {
        "zh": "✅ 可视化配置完成。",
        "en": "✅ GUI configuration done.",
    },
    "camera_title": {"zh": "相机配置", "en": "CAMERA CONFIGURATION"},
    "camera_try_gui": {
        "zh": "尝试使用可视化方式配置相机（DearPyGui）...",
        "en": "Trying to configure cameras using GUI (DearPyGui)...",
    },
    "camera_gui_unavailable": {
        "zh": "可视化不可用或失败，回退到 hotplug 方式。",
        "en": "GUI unavailable/failed, falling back to hotplug method.",
    },
    "camera_gui_done": {
        "zh": "✅ 可视化配置完成：{mapping}",
        "en": "✅ GUI configuration done: {mapping}",
    },
    "front_camera": {
        "zh": "[前置相机] out_key 固定为 observation.images.front",
        "en": "[Front camera] out_key fixed: observation.images.front",
    },
    "wrist_camera": {
        "zh": "[腕部相机] out_key 固定为 observation.images.wrist",
        "en": "[Wrist camera] out_key fixed: observation.images.wrist",
    },
    "unplug_camera": {
        "zh": "请拔掉该相机，然后按回车...",
        "en": "Please unplug the camera device, then press Enter...",
    },
    "plug_camera": {
        "zh": "请插入该相机，然后按回车...",
        "en": "Please plug in the camera device now, then press Enter...",
    },
    "detected_cams": {
        "zh": "检测到相机设备：{devices}",
        "en": "Detected camera devices: {devices}",
    },
    "new_cams": {
        "zh": "新增相机设备：{devices}",
        "en": "New camera devices: {devices}",
    },
    "no_new_cam": {
        "zh": "❌ 未检测到新增相机设备",
        "en": "❌ No new camera devices detected",
    },
    "test_cam_ok": {"zh": "相机可采集：{dev}", "en": "Camera capture OK: {dev}"},
    "test_cam_fail": {"zh": "相机不可用：{dev}", "en": "Camera not usable: {dev}"},
    "use_cam": {"zh": "✅ 使用相机设备：{dev}", "en": "✅ Using camera device: {dev}"},
    "no_cam_work": {
        "zh": "❌ 没有相机设备可以成功采集图像",
        "en": "❌ No camera devices could capture images",
    },
    "robot_title": {"zh": "机器人串口配置", "en": "ROBOT SERIAL CONFIGURATION"},
    "unplug_serial": {
        "zh": "请拔掉串口设备，然后按回车...",
        "en": "Please unplug the serial device, then press Enter...",
    },
    "plug_serial": {
        "zh": "请插入串口设备，然后按回车...",
        "en": "Please plug in the serial device now, then press Enter...",
    },
    "new_serial": {
        "zh": "新增串口设备：{devices}",
        "en": "New serial devices: {devices}",
    },
    "no_new_serial": {
        "zh": "❌ 未检测到新增串口设备",
        "en": "❌ No new serial device detected",
    },
    "multi_new_serial": {
        "zh": "❌ 检测到多个新增串口设备：{devices}",
        "en": "❌ Found multiple new serial devices: {devices}",
    },
    "serial_detected": {
        "zh": "✅ 检测到串口设备：{dev}",
        "en": "✅ Detected serial device: {dev}",
    },
    "chmod_serial": {
        "zh": "正在设置串口权限 chmod 666：{dev}",
        "en": "Setting serial permission chmod 666: {dev}",
    },
    "serial_try_gui": {
        "zh": "尝试使用可视化方式检测串口（DearPyGui + pyserial）...",
        "en": "Trying to detect serial port using GUI (DearPyGui + pyserial)...",
    },
    "serial_gui_unavailable": {
        "zh": "可视化不可用或失败，回退到命令行插拔检测方式。",
        "en": "GUI unavailable/failed, falling back to CLI hotplug detection.",
    },
    "serial_gui_done": {
        "zh": "✅ 可视化串口检测完成。端口：{dev}",
        "en": "✅ GUI serial detection done. Port: {dev}",
    },
    "robot_done": {
        "zh": "✅ 串口配置完成。端口：{dev}",
        "en": "✅ Serial configuration done. Port: {dev}",
    },
    "calib_title": {"zh": "机械臂标定", "en": "ROBOT ARM CALIBRATION"},
    "calib_tip1": {
        "zh": "标定会生成该机械臂专属的标定参数文件。",
        "en": "Calibration will generate robot-specific calibration parameters.",
    },
    "calib_tip2": {
        "zh": "为避免失败，请先将机械臂调到零位。",
        "en": "To avoid failure, please move the robot arm to zero position first.",
    },
    "calib_confirm": {"zh": "开始标定？(y/N): ", "en": "Start calibration? (y/N): "},
    "calib_cancel": {"zh": "已取消标定。", "en": "Calibration cancelled."},
    "calib_ok": {"zh": "✅ 标定完成", "en": "✅ Calibration completed successfully"},
    "calib_fail": {
        "zh": "❌ 标定失败，返回码：{code}",
        "en": "❌ Calibration failed, return code: {code}",
    },
    "dir_missing": {
        "zh": "目录不存在：{path}",
        "en": "Directory does not exist: {path}",
    },
}


# ----------------------------
# Input helper (flush prompt)
# ----------------------------
def prompt_input(prompt: str = "") -> str:
    if prompt:
        print(prompt, end="", flush=True)
    return input()


def choose_language() -> str:
    ans = prompt_input(I18N["choose_lang"]["zh"]).strip()
    if ans == "2":
        return "en"
    if ans == "1" or ans == "":
        return "zh"
    print(I18N["invalid_lang_default"]["zh"])
    return "zh"


def t(key: str, **kwargs) -> str:
    item = I18N.get(key)
    s = item.get(LANG, item.get("zh", key)) if item else key
    return s.format(**kwargs) if kwargs else s


def log_info(key: str, **kwargs) -> None:
    logging.info(t(key, **kwargs))


# ----------------------------
# YAML helpers
# ----------------------------
def load_yaml_config(filepath: Path) -> Dict[str, Any]:
    try:
        log_info("load_cfg", path=str(filepath))
        if not filepath.exists():
            log_info("file_not_found", path=str(filepath))
            return {}
        with filepath.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        log_info(
            "Failed to load YAML file {path}: {err}", path=str(filepath), err=str(e)
        )
        sys.exit(1)


def save_yaml_config(filepath: Path, config: Dict[str, Any]) -> None:
    try:
        log_info("save_cfg", path=str(filepath))
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with filepath.open("w", encoding="utf-8") as f:
            yaml.safe_dump(
                config, f, default_flow_style=False, allow_unicode=True, sort_keys=False
            )
    except Exception as e:
        log_info(
            "Failed to save YAML file {path}: {err}", path=str(filepath), err=str(e)
        )
        sys.exit(1)


def input_with_default(prompt: str, default_value: str) -> str:
    try:
        msg = f"{prompt} (default: [{default_value}], Enter to use default): "
        user_input = prompt_input(msg).strip()
        return user_input if user_input else str(default_value)
    except KeyboardInterrupt:
        print(t("cancelled"))
        raise SystemExit(0)


# ----------------------------
# Option 1: Device settings (RynnBot) - ONLY update 4 fields under rynnbot
# ----------------------------
def configure_device_settings_gui(cfg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if not DPG_AVAILABLE:
        return None

    if sys.platform.startswith("linux") and not (
        ("DISPLAY" in os.environ) or ("WAYLAND_DISPLAY" in os.environ)
    ):
        return None

    rb = cfg.get("rynnbot")
    if not isinstance(rb, dict):
        rb = {}
        cfg["rynnbot"] = rb

    product_key = str(rb.get("product_key", "") or "")
    device_name = str(rb.get("device_name", "") or "")
    device_secret = str(rb.get("device_secret", "") or "")
    http_url = str(rb.get("http_url", "") or "")

    state = {"saved": False, "running": True}

    def on_save(sender, app_data, user_data):
        rb["product_key"] = dpg.get_value("rb_product_key").strip()
        rb["device_name"] = dpg.get_value("rb_device_name").strip()
        rb["device_secret"] = dpg.get_value("rb_device_secret").strip()
        rb["http_url"] = dpg.get_value("rb_http_url").strip()
        state["saved"] = True
        state["running"] = False

    def on_cancel(sender, app_data, user_data):
        state["running"] = False

    dpg.create_context()

    FONT_SIZE = 26
    font_path = find_font_path()
    with dpg.font_registry():
        if font_path and os.path.exists(font_path):
            fnt = dpg.add_font(font_path, FONT_SIZE)
            dpg.bind_font(fnt)

    with dpg.window(label="RynnBot Device Settings", width=820, height=520):
        dpg.add_text("Edit RynnBot fields (only these four will be updated):")
        dpg.add_separator()
        dpg.add_spacer(height=8)

        dpg.add_text("product_key")
        dpg.add_input_text(tag="rb_product_key", default_value=product_key, width=-1)

        dpg.add_spacer(height=8)
        dpg.add_text("device_name")
        dpg.add_input_text(tag="rb_device_name", default_value=device_name, width=-1)

        dpg.add_spacer(height=8)
        dpg.add_text("device_secret")
        dpg.add_input_text(
            tag="rb_device_secret", default_value=device_secret, width=-1
        )

        dpg.add_spacer(height=8)
        dpg.add_text("http_url")
        dpg.add_input_text(tag="rb_http_url", default_value=http_url, width=-1)

        dpg.add_spacer(height=18)
        with dpg.group(horizontal=True):
            dpg.add_button(label="Save", width=140, height=44, callback=on_save)
            dpg.add_button(label="Cancel", width=140, height=44, callback=on_cancel)

    _safe_create_viewport(
        title="RynnBot Device Settings", width=860, height=560, dpi_aware=True
    )
    dpg.setup_dearpygui()
    dpg.show_viewport()

    while dpg.is_dearpygui_running() and state["running"]:
        dpg.render_dearpygui_frame()

    dpg.destroy_context()

    if not state["saved"]:
        return None
    return cfg


def configure_device_settings() -> None:
    log_info("device_title")
    print("\n" + "=" * 50)
    print(f"        {t('device_title')}")
    print("=" * 50)

    cfg = load_yaml_config(RYNNBOT_CONFIG_PATH)

    try:
        log_info("device_try_gui")
        new_cfg = configure_device_settings_gui(cfg)
    except Exception:
        new_cfg = None

    if new_cfg is not None:
        save_yaml_config(RYNNBOT_CONFIG_PATH, new_cfg)
        log_info("device_gui_done")
        print(t("saved", path=str(RYNNBOT_CONFIG_PATH)))
        return

    log_info("device_gui_unavailable")

    rb = cfg.get("rynnbot")
    if not isinstance(rb, dict):
        rb = {}
        cfg["rynnbot"] = rb

    rb["product_key"] = input_with_default(
        t("prompt_product_key"), rb.get("product_key", "")
    )
    rb["device_name"] = input_with_default(
        t("prompt_device_name"), rb.get("device_name", "")
    )
    rb["device_secret"] = input_with_default(
        t("prompt_device_secret"), rb.get("device_secret", "")
    )
    rb["http_url"] = input_with_default(t("prompt_http_url"), rb.get("http_url", ""))

    save_yaml_config(RYNNBOT_CONFIG_PATH, cfg)
    print(t("saved", path=str(RYNNBOT_CONFIG_PATH)))


# ----------------------------
# Option 2: Camera settings - GUI preferred, fallback to hotplug
# ----------------------------
CAM_OUT_KEYS = ["observation.images.front", "observation.images.wrist"]


def detect_camera_devices_linux() -> List[str]:
    devices = sorted(str(p) for p in Path("/dev").glob("video*"))
    log_info("detected_cams", devices=devices)
    return devices


def test_camera_device_linux(device_path: str) -> bool:
    try:
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            log_info("test_cam_fail", dev=device_path)
            return False

        ok = False
        for _ in range(10):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                ok = True
                break
            time.sleep(0.05)

        cap.release()
        if not ok:
            log_info("test_cam_fail", dev=device_path)
            return False

        log_info("test_cam_ok", dev=device_path)
        return True
    except Exception:
        log_info("test_cam_fail", dev=device_path)
        return False


def wait_for_camera_device_change_video() -> str:
    if not sys.platform.startswith("linux"):
        raise SystemExit("Linux only for hotplug camera detection.")

    try:
        prompt_input(t("unplug_camera"))
        time.sleep(0.5)
        before = set(detect_camera_devices_linux())

        prompt_input(t("plug_camera"))
        time.sleep(0.5)
        after = set(detect_camera_devices_linux())

        new_devices = sorted(after - before)
        log_info("new_cams", devices=new_devices)

        if not new_devices:
            log_info("no_new_cam")
            raise SystemExit(0)

        for dev in new_devices:
            try:
                subprocess.run(["sudo", "chmod", "666", dev], check=True)
            except Exception:
                pass

            if test_camera_device_linux(dev):
                print(t("use_cam", dev=dev))
                return dev

        log_info("no_cam_work")
        raise SystemExit(0)

    except KeyboardInterrupt:
        print(t("cancelled"))
        raise SystemExit(0)


def _update_camera_device_id_only(
    cfg: Dict[str, Any], out_key: str, device_id: str
) -> None:
    servers = cfg.get("servers")
    if not isinstance(servers, list):
        log_info("so101_config.yaml: servers is missing or not a list")
        raise SystemExit(0)

    sensor_server = None
    for s in servers:
        if isinstance(s, dict) and s.get("name") == "sensor_server":
            sensor_server = s
            break
    if sensor_server is None:
        log_info("so101_config.yaml: sensor_server not found")
        raise SystemExit(0)

    inputs = sensor_server.get("inputs")
    if not isinstance(inputs, list):
        log_info("so101_config.yaml: sensor_server.inputs is missing or not a list")
        raise SystemExit(0)

    for item in inputs:
        if not isinstance(item, dict):
            continue
        params = item.get("params")
        if not isinstance(params, dict):
            continue
        if params.get("out_key") != out_key:
            continue

        init_args = params.get("init_args")
        if not isinstance(init_args, dict):
            init_args = {}
            params["init_args"] = init_args

        init_args["device_id"] = device_id
        return

    log_info(f"so101_config.yaml: camera entry not found for out_key={out_key}")
    raise SystemExit(0)


@contextmanager
def silent_opencv_probe(silent_stderr: bool = True):
    old_lvl = None
    used = None

    try:
        if (
            hasattr(cv2, "utils")
            and hasattr(cv2.utils, "logging")
            and hasattr(cv2.utils.logging, "setLogLevel")
        ):
            used = "utils.logging"
            if hasattr(cv2.utils.logging, "getLogLevel"):
                old_lvl = cv2.utils.logging.getLogLevel()
            cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
        elif hasattr(cv2, "setLogLevel"):
            used = "setLogLevel"
            if hasattr(cv2, "getLogLevel"):
                old_lvl = cv2.getLogLevel()
            cv2.setLogLevel(0)
    except Exception:
        pass

    if not silent_stderr:
        try:
            yield
        finally:
            pass
        return

    devnull_fd = None
    saved_stderr_fd = None
    try:
        saved_stderr_fd = os.dup(2)
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, 2)
        yield
    finally:
        try:
            if saved_stderr_fd is not None:
                os.dup2(saved_stderr_fd, 2)
        finally:
            if saved_stderr_fd is not None:
                os.close(saved_stderr_fd)
            if devnull_fd is not None:
                os.close(devnull_fd)

        try:
            if used == "utils.logging" and old_lvl is not None:
                cv2.utils.logging.setLogLevel(old_lvl)
            elif used == "setLogLevel" and old_lvl is not None:
                cv2.setLogLevel(old_lvl)
        except Exception:
            pass


def _make_capture(index: int) -> cv2.VideoCapture:
    source = f"/dev/video{index}" if sys.platform.startswith("linux") else index

    if sys.platform.startswith("linux"):
        backends: List[Optional[int]] = [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, None]
    elif sys.platform.startswith("win"):
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, None]
    elif sys.platform == "darwin":
        backends = [cv2.CAP_AVFOUNDATION, None]
    else:
        backends = [None]

    last_cap: Optional[cv2.VideoCapture] = None
    for be in backends:
        cap = cv2.VideoCapture(source) if be is None else cv2.VideoCapture(source, be)
        last_cap = cap
        if cap is not None and cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            return cap
        try:
            cap.release()
        except Exception:
            pass

    return last_cap if last_cap is not None else cv2.VideoCapture(source)


def probe_camera_indices(max_probe: int = 8) -> List[int]:
    ok: List[int] = []
    for i in range(max_probe):
        cap = None
        try:
            with silent_opencv_probe(silent_stderr=True):
                cap = _make_capture(i)
                if cap is None or not cap.isOpened():
                    continue

                good = False
                for _ in range(3):
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        good = True
                        break
                    time.sleep(0.03)

            if good:
                print(t("use_cam", dev=i))
                ok.append(i)

        except Exception:
            pass
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
    return ok


def find_font_path() -> Optional[str]:
    if sys.platform.startswith("win"):
        candidates = [
            r"C:\Windows\Fonts\msyh.ttc",
            r"C:\Windows\Fonts\simhei.ttf",
            r"C:\Windows\Fonts\segoeui.ttf",
            r"C:\Windows\Fonts\arial.ttf",
        ]
    elif sys.platform == "darwin":
        candidates = [
            "/System/Library/Fonts/PingFang.ttc",
            "/System/Library/Fonts/Supplemental/Arial.ttf",
        ]
    else:
        candidates = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None


def _safe_create_viewport(**kwargs):
    try:
        return dpg.create_viewport(**kwargs)
    except Exception:
        safe_keys = {
            "title",
            "width",
            "height",
            "x_pos",
            "y_pos",
            "min_width",
            "min_height",
            "max_width",
            "max_height",
            "resizable",
            "vsync",
            "always_on_top",
            "decorated",
            "clear_color",
            "disable_close",
            "small_icon",
            "large_icon",
        }
        filtered = {k: v for k, v in kwargs.items() if k in safe_keys}
        return dpg.create_viewport(**filtered)


# ----------------------------
# GUI camera mapper (multithread capture)
# ----------------------------
def configure_cameras_gui(max_probe: int = 8) -> Optional[Dict[str, str]]:
    """
    Returns mapping: out_key -> device_id (string)

    Linux: device_id will prefer the mapped /dev/videoX (NOT f"/dev/video{index}" guess).
    Else: uses numeric index as string.
    """
    if not DPG_AVAILABLE:
        return None

    if sys.platform.startswith("linux") and not (
        ("DISPLAY" in os.environ) or ("WAYLAND_DISPLAY" in os.environ)
    ):
        return None

    cam_indices = probe_camera_indices(max_probe)
    if not cam_indices:
        return None

    tex_w, tex_h = 200, 200
    ui_fps = 30

    state = {
        "selected_out_key": CAM_OUT_KEYS[0],
        "binding": {},  # out_key -> cam_index(int)
        "running": True,
        "saved": False,
    }

    def binding_ok() -> bool:
        return all(ok in state["binding"] for ok in CAM_OUT_KEYS)

    def binding_text() -> str:
        return "\n".join(
            f"{ok}  ->  {state['binding'].get(ok, '(unbound)')}" for ok in CAM_OUT_KEYS
        )

    def on_select_out_key(sender, app_data, user_data):
        if isinstance(app_data, str) and app_data:
            state["selected_out_key"] = app_data
            dpg.set_value("selected_out_key", state["selected_out_key"])

    def on_bind_camera(sender, app_data, user_data):
        cam = int(user_data)
        ok = state["selected_out_key"]
        state["binding"][ok] = cam
        dpg.set_value("binding_view", binding_text())

    def on_clear(sender, app_data, user_data):
        state["binding"].clear()
        dpg.set_value("binding_view", binding_text())

    def on_save_and_quit(sender, app_data, user_data):
        if not binding_ok():
            dpg.configure_item("modal_need_all", show=True)
            return
        state["saved"] = True
        state["running"] = False

    def on_quit(sender, app_data, user_data):
        state["running"] = False

    with silent_opencv_probe(silent_stderr=True):
        caps: Dict[int, cv2.VideoCapture] = {i: _make_capture(i) for i in cam_indices}

    dpg.create_context()

    FONT_SIZE = 26
    font_path = find_font_path()
    with dpg.font_registry():
        if font_path and os.path.exists(font_path):
            fnt = dpg.add_font(font_path, FONT_SIZE)
            dpg.bind_font(fnt)

    def empty_rgba():
        return [0.0] * (tex_w * tex_h * 4)

    texture_tags: Dict[int, str] = {}
    with dpg.texture_registry(show=False):
        for cam in cam_indices:
            tag = f"tex_cam_{cam}"
            texture_tags[cam] = tag
            dpg.add_dynamic_texture(tex_w, tex_h, empty_rgba(), tag=tag)

    with dpg.window(
        label="Need mapping",
        modal=True,
        show=False,
        tag="modal_need_all",
        no_resize=True,
    ):
        dpg.add_text("Please bind BOTH cameras (front & wrist) before saving.")
        dpg.add_button(
            label="OK",
            width=120,
            height=35,
            callback=lambda: dpg.configure_item("modal_need_all", show=False),
        )

    with dpg.window(label="SO101 Camera Mapper", width=1200, height=780):
        with dpg.group(horizontal=True):
            with dpg.child_window(width=460, height=-1):
                dpg.add_text("1) Select out_key")
                dpg.add_separator()
                dpg.add_listbox(
                    items=CAM_OUT_KEYS,
                    num_items=len(CAM_OUT_KEYS),
                    callback=on_select_out_key,
                    width=-1,
                )
                dpg.add_spacer(height=10)
                dpg.add_text("Selected out_key:")
                dpg.add_input_text(
                    tag="selected_out_key",
                    default_value=state["selected_out_key"],
                    readonly=True,
                    width=-1,
                )

                dpg.add_spacer(height=14)
                dpg.add_text("3) Current mapping")
                dpg.add_separator()
                dpg.add_input_text(
                    tag="binding_view",
                    default_value=binding_text(),
                    multiline=True,
                    readonly=True,
                    height=220,
                    width=-1,
                )

                dpg.add_spacer(height=14)
                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="Clear", width=130, height=40, callback=on_clear
                    )
                    dpg.add_button(
                        label="Save & Quit",
                        width=160,
                        height=40,
                        callback=on_save_and_quit,
                    )
                    dpg.add_button(label="Quit", width=90, height=40, callback=on_quit)

            with dpg.child_window(width=-1, height=-1):
                dpg.add_text("2) Click bind camera to outkey (front & wrist))")
                dpg.add_separator()

                cols = 2 if len(cam_indices) <= 4 else 3
                row_parent = None
                for i, cam in enumerate(cam_indices):
                    if i % cols == 0:
                        dpg.add_spacer(height=8)
                        row_parent = dpg.add_group(horizontal=True)

                    with dpg.group(parent=row_parent):
                        dpg.add_text(f"Camera index: {cam}")
                        dpg.add_image(texture_tags[cam])
                        dpg.add_button(
                            label="Click to bind out_key",
                            width=tex_w,
                            height=38,
                            callback=on_bind_camera,
                            user_data=cam,
                        )
                        dpg.add_spacer(height=12)

    _safe_create_viewport(
        title="SO101 Camera Mapper", width=1220, height=820, dpi_aware=True
    )
    dpg.setup_dearpygui()
    dpg.show_viewport()

    stop_event = threading.Event()
    latest: Dict[int, Optional[np.ndarray]] = {cam: None for cam in cam_indices}
    locks: Dict[int, threading.Lock] = {cam: threading.Lock() for cam in cam_indices}

    def capture_worker(cam: int, cap: cv2.VideoCapture):
        while not stop_event.is_set():
            if cap is None or not cap.isOpened():
                time.sleep(0.05)
                continue
            with silent_opencv_probe(silent_stderr=True):
                ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue
            try:
                frame = cv2.resize(frame, (tex_w, tex_h), interpolation=cv2.INTER_AREA)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
                data = (frame.astype(np.float32) / 255.0).ravel()
                with locks[cam]:
                    latest[cam] = data
            except Exception:
                time.sleep(0.01)

    threads: List[threading.Thread] = []
    for cam, cap in caps.items():
        th = threading.Thread(target=capture_worker, args=(cam, cap), daemon=True)
        th.start()
        threads.append(th)

    frame_interval = 1.0 / max(1, ui_fps)
    last = 0.0

    while dpg.is_dearpygui_running() and state["running"]:
        now = time.time()
        if now - last >= frame_interval:
            last = now
            for cam in cam_indices:
                with locks[cam]:
                    data = latest[cam]
                if data is not None:
                    dpg.set_value(texture_tags[cam], data)

        dpg.render_dearpygui_frame()

    stop_event.set()
    for th in threads:
        th.join(timeout=0.5)

    for cap in caps.values():
        try:
            cap.release()
        except Exception:
            pass
    dpg.destroy_context()

    if not state["saved"]:
        return None

    mapping: Dict[str, str] = {}
    for ok_key, idx in state["binding"].items():
        mapping[ok_key] = str(idx)

    if not all(k in mapping for k in CAM_OUT_KEYS):
        return None
    return mapping


def configure_cameras() -> None:
    log_info("camera_title")
    print("\n" + "=" * 50)
    print(f"        {t('camera_title')}")
    print("=" * 50)

    cfg = load_yaml_config(SO101_RCP_CONFIG_PATH)

    mapping: Optional[Dict[str, str]] = None
    try:
        log_info("camera_try_gui")
        mapping = configure_cameras_gui(max_probe=8)
    except Exception:
        mapping = None

    if mapping:
        log_info("camera_gui_done", mapping=mapping)
        _update_camera_device_id_only(
            cfg, "observation.images.front", mapping["observation.images.front"]
        )
        _update_camera_device_id_only(
            cfg, "observation.images.wrist", mapping["observation.images.wrist"]
        )
        save_yaml_config(SO101_RCP_CONFIG_PATH, cfg)
        print(t("saved", path=str(SO101_RCP_CONFIG_PATH)))
        return

    log_info("camera_gui_unavailable")
    print("\n" + t("front_camera"))
    front_dev = wait_for_camera_device_change_video()

    print("\n" + t("wrist_camera"))
    wrist_dev = wait_for_camera_device_change_video()

    _update_camera_device_id_only(cfg, "observation.images.front", front_dev)
    _update_camera_device_id_only(cfg, "observation.images.wrist", wrist_dev)

    save_yaml_config(SO101_RCP_CONFIG_PATH, cfg)
    print(t("saved", path=str(SO101_RCP_CONFIG_PATH)))


# ----------------------------
# Option 3: Robot serial settings
# ----------------------------
def detect_serial_devices_linux() -> List[str]:
    devs = []
    devs += [str(p) for p in Path("/dev").glob("ttyACM*")]
    devs += [str(p) for p in Path("/dev").glob("ttyUSB*")]
    return sorted(set(devs))


def wait_for_serial_device_change_linux() -> str:
    if not sys.platform.startswith("linux"):
        raise SystemExit("Linux only for CLI hotplug serial detection.")

    try:
        prompt_input(t("unplug_serial"))
        time.sleep(0.5)
        before = set(detect_serial_devices_linux())

        prompt_input(t("plug_serial"))
        time.sleep(0.5)
        after = set(detect_serial_devices_linux())

        new_devices = sorted(after - before)
        log_info("new_serial", devices=new_devices)

        if len(new_devices) == 0:
            raise SystemExit(t("no_new_serial"))
        if len(new_devices) > 1:
            raise SystemExit(t("multi_new_serial", devices=new_devices))

        dev = new_devices[0]
        print(t("serial_detected", dev=dev))
        return dev

    except KeyboardInterrupt:
        print(t("cancelled"))
        raise SystemExit(0)


def _scan_serial_ports_pyserial() -> Dict[str, Dict[str, str]]:
    if not PYSERIAL_AVAILABLE:
        return {}
    out: Dict[str, Dict[str, str]] = {}
    for p in list_ports.comports():
        dev = p.device
        out[dev] = {
            "device": dev,
            "description": getattr(p, "description", "") or "",
            "hwid": getattr(p, "hwid", "") or "",
        }
    return out


def _fmt_ports(title: str, ports: Dict[str, Dict[str, str]]) -> str:
    lines = [title]
    if not ports:
        lines.append("  (none)")
        return "\n".join(lines)
    for dev in sorted(ports.keys()):
        info = ports[dev]
        extra = " | ".join(
            x for x in [info.get("description", ""), info.get("hwid", "")] if x
        )
        lines.append(f"  {dev}" + (f" | {extra}" if extra else ""))
    return "\n".join(lines)


def configure_serial_gui_hotplug(refresh_interval: float = 0.1) -> Optional[str]:
    if not DPG_AVAILABLE or not PYSERIAL_AVAILABLE:
        return None

    if sys.platform.startswith("linux") and not (
        ("DISPLAY" in os.environ) or ("WAYLAND_DISPLAY" in os.environ)
    ):
        return None

    class App:
        def __init__(self) -> None:
            self.before: Dict[str, Dict[str, str]] = {}
            self.after: Dict[str, Dict[str, str]] = {}
            self.new: Dict[str, Dict[str, str]] = {}
            self.selected: Optional[str] = None
            self._last_current_set: set[str] = set()
            self.running = True
            self.saved = False

        def autorefresh_current(self) -> None:
            cur = _scan_serial_ports_pyserial()
            cur_set = set(cur.keys())
            if cur_set == self._last_current_set:
                return
            self._last_current_set = cur_set
            dpg.set_value(
                "txt_current", _fmt_ports("Current ports (auto refresh 0.1s):", cur)
            )

        def mark_unplugged(self) -> None:
            self.before = _scan_serial_ports_pyserial()
            self.after = {}
            self.new = {}
            self.selected = None

            dpg.set_value(
                "txt_before", _fmt_ports("Baseline (after unplug):", self.before)
            )
            dpg.set_value("txt_after", "After plug-in:\n  (not checked yet)")
            dpg.set_value("txt_new", "New ports:\n  (not checked yet)")
            dpg.configure_item("list_new", items=[])
            dpg.set_value("txt_selected", "Selected port: (none)")
            dpg.set_value(
                "txt_status",
                "Baseline recorded. Now plug in the device, then click Step 2.",
            )

        def check_new(self) -> None:
            self.after = _scan_serial_ports_pyserial()
            before_set = set(self.before.keys())
            after_set = set(self.after.keys())
            new_devs = sorted(after_set - before_set)

            self.new = {d: self.after[d] for d in new_devs if d in self.after}

            dpg.set_value("txt_after", _fmt_ports("After plug-in:", self.after))
            dpg.set_value("txt_new", _fmt_ports("New ports:", self.new))
            dpg.configure_item("list_new", items=new_devs)

            if not new_devs:
                dpg.set_value(
                    "txt_status",
                    "No new port detected. Wait 1-2s and click Step 2 again.",
                )
                self.selected = None
                dpg.set_value("txt_selected", "Selected port: (none)")
                return

            if len(new_devs) == 1:
                self.selected = new_devs[0]
                dpg.set_value("list_new", self.selected)
                dpg.set_value("txt_selected", f"Selected port: {self.selected}")
                dpg.set_value(
                    "txt_status", "Detected exactly one new port (auto-selected)."
                )
            else:
                self.selected = None
                dpg.set_value("txt_selected", "Selected port: (select from list)")
                dpg.set_value(
                    "txt_status",
                    f"Detected {len(new_devs)} new ports. Please select one.",
                )

        def on_select_new(self, sender, app_data, user_data) -> None:
            if isinstance(app_data, str) and app_data:
                self.selected = app_data
                dpg.set_value("txt_selected", f"Selected port: {self.selected}")

        def use_selected(self) -> None:
            if not self.selected:
                dpg.set_value("txt_status", "No port selected.")
                return
            dpg.set_value("txt_status", f"Using: {self.selected}")
            self.saved = True
            self.running = False

    app = App()

    dpg.create_context()

    FONT_SIZE = 26
    font_path = find_font_path()
    if font_path and os.path.exists(font_path):
        with dpg.font_registry():
            fnt = dpg.add_font(font_path, FONT_SIZE)
        dpg.bind_font(fnt)

    with dpg.window(
        label="Serial Hotplug Mapper (Auto Refresh)", width=1020, height=760
    ):
        dpg.add_text("Tutorial:")
        dpg.add_text("  Step 1) Unplug the serial device. Click: 'Record baseline'")
        dpg.add_text("  Step 2) Plug the device in. Wait a moment. Click: 'Check new'")
        dpg.add_text(
            "  Step 3) If multiple ports appear, select one. Click: 'Use Selected'"
        )
        dpg.add_separator()

        with dpg.group(horizontal=True):
            dpg.add_button(
                label="Step 1) Record baseline",
                width=360,
                height=46,
                callback=lambda: app.mark_unplugged(),
            )
            dpg.add_button(
                label="Step 2) Check new",
                width=280,
                height=46,
                callback=lambda: app.check_new(),
            )
            dpg.add_button(
                label="Step 3) Use Selected",
                width=260,
                height=46,
                callback=lambda: app.use_selected(),
            )

        dpg.add_spacer(height=6)
        dpg.add_text("Auto-refresh is ON (0.1s).", tag="txt_status")
        dpg.add_spacer(height=8)

        with dpg.group(horizontal=True):
            with dpg.child_window(width=500, height=560):
                dpg.add_input_text(
                    tag="txt_current",
                    multiline=True,
                    readonly=True,
                    width=-1,
                    height=240,
                )
                dpg.add_spacer(height=8)
                dpg.add_input_text(
                    tag="txt_before",
                    multiline=True,
                    readonly=True,
                    width=-1,
                    height=150,
                )
                dpg.add_spacer(height=8)
                dpg.add_input_text(
                    tag="txt_after", multiline=True, readonly=True, width=-1, height=150
                )

            with dpg.child_window(width=-1, height=560):
                dpg.add_input_text(
                    tag="txt_new", multiline=True, readonly=True, width=-1, height=240
                )
                dpg.add_text("Select new port (if multiple):")
                dpg.add_listbox(
                    tag="list_new",
                    items=[],
                    num_items=8,
                    width=-1,
                    callback=app.on_select_new,
                )
                dpg.add_spacer(height=10)
                dpg.add_text("Selected port: (none)", tag="txt_selected")

    dpg.create_viewport(title="Serial Hotplug Mapper", width=1060, height=820)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    dpg.set_value("txt_before", "Baseline (after unplug):\n  (click Step 1)")
    dpg.set_value("txt_after", "After plug-in:\n  (click Step 2)")
    dpg.set_value("txt_new", "New ports:\n  (click Step 2 after plug-in)")
    app.autorefresh_current()

    last_refresh = 0.0
    while dpg.is_dearpygui_running() and app.running:
        now = time.time()
        if now - last_refresh >= refresh_interval:
            last_refresh = now
            app.autorefresh_current()
        dpg.render_dearpygui_frame()

    dpg.destroy_context()

    if not app.saved:
        return None
    return app.selected


def configure_robot() -> None:
    log_info("robot_title")
    print("\n" + "=" * 50)
    print(f"        {t('robot_title')}")
    print("=" * 50)

    dev: Optional[str] = None

    try:
        log_info("serial_try_gui")
        dev = configure_serial_gui_hotplug(refresh_interval=0.1)
    except Exception:
        dev = None

    if dev:
        log_info("serial_gui_done", dev=dev)
    else:
        log_info("serial_gui_unavailable")
        dev = wait_for_serial_device_change_linux()

    low_cfg = load_yaml_config(SO101_LOWLEVEL_CONFIG_PATH)
    robot = low_cfg.get("robot")
    if not isinstance(robot, dict):
        robot = {}
        low_cfg["robot"] = robot

    robot["port"] = dev

    save_yaml_config(SO101_LOWLEVEL_CONFIG_PATH, low_cfg)
    print(t("saved", path=str(SO101_LOWLEVEL_CONFIG_PATH)))
    print(t("robot_done", dev=dev))

    if sys.platform.startswith("linux"):
        try:
            print(t("chmod_serial", dev=dev))
            subprocess.run(["sudo", "chmod", "666", dev], check=True)
        except Exception:
            pass


def calibrate_robot_arm() -> None:
    log_info("calib_title")
    print("\n" + "=" * 50)
    print(f"        {t('calib_title')}")
    print("=" * 50)

    try:
        confirm = prompt_input(t("calib_confirm")).strip().lower()
        if confirm not in ["y", "yes"]:
            print(t("calib_cancel"))
            return

        if not SO101_ROBOT_DIR.exists():
            raise SystemExit(t("dir_missing", path=str(SO101_ROBOT_DIR)))

        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "scripts.calibrate",
                "--arm",
                "follower",
                "--lang",
                LANG,
            ],
            cwd=str(SO101_ROBOT_DIR),
        )

        if result.returncode == 0:
            print(t("calib_ok"))
        else:
            raise SystemExit(t("calib_fail", code=result.returncode))

    except KeyboardInterrupt:
        print(t("cancelled"))
        raise SystemExit(0)


def main():
    global LANG
    LANG = choose_language()

    print("=" * 60)
    print(f"                  {t('tool_title')}")
    print("=" * 60)
    print(t("welcome"))

    while True:
        print("\n" + "-" * 50)
        print(f"           {t('menu_title')}")
        print("-" * 50)
        print(t("menu_desc"))
        print(t("menu_1"))
        print(t("menu_2"))
        print(t("menu_3"))
        print(t("menu_4"))
        print(t("menu_5"))
        print(t("menu_q"))
        print("-" * 50)

        try:
            choice = prompt_input(t("menu_prompt")).strip()
            logging.info(f"[choice]={choice}")

            if choice == "1":
                configure_device_settings()
            elif choice == "2":
                configure_cameras()
            elif choice == "3":
                configure_robot()
            elif choice == "4":
                calibrate_robot_arm()
            elif choice == "5":
                configure_device_settings()
                configure_cameras()
                configure_robot()
                calibrate_robot_arm()
            elif choice == "q":
                print(t("exit"))
                break
            else:
                print(t("invalid_choice", choice=choice))
                log_info("invalid_choice", choice=choice)

        except KeyboardInterrupt:
            print(t("cancelled"))
            raise SystemExit(0)


if __name__ == "__main__":
    main()
