"""
Language support for setup scripts.
Provides bilingual (English/Chinese) messages for initial setup experience.
"""

import os

# Global language setting - read from env var or default to English
_current_lang = os.environ.get("RYNNRCP_LANG", "en")

MESSAGES = {
    "en": {
        # Common
        "lang_select": "Select language / 选择语言:",
        "lang_english": "  1. English",
        "lang_chinese": "  2. 中文",
        "lang_prompt": "Choice/选择 (1/2, default=1): ",

        # find_motor_port.py
        "detect_port_prompt": "Do you want to detect the port for {dev_type}? ({default}, default={default_val}): ",
        "skip_detection": "Skipping {dev_type} port detection.\n",
        "usb_remove_prompt": "\nPlease make sure that the USB cable is REMOVED and press Enter to continue.",
        "usb_connect_prompt": "Please connect the USB cable and press Enter to continue.",
        "waiting_device": "Waiting for device recognition...",
        "detected_port": "\n✓ Detected port for {dev_type}: '{port}'",
        "detected_port_env": "  DETECTED_PORT={port}",
        "updated_follower_port": "✓ Updated {config_path} with follower port: {port}",
        "updated_leader_port": "✓ Updated {config_path} with leader port: {port}",
        "no_port_diff": "\n⚠ Warning: Could not detect the port for {dev_type}. No difference was found.\n",
        "too_many_ports": "\n⚠ Warning: Could not detect the port for {dev_type}. Too many ports found ({count} ports): {ports}.\n",
        "setting_permissions": "Setting permissions for detected ports...",
        "adding_to_dialout": "Adding user {user} to dialout group...",
        "added_to_dialout": "✅ Added {user} to dialout group",
        "dialout_next_login": "ℹ️ Group membership will be active on next login, using chmod for immediate access.",
        "dialout_add_failed": "⚠️ Failed to add user to dialout group: {error}",
        "dialout_error": "⚠️ Error adding user to dialout group: {error}",
        "already_in_dialout": "✅ User {user} is already in dialout group",
        "port_accessible": "✅ Port {port} is already accessible",
        "setting_port_perms": "Setting permissions for {port}...",
        "perms_set": "✅ Permissions set for {port}",
        "perms_timeout": "⚠️ Timeout setting permissions for {port}",
        "perms_failed": "⚠️ Failed to set permissions for {port}: {error}",
        "perms_hint": "ℹ️ Try running the script with sudo or add your user to the dialout group manually:",
        "perms_hint_cmd": "   sudo usermod -a -G dialout $USER",
        "port_not_found": "⚠️ Port {port} not found, skipping chmod",
        "config_update_failed": "⚠ Warning: Could not update config file: {error}",
        "manual_set_ports": "  Please manually set ports in {config_path}",
        "port_summary": "\n" + "=" * 60 + "\nPort Detection Summary:\n" + "=" * 60,
        "follower_port": "  Follower port: {port}",
        "leader_port": "  Leader port: {port}",
        "dev_type_follower": "follower",
        "dev_type_leader": "leader",
        "not_detected": "Not detected or skipped",

        # find_camera_port.py
        "camera_detection": "🎯 Camera Detection",
        "hardcoded_detection": "🔍 Hardcoded camera detection - using platform-specific camera assignment...",
        "found_cameras_ubuntu": "Found {count} external cameras on Ubuntu",
        "found_cameras_macos": "Found {count} cameras on macOS",
        "assigned_front": "✅ Assigned front camera: {camera_id}",
        "assigned_wrist": "✅ Assigned wrist camera: {camera_id}",
        "not_enough_cameras": "⚠️ Only found {count} cameras, need at least 2",
        "updated_front_camera": "✅ Updated front camera port: {camera_id}",
        "updated_wrist_camera": "✅ Updated wrist camera port: {camera_id}",
        "camera_config_saved": "💾 Camera configuration saved to {config_path}",
        "camera_config_failed": "❌ Warning: Could not update camera config file: {error}",
        "manual_set_cameras": "  Please manually set camera ports in {config_path}",
        "camera_detection_done": "✅ Camera detection completed using hardcoded assignment",
        "detected_cameras": "\n--- Detected Cameras ---",
        "camera_num": "Camera #{num}:",
        "searching_opencv": "Searching for OpenCV cameras...",
        "found_opencv": "Found {count} OpenCV cameras.",
        "filtered_cameras": "Filtered {filtered} external cameras from {total} total cameras",
        "no_cameras_detected": "No OpenCV cameras were detected.",

        # calibrate.py
        "calib_loading_config": "Loaded configuration from {config_path}",
        "calib_no_config": "No follower or leader configuration found in config file",
        "calib_expected_section": "Expected 'robot' or 'teleoperate' section with 'port' specified",
        "calib_both_found": "Both follower and leader configurations found",
        "calib_choose_device": "\nWhich device would you like to calibrate? (1/2/3, default=1): ",
        "calib_option_follower": "  1. Follower (SO101)",
        "calib_option_leader": "  2. Leader (SO101)",
        "calib_option_both": "  3. Both (follower first, then leader)",
        "calib_selected_leader": "User selected: Calibrate leader only",
        "calib_selected_both": "User selected: Calibrate both devices",
        "calib_selected_follower": "User selected: Calibrate follower only",
        "calib_follower_found": "Follower configuration found, calibrating follower",
        "calib_leader_found": "Leader configuration found, calibrating leader",
        "calib_prompt": "Do you want to calibrate {dev_type}? ({default}, default={default_val}): ",
        "calib_skip": "Skipping {dev_type} calibration.\n",
        "calib_header_follower": "CALIBRATING FOLLOWER",
        "calib_header_leader": "CALIBRATING LEADER",
        "calib_info": "Calibrating {dev_type}: type={type}, id={id}, port={port}",
        "calib_file_path": "Calibration file: {path}",
        "calib_process_failed": "Calibration process failed: {error}",
        "calib_use_existing": "Press ENTER to use provided calibration file for {id}, or type 'c' and press ENTER to run calibration: ",
        "calib_writing_file": "Writing calibration file for {id} to the motors",
        "calib_running": "\nRunning calibration of {device}",
        "calib_move_middle": "Move {device} to the middle of its range of motion and press ENTER....",
        "calib_move_range": "Move all joints sequentially through their entire ranges of motion.\nRecording positions. Press ENTER to stop...",
        "calib_saved": "Calibration saved to {path}",
        "calib_connect_motor": "Connect the controller board to the '{motor}' motor only and press enter.",
        "calib_motor_id_set": "'{motor}' motor id set to {id}",
        "calib_summary": "\n" + "=" * 60 + "\nCalibration Summary:\n" + "=" * 60,
        "calib_follower_status": "  Follower: {status}",
        "calib_leader_status": "  Leader: {status}",
        "calib_status_done": "Calibrated",
        "calib_status_skipped": "Skipped",
        "calib_status_failed": "Failed",
        "calib_status_no_config": "No configuration",

        # show_joint_angles.py
        "show_angles_title": "SO101 Joint Angles Monitor",
        "show_angles_no_config": "No follower or leader configuration found in config file",
        "show_angles_select_device": "Select device to monitor (1/2, default=1): ",
        "show_angles_option_follower": "  1. Follower (SO101)",
        "show_angles_option_leader": "  2. Leader (SO101)",
        "show_angles_header": "=== SO101 Joint Angles ({device}) ===",
        "show_angles_column_headers": "          Joint     Raw Encoder    Radians",
        "show_angles_connecting": "Connecting to {device}...",
        "show_angles_connected": "Connected to {device}",
        "show_angles_monitoring": "Monitoring {count} joints",
        "show_angles_press_ctrl_c": "Press Ctrl+C to stop",
        "show_angles_stopping": "Stopping monitoring...",
        "show_angles_loop_error": "Error in monitoring loop: {error}",
        "show_angles_continuing": "Continuing...",
        "show_angles_error": "Error: {error}",
        "show_angles_troubleshooting": "Troubleshooting tips:",
        "show_angles_tip_1": "1. Check that the device is connected via USB",
        "show_angles_tip_2": "2. Verify the USB port in configs/so101.yaml",
        "show_angles_tip_3": "3. Check that you have permission to access the USB port",
        "show_angles_tip_4": "4. Ensure the device is powered on",
        "show_angles_tip_5": "5. Try running: ls /dev/tty* to see available ports",
        "show_angles_full_trace": "Full error trace:",
        "show_angles_disconnecting": "Disconnecting...",
        "show_angles_disconnected": "Disconnected successfully",
        "show_angles_disconnect_error": "Error during disconnect: {error}",
    },
    "zh": {
        # Common
        "lang_select": "Select language / 选择语言:",
        "lang_english": "  1. English",
        "lang_chinese": "  2. 中文",
        "lang_prompt": "Choice/选择 (1/2, default=1): ",

        # find_motor_port.py
        "detect_port_prompt": "是否检测 {dev_type} 的端口？({default}, 默认={default_val}): ",
        "skip_detection": "跳过 {dev_type} 端口检测。\n",
        "usb_remove_prompt": "\n请确保USB线已拔出，然后按Enter继续。",
        "usb_connect_prompt": "请连接USB线，然后按Enter继续。",
        "waiting_device": "正在等待设备识别...",
        "detected_port": "\n✓ 检测到 {dev_type} 的端口：'{port}'",
        "detected_port_env": "  DETECTED_PORT={port}",
        "updated_follower_port": "✓ 已更新 {config_path} 从臂端口：{port}",
        "updated_leader_port": "✓ 已更新 {config_path} 主臂端口：{port}",
        "no_port_diff": "\n⚠ 警告：无法检测 {dev_type} 的端口，未发现差异。\n",
        "too_many_ports": "\n⚠ 警告：无法检测 {dev_type} 的端口，发现过多端口（{count}个）：{ports}。\n",
        "setting_permissions": "正在设置检测到的端口权限...",
        "adding_to_dialout": "正在将用户 {user} 添加到 dialout 组...",
        "added_to_dialout": "✅ 已将 {user} 添加到 dialout 组",
        "dialout_next_login": "ℹ️ 组成员资格将在下次登录后生效，现使用 chmod 进行临时访问。",
        "dialout_add_failed": "⚠️ 添加用户到 dialout 组失败：{error}",
        "dialout_error": "⚠️ 添加用户到 dialout 组时出错：{error}",
        "already_in_dialout": "✅ 用户 {user} 已在 dialout 组中",
        "port_accessible": "✅ 端口 {port} 已可访问",
        "setting_port_perms": "正在设置 {port} 的权限...",
        "perms_set": "✅ 已设置 {port} 的权限",
        "perms_timeout": "⚠️ 设置 {port} 权限超时",
        "perms_failed": "⚠️ 设置 {port} 权限失败：{error}",
        "perms_hint": "ℹ️ 请尝试使用 sudo 运行脚本，或手动将用户添加到 dialout 组：",
        "perms_hint_cmd": "   sudo usermod -a -G dialout $USER",
        "port_not_found": "⚠️ 未找到端口 {port}，跳过 chmod",
        "config_update_failed": "⚠ 警告：无法更新配置文件：{error}",
        "manual_set_ports": "  请手动在 {config_path} 中设置端口",
        "port_summary": "\n" + "=" * 60 + "\n端口检测汇总:\n" + "=" * 60,
        "follower_port": "  从臂端口：{port}",
        "leader_port": "  主臂端口：{port}",
        "dev_type_follower": "从臂",
        "dev_type_leader": "主臂",
        "not_detected": "未检测到或已跳过",

        # find_camera_port.py
        "camera_detection": "🎯 摄像头检测",
        "hardcoded_detection": "🔍 硬编码摄像头检测 - 使用平台特定的摄像头分配...",
        "found_cameras_ubuntu": "在 Ubuntu 上找到 {count} 个外部摄像头",
        "found_cameras_macos": "在 macOS 上找到 {count} 个摄像头",
        "assigned_front": "✅ 已分配前置摄像头：{camera_id}",
        "assigned_wrist": "✅ 已分配腕部摄像头：{camera_id}",
        "not_enough_cameras": "⚠️ 仅找到 {count} 个摄像头，至少需要2个",
        "updated_front_camera": "✅ 已更新前置摄像头端口：{camera_id}",
        "updated_wrist_camera": "✅ 已更新腕部摄像头端口：{camera_id}",
        "camera_config_saved": "💾 摄像头配置已保存到 {config_path}",
        "camera_config_failed": "❌ 警告：无法更新摄像头配置文件：{error}",
        "manual_set_cameras": "  请手动在 {config_path} 中设置摄像头端口",
        "camera_detection_done": "✅ 使用硬编码分配完成摄像头检测",
        "detected_cameras": "\n--- 检测到的摄像头 ---",
        "camera_num": "摄像头 #{num}:",
        "searching_opencv": "正在搜索 OpenCV 摄像头...",
        "found_opencv": "找到 {count} 个 OpenCV 摄像头。",
        "filtered_cameras": "从 {total} 个摄像头中筛选出 {filtered} 个外部摄像头",
        "no_cameras_detected": "未检测到 OpenCV 摄像头。",

        # calibrate.py
        "calib_loading_config": "已从 {config_path} 加载配置",
        "calib_no_config": "配置文件中未找到从臂或主臂配置",
        "calib_expected_section": "需要配置 'robot' 或 'teleoperate' 部分，并指定 'port'",
        "calib_both_found": "发现从臂和主臂配置",
        "calib_choose_device": "\n要校准哪个设备？(1/2/3, 默认=1): ",
        "calib_option_follower": "  1. 从臂 (SO101)",
        "calib_option_leader": "  2. 主臂 (SO101)",
        "calib_option_both": "  3. 两者（先校准从臂，再校准主臂）",
        "calib_selected_leader": "用户选择：仅校准主臂",
        "calib_selected_both": "用户选择：校准两个设备",
        "calib_selected_follower": "用户选择：仅校准从臂",
        "calib_follower_found": "发现从臂配置，正在校准从臂",
        "calib_leader_found": "发现主臂配置，正在校准主臂",
        "calib_prompt": "是否校准 {dev_type}？({default}, 默认={default_val}): ",
        "calib_skip": "跳过 {dev_type} 校准。\n",
        "calib_header_follower": "校准从臂",
        "calib_header_leader": "校准主臂",
        "calib_info": "正在校准 {dev_type}：类型={type}，ID={id}，端口={port}",
        "calib_file_path": "校准文件：{path}",
        "calib_process_failed": "校准过程失败：{error}",
        "calib_use_existing": "按 Enter 使用 {id} 的现有校准文件，或输入 'c' 并按 Enter 运行校准：",
        "calib_writing_file": "正在将 {id} 的校准文件写入电机",
        "calib_running": "\n正在校准 {device}",
        "calib_move_middle": "将 {device} 移动到其运动范围的中间位置，然后按 Enter....",
        "calib_move_range": "依次将所有关节移动到其整个运动范围。\n正在记录位置。按 Enter 停止...",
        "calib_saved": "校准已保存到 {path}",
        "calib_connect_motor": "仅将控制板连接到 '{motor}' 电机，然后按 Enter。",
        "calib_motor_id_set": "'{motor}' 电机 ID 已设置为 {id}",
        "calib_summary": "\n" + "=" * 60 + "\n校准汇总:\n" + "=" * 60,
        "calib_follower_status": "  从臂：{status}",
        "calib_leader_status": "  主臂：{status}",
        "calib_status_done": "已校准",
        "calib_status_skipped": "已跳过",
        "calib_status_failed": "失败",
        "calib_status_no_config": "无配置",

        # show_joint_angles.py
        "show_angles_title": "SO101 关节角度监控",
        "show_angles_no_config": "配置文件中未找到从臂或主臂配置",
        "show_angles_select_device": "选择要监控的设备 (1/2, 默认=1): ",
        "show_angles_option_follower": "  1. 从臂 (SO101)",
        "show_angles_option_leader": "  2. 主臂 (SO101)",
        "show_angles_header": "=== SO101 关节角度 ({device}) ===",
        "show_angles_column_headers": "          关节     原始编码值     弧度",
        "show_angles_connecting": "正在连接 {device}...",
        "show_angles_connected": "已连接到 {device}",
        "show_angles_monitoring": "正在监控 {count} 个关节",
        "show_angles_press_ctrl_c": "按 Ctrl+C 停止",
        "show_angles_stopping": "正在停止监控...",
        "show_angles_loop_error": "监控循环错误：{error}",
        "show_angles_continuing": "继续...",
        "show_angles_error": "错误：{error}",
        "show_angles_troubleshooting": "故障排除提示：",
        "show_angles_tip_1": "1. 检查设备是否通过USB连接",
        "show_angles_tip_2": "2. 验证 configs/so101.yaml 中的USB端口",
        "show_angles_tip_3": "3. 检查是否有访问USB端口的权限",
        "show_angles_tip_4": "4. 确保设备已开机",
        "show_angles_tip_5": "5. 尝试运行: ls /dev/tty* 查看可用端口",
        "show_angles_full_trace": "完整错误跟踪：",
        "show_angles_disconnecting": "正在断开连接...",
        "show_angles_disconnected": "已成功断开连接",
        "show_angles_disconnect_error": "断开连接时出错：{error}",
    }
}


def set_language(lang: str):
    """Set the current language."""
    global _current_lang
    _current_lang = lang if lang in ("en", "zh") else "en"


def get_language() -> str:
    """Get the current language."""
    return _current_lang


def t(key: str, **kwargs) -> str:
    """
    Get translated message by key.

    Args:
        key: Message key
        **kwargs: Format arguments for the message

    Returns:
        Translated and formatted message
    """
    msg = MESSAGES.get(_current_lang, MESSAGES["en"]).get(key, key)
    if kwargs:
        return msg.format(**kwargs)
    return msg


def select_language():
    """
    Prompt user to select language if not already set via environment variable.
    Called at the start of standalone Python scripts.
    """
    global _current_lang

    # If already set via env var, don't prompt
    if os.environ.get("RYNNRCP_LANG"):
        return

    print(t("lang_select"))
    print(t("lang_english"))
    print(t("lang_chinese"))
    choice = input(t("lang_prompt")).strip()

    if choice == "2":
        _current_lang = "zh"
    else:
        _current_lang = "en"
