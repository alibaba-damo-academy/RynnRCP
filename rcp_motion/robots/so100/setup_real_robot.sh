#!/bin/bash

# ----------------------------
# Step 1: 检测并激活虚拟环境
# ----------------------------
if [ -z "$VIRTUAL_ENV" ]; then
    echo "⚠️ 当前未处于虚拟环境中，正在尝试激活..."

    # 检测操作系统
    OS_TYPE=$(uname -s)
    if [ "$OS_TYPE" = "Linux" ]; then
        VENV_DIR=".venv"
    elif [ "$OS_TYPE" = "Darwin" ]; then
        VENV_DIR="venv"
    else
        echo "❌ 不支持的操作系统：$OS_TYPE"
        exit 1
    fi

    # 检查虚拟环境是否存在
    if [ ! -d "$VENV_DIR" ]; then
        echo "❌ 虚拟环境 $VENV_DIR 不存在，正在创建..."
        SETUP_SCRIPT="setup_motion.sh"

        if [ -f "$SETUP_SCRIPT" ]; then
            chmod +x "$SETUP_SCRIPT"
            ./"$SETUP_SCRIPT"
            if [ $? -ne 0 ]; then
                echo "❌ 创建虚拟环境失败！"
                exit 1
            fi
        else
            echo "❌ 未找到 setup_motion.sh 脚本！"
            exit 1
        fi
    fi

    # 激活虚拟环境
    ACTIVATE_SCRIPT="$VENV_DIR/bin/activate"
    if [ -f "$ACTIVATE_SCRIPT" ]; then
        source "$ACTIVATE_SCRIPT"
        echo "✅ 虚拟环境已激活。"
    else
        echo "❌ 无法找到虚拟环境激活脚本 $ACTIVATE_SCRIPT"
        exit 1
    fi
else
    echo "✅ 当前已在虚拟环境中。"
fi

# ----------------------------
# Step 2: 运行 Python 脚本并捕获串口信息
# ----------------------------
echo "正在运行 Python 脚本以检测串口..."
# output=$(python -m scripts.find_write_port 2>&1)
output=$(python -m scripts.find_write_port 2>&1 | tee /dev/tty)

# 提取检测到的串口
detected_port=$(echo "$output" | grep -oP 'DETECTED_PORT=\K[^ ]+')
echo "🔍 检测到的串口路径：$detected_port"

if [[ -z "$detected_port" ]]; then
    echo "❌ 未检测到串口设备！"
    echo "原始输出："
    echo "$output"
    exit 1
fi

# ----------------------------
# Step 3: 获取设备的 Vendor ID 和 Product ID
# ----------------------------
# vendor_id=$(udevadm info --query=all --name="$detected_port" | grep -i idVendor | awk -F= '{print $2}')
# product_id=$(udevadm info --query=all --name="$detected_port" | grep -i idProduct | awk -F= '{print $2}')
vendor_id=$(udevadm info --query=all --name="$detected_port" | grep -i 'ID_VENDOR_ID' | awk -F= '{print $2}')
product_id=$(udevadm info --query=all --name="$detected_port" | grep -i 'ID_MODEL_ID' | awk -F= '{print $2}')

if [[ -z "$vendor_id" || -z "$product_id" ]]; then
    echo "❌ 无法获取设备的 Vendor ID 或 Product ID！"
    exit 1
fi

# ----------------------------
# Step 4: 创建 udev 规则文件（持久化权限）
# ----------------------------
udev_rule="/etc/udev/rules.d/99-mechanical-arm.rules"
if [[ ! -f "$udev_rule" ]]; then
    echo "正在创建 udev 规则文件 $udev_rule..."
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor_id\", ATTRS{idProduct}==\"$product_id\", MODE=\"0666\", GROUP=\"dialout\"" | sudo tee "$udev_rule" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "✅ udev 规则已创建！请重新插拔设备以应用规则。"
else
    echo "ℹ️ udev 规则文件已存在，跳过创建。"
fi

# ----------------------------
# Step 5: 检查用户是否在 dialout 组
# ----------------------------
if ! groups $USER | grep -q '\bdialout\b'; then
    echo "⚠️ 用户不在 dialout 组，正在添加..."
    sudo usermod -a -G dialout $USER
    echo "✅ 请重新登录以应用组权限。"
fi

echo "✅ 脚本执行完成！"
