#!/bin/bash

set -e

# ----------------------------
# Language Selection / 语言选择
# ----------------------------
echo "Select language / 选择语言:"
echo "  1. English"
echo "  2. 中文"
read -p "Choice/选择 (1/2, default=1): " lang_choice

if [ "$lang_choice" = "2" ]; then
    LANG_ZH=true
    export RYNNRCP_LANG="zh"
else
    LANG_ZH=false
    export RYNNRCP_LANG="en"
fi

# Bilingual message helper
msg() {
    if [ "$LANG_ZH" = true ]; then
        echo "$2"
    else
        echo "$1"
    fi
}

# Function to detect Python path for macOS
detect_python_path() {
    local python_path=""
    if [[ "$OSTYPE" == "darwin"* ]]; then
        if command -v brew &> /dev/null; then
            BREW_PYTHON="$(brew --prefix python@3.10 2>/dev/null)/bin/python3.10"
            if [[ -x "$BREW_PYTHON" ]]; then
                python_path="$BREW_PYTHON"
                # msg "🍺 Using Homebrew Python 3.10 (recommended for       )" "🍺 使用 Homebrew Python 3.10（MuJoCo 推荐）"
            fi
        fi
        if [[ -z "$python_path" && -x "/Library/Frameworks/Python.framework/Versions/3.10/bin/python3.10" ]]; then
            python_path="/Library/Frameworks/Python.framework/Versions/3.10/bin/python3.10"
            # msg "🐍 Using Framework Python 3.10" "🐍 使用 Framework Python 3.10"
        fi
    fi
    echo "$python_path"
}

# ----------------------------
# Linux prerequisites (LCM + build tools)
# ----------------------------
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if command -v sudo >/dev/null 2>&1; then
        echo "🔧 Installing Linux prerequisites: liblcm-dev, build-essential ..."
        sudo apt-get update
        sudo apt-get install -y liblcm-dev build-essential
    else
        echo "❌ sudo not found. Please install prerequisites manually:"
        echo "  apt-get update && apt-get install -y liblcm-dev build-essential"
        exit 1
    fi
fi

# ----------------------------
# Setup script for RynnRCP using uv
# ----------------------------
msg "🚀 Setting up RynnRCP package with uv..." "🚀 正在使用 uv 设置 RynnRCP 包..."

install_uv() {
    msg "🔧 Installing uv..." "🔧 正在安装 uv..."
    msg "OSType:${OSTYPE}" "OSType:${OSTYPE}"
    if [[ "$OSTYPE" == "darwin"* ]]; then
        if command -v brew &> /dev/null; then
            msg "🍺 Installing uv using Homebrew..." "🍺 使用 Homebrew 安装 uv..."
            brew install uv
        else
            msg "🐍 Installing uv using Python pip..." "🐍 使用 Python pip 安装 uv..."
            pip install uv
        fi
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        msg "🌐 Installing uv using official installer..." "🌐 使用官方安装程序安装 uv..."
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
    elif [[ "$OSTYPE" == "msys"*  || "$OSTYPE" == "win32"* ]]; then
        # Git Bash on Windows
        msg "Detected Git Bash on Windows. Installing uv via official installer..." "检测到 Git Bash (Windows)，通过官方方式安装 uv..."
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.local/bin:$PATH"

        if command -v uv &> /dev/null; then
            msg "✅ uv is already installed" "✅ uv 已安装"
        fi
    else
        msg "🐍 Installing uv using Python pip..." "🐍 使用 Python pip 安装 uv..."
        pip install uv
    fi

    if ! command -v uv &> /dev/null; then
        msg "❌ Failed to install uv" "❌ uv 安装失败"
        exit 1
    fi

    msg "✅ uv installed successfully" "✅ uv 安装成功"
}

if ! command -v uv &> /dev/null; then
    msg "⚠️ uv is not installed." "⚠️ uv 未安装。"
    install_uv
else
    msg "✅ uv is already installed" "✅ uv 已安装"
fi

if [[ ! -f "setup_rcp.sh" ]]; then
    msg "❌ Error: Please run this script from RynnRCP/ root directory" "❌ 错误：请从 RynnRCP/ 根目录运行此脚本"
    msg "Current directory: $(pwd)" "当前目录：$(pwd)"
    exit 1
fi

# ----------------------------
# Virtual Environment Setup
# ----------------------------
VENV_DIR="venv"

PYTHON_PATH="$(detect_python_path)"
if [[ -n "$PYTHON_PATH" ]]; then
    msg "🐍 Using Python: $PYTHON_PATH" "🐍 使用 Python：$PYTHON_PATH"
fi

msg "📦 Checking virtual environment..." "📦 正在检查虚拟环境..."

if [ -d "$VENV_DIR" ] && [ -f "$VENV_DIR/pyvenv.cfg" ]; then
    msg "⚠️  Virtual environment already exists." "⚠️  虚拟环境已存在。"
    read -p "Do you want to recreate it? This will delete the existing environment (y/N): " recreate_choice
    if [[ "$recreate_choice" =~ ^[Yy]$ ]]; then
        msg "🗑️  Removing existing virtual environment..." "🗑️  正在删除现有虚拟环境..."
        rm -rf "$VENV_DIR"
        
        msg "📦 Creating new virtual environment with uv..." "📦 正在使用 uv 创建新的虚拟环境..."
        if [[ -n "$PYTHON_PATH" ]]; then
            uv venv $VENV_DIR --python "$PYTHON_PATH"
        else
            uv venv $VENV_DIR --python 3.10
        fi
    else
        msg "✅ Using existing virtual environment." "✅ 使用现有虚拟环境。"
    fi
fi

if [ ! -d "$VENV_DIR" ]; then
    msg "📦 Creating virtual environment with uv..." "📦 正在使用 uv 创建虚拟环境..."
    if [[ -n "$PYTHON_PATH" ]]; then
        uv venv "$VENV_DIR" --python "$PYTHON_PATH"
    else
        uv venv "$VENV_DIR" --python 3.10
    fi
fi

if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
    # Windows Git Bash - 使用 Scripts 目录
    source "$VENV_DIR/Scripts/activate"
else
    source "$VENV_DIR/bin/activate"
fi

msg "✅ Virtual environment activated: $(basename $VIRTUAL_ENV)" "✅ 虚拟环境已激活：$(basename $VIRTUAL_ENV)"

export PATH="$VIRTUAL_ENV/bin:$PATH"
export UV_VIRTUAL_ENV="$(pwd)/$VENV_DIR"

# ----------------------------
# Install RCP Package
# ----------------------------
msg "🔧 Installing RynnRCP package (all dependencies)..." "🔧 正在安装 RynnRCP 包（全部依赖）..."
uv pip install -e "."

# ----------------------------
# Install Development Tools
# ----------------------------
msg "🔧 Installing development tools..." "🔧 正在安装开发工具..."
uv pip install -e ".[dev]"

# ----------------------------
# macOS: Install PyObjC for AVFoundation uniqueID / hotplug tools
# ----------------------------
if [[ "$OSTYPE" == "darwin"* ]]; then
    msg "🍎 macOS detected: installing pyobjc (for AVFoundation)..." "🍎 检测到 macOS：正在安装 pyobjc（用于 AVFoundation）..."
    uv pip install pyobjc
fi

# ----------------------------
# Fix pip path if needed
# ----------------------------
if [[ "$(which pip)" != *"$VENV_DIR"* ]]; then
    msg "⚠️  Pip is not pointing to venv, installing pip in venv..." "⚠️  Pip 未指向 venv，正在 venv 中安装 pip..."
    uv pip install pip
fi

# ----------------------------
# Build Protocol and LCM Messages   
# ----------------------------
echo ""
msg "🔧 Generating Protocol Definitions..." "🔧 正在生成协议定义..."
python ./scripts/gen_proto_msg.py || {
    msg "❌ Protocol Definitions generation failed." "❌ 协议定义生成失败。"
    exit 1
}

# Handle LCM message generation with platform-specific behavior
if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
    # On Windows, LCM generation might fail due to compatibility issues, warn but continue
    msg "⚠️  lcm-gen not found on Windows, skipping LCM message generation." "⚠️  Windows 上未找到 lcm-gen，跳过 LCM 消息生成。"
    msg "💡  You may need to install LCM separately if LCM functionality is required." "💡  如果需要 LCM 功能，您可能需要单独安装 LCM。"
else
    # On Unix-like systems, lcm-gen should work, so treat failure as error
    python ./scripts/gen_lcm_msg.py || {
        msg "❌ LCM Messages generation failed." "❌ LCM消息生成失败。"
        exit 1
    }
fi

# ----------------------------
# Verification
# ----------------------------
msg "✅ Verifying installation..." "✅ 正在验证安装..."

python -c "import rynnrcp; print('✅ rynnrcp package installed successfully')" || {
    msg "❌ rynnrcp package installation failed" "❌ rynnrcp 包安装失败"
    exit 1
}

python -c "import rcp_motion; print('✅ rcp_motion package installed successfully')" || {
    msg "❌ rcp_motion package installation failed" "❌ rcp_motion 包安装失败"
    exit 1
}

python -c "import mujoco; print('✅ mujoco package installed successfully')" || {
    msg "❌ mujoco package installation failed" "❌ mujoco 包安装失败"
}

# ----------------------------
# Setup Complete
# ----------------------------
echo ""
msg "✅ Setup completed successfully!" "✅ 设置成功完成！"
echo ""
msg "🔄 To activate the environment in the future:" "🔄 以后激活环境："
if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
  echo "  source $VENV_DIR/Scripts/activate"
else
  echo "  source $VENV_DIR/bin/activate"
fi
echo ""
msg "🎯 Quick start commands (after activation):" "🎯 快速启动命令（激活后）："
echo ""

echo ""
msg "💡 For development:" "💡 开发时："
echo "  black .                    # Format code"
echo "  pytest tests/              # Run tests"
echo ""
exit 0
