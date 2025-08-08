#!/bin/bash

set -e

echo "🚀 Setting up RynnMotion-LeRobot/LeKiwi Motion Control System..."

if [[ ! -f "setup_motion.sh" ]] || [[ ! -d "../../../common/lcm" ]]; then
    echo "❌ Error: Please run this script from robots/lerobot/ directory"
    echo "Current directory: $(pwd)"
    exit 1
fi


if [[ "$OSTYPE" == "darwin"* ]]; then
    VENV_DIR="venv"
else
    VENV_DIR=".venv"
fi

if [[ ! -d "$VENV_DIR" ]]; then
    echo "📦 Creating virtual environment in $VENV_DIR..."
    if command -v python3 &> /dev/null; then
        echo "Using python3 to create virtual environment..."
        python3 -m venv "$VENV_DIR"
    elif command -v python &> /dev/null; then
        echo "Using python to create virtual environment..."
        python -m venv "$VENV_DIR"
    else
        echo "❌ Error: Neither python3 nor python command found"
        echo "Please install Python 3.8+ and try again"
        exit 1
    fi
    echo "✓ Virtual environment created"
fi

echo "🔄 Activating project virtual environment..."
source "$VENV_DIR/bin/activate"
echo "✓ Virtual environment activated: $(basename $VIRTUAL_ENV)"

echo "📦 Upgrading pip..."
python -m pip install --upgrade pip

echo "📦 Installing core requirements..."
pip install -r requirements.txt

echo "🔧 Installing build tools..."
pip install setuptools wheel build

echo "📦 Installing RynnMotion-LeRobot package in development mode..."
pip install -e .

echo "🔧 Register the current virtual environment as a Jupyter kernel."
python -m ipykernel install --user --name=.venv --display-name "Python (.venv)"

echo ""
echo "✅ RynnMotion-LeRobot setup complete!"
echo ""
echo "🔄 IMPORTANT: Activate the virtual environment to use the system:"
echo "  source $VENV_DIR/bin/activate"
echo ""
echo "🎯 Quick start commands (after activation):"
echo "  # Find USB port for real robot"
echo "  python -m scripts.find_write_port"
echo ""
echo "  # Run unified controller with sim/mock/real robot"
echo "  python -m scripts.unified_controller --mode sim/mock/real"
echo ""
echo "💡 TIP: Enable plot settings in config.yaml to optimize your VLA policy and motion control!"
echo "      - Joint trajectory analysis for control loop performance tuning"
echo "      - Interpolator visualization for action chunking optimization"
echo "📖 For detailed usage, see README.md"
