#!/bin/bash
# Script to launch nodes
set -e

CURRENT_DIR="$(pwd)"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT_DIR="$(dirname "$SCRIPT_DIR")"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# Temporary set LD_LIBRARY_PATH for local libs
export LD_LIBRARY_PATH=\
"${PROJECT_ROOT_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src:"\
"${PROJECT_ROOT_DIR}/build/third_party/lcm-1.5.0/build/lcm:"\
"${PROJECT_ROOT_DIR}/build/third_party/libwebsockets-4.0.20/build/lib:"\
"${PROJECT_ROOT_DIR}/build/third_party/libyaml-cpp-0.7.0/build/lib:"\
"${PROJECT_ROOT_DIR}/build/third_party/libyaml-cpp-0.7.0/build:"\
"${PROJECT_ROOT_DIR}/build/third_party/openssl-3.2.0:"\
"${PROJECT_ROOT_DIR}/build/rcp_framework/cpp/robot_server:"\
"$LD_LIBRARY_PATH"

# Check for DEBUG mode
if [[ "$1" == "DEBUG" ]]; then
    LOG_OUTPUT="/dev/stdout"
else
    LOG_OUTPUT="/dev/null"
fi

log "Starting robot motion..."
cd $PROJECT_ROOT_DIR

# Subshells allow background processes to run independently, 
# enabling graceful signal handling like SIGINT for proper termination.

cd robot_motion/robots/lerobot
# Start the robot motion process in the background using a subshell
( 
    python3 -m scripts.unified_controller --mode real  >> "$LOG_OUTPUT" 2>&1
) &
MOTION_PID=$!
log "Robot motion started with PID: $MOTION_PID"

log "Starting server node..."
cd $PROJECT_ROOT_DIR

# Start the robot server process in the background using a subshell
( 
    ./build/rcp_framework/robots/so100/so100_servers/so100_servers \
    ./rcp_framework/robots/so100/config/device_config.yaml \
    ./common/config/glog_config.yaml  >> "$LOG_OUTPUT" 2>&1
) &
SERVER_PID=$!
log "Server started with PID: $SERVER_PID"

log "Starting camera node..."
cd $PROJECT_ROOT_DIR

# Start the camera node process in the background using a subshell
( 
    python3 -m rcp_framework.robots.so100.camera_node.camera_node \
    --camera_config rcp_framework/robots/so100/config/cameras.yaml \
    --log_config common/config/glog_config.yaml  >> "$LOG_OUTPUT" 2>&1
) &
CAMERA_PID=$!
log "Camera node started with PID: $CAMERA_PID"

log "Log file to path: ~/RynnRcplog/"

cleanup() {
    log "Stopping processes..."
    # Send SIGINT signal to the processes to allow them to terminate gracefully
    kill -SIGINT $SERVER_PID $CAMERA_PID $MOTION_PID
    wait $SERVER_PID $CAMERA_PID $MOTION_PID 2>/dev/null
    log "Processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

log "Monitoring processes..."
while true; do
    if ! kill -0 $MOTION_PID 2>/dev/null; then
        echo -e "\033[31m*** ERROR: RobotMotion process (PID: $MOTION_PID) has stopped! ***\033[0m"
    fi
    if ! kill -0 $SERVER_PID 2>/dev/null; then
        echo -e "\033[31m*** ERROR: RobotServer process (PID: $SERVER_PID) has stopped! ***\033[0m"
    fi
    if ! kill -0 $CAMERA_PID 2>/dev/null; then
        echo -e "\033[31m*** ERROR: Camera node process (PID: $CAMERA_PID) has stopped! ***\033[0m"
    fi
    sleep 3
done

cd $CURRENT_DIR
