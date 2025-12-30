#!/bin/bash
set -e

CURRENT_DIR="$(pwd)"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT_DIR="$(dirname "$SCRIPT_DIR")"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1"
}

OS_TYPE=$(uname)

if [[ "$OS_TYPE" == "Darwin" ]]; then
    export DYLD_LIBRARY_PATH=\
"${PROJECT_ROOT_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src:"\
"${PROJECT_ROOT_DIR}/build/rcp_framework/cpp/robot_server:"\
"$DYLD_LIBRARY_PATH"
    
    if [[ -d "/opt/homebrew/lib" ]]; then
        export DYLD_LIBRARY_PATH="/opt/homebrew/lib:$DYLD_LIBRARY_PATH"
    elif [[ -d "/usr/local/lib" ]]; then
        export DYLD_LIBRARY_PATH="/usr/local/lib:$DYLD_LIBRARY_PATH"
    fi
    
    log "Using macOS library paths"
else
    export LD_LIBRARY_PATH=\
"${PROJECT_ROOT_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/src:"\
"${PROJECT_ROOT_DIR}/build/third_party/lcm-1.5.0/build/lcm:"\
"${PROJECT_ROOT_DIR}/build/third_party/libwebsockets-4.0.20/build/lib:"\
"${PROJECT_ROOT_DIR}/build/third_party/libyaml-cpp-0.7.0/build/lib:"\
"${PROJECT_ROOT_DIR}/build/third_party/libyaml-cpp-0.7.0/build:"\
"${PROJECT_ROOT_DIR}/build/third_party/openssl-3.2.0:"\
"${PROJECT_ROOT_DIR}/build/rcp_framework/cpp/robot_server:"\
"$LD_LIBRARY_PATH"
    
    log "Using Linux library paths"
fi

if [[ "$1" == "DEBUG" ]]; then
    LOG_OUTPUT="/dev/stdout"
else
    LOG_OUTPUT="/dev/null"
fi

log "Starting robot motion..."
cd $PROJECT_ROOT_DIR/rcp_framework/robots/xtrainer

( 
    python3 -m motion_node.xtrainer  >> "$LOG_OUTPUT" 2>&1
) &
MOTION_PID=$!
log "Realman motion started with PID: $MOTION_PID"

log "Starting server node..."
cd $PROJECT_ROOT_DIR

( 
    ./build/rcp_framework/robots/xtrainer/xtrainer_servers/xtrainer_servers \
    ./rcp_framework/robots/xtrainer/config/device_config.yaml \
    ./common/config/glog_config.yaml  >> "$LOG_OUTPUT" 2>&1
) &
SERVER_PID=$!
log "Server started with PID: $SERVER_PID"

log "Log file to path: ~/RynnRcplog/"

cleanup() {
    log "Stopping processes..."
    wait $SERVER_PID $MOTION_PID 2>/dev/null
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
    sleep 3
done

cd $CURRENT_DIR
