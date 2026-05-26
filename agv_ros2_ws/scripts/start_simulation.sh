#!/bin/bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

export AGV_SIMULATION=1

ros2 launch agv_config config_manager.launch.py &
CONFIG_PID=$!

ros2 launch agv_navigation navigation.launch.py simulation:=true &
NAV_PID=$!

ros2 launch agv_io_controller io_controller.launch.py simulation:=true &
IO_PID=$!

ros2 launch agv_plc_bridge plc_bridge.launch.py simulation:=true &
PLC_PID=$!

ros2 launch agv_vision vision.launch.py simulation:=true &
VISION_PID=$!

ros2 launch agv_web_server web_server.launch.py &
WEB_PID=$!

ros2 launch agv_connectivity connectivity.launch.py &
CONN_PID=$!

ros2 launch agv_power_manager power_manager.launch.py simulate:=true &
POWER_PID=$!

ros2 launch agv_path_planner path_planner.launch.py &
PLANNER_PID=$!

ros2 launch agv_video_stream video_stream.launch.py &
VIDEO_PID=$!

ros2 launch agv_3d_scanner 3d_scanner.launch.py &
SCANNER_PID=$!

wait $CONFIG_PID $NAV_PID $IO_PID $PLC_PID $VISION_PID $WEB_PID $CONN_PID $POWER_PID $PLANNER_PID $VIDEO_PID $SCANNER_PID
