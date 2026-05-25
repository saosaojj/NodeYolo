#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /agv_ros2_ws/install/setup.bash ]; then
    source /agv_ros2_ws/install/setup.bash
fi

exec "$@"
