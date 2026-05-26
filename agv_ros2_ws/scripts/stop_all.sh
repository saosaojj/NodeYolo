#!/bin/bash

pkill -SIGINT -f "ros2 launch" 2>/dev/null || true
pkill -SIGINT -f "ros2 run" 2>/dev/null || true

sleep 2

pkill -SIGTERM -f "ros2 launch" 2>/dev/null || true
pkill -SIGTERM -f "ros2 run" 2>/dev/null || true
pkill -SIGTERM -f "_ros2" 2>/dev/null || true

sleep 2

pkill -9 -f "ros2" 2>/dev/null || true

echo "All ROS2 processes stopped."
