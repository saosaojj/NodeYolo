FROM ros:humble-ros-base AS builder

RUN apt-get update && apt-get install -y \
    python3-pip \
    libgpiod2 \
    bluetooth \
    network-manager \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /agv_ros2_ws
COPY . /agv_ros2_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    libgpiod2 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /agv_ros2_ws/install /agv_ros2_ws/install
COPY --from=builder /agv_ros2_ws/src /agv_ros2_ws/src

RUN pip3 install --no-cache-dir \
    pymodbus>=3.5.0 \
    ultralytics>=8.0.0 \
    opencv-python-headless>=4.8.0 \
    fastapi>=0.104.0 \
    "uvicorn[standard]>=0.24.0" \
    websockets>=12.0 \
    pyyaml>=6.0 \
    gpiod>=2.0.0 \
    numpy>=1.24.0

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

EXPOSE 8080 5020

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "agv_navigation", "navigation.launch.py"]
