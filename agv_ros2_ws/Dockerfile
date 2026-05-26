# =====================================================
# AGV ROS2 Docker 多阶段构建优化版本
# 作者: AGV Development Team
# 版本: 2.0
# =====================================================

# =====================================================
# 第一阶段: 构建阶段
# =====================================================
FROM ros:humble-ros-base-jammy AS builder

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    PYTHONUNBUFFERED=1

# 安装构建依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    wget \
    curl \
    build-essential \
    libboost-system-dev \
    libboost-thread-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# 安装Python依赖（构建阶段）
COPY requirements.txt /tmp/build_requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/build_requirements.txt || true

# 设置工作目录
WORKDIR /agv_ros2_ws

# 复制源代码
COPY . /agv_ros2_ws

# 初始化rosdep
RUN rosdep update || true

# 安装ROS依赖
RUN rosdep install --from-paths src --ignore-src -r -y || echo "Warning: Some ROS dependencies may be missing"

# 构建项目
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS='-O3 -march=native' \
    --mixin \
        parallel-make \
    --event-handlers console_direct+" 2>&1 | tee /tmp/build.log || true

# 保存构建日志
RUN if [ -f /tmp/build.log ]; then \
    mkdir -p /agv_ros2_ws/.docker_artifacts && \
    cp /tmp/build.log /agv_ros2_ws/.docker_artifacts/build.log || true; \
    fi

# =====================================================
# 第二阶段: 运行阶段 - 完整版（推荐用于开发和测试）
# =====================================================
FROM ros:humble-ros-base-jammy AS runtime-full

# 标签
LABEL maintainer="AGV Team" \
      version="2.0" \
      description="AGV ROS2 Full Runtime with all dependencies"

# 环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=humble \
    AGV_CONFIG_DIR=/agv_config \
    AGV_DATA_DIR=/agv_data \
    AGV_LOG_DIR=/agv_logs

# 安装运行时依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    libgpiod2 \
    libgpiod-dev \
    libbluetooth3 \
    libbluetooth-dev \
    bluez \
    dbus \
    iw \
    wireless-tools \
    iputils-ping \
    net-tools \
    curl \
    wget \
    vim \
    htop \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# 安装Python运行时依赖
RUN pip3 install --no-cache-dir --break-system-packages \
    pymodbus==3.6.8 \
    opencv-python-headless==4.9.0.80 \
    fastapi==0.110.0 \
    uvicorn[standard]==0.27.1 \
    websockets==12.0 \
    pyyaml==6.0.1 \
    numpy==1.26.4 \
    python-multipart==0.0.9 \
    aiofiles==23.2.1 \
    psutil==5.9.8

# 复制构建产物
COPY --from=builder /agv_ros2_ws/install /agv_ros2_ws/install
COPY --from=builder /agv_ros2_ws/src /agv_ros2_ws/src

# 创建必要的目录
RUN mkdir -p ${AGV_CONFIG_DIR} \
             ${AGV_DATA_DIR} \
             ${AGV_LOG_DIR} \
             /agv_ros2_ws/models \
             /agv_ros2_ws/datasets

# 复制入口脚本
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# 健康检查脚本
COPY docker-healthcheck.sh /docker-healthcheck.sh
RUN chmod +x /docker-healthcheck.sh

# 暴露端口
EXPOSE 8080 5020 5000 9090

# 健康检查
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD /docker-healthcheck.sh || exit 1

# 入口点
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["launch_all"]

# =====================================================
# 第三阶段: 精简版（仅Web服务）
# =====================================================
FROM python:3.10-slim AS runtime-web

# 标签
LABEL maintainer="AGV Team" \
      version="2.0" \
      description="AGV Web Server Only"

# 环境变量
ENV PYTHONUNBUFFERED=1 \
    AGV_CONFIG_DIR=/agv_config \
    AGV_DATA_DIR=/agv_data \
    AGV_LOG_DIR=/agv_logs

# 安装运行时依赖
RUN apt-get update && apt-get install -y \
    libgpiod2 \
    curl \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# 安装Python依赖
RUN pip3 install --no-cache-dir --break-system-packages \
    pymodbus==3.6.8 \
    opencv-python-headless==4.9.0.80 \
    fastapi==0.110.0 \
    uvicorn[standard]==0.27.1 \
    websockets==12.0 \
    pyyaml==6.0.1 \
    numpy==1.26.4

# 复制应用代码
COPY --from=builder /agv_ros2_ws/install /agv_ros2_ws/install
COPY --from=builder /agv_ros2_ws/src/agv_web_server /app

WORKDIR /app

# 创建目录
RUN mkdir -p ${AGV_CONFIG_DIR} ${AGV_DATA_DIR} ${AGV_LOG_DIR}

# 暴露端口
EXPOSE 8080

# 健康检查
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD curl -f http://localhost:8080/api/v1/system/health || exit 1

# 入口点
CMD ["python3", "-m", "uvicorn", "web_server_node:app", "--host", "0.0.0.0", "--port", "8080"]

# =====================================================
# 第四阶段: 开发版本
# =====================================================
FROM ros:humble-ros-base-jammy AS development

# 标签
LABEL maintainer="AGV Team" \
      version="2.0" \
      description="AGV ROS2 Development Environment"

# 环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    EDITOR=vim

# 安装开发工具
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-virtualenv \
    git \
    vim \
    nano \
    htop \
    tree \
    mc \
    net-tools \
    iputils-ping \
    curl \
    wget \
    build-essential \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# 安装Python开发依赖
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

# 复制源代码（保持可编辑状态）
COPY . /agv_ros2_ws

# 工作目录
WORKDIR /agv_ros2_ws

# 入口点
ENTRYPOINT ["/bin/bash"]
