# AGV ROS2 Docker 部署指南

## 📋 目录

- [快速开始](#快速开始)
- [系统要求](#系统要求)
- [部署方式](#部署方式)
  - [方式一：使用部署脚本（推荐）](#方式一使用部署脚本推荐)
  - [方式二：手动部署](#方式二手动部署)
  - [方式三：Docker Compose](#方式三docker-compose)
- [环境配置](#环境配置)
- [服务说明](#服务说明)
- [监控和维护](#监控和维护)
- [故障排除](#故障排除)
- [安全建议](#安全建议)

---

## 🚀 快速开始

### 1. 克隆项目
```bash
git clone <repository_url>
cd agv_ros2_ws
```

### 2. 快速部署
```bash
# 1. 赋予脚本执行权限
chmod +x deploy.sh monitor.sh docker-entrypoint.sh docker-healthcheck.sh

# 2. 构建镜像
./deploy.sh build

# 3. 部署生产环境
./deploy.sh deploy-prod

# 4. 查看状态
./deploy.sh status

# 5. 打开浏览器访问
# http://localhost:8080
```

### 3. 查看监控
```bash
# 单次检查
./monitor.sh --single

# 持续监控
./monitor.sh --monitor
```

---

## 💻 系统要求

### 硬件要求

| 组件 | 最低要求 | 推荐配置 |
|------|---------|---------|
| CPU | 4核心 | 8核心+ |
| 内存 | 8GB | 16GB+ |
| 磁盘 | 20GB | 50GB+ SSD |
| 网络 | 100Mbps | 1Gbps |

### 软件要求

| 软件 | 版本 | 说明 |
|------|------|------|
| Docker | 20.10+ | 容器引擎 |
| Docker Compose | 2.0+ | 容器编排 |
| NVIDIA Driver | 525+ | GPU支持（可选） |
| CUDA | 11.8+ | GPU加速（可选） |

### 操作系统

- **生产环境**: Ubuntu 22.04 LTS
- **开发环境**: Ubuntu 20.04 / 22.04
- **测试环境**: macOS / Linux / Windows (WSL2)

---

## 📦 部署方式

### 方式一：使用部署脚本（推荐）

#### 生产环境部署

```bash
# 1. 构建镜像
./deploy.sh build

# 2. 部署
./deploy.sh deploy-prod

# 3. 查看日志
./deploy.sh logs

# 4. 查看状态
./deploy.sh status

# 5. 健康检查
./deploy.sh health
```

#### 开发环境部署

```bash
./deploy.sh deploy-dev

# 进入容器
docker exec -it agv_dev bash

# 在容器内重新构建
colcon build
source install/setup.bash
```

#### 测试环境部署

```bash
./deploy.sh deploy-test

# 访问地址
http://localhost:8082
```

### 方式二：手动部署

#### 1. 构建镜像

```bash
# 完整版镜像
docker build --target runtime-full -t agv_ros2:full .

# 精简版镜像（仅Web）
docker build --target runtime-web -t agv_ros2:web .

# 开发版镜像
docker build --target development -t agv_ros2:dev .
```

#### 2. 创建网络和卷

```bash
# 创建网络
docker network create agv_network

# 创建卷
docker volume create agv_config
docker volume create agv_data
docker volume create agv_logs
```

#### 3. 运行容器

```bash
# 运行核心服务
docker run -d \
  --name agv_core \
  --network host \
  --privileged \
  -v agv_config:/agv_config \
  -v agv_data:/agv_data \
  -v agv_logs:/agv_logs \
  -v /dev:/dev \
  -e ROS_DOMAIN_ID=30 \
  agv_ros2:full

# 运行Web服务
docker run -d \
  --name agv_web \
  --network host \
  -p 8080:8080 \
  -v agv_config:/agv_config \
  -v agv_data:/agv_data \
  -v agv_logs:/agv_logs \
  agv_ros2:full \
  launch_web
```

### 方式三：Docker Compose

#### 基本命令

```bash
# 启动所有服务
docker-compose up -d

# 启动特定服务
docker-compose up -d agv_core

# 停止所有服务
docker-compose down

# 查看日志
docker-compose logs -f

# 查看状态
docker-compose ps
```

#### 使用不同配置文件

```bash
# 生产环境
docker-compose -f docker-compose.prod.yml up -d

# 开发环境
docker-compose -f docker-compose.dev.yml up -d

# 测试环境
docker-compose -f docker-compose.test.yml up -d
```

---

## ⚙️ 环境配置

### 1. 创建环境文件

```bash
cp .env.example .env
```

### 2. 配置参数说明

```env
# ===================================================
# ROS2 配置
# ===================================================
ROS_DOMAIN_ID=30                    # ROS2 域ID，多个机器人不要相同
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # ROS中间件实现

# ===================================================
# AGV Web 服务配置
# ===================================================
AGV_WEB_HOST=0.0.0.0               # Web服务监听地址
AGV_WEB_PORT=8080                   # Web服务端口
AGV_WEB_JWT_SECRET=your_secret_key # JWT密钥（生产环境必须修改）

# ===================================================
# 功能开关
# ===================================================
ENABLE_VISION=false                 # 是否启用视觉系统
ENABLE_SIMULATION=true              # 是否启用仿真模式

# ===================================================
# 摄像头配置
# ===================================================
CAMERA_DEVICE=0                     # 摄像头设备索引或RTSP地址
CAMERA_USE_RTSP=false               # 是否使用RTSP流
CAMERA_FPS=30                       # 帧率
CAMERA_WIDTH=640                    # 宽度
CAMERA_HEIGHT=480                  # 高度

# ===================================================
# PLC 配置
# ===================================================
PLC_IP=127.0.0.1                   # PLC IP地址
PLC_PORT=502                        # PLC端口
PLC_SLAVE_ID=1                      # 从站ID
```

### 3. GPU配置（可选）

如果使用GPU加速，需要安装NVIDIA Container Toolkit：

```bash
# 安装 NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

---

## 🏗️ 服务说明

### 服务架构

```
┌─────────────────────────────────────────────────┐
│                   AGV System                     │
├─────────────────────────────────────────────────┤
│                                                   │
│  ┌─────────────┐  ┌─────────────┐               │
│  │  agv_core   │  │  agv_vision │ (GPU)          │
│  │  (核心服务)  │  │  (视觉识别)  │               │
│  └──────┬──────┘  └──────┬──────┘               │
│         │                │                        │
│         └────────┬───────┘                        │
│                  │                                │
│  ┌──────────────▼────────────────┐                │
│  │        agv_web (Web服务)      │                │
│  │  • FastAPI REST API         │                │
│  │  • WebSocket 实时通信        │                │
│  │  • SQLite 数据存储          │                │
│  └──────────────────────────────┘                │
│                                                   │
│  ┌──────────────┐  ┌──────────────┐             │
│  │  agv_web_light │  │    agv_dev    │             │
│  │  (仅Web服务)   │  │   (开发环境)   │             │
│  └──────────────┘  └──────────────┘             │
│                                                   │
└─────────────────────────────────────────────────┘
```

### 服务详情

#### agv_core（核心服务）

- **功能**: ROS2核心节点、导航、IO控制、PLC通信
- **端口**: 无（使用host网络）
- **资源**: CPU密集型
- **依赖**: GPIO、串口等硬件设备

#### agv_vision（视觉服务）

- **功能**: 视觉识别、目标检测
- **GPU需求**: 需要NVIDIA GPU
- **依赖**: agv_core

#### agv_web（Web服务）

- **功能**: REST API、WebSocket、前端界面
- **端口**: 8080 (Web), 5020 (PLC)
- **API文档**: http://localhost:8080/docs

#### agv_web_light（轻量级Web服务）

- **功能**: 仅提供Web服务（无ROS依赖）
- **端口**: 8081
- **适用**: 纯Web开发、测试

#### agv_dev（开发环境）

- **功能**: 交互式开发环境
- **访问**: `docker exec -it agv_dev bash`

---

## 🔧 监控和维护

### 监控工具

```bash
# 使用监控脚本
./monitor.sh --single     # 单次检查
./monitor.sh --monitor    # 持续监控（Ctrl+C退出）

# Docker stats
docker stats

# 查看特定服务日志
docker logs -f agv_core
docker logs -f agv_web
```

### 日志管理

```bash
# 查看所有日志
docker-compose logs

# 查看特定服务日志
docker-compose logs -f agv_core

# 导出日志
docker-compose logs > agv_logs_$(date +%Y%m%d_%H%M%S).txt
```

### 数据库管理

数据库文件位置: `/root/.agv_web_config/agv_data.db`

```bash
# 进入容器
docker exec -it agv_web bash

# 使用sqlite3
sqlite3 /root/.agv_web_config/agv_data.db

# 查看表
sqlite> .tables

# 查看配置历史
sqlite> SELECT * FROM config_history LIMIT 10;

# 查看仿真状态
sqlite> SELECT * FROM simulation_state;
```

### 数据备份

```bash
# 备份配置
docker cp agv_core:/agv_config ./backup_config_$(date +%Y%m%d)

# 备份数据库
docker cp agv_core:/root/.agv_web_config/agv_data.db ./backup_db_$(date +%Y%m%d).db

# 备份所有数据
tar -czf backup_$(date +%Y%m%d_%H%M%S).tar.gz volumes/
```

---

## 🔍 故障排除

### 常见问题

#### 1. 容器无法启动

```bash
# 查看详细日志
docker logs -f agv_core

# 检查Docker状态
systemctl status docker

# 检查端口占用
netstat -tuln | grep 8080
```

#### 2. Web服务无法访问

```bash
# 检查容器状态
docker ps -a | grep agv_web

# 检查端口映射
docker port agv_web

# 测试服务响应
curl http://localhost:8080/api/v1/system/health
```

#### 3. ROS节点无法通信

```bash
# 检查ROS_DOMAIN_ID
docker exec agv_core bash -c 'echo $ROS_DOMAIN_ID'

# 查看ROS节点
docker exec agv_core bash -c 'source /opt/ros/humble/setup.bash && ros2 node list'

# 查看ROS话题
docker exec agv_core bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'
```

#### 4. GPU不可用

```bash
# 检查NVIDIA驱动
nvidia-smi

# 检查CUDA版本
nvcc --version

# 测试Docker GPU支持
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### 性能优化

#### 1. 增加内存限制

编辑 `docker-compose.yml`:

```yaml
services:
  agv_core:
    mem_limit: 8g
    mem_reservation: 4g
```

#### 2. CPU优先级

```yaml
services:
  agv_core:
    cpus: 4
    cpu_period: 100000
    cpu_quota: 400000
```

#### 3. 自动重启

```yaml
services:
  agv_core:
    restart: always
    restart_policy:
      max_attempts: 3
```

---

## 🔒 安全建议

### 1. 修改默认密钥

```bash
# 生成强密钥
openssl rand -base64 32

# 编辑 .env 文件
vim .env
# 修改 AGV_WEB_JWT_SECRET=your_new_secret_key
```

### 2. 限制网络访问

```bash
# 使用防火墙
sudo ufw allow 8080/tcp
sudo ufw deny 8080
```

### 3. 定期更新

```bash
# 拉取最新代码
git pull

# 重新构建镜像
./deploy.sh build

# 滚动更新
docker-compose up -d --build
```

### 4. 监控安全事件

```bash
# 查看登录日志
docker exec agv_core last

# 查看认证日志
docker exec agv_core cat /var/log/auth.log | grep -i fail
```

---

## 📊 部署检查清单

### 部署前

- [ ] 硬件要求满足
- [ ] Docker 和 Docker Compose 已安装
- [ ] NVIDIA驱动已安装（GPU环境）
- [ ] 防火墙已配置
- [ ] 配置文件已创建

### 部署中

- [ ] 镜像构建成功
- [ ] 服务启动成功
- [ ] 端口可访问
- [ ] 数据库初始化成功
- [ ] ROS节点运行正常

### 部署后

- [ ] Web界面可访问
- [ ] 健康检查通过
- [ ] API文档可访问
- [ ] 数据持久化正常
- [ ] 日志正常记录

---

## 📞 技术支持

如遇到问题，请提供以下信息：

1. 操作系统和内核版本
2. Docker 版本
3. 完整的错误日志
4. 部署配置文件
5. 复现步骤

---

## 📄 许可证

本项目采用 MIT 许可证 - 详见 LICENSE 文件

---

## 👥 贡献者

- AGV Development Team

---

**最后更新**: 2026-05-26
**版本**: 2.0
