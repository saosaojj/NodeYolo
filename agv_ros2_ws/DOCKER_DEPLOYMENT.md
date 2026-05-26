# AGV ROS2 Docker 详细部署指南

## 📋 目录

- [概述](#概述)
- [系统要求](#系统要求)
- [安装Docker](#安装docker)
- [获取项目代码](#获取项目代码)
- [部署流程](#部署流程)
- [配置说明](#配置说明)
- [服务管理](#服务管理)
- [监控和维护](#监控和维护)
- [常见问题](#常见问题)
- [故障排除](#故障排除)
- [安全加固](#安全加固)

---

## 1. 概述

AGV ROS2是一个基于ROS2 Humble的完整AGV系统，提供以下功能：

- ✅ ROS2节点通信和管理
- ✅ 自主导航和路径规划
- ✅ Web前端控制界面
- ✅ 摄像头和视觉识别
- ✅ PLC通信和IO控制
- ✅ 仿真测试功能
- ✅ SQLite数据库持久化
- ✅ 完整的Docker部署方案

---

## 2. 系统要求

### 2.1 硬件要求

| 组件 | 最低配置 | 推荐配置 | 说明 |
|------|---------|---------|------|
| **CPU** | 4核 | 8核+ | 越多越好，ROS2节点较多 |
| **内存** | 8GB | 16GB+ | 建议使用16GB以上 |
| **存储** | 20GB HDD | 50GB SSD | SSD提供更好的性能 |
| **网络** | 100Mbps | 1Gbps | 用于实时通信和视频流 |
| **GPU** | 可选 | NVIDIA显卡 | 视觉识别需要 |

### 2.2 软件要求

| 软件 | 最低版本 | 推荐版本 | 说明 |
|------|---------|---------|------|
| **Docker** | 20.10 | 23.0+ | 容器引擎 |
| **Docker Compose** | 2.0 | 2.18+ | 容器编排工具 |
| **操作系统** | Ubuntu 20.04 | Ubuntu 22.04 | 推荐生产环境 |
| **NVIDIA驱动** | 525+ | 535+ | 仅GPU环境需要 |

### 2.3 网络要求

- 8080端口：Web服务（必须开放）
- 5020端口：PLC通信（可选）
- ROS2通信端口：动态分配（使用host网络模式）

---

## 3. 安装Docker

### 3.1 Ubuntu系统安装Docker

```bash
# 更新系统包索引
sudo apt-get update

# 安装依赖
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# 添加Docker官方GPG密钥
sudo mkdir -m 0755 -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# 设置Docker软件源
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 安装Docker Engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 验证安装
sudo docker --version
sudo docker compose version

# 启用Docker服务
sudo systemctl enable docker
sudo systemctl start docker

# 添加当前用户到docker组
sudo usermod -aG docker $USER

# 注销并重新登录，或运行
newgrp docker
```

### 3.2 安装NVIDIA Container Toolkit（GPU环境）

```bash
# 添加NVIDIA GPG密钥
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# 设置源
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 安装
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 配置Docker
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 测试GPU
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

---

## 4. 获取项目代码

### 4.1 克隆仓库

```bash
# 克隆项目
git clone <your-github-repository-url>
cd agv_ros2_ws

# 查看项目结构
ls -la
```

### 4.2 检查文件完整性

确保以下文件存在：

```
agv_ros2_ws/
├── Dockerfile
├── docker-compose.yml
├── docker-compose.prod.yml
├── docker-compose.dev.yml
├── docker-compose.test.yml
├── docker-entrypoint.sh
├── docker-healthcheck.sh
├── deploy.sh
├── monitor.sh
├── .env.example
├── .gitignore
├── requirements.txt
├── README.md
├── DEPLOYMENT_GUIDE.md
└── src/
    ├── agv_web_server/
    ├── agv_web_frontend/
    └── ...
```

---

## 5. 部署流程

### 5.1 准备工作

```bash
# 1. 进入项目目录
cd /path/to/agv_ros2_ws

# 2. 赋予脚本执行权限
chmod +x deploy.sh
chmod +x monitor.sh
chmod +x docker-entrypoint.sh
chmod +x docker-healthcheck.sh

# 3. 配置环境变量（可选）
cp .env.example .env
# 编辑 .env 文件，根据需要修改配置
```

### 5.2 方式一：使用部署脚本（推荐）

#### 生产环境部署

```bash
# 构建Docker镜像
./deploy.sh build

# 部署生产环境
./deploy.sh deploy-prod

# 查看状态
./deploy.sh status

# 查看日志
./deploy.sh logs

# 执行健康检查
./deploy.sh health
```

#### 开发环境部署

```bash
# 部署开发环境
./deploy.sh deploy-dev

# 进入开发容器
docker exec -it agv_dev bash

# 在容器内
cd /agv_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 启动Web服务
ros2 launch agv_web_server web_server.launch.py
```

#### 测试环境部署

```bash
# 部署测试环境
./deploy.sh deploy-test

# 访问测试服务
# http://localhost:8082
```

### 5.3 方式二：使用Docker Compose

#### 生产环境

```bash
# 启动所有服务
docker-compose -f docker-compose.prod.yml up -d

# 查看日志
docker-compose -f docker-compose.prod.yml logs -f

# 查看状态
docker-compose -f docker-compose.prod.yml ps

# 停止服务
docker-compose -f docker-compose.prod.yml down
```

#### 完整环境（所有服务）

```bash
# 启动所有服务
docker-compose up -d

# 查看服务状态
docker-compose ps

# 查看特定服务日志
docker-compose logs -f agv_web

# 进入容器
docker exec -it agv_core bash
```

### 5.4 方式三：手动构建和运行

#### 步骤1：构建镜像

```bash
# 构建完整版镜像
docker build --target runtime-full -t agv_ros2:full .

# 构建精简版镜像
docker build --target runtime-web -t agv_ros2:web .

# 构建开发版镜像
docker build --target development -t agv_ros2:dev .

# 查看构建的镜像
docker images | grep agv_ros2
```

#### 步骤2：创建数据卷

```bash
# 创建必要的目录
mkdir -p volumes/config
mkdir -p volumes/data
mkdir -p volumes/logs
mkdir -p volumes/models
```

#### 步骤3：运行容器

```bash
# 运行核心服务
docker run -d \
  --name agv_core \
  --network host \
  --privileged \
  -v $(pwd)/volumes/config:/agv_config \
  -v $(pwd)/volumes/data:/agv_data \
  -v $(pwd)/volumes/logs:/agv_logs \
  -v /dev:/dev \
  -v /sys/class/gpio:/sys/class/gpio \
  -v /var/run/dbus:/var/run/dbus \
  -e ROS_DOMAIN_ID=30 \
  -e AGV_CONFIG_DIR=/agv_config \
  -e AGV_DATA_DIR=/agv_data \
  -e AGV_LOG_DIR=/agv_logs \
  --restart unless-stopped \
  agv_ros2:full \
  launch_all

# 运行Web服务
docker run -d \
  --name agv_web \
  --network host \
  -p 8080:8080 \
  -v $(pwd)/volumes/config:/agv_config \
  -v $(pwd)/volumes/data:/agv_data \
  -v $(pwd)/volumes/logs:/agv_logs \
  -e ROS_DOMAIN_ID=30 \
  -e AGV_WEB_HOST=0.0.0.0 \
  -e AGV_WEB_PORT=8080 \
  --restart unless-stopped \
  agv_ros2:full \
  launch_web

# 查看容器状态
docker ps -a
```

---

## 6. 配置说明

### 6.1 环境变量配置

复制和编辑配置文件：

```bash
cp .env.example .env
vim .env
```

#### 常用配置项

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `ROS_DOMAIN_ID` | 30 | ROS2域ID，多机器人系统需不同 |
| `RMW_IMPLEMENTATION` | rmw_cyclonedds_cpp | ROS2中间件实现 |
| `AGV_WEB_HOST` | 0.0.0.0 | Web服务监听地址 |
| `AGV_WEB_PORT` | 8080 | Web服务端口 |
| `AGV_WEB_JWT_SECRET` | change_me | JWT密钥，生产环境必须修改 |
| `ENABLE_VISION` | false | 是否启用视觉识别 |
| `ENABLE_SIMULATION` | true | 是否启用仿真模式 |
| `CAMERA_DEVICE` | 0 | 摄像头设备索引或RTSP地址 |
| `CAMERA_USE_RTSP` | false | 是否使用RTSP流 |
| `CAMERA_FPS` | 30 | 摄像头帧率 |
| `PLC_IP` | 127.0.0.1 | PLC设备IP地址 |
| `PLC_PORT` | 502 | PLC通信端口 |
| `PLC_SLAVE_ID` | 1 | PLC从站ID |

#### 生产环境关键配置

```env
# 生产环境必须修改的配置
AGV_WEB_JWT_SECRET=your_strong_secret_key_here

# 关闭仿真模式（使用真实硬件）
ENABLE_SIMULATION=false

# 根据需要启用视觉识别
ENABLE_VISION=true
```

### 6.2 数据库配置

数据库位置：`/root/.agv_web_config/agv_data.db`

```bash
# 进入容器查看数据库
docker exec -it agv_core bash

# 使用SQLite
sqlite3 /root/.agv_web_config/agv_data.db

# 查看表
sqlite> .tables

# 查询仿真状态
sqlite> SELECT * FROM simulation_state;

# 查询配置历史
sqlite> SELECT * FROM config_history ORDER BY timestamp DESC LIMIT 10;
```

### 6.3 持久化配置

所有数据都存储在`volumes/`目录中：

```
volumes/
├── config/   # 配置文件
├── data/     # 数据库和数据
├── logs/     # 日志文件
└── models/   # 模型文件
```

备份数据：

```bash
# 备份所有数据
tar -czf agv_backup_$(date +%Y%m%d_%H%M%S).tar.gz volumes/

# 恢复数据
tar -xzf agv_backup_20240101_120000.tar.gz
```

---

## 7. 服务管理

### 7.1 服务状态检查

```bash
# 方式1：使用部署脚本
./deploy.sh status

# 方式2：使用Docker Compose
docker-compose ps

# 方式3：查看所有容器
docker ps -a
```

### 7.2 启动和停止服务

```bash
# 启动所有服务
docker-compose up -d

# 停止所有服务
docker-compose down

# 重启特定服务
docker-compose restart agv_web

# 查看服务日志
docker-compose logs -f agv_core
```

### 7.3 进入容器调试

```bash
# 进入核心服务容器
docker exec -it agv_core bash

# 进入Web服务容器
docker exec -it agv_web bash

# 进入开发环境容器
docker exec -it agv_dev bash

# 在容器内检查ROS2节点
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list
```

### 7.4 使用入口脚本命令

```bash
# 进入容器后使用
docker exec -it agv_core /docker-entrypoint.sh help

# 显示服务状态
docker exec -it agv_core /docker-entrypoint.sh status

# 启动特定服务
docker exec -it agv_core /docker-entrypoint.sh launch_web
```

---

## 8. 监控和维护

### 8.1 使用监控脚本

```bash
# 单次检查
./monitor.sh --single

# 持续监控（默认5秒间隔）
./monitor.sh --monitor

# 持续监控（10秒间隔）
./monitor.sh --monitor --interval 10
```

监控内容：
- 容器运行状态
- CPU和内存使用
- 磁盘空间
- Web服务响应
- ROS2节点数量

### 8.2 日志管理

```bash
# 查看所有服务的日志
docker-compose logs -f

# 查看特定服务的日志
docker-compose logs -f agv_web

# 查看最近100行日志
docker-compose logs --tail=100 agv_core

# 查找特定关键词
docker-compose logs agv_core | grep -i error

# 导出日志
docker-compose logs > agv_logs_$(date +%Y%m%d_%H%M%S).txt

# 查看容器日志文件
docker logs --details agv_core
```

### 8.3 性能监控

```bash
# 查看Docker资源使用
docker stats

# 查看特定容器资源
docker stats agv_core agv_web

# 使用htop（需要进入容器）
docker exec -it agv_core htop
```

### 8.4 定期维护任务

建议设置定期维护：

```bash
# 清理未使用的Docker资源
docker system prune -f

# 清理日志（保留最近的）
docker-compose logs --tail=1000 > recent_logs.txt

# 备份数据
tar -czf backup_$(date +%Y%m%d).tar.gz volumes/
```

---

## 9. 常见问题

### 9.1 端口被占用

**问题**：启动服务时提示端口被占用

```bash
# 检查端口占用
sudo netstat -tulpn | grep 8080
# 或
sudo lsof -i :8080

# 解决方案1：修改端口
# 编辑 .env 文件，修改 AGV_WEB_PORT

# 解决方案2：停止占用端口的进程
sudo kill -9 <PID>

# 解决方案3：使用其他端口
docker-compose -f docker-compose.test.yml up -d
# 测试环境使用8082端口
```

### 9.2 权限问题

**问题**：容器无法访问设备或文件

```bash
# 检查Docker是否以正确权限运行
groups | grep docker

# 如果不在docker组中
sudo usermod -aG docker $USER
newgrp docker

# 检查文件权限
ls -la volumes/

# 修改权限
chmod -R 755 volumes/
```

### 9.3 镜像构建失败

**问题**：Docker构建过程中出错

```bash
# 清理旧镜像
docker system prune -a

# 查看构建日志
docker build --no-cache --progress=plain -t agv_ros2:full .

# 分步调试构建
docker build --target builder -t agv_ros2:builder .
docker run -it agv_ros2:builder bash
# 在容器内手动执行构建步骤
```

### 9.4 容器无法启动

**问题**：容器启动后立即停止

```bash
# 查看容器日志
docker logs agv_core

# 查看容器详细信息
docker inspect agv_core

# 手动启动容器进行调试
docker run -it --rm agv_ros2:full bash
# 在容器内检查问题
```

### 9.5 ROS2节点无法通信

**问题**：ROS2节点无法相互发现

```bash
# 检查ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID
docker exec agv_core bash -c 'echo $ROS_DOMAIN_ID'

# 确保所有容器使用相同的网络模式
# 推荐使用 --network host
# 或确保容器在同一Docker网络中

# 检查节点
docker exec -it agv_core bash -c '
source /opt/ros/humble/setup.bash
source /agv_ros2_ws/install/setup.bash 2>/dev/null
ros2 node list
'
```

---

## 10. 故障排除

### 10.1 诊断脚本

创建诊断脚本：

```bash
#!/bin/bash
echo "=== AGV System Diagnostic ==="
echo ""

echo "1. Docker Status:"
docker --version
docker compose version
echo ""

echo "2. Container Status:"
docker ps -a
echo ""

echo "3. Resource Usage:"
docker stats --no-stream
echo ""

echo "4. Logs (Last 10 lines):"
docker logs --tail=10 agv_core 2>/dev/null || echo "No logs for agv_core"
docker logs --tail=10 agv_web 2>/dev/null || echo "No logs for agv_web"
echo ""

echo "5. Disk Space:"
df -h
echo ""
```

### 10.2 重置系统

如果系统出现严重问题：

```bash
# 备份数据
tar -czf backup_before_reset.tar.gz volumes/

# 停止所有服务
docker-compose down -v

# 清理所有容器和镜像
docker system prune -a -f

# 重新构建和部署
./deploy.sh build
./deploy.sh deploy-prod

# 恢复数据（如果需要）
rm -rf volumes/
tar -xzf backup_before_reset.tar.gz
docker-compose up -d
```

### 10.3 常见错误代码

| 错误代码 | 可能原因 | 解决方案 |
|---------|---------|---------|
| 127 | 命令未找到 | 检查脚本执行权限和PATH |
| 137 | 内存不足 | 增加容器内存限制或关闭其他程序 |
| 255 | 执行错误 | 查看详细日志排查问题 |

---

## 11. 安全加固

### 11.1 基本安全措施

1. **修改默认密钥**

```bash
# 生成强密钥
openssl rand -base64 32

# 在.env文件中设置
AGV_WEB_JWT_SECRET=your_strong_generated_key
```

2. **限制网络访问**

```bash
# 使用UFW防火墙
sudo ufw enable
sudo ufw allow 8080/tcp
sudo ufw allow 22/tcp  # SSH
sudo ufw default deny incoming
```

3. **定期更新**

```bash
# 更新系统
sudo apt-get update && sudo apt-get upgrade -y

# 重新构建Docker镜像
./deploy.sh build

# 滚动更新
docker-compose up -d --build
```

### 11.2 Docker安全最佳实践

```bash
# 不要以root用户运行容器（生产环境）
# 在Dockerfile中添加非root用户

# 限制容器资源
# 在docker-compose.yml中添加
mem_limit: 4g
cpus: 4

# 使用只读文件系统
# 在docker-compose.yml中添加
read_only: true
tmpfs:
  - /tmp
  - /var/run
```

### 11.3 监控安全事件

```bash
# 查看认证日志
docker exec agv_core cat /var/log/auth.log | grep -i fail

# 检查容器内的异常进程
docker top agv_core

# 检查网络连接
docker exec -it agv_core netstat -tulpn
```

---

## 12. 高级配置

### 12.1 使用HTTPS

```bash
# 使用Let's Encrypt证书
# 1. 安装certbot
sudo apt-get install certbot

# 2. 获取证书
sudo certbot certonly --standalone -d your-domain.com

# 3. 更新docker-compose.yml配置Web服务器使用HTTPS
```

### 12.2 配置反向代理

使用Nginx作为反向代理：

```nginx
# nginx.conf
server {
    listen 80;
    server_name your-domain.com;

    location / {
        proxy_pass http://localhost:8080;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        # WebSocket支持
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

### 12.3 日志轮转

配置日志轮转：

```bash
# /etc/logrotate.d/agv-ros2
/path/to/agv_ros2_ws/volumes/logs/*.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 0644 root root
}
```

---

## 附录

### A. 快速参考命令

```bash
# Docker常用命令
docker ps -a                    # 查看所有容器
docker logs -f <container>      # 查看容器日志
docker exec -it <container> bash # 进入容器
docker stats                    # 资源使用
docker images                   # 查看镜像
docker system prune             # 清理资源

# Docker Compose命令
docker-compose up -d            # 启动服务
docker-compose down             # 停止服务
docker-compose logs -f          # 查看日志
docker-compose ps               # 查看状态
docker-compose restart         # 重启服务

# 部署脚本命令
./deploy.sh build              # 构建镜像
./deploy.sh deploy-prod        # 生产部署
./deploy.sh status             # 查看状态
./deploy.sh health             # 健康检查

# 监控脚本命令
./monitor.sh --single          # 单次检查
./monitor.sh --monitor        # 持续监控
```

### B. 文件和目录说明

| 路径 | 说明 |
|------|------|
| `/agv_config` | 配置文件目录 |
| `/agv_data` | 数据文件目录 |
| `/agv_logs` | 日志文件目录 |
| `/root/.agv_web_config` | Web服务配置和数据库 |
| `/agv_ros2_ws` | 工作空间根目录 |

---

**文档版本**: 1.0
**最后更新**: 2026-05-26
**维护者**: AGV Development Team
