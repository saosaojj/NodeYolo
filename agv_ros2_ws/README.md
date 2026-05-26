# AGV ROS2 Project

<div align="center">

![AGV ROS2](https://img.shields.io/badge/AGV-ROS2-blue.svg)
![Version](https://img.shields.io/badge/Version-2.0-green.svg)
![Docker](https://img.shields.io/badge/Docker-Supported-blue.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

一个基于ROS2 Humble的完整AGV（自动导引车）系统，包含Web界面、仿真测试、数据库存储等功能。

</div>

## 📋 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [快速开始](#快速开始)
- [项目结构](#项目结构)
- [Docker部署](#docker部署)
- [开发指南](#开发指南)
- [API文档](#api文档)
- [贡献指南](#贡献指南)
- [许可证](#许可证)

---

## 🚀 项目简介

AGV ROS2是一个功能完整的自动导引车系统，基于ROS2 Humble构建。该系统包含以下核心功能：

- ✅ ROS2节点管理和通信
- ✅ 导航和路径规划
- ✅ Web前端控制界面
- ✅ 摄像头和视觉识别
- ✅ PLC通信和IO控制
- ✅ 仿真测试功能（无需硬件）
- ✅ SQLite数据库持久化
- ✅ 完整的Docker部署方案

---

## ✨ 功能特性

### 核心功能

| 功能 | 描述 |
|------|------|
| **导航系统** | 基于ROS2的自主导航和路径规划 |
| **Web界面** | 现代化的响应式Web控制界面 |
| **仿真模式** | 无需硬件即可完整测试系统 |
| **数据库存储** | SQLite持久化存储配置和数据 |
| **PLC通信** | Modbus TCP PLC通信 |
| **视觉识别** | 基于YOLO的视觉识别系统 |
| **健康检查** | 完整的服务健康监控 |
| **Docker部署** | 一键部署到生产环境 |

### 仿真功能

- 📷 摄像头仿真（彩色渐变图像）
- 🔧 PLC仿真（随机线圈和寄存器）
- 👁️ 视觉识别仿真
- 💾 配置历史记录
- 📊 数据持久化

---

## 🏁 快速开始

### 前提条件

- Docker 20.10+
- Docker Compose 2.0+
- 至少8GB RAM
- 20GB+ 磁盘空间

### 一键部署

```bash
# 1. 克隆项目
git clone <your-repository-url>
cd agv_ros2_ws

# 2. 赋予执行权限
chmod +x deploy.sh monitor.sh docker-entrypoint.sh docker-healthcheck.sh

# 3. 构建并部署
./deploy.sh build
./deploy.sh deploy-prod

# 4. 查看状态
./deploy.sh status

# 5. 访问Web界面
# 浏览器打开: http://localhost:8080
```

### 验证部署

```bash
# 查看服务状态
./deploy.sh status

# 执行健康检查
./deploy.sh health

# 查看监控
./monitor.sh --single
```

---

## 📁 项目结构

```
agv_ros2_ws/
├── src/                                      # 源码目录
│   ├── agv_web_server/                     # Web服务模块
│   │   ├── agv_web_server/               # 核心代码
│   │   │   ├── __init__.py
│   │   │   ├── config_manager.py         # 配置管理
│   │   │   ├── database_manager.py       # 数据库管理
│   │   │   ├── camera_manager.py         # 摄像头管理
│   │   │   ├── plc_manager.py            # PLC管理
│   │   │   ├── api_routes.py             # API路由
│   │   │   ├── web_server_node.py        # Web服务节点
│   │   │   └── websocket_handler.py      # WebSocket处理
│   │   ├── config/                        # 配置文件
│   │   ├── launch/                        # 启动文件
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── agv_web_frontend/                   # 前端模块
│   │   ├── index.html                    # 主页面
│   │   ├── css/style.css                 # 样式文件
│   │   └── js/app.js                     # 前端逻辑
│   │
│   ├── agv_navigation/                     # 导航模块
│   ├── agv_plc_bridge/                     # PLC桥接模块
│   ├── agv_io_controller/                  # IO控制模块
│   ├── agv_vision/                         # 视觉识别模块
│   ├── agv_interfaces/                     # ROS2接口定义
│   └── ... (其他模块)
│
├── Dockerfile                               # 多阶段构建文件
├── docker-compose.yml                       # 完整服务编排
├── docker-compose.prod.yml                  # 生产环境配置
├── docker-compose.dev.yml                   # 开发环境配置
├── docker-compose.test.yml                  # 测试环境配置
├── docker-entrypoint.sh                     # 智能入口脚本
├── docker-healthcheck.sh                    # 健康检查脚本
├── deploy.sh                               # 一键部署脚本
├── monitor.sh                              # 监控脚本
├── .env.example                            # 环境配置模板
├── .gitignore                              # Git忽略文件
├── requirements.txt                        # Python依赖
├── DEPLOYMENT_GUIDE.md                     # 完整部署指南
└── volumes/                                # 数据持久化目录
    ├── config/
    ├── data/
    ├── logs/
    └── models/
```

---

## 🐳 Docker部署

### 部署方式

#### 方式一：使用部署脚本（推荐）

```bash
# 生产环境
./deploy.sh deploy-prod

# 开发环境
./deploy.sh deploy-dev

# 测试环境
./deploy.sh deploy-test
```

#### 方式二：使用Docker Compose

```bash
# 启动所有服务
docker-compose up -d

# 启动特定服务
docker-compose up -d agv_core

# 停止服务
docker-compose down
```

#### 方式三：手动构建和运行

```bash
# 构建镜像
docker build --target runtime-full -t agv_ros2:full .

# 运行容器
docker run -d --name agv_core --network host --privileged agv_ros2:full
```

### Docker服务说明

| 服务 | 端口 | 描述 |
|------|------|------|
| `agv_core` | - | 核心服务（ROS2节点） |
| `agv_web` | 8080 | Web服务和API |
| `agv_vision` | - | 视觉识别（GPU） |
| `agv_web_light` | 8081 | 轻量版Web服务 |
| `agv_dev` | - | 开发环境 |

### 环境变量配置

复制环境配置模板：

```bash
cp .env.example .env
```

编辑配置：

```env
# ROS2配置
ROS_DOMAIN_ID=30
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Web服务配置
AGV_WEB_HOST=0.0.0.0
AGV_WEB_PORT=8080
AGV_WEB_JWT_SECRET=your_secret_key

# 功能开关
ENABLE_VISION=false
ENABLE_SIMULATION=true
```

### 健康检查

系统内置了完整的健康检查：

```bash
# 使用部署脚本
./deploy.sh health

# 使用监控脚本
./monitor.sh --single
```

### 日志管理

```bash
# 查看所有服务日志
docker-compose logs -f

# 查看特定服务
docker-compose logs -f agv_web

# 查看最近100行
docker-compose logs --tail=100
```

---

## 👨‍💻 开发指南

### 开发环境设置

```bash
# 1. 启动开发环境
./deploy.sh deploy-dev

# 2. 进入容器
docker exec -it agv_dev bash

# 3. 在容器内
cd /agv_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 4. 运行节点
ros2 launch agv_web_server web_server.launch.py
```

### 代码规范

- 使用PEP8代码风格
- 添加完整的中文注释
- 编写单元测试
- 提交前运行代码检查

### 添加新功能

1. 在相应模块中创建新文件
2. 编写API路由（如果需要）
3. 更新前端界面
4. 编写测试用例
5. 更新文档

---

## 📚 API文档

### 基础API

| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/` | 主页 |
| GET | `/docs` | API文档（Swagger） |
| GET | `/api/v1/system/health` | 健康检查 |

### 仿真API

| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/api/v1/simulation/status` | 获取仿真状态 |
| POST | `/api/v1/simulation/status` | 设置仿真状态 |
| POST | `/api/v1/simulation/camera` | 启用/禁用摄像头仿真 |
| POST | `/api/v1/simulation/plc` | 启用/禁用PLC仿真 |

### 数据查询API

| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/api/v1/data/config/history` | 配置历史记录 |
| GET | `/api/v1/data/agv/history` | AGV状态历史 |
| POST | `/api/v1/data/cleanup` | 清理过期数据 |

更多API文档请访问：`http://localhost:8080/docs`

---

## 🛠️ 监控和维护

### 使用监控脚本

```bash
# 单次检查
./monitor.sh --single

# 持续监控
./monitor.sh --monitor

# 设置检查间隔
./monitor.sh --monitor --interval 10
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

### 系统更新

```bash
# 拉取最新代码
git pull

# 重新构建
./deploy.sh build

# 滚动更新
docker-compose up -d --build
```

---

## 📖 详细文档

更多详细信息请查看：

- [部署指南](DEPLOYMENT_GUIDE.md) - 完整的部署和配置文档
- [API文档](http://localhost:8080/docs) - 自动生成的API文档
- [Docker Compose参考](docker-compose.yml) - Docker服务配置

---

## 🤝 贡献指南

我们欢迎任何形式的贡献！

1. Fork本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

---

## 🔒 安全建议

1. **修改默认密钥**：在生产环境中必须修改 `AGV_WEB_JWT_SECRET`
2. **限制网络访问**：使用防火墙限制不必要的端口访问
3. **定期更新**：保持系统和依赖库更新
4. **备份数据**：定期备份配置和数据库
5. **监控日志**：定期检查系统日志

---

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 👥 贡献者

- AGV Development Team

---

## 📞 支持

如有问题，请查看：

- [常见问题](DEPLOYMENT_GUIDE.md#故障排除)
- [GitHub Issues](../../issues)

---

## 📊 项目状态

| 状态 | 描述 |
|------|------|
| ✅ | 代码完整，包含所有功能 |
| ✅ | Docker部署方案完整 |
| ✅ | 文档完整 |
| ✅ | 仿真测试功能完整 |
| ✅ | 数据库持久化实现 |

---

**最后更新**: 2026-05-26
**版本**: 2.0
