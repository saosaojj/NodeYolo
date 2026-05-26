#!/bin/bash
# =====================================================
# AGV Docker 入口脚本
# =====================================================
set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} ${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} ${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} ${RED}[ERROR]${NC} $1" >&2
}

# 打印启动横幅
print_banner() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}   AGV ROS2 System v2.0${NC}"
    echo -e "${GREEN}   Docker Container${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
}

# 初始化ROS环境
init_ros() {
    log_info "初始化ROS2环境..."

    # 检查ROS安装
    if [ ! -f /opt/ros/humble/setup.bash ]; then
        log_error "ROS2 Humble 未安装"
        exit 1
    fi

    source /opt/ros/humble/setup.bash

    # 检查工作空间
    if [ -f /agv_ros2_ws/install/setup.bash ]; then
        log_info "加载AGV工作空间..."
        source /agv_ros2_ws/install/setup.bash
    else
        log_warn "AGV工作空间未构建，使用源代码模式"
        if [ -f /agv_ros2_ws/src/setup.bash ]; then
            source /agv_ros2_ws/src/setup.bash
        fi
    fi

    log_info "ROS2环境初始化完成"
}

# 创建必要的目录
create_directories() {
    log_info "创建必要的目录..."

    mkdir -p /agv_config
    mkdir -p /agv_data
    mkdir -p /agv_logs
    mkdir -p /root/.agv_web_config

    # 设置权限
    chmod -R 755 /agv_config
    chmod -R 755 /agv_data
    chmod -R 755 /agv_logs
    chmod -R 755 /root/.agv_web_config

    log_info "目录创建完成"
}

# 检查网络
check_network() {
    log_info "检查网络连接..."
    if ping -c 1 8.8.8.8 > /dev/null 2>&1; then
        log_info "网络连接正常"
    else
        log_warn "无法访问外部网络（可能在内网环境中）"
    fi
}

# 启动Web服务
start_web_server() {
    log_info "启动Web服务..."
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
    export AGV_WEB_HOST=${AGV_WEB_HOST:-0.0.0.0}
    export AGV_WEB_PORT=${AGV_WEB_PORT:-8080}

    ros2 launch agv_web_server web_server.launch.py &
    local web_pid=$!

    log_info "Web服务已启动 (PID: $web_pid)"
}

# 启动导航
start_navigation() {
    log_info "启动导航系统..."
    ros2 launch agv_navigation navigation.launch.py &
    local nav_pid=$!

    log_info "导航系统已启动 (PID: $nav_pid)"
}

# 启动IO控制
start_io_controller() {
    log_info "启动IO控制器..."
    ros2 launch agv_io_controller io_controller.launch.py &
    local io_pid=$!

    log_info "IO控制器已启动 (PID: $io_pid)"
}

# 启动PLC桥接
start_plc_bridge() {
    log_info "启动PLC桥接..."
    ros2 launch agv_plc_bridge plc_bridge.launch.py &
    local plc_pid=$!

    log_info "PLC桥接已启动 (PID: $plc_pid)"
}

# 启动视觉识别
start_vision() {
    log_info "启动视觉识别系统..."
    ros2 launch agv_vision vision.launch.py &
    local vision_pid=$!

    log_info "视觉识别系统已启动 (PID: $vision_pid)"
}

# 启动所有服务
start_all() {
    log_info "启动所有AGV服务..."

    init_ros
    create_directories
    check_network

    # 启动各项服务
    start_navigation
    sleep 2

    start_io_controller
    sleep 1

    start_plc_bridge
    sleep 1

    start_web_server
    sleep 1

    # 可选启动视觉系统
    if [ "${ENABLE_VISION}" = "true" ]; then
        start_vision
    fi

    log_info "所有服务启动完成"
    log_info "Web服务地址: http://${AGV_WEB_HOST:-0.0.0.0}:${AGV_WEB_PORT:-8080}"

    # 等待所有后台进程
    wait
}

# 显示帮助信息
show_help() {
    print_banner
    echo "用法: docker-entrypoint.sh [命令]"
    echo ""
    echo "可用命令:"
    echo "  launch_all       启动所有服务（默认）"
    echo "  launch_web       仅启动Web服务"
    echo "  launch_nav       仅启动导航系统"
    echo "  launch_io        仅启动IO控制器"
    echo "  launch_plc       仅启动PLC桥接"
    echo "  launch_vision    仅启动视觉系统"
    echo "  dev              开发模式（交互式bash）"
    echo "  status           显示服务状态"
    echo "  help             显示此帮助信息"
    echo ""
    echo "环境变量:"
    echo "  ROS_DOMAIN_ID    ROS2域ID（默认: 30）"
    echo "  AGV_WEB_HOST     Web服务监听地址（默认: 0.0.0.0）"
    echo "  AGV_WEB_PORT     Web服务端口（默认: 8080）"
    echo "  ENABLE_VISION    启用视觉系统（默认: false）"
    echo ""
}

# 显示状态
show_status() {
    print_banner
    echo "AGV服务状态"
    echo "================"
    echo ""

    # 检查ROS节点
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        if [ -f /agv_ros2_ws/install/setup.bash ]; then
            source /agv_ros2_ws/install/setup.bash

            echo -e "${GREEN}ROS2节点:${NC}"
            ros2 node list 2>/dev/null || echo "  无法获取节点列表"
            echo ""

            echo -e "${GREEN}ROS2话题:${NC}"
            ros2 topic list 2>/dev/null | head -10 || echo "  无法获取话题列表"
            echo ""
        fi
    fi

    # 检查进程
    echo -e "${GREEN}运行中的进程:${NC}"
    ps aux | grep -E "ros2|agv_" | grep -v grep || echo "  无ROS进程运行"
    echo ""

    # 检查数据库
    echo -e "${GREEN}数据库状态:${NC}"
    if [ -f /root/.agv_web_config/agv_data.db ]; then
        echo "  ✓ 数据库文件存在"
    else
        echo "  ✗ 数据库文件不存在"
    fi
    echo ""

    # 检查端口
    echo -e "${GREEN}监听端口:${NC}"
    if command -v netstat &> /dev/null; then
        netstat -tuln | grep -E "8080|5020" || echo "  无相关端口监听"
    elif command -v ss &> /dev/null; then
        ss -tuln | grep -E "8080|5020" || echo "  无相关端口监听"
    fi
}

# 主程序
main() {
    print_banner

    local command=${1:-launch_all}

    case $command in
        launch_all)
            log_info "启动完整AGV系统..."
            start_all
            ;;
        launch_web)
            init_ros
            create_directories
            start_web_server
            wait
            ;;
        launch_nav)
            init_ros
            start_navigation
            wait
            ;;
        launch_io)
            init_ros
            start_io_controller
            wait
            ;;
        launch_plc)
            init_ros
            start_plc_bridge
            wait
            ;;
        launch_vision)
            init_ros
            start_vision
            wait
            ;;
        dev)
            log_info "进入开发模式..."
            exec /bin/bash
            ;;
        status)
            show_status
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            log_error "未知命令: $command"
            show_help
            exit 1
            ;;
    esac
}

# 执行主程序
main "$@"
