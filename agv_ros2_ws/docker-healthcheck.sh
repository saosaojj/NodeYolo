#!/bin/bash
# =====================================================
# AGV Docker 健康检查脚本
# =====================================================

set -e

# 日志函数
log_info() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [HEALTHCHECK] INFO: $1"
}

log_error() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [HEALTHCHECK] ERROR: $1" >&2
}

# 检查ROS2环境
check_ros() {
    log_info "检查ROS2环境..."
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        if command -v ros2 &> /dev/null; then
            log_info "ROS2 命令可用"
            return 0
        else
            log_error "ROS2 命令不可用"
            return 1
        fi
    else
        log_error "ROS2 Humble 未安装"
        return 1
    fi
}

# 检查Web服务
check_web_service() {
    log_info "检查Web服务..."
    if curl -f -s http://localhost:8080/api/v1/system/health > /dev/null 2>&1; then
        log_info "Web服务健康"
        return 0
    else
        log_error "Web服务不健康或未响应"
        return 1
    fi
}

# 检查ROS2节点
check_ros_nodes() {
    log_info "检查ROS2节点..."
    source /opt/ros/humble/setup.bash
    if [ -f /agv_ros2_ws/install/setup.bash ]; then
        source /agv_ros2_ws/install/setup.bash
        # 检查关键节点是否存在
        if ros2 node list 2>/dev/null | grep -q "web_server_node\|navigation_node\|plc_node"; then
            log_info "关键ROS2节点运行中"
            return 0
        else
            log_error "关键ROS2节点未运行"
            return 1
        fi
    else
        log_error "ROS2工作空间未构建"
        return 1
    fi
}

# 检查数据库
check_database() {
    log_info "检查数据库..."
    if [ -f /root/.agv_web_config/agv_data.db ]; then
        # 检查SQLite数据库是否可读
        if python3 -c "import sqlite3; conn = sqlite3.connect('/root/.agv_web_config/agv_data.db'); conn.close()" 2>/dev/null; then
            log_info "数据库健康"
            return 0
        else
            log_error "数据库不可访问"
            return 1
        fi
    else
        log_info "数据库文件不存在（首次运行）"
        return 0
    fi
}

# 检查日志目录
check_logs() {
    log_info "检查日志目录..."
    if [ -d /agv_logs ]; then
        log_info "日志目录存在"
        return 0
    else
        log_error "日志目录不存在"
        return 1
    fi
}

# 主要检查逻辑
main() {
    log_info "开始健康检查..."

    # 如果是Web服务容器，只检查Web服务
    if [ "$1" = "web" ]; then
        check_web_service
        exit $?
    fi

    # 否则执行完整检查
    local exit_code=0

    check_ros || exit_code=1
    check_database || exit_code=1
    check_logs || exit_code=1
    check_ros_nodes || exit_code=1

    if [ $exit_code -eq 0 ]; then
        log_info "所有检查通过"
    else
        log_error "部分检查失败"
    fi

    exit $exit_code
}

# 执行检查
main "$@"
