#!/bin/bash
# =====================================================
# AGV Docker 监控脚本
# =====================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 监控配置
CHECK_INTERVAL=5
LOG_FILE="/tmp/agv_monitor.log"

# 日志函数
log() {
    echo -e "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# 检查服务健康状态
check_service_health() {
    local service=$1
    if docker ps --filter "name=$service" --filter "status=running" | grep -q "$service"; then
        return 0
    else
        return 1
    fi
}

# 获取服务CPU使用率
get_cpu_usage() {
    local service=$1
    docker stats --no-stream --format "{{.CPUPerc}}" $service 2>/dev/null | sed 's/%//' || echo "0"
}

# 获取服务内存使用
get_memory_usage() {
    local service=$1
    docker stats --no-stream --format "{{.MemUsage}}" $service 2>/dev/null || echo "N/A"
}

# 获取容器运行时间
get_uptime() {
    local service=$1
    docker inspect --format='{{.State.StartedAt}}' $service 2>/dev/null
}

# 格式化运行时间
format_uptime() {
    local started_at=$1
    local start_time=$(date -d "$started_at" +%s 2>/dev/null || echo 0)
    local now=$(date +%s)
    local uptime=$((now - start_time))

    local days=$((uptime / 86400))
    local hours=$(( (uptime % 86400) / 3600 ))
    local minutes=$(( (uptime % 3600) / 60 ))

    if [ $days -gt 0 ]; then
        echo "${days}d ${hours}h ${minutes}m"
    elif [ $hours -gt 0 ]; then
        echo "${hours}h ${minutes}m"
    else
        echo "${minutes}m"
    fi
}

# 检查Web服务响应时间
check_web_response() {
    local start_time=$(date +%s%N)
    if curl -f -s http://localhost:8080/api/v1/system/health > /dev/null 2>&1; then
        local end_time=$(date +%s%N)
        local response_time=$(( (end_time - start_time) / 1000000 ))
        echo "$response_time ms"
    else
        echo "无法连接"
    fi
}

# 检查磁盘空间
check_disk_space() {
    df -h / | awk 'NR==2 {print $5 " used (" $4 " available)"}'
}

# 检查内存使用
check_memory() {
    free -h | awk 'NR==2 {print "Used: " $3 " / Total: " $2}'
}

# 检查ROS节点
check_ros_nodes() {
    docker exec agv_core bash -c "source /opt/ros/humble/setup.bash && source /agv_ros2_ws/install/setup.bash 2>/dev/null && ros2 node list" 2>/dev/null | wc -l
}

# 生成监控报告
generate_report() {
    clear
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}           AGV Docker 监控系统${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${BLUE}时间: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${BLUE}系统信息:${NC}"
    echo -e "  • 磁盘空间: $(check_disk_space)"
    echo -e "  • 内存使用: $(check_memory)"
    echo ""
    echo -e "${BLUE}服务状态:${NC}"

    local services=("agv_core" "agv_vision" "agv_web" "agv_web_light" "ag_dev")
    local all_healthy=true

    for service in "${services[@]}"; do
        if docker ps -a --filter "name=$service" --format "{{.Names}}" | grep -q "^${service}$"; then
            if check_service_health $service; then
                local cpu=$(get_cpu_usage $service)
                local mem=$(get_memory_usage $service)
                local started=$(get_uptime $service)
                local uptime=$(format_uptime "$started")

                echo -e "  ${GREEN}✓${NC} $service"
                echo -e "      CPU: ${cpu}% | 内存: $mem | 运行时间: $uptime"
            else
                echo -e "  ${RED}✗${NC} $service (已停止)"
                all_healthy=false
            fi
        fi
    done

    echo ""
    echo -e "${BLUE}Web服务:${NC}"
    echo -e "  • 响应时间: $(check_web_response)"
    echo ""

    echo -e "${BLUE}ROS2节点数量:${NC}"
    local node_count=$(check_ros_nodes)
    echo -e "  • 运行中的节点: $node_count"
    echo ""

    if $all_healthy; then
        echo -e "${GREEN}✓ 所有服务运行正常${NC}"
    else
        echo -e "${RED}✗ 部分服务异常${NC}"
    fi

    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo ""
}

# 持续监控模式
continuous_monitor() {
    log "${BLUE}开始持续监控模式${NC}"
    log "按 Ctrl+C 停止监控"

    while true; do
        generate_report
        sleep $CHECK_INTERVAL
    done
}

# 单次检查
single_check() {
    generate_report
}

# 显示帮助
show_help() {
    echo ""
    echo "AGV Docker 监控脚本"
    echo "==================="
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "可用选项:"
    echo "  -m, --monitor      持续监控模式"
    echo "  -s, --single      单次检查"
    echo "  -i, --interval    设置检查间隔（秒，默认: 5）"
    echo "  -h, --help        显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 --single"
    echo "  $0 --monitor"
    echo "  $0 --monitor --interval 10"
    echo ""
}

# 主程序
main() {
    local mode="single"
    CHECK_INTERVAL=5

    # 解析参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -m|--monitor)
                mode="continuous"
                shift
                ;;
            -s|--single)
                mode="single"
                shift
                ;;
            -i|--interval)
                CHECK_INTERVAL="$2"
                shift 2
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                echo "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # 执行监控
    case $mode in
        continuous)
            continuous_monitor
            ;;
        single)
            single_check
            ;;
    esac
}

main "$@"
