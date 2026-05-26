#!/bin/bash
# =====================================================
# AGV Docker 部署脚本
# =====================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

# 检查Docker是否安装
check_docker() {
    log_info "检查Docker环境..."
    if ! command -v docker &> /dev/null; then
        log_error "Docker未安装"
        exit 1
    fi
    if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
        log_error "Docker Compose未安装"
        exit 1
    fi
    log_info "Docker环境检查完成"
}

# 创建必要的目录
create_directories() {
    log_info "创建部署目录..."
    mkdir -p volumes/config
    mkdir -p volumes/data
    mkdir -p volumes/logs
    mkdir -p volumes/models
    log_info "目录创建完成"
}

# 构建镜像
build_images() {
    log_info "构建Docker镜像..."
    if command -v docker-compose &> /dev/null; then
        docker-compose build --no-cache
    else
        docker compose build --no-cache
    fi
    log_info "镜像构建完成"
}

# 部署生产环境
deploy_production() {
    log_info "部署生产环境..."
    create_directories

    if command -v docker-compose &> /dev/null; then
        docker-compose -f docker-compose.prod.yml up -d
    else
        docker compose -f docker-compose.prod.yml up -d
    fi

    log_info "生产环境部署完成"
    log_info "Web服务地址: http://localhost:8080"
}

# 部署开发环境
deploy_development() {
    log_info "部署开发环境..."
    create_directories

    if command -v docker-compose &> /dev/null; then
        docker-compose -f docker-compose.dev.yml up -d
    else
        docker compose -f docker-compose.dev.yml up -d
    fi

    log_info "开发环境部署完成"
    log_info "使用命令进入容器: docker exec -it agv_dev bash"
}

# 部署测试环境
deploy_testing() {
    log_info "部署测试环境..."
    create_directories

    if command -v docker-compose &> /dev/null; then
        docker-compose -f docker-compose.test.yml up -d
    else
        docker compose -f docker-compose.test.yml up -d
    fi

    log_info "测试环境部署完成"
    log_info "Web服务地址: http://localhost:8082"
}

# 停止服务
stop_services() {
    log_info "停止所有服务..."

    if command -v docker-compose &> /dev/null; then
        docker-compose down
    else
        docker compose down
    fi

    log_info "服务已停止"
}

# 查看日志
view_logs() {
    local service=${1:-agv_core}

    log_info "查看 $service 日志..."
    docker logs -f $service
}

# 查看状态
view_status() {
    log_info "查看服务状态..."

    if command -v docker-compose &> /dev/null; then
        docker-compose ps
    else
        docker compose ps
    fi
}

# 健康检查
health_check() {
    log_info "执行健康检查..."

    local services=("agv_core" "agv_web" "agv_vision")
    local all_healthy=true

    for service in "${services[@]}"; do
        if docker ps --filter "name=$service" --filter "status=running" | grep -q "$service"; then
            log_info "✓ $service 运行正常"
        else
            log_warn "✗ $service 未运行"
            all_healthy=false
        fi
    done

    if $all_healthy; then
        log_info "所有服务健康"
        return 0
    else
        log_error "部分服务不健康"
        return 1
    fi
}

# 显示帮助
show_help() {
    echo ""
    echo "AGV Docker 部署脚本"
    echo "=================="
    echo ""
    echo "用法: $0 [命令]"
    echo ""
    echo "可用命令:"
    echo "  build              构建Docker镜像"
    echo "  deploy-prod        部署生产环境"
    echo "  deploy-dev         部署开发环境"
    echo "  deploy-test        部署测试环境"
    echo "  stop               停止所有服务"
    echo "  logs [服务名]      查看日志（默认: agv_core）"
    echo "  status             查看服务状态"
    echo "  health             执行健康检查"
    echo "  help               显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 build"
    echo "  $0 deploy-prod"
    echo "  $0 logs agv_web"
    echo ""
}

# 主程序
main() {
    local command=${1:-help}

    case $command in
        build)
            check_docker
            build_images
            ;;
        deploy-prod)
            check_docker
            deploy_production
            ;;
        deploy-dev)
            check_docker
            deploy_development
            ;;
        deploy-test)
            check_docker
            deploy_testing
            ;;
        stop)
            check_docker
            stop_services
            ;;
        logs)
            view_logs ${2:-agv_core}
            ;;
        status)
            check_docker
            view_status
            ;;
        health)
            check_docker
            health_check
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

main "$@"
