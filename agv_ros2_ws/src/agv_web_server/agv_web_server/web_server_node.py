#!/usr/bin/env python3
"""
AGV Web 服务节点

基于 FastAPI 的 Web 服务节点，提供 REST API 和 WebSocket 支持，
集成摄像头和 PLC 管理器，自动发现并挂载前端静态文件。
"""

import threading
import os
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from rclpy.node import Node

from agv_web_server.ros_bridge import RosBridge
from agv_web_server.api_routes import create_api_router, camera_mgr, plc_mgr, db_mgr
from agv_web_server.websocket_handler import register_websocket


def _find_frontend_dir():
    """
    自动查找前端静态文件目录

    按优先级搜索多个可能的位置：
    1. agv_web_frontend ROS 包的 share 目录
    2. 源代码目录中的 agv_web_frontend
    3. 相对于工作空间的路径

    Returns:
        str or None: 找到的前端目录路径，未找到返回 None
    """
    candidates = []

    # 方式1: 通过 ament_index 查找已安装的包
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('agv_web_frontend')
        candidates.append(share_dir)
    except Exception:
        pass

    # 方式2: 源代码目录（开发模式）
    # 从当前文件位置向上查找
    current_file = os.path.abspath(__file__)
    # web_server_node.py -> agv_web_server -> src -> agv_ros2_ws
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_file)))
    candidates.append(os.path.join(src_dir, 'agv_web_frontend'))

    # 方式3: install 目录
    install_dir = os.path.dirname(os.path.dirname(os.path.dirname(src_dir)))
    if install_dir.endswith('install'):
        candidates.append(os.path.join(install_dir, 'agv_web_frontend', 'share', 'agv_web_frontend'))

    # 方式4: 工作空间根目录下的常见路径
    ws_dir = os.path.dirname(src_dir)
    candidates.append(os.path.join(ws_dir, 'src', 'agv_web_frontend'))
    candidates.append(os.path.join(ws_dir, 'install', 'agv_web_frontend', 'share', 'agv_web_frontend'))

    # 方式5: Docker 容器中的路径
    candidates.append('/agv_ros2_ws/src/agv_web_frontend')
    candidates.append('/agv_ros2_ws/install/agv_web_frontend/share/agv_web_frontend')

    # 逐个检查候选路径
    for path in candidates:
        index_html = os.path.join(path, 'index.html')
        if os.path.exists(index_html):
            return os.path.abspath(path)

    return None


class WebServerNode(Node):
    """
    Web 服务节点类

    负责启动 FastAPI 服务器，初始化 ROS 桥接，
    管理摄像头和 PLC 管理器的生命周期。
    """

    def __init__(self):
        """
        初始化 Web 服务节点
        """
        super().__init__('web_server_node')

        # 声明参数
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('cors_origins', ['*'])
        self.declare_parameter('static_dir', '')
        self.declare_parameter('jwt_secret', 'agv_secret_key_change_in_production')

        # 获取参数值
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.cors_origins = self.get_parameter('cors_origins').get_parameter_value().string_array_value
        self.static_dir = self.get_parameter('static_dir').get_parameter_value().string_value
        self.jwt_secret = self.get_parameter('jwt_secret').get_parameter_value().string_value

        # 创建 FastAPI 应用
        self.app = FastAPI(title='AGV Web Server', version='1.0.0')

        # 配置 CORS 中间件
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=list(self.cors_origins),
            allow_credentials=True,
            allow_methods=['*'],
            allow_headers=['*'],
        )

        # 初始化 ROS 桥接
        self.ros_bridge = RosBridge(self)

        # 创建并注册 API 路由
        api_router = create_api_router(self.ros_bridge)
        self.app.include_router(api_router)

        # 注册 WebSocket 处理
        register_websocket(self.app, self.ros_bridge)

        # 查找前端静态文件目录
        frontend_dir = self._resolve_frontend_dir()

        # 挂载静态文件和前端页面
        if frontend_dir and os.path.isdir(frontend_dir):
            self._mount_frontend(frontend_dir)
        else:
            self.get_logger().warn(
                '前端静态文件目录未找到！Web 界面将不可用。'
                '请确保 agv_web_frontend 包已正确构建和安装。'
            )
            # 提供一个基本的 API 状态页面作为后备
            @self.app.get("/")
            async def api_only_index():
                return {
                    "service": "AGV Web Server",
                    "version": "1.0.0",
                    "status": "running",
                    "note": "前端文件未找到，仅 API 模式运行。请构建 agv_web_frontend 包。",
                    "api_docs": f"http://{self.host}:{self.port}/docs"
                }

        # 启动摄像头管理器
        try:
            camera_mgr.start()
            self.get_logger().info('Camera manager started')
        except Exception as e:
            self.get_logger().error(f'Failed to start camera manager: {e}')

        # 启动 PLC 管理器
        try:
            plc_mgr.start()
            self.get_logger().info('PLC manager started')
        except Exception as e:
            self.get_logger().error(f'Failed to start PLC manager: {e}')

        # 从数据库加载仿真状态
        try:
            sim_status = db_mgr.get_simulation_state()
            if sim_status.get('camera_simulation'):
                camera_mgr.set_simulation(True)
                self.get_logger().info('Camera simulation enabled')
            if sim_status.get('plc_simulation'):
                plc_mgr.set_simulation(True)
                self.get_logger().info('PLC simulation enabled')
        except Exception as e:
            self.get_logger().error(f'Failed to load simulation status: {e}')

        # 在后台线程中启动 Uvicorn 服务器
        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()

        self.get_logger().info(
            f'Web server started at http://{self.host}:{self.port}'
        )

    def _resolve_frontend_dir(self):
        """
        解析前端静态文件目录路径

        优先使用 static_dir 参数，如果为空则自动搜索。

        Returns:
            str or None: 前端目录路径
        """
        # 优先使用参数指定的路径
        if self.static_dir and os.path.isdir(self.static_dir):
            index_html = os.path.join(self.static_dir, 'index.html')
            if os.path.exists(index_html):
                self.get_logger().info(
                    f'使用参数指定的前端目录: {self.static_dir}'
                )
                return self.static_dir
            else:
                self.get_logger().warn(
                    f'参数 static_dir={self.static_dir} 目录存在但缺少 index.html'
                )

        # 自动搜索前端目录
        frontend_dir = _find_frontend_dir()
        if frontend_dir:
            self.get_logger().info(
                f'自动发现前端目录: {frontend_dir}'
            )
            return frontend_dir

        return None

    def _mount_frontend(self, frontend_dir):
        """
        挂载前端静态文件到 FastAPI

        Args:
            frontend_dir: 前端静态文件目录路径
        """
        # 挂载静态文件到 /static（CSS、JS 等资源）
        self.app.mount(
            "/static",
            StaticFiles(directory=frontend_dir),
            name="static"
        )
        self.get_logger().info(f'Static files served from {frontend_dir} at /static')

        # 根路径返回 index.html
        @self.app.get("/")
        async def get_index():
            index_path = os.path.join(frontend_dir, "index.html")
            if os.path.exists(index_path):
                return FileResponse(index_path)
            return {"message": "index.html not found"}

        # 处理前端路由的 fallback（SPA 支持）
        @self.app.get("/{page_name}")
        async def get_page(page_name: str):
            """前端页面路由 fallback"""
            # 如果请求的是静态资源文件（有扩展名），返回 404
            if '.' in page_name:
                return {"detail": "Not found"}
            # 否则返回 index.html（SPA 模式）
            index_path = os.path.join(frontend_dir, "index.html")
            if os.path.exists(index_path):
                return FileResponse(index_path)
            return {"detail": "Not found"}

        self.get_logger().info(
            f'前端页面已挂载，访问 http://{self.host}:{self.port} 查看'
        )

    def destroy_node(self):
        """
        销毁节点，清理资源

        停止摄像头和 PLC 管理器，关闭 Web 服务器，
        然后调用基类的 destroy_node。
        """
        # 停止摄像头管理器
        try:
            camera_mgr.stop()
            self.get_logger().info('Camera manager stopped')
        except Exception as e:
            self.get_logger().error(f'Failed to stop camera manager: {e}')

        # 停止 PLC 管理器
        try:
            plc_mgr.stop()
            self.get_logger().info('PLC manager stopped')
        except Exception as e:
            self.get_logger().error(f'Failed to stop PLC manager: {e}')

        # 调用基类销毁方法
        super().destroy_node()

    def _run_server(self):
        """
        运行 Uvicorn Web 服务器

        在后台线程中执行，不阻塞 ROS 节点的 spin。
        """
        uvicorn.run(self.app, host=self.host, port=self.port, log_level='info')


def main(args=None):
    """
    主函数

    初始化 ROS，创建并运行 Web 服务节点。
    """
    import rclpy
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
