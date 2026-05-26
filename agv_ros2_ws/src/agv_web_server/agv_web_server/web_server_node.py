#!/usr/bin/env python3
"""
AGV Web 服务节点

基于 FastAPI 的 Web 服务节点，提供 REST API 和 WebSocket 支持，
集成摄像头和 PLC 管理器。
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

        # 挂载静态文件和前端页面
        if self.static_dir and os.path.isdir(self.static_dir):
            # 挂载静态文件到 /static
            self.app.mount(
                "/static",
                StaticFiles(directory=self.static_dir),
                name="static"
            )
            self.get_logger().info(f'Static files served from {self.static_dir} at /static')

            # 添加根路径路由，返回 index.html
            @self.app.get("/")
            async def get_index():
                index_path = os.path.join(self.static_dir, "index.html")
                if os.path.exists(index_path):
                    return FileResponse(index_path)
                return {"message": "index.html not found in static_dir"}
        else:
            self.get_logger().info(
                'No static_dir specified or directory does not exist. '
                'Frontend will not be served by this server.'
            )

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
