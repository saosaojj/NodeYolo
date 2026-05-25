import threading
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from rclpy.node import Node
from agv_web_server.ros_bridge import RosBridge
from agv_web_server.api_routes import create_api_router
from agv_web_server.websocket_handler import register_websocket


class WebServerNode(Node):

    def __init__(self):
        super().__init__('web_server_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('cors_origins', ['*'])
        self.declare_parameter('static_dir', '')
        self.declare_parameter('jwt_secret', 'agv_secret_key_change_in_production')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.cors_origins = self.get_parameter('cors_origins').get_parameter_value().string_array_value
        self.static_dir = self.get_parameter('static_dir').get_parameter_value().string_value
        self.jwt_secret = self.get_parameter('jwt_secret').get_parameter_value().string_value

        self.app = FastAPI(title='AGV Web Server', version='1.0.0')

        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=list(self.cors_origins),
            allow_credentials=True,
            allow_methods=['*'],
            allow_headers=['*'],
        )

        self.ros_bridge = RosBridge(self)

        api_router = create_api_router(self.ros_bridge)
        self.app.include_router(api_router)

        register_websocket(self.app, self.ros_bridge)

        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()

        self.get_logger().info(
            f'Web server started at http://{self.host}:{self.port}'
        )

    def _run_server(self):
        uvicorn.run(self.app, host=self.host, port=self.port, log_level='info')


def main(args=None):
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
