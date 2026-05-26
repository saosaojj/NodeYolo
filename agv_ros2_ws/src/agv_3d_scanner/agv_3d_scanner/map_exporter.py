# 地图导出模块，负责调用地图生成服务并导出3D扫描地图文件
import rclpy
from rclpy.node import Node
from agv_interfaces.srv import GenerateScanMap
import os
import time
import numpy as np


# 地图导出节点类，作为客户端调用地图生成服务，将点云数据导出为地图文件
class MapExporter(Node):

    def __init__(self):
        super().__init__('map_exporter')

        # 声明导出相关参数：默认导出路径和默认导出格式
        self.declare_parameter('default_export_path', '/tmp/agv_maps')
        self.declare_parameter('default_format', 'pcd')

        self.default_export_path = self.get_parameter('default_export_path').value
        self.default_format = self.get_parameter('default_format').value

        # 创建地图生成服务的客户端
        self.generate_map_client = self.create_client(GenerateScanMap, '/scanner/generate_map')

        # 等待地图生成服务可用
        while not self.generate_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for generate_map service...')

        self.get_logger().info('MapExporter initialized')

    # 导出地图方法，调用地图生成服务并等待结果
    def export_map(self, map_name, export_path=None, fmt=None, include_path=True):
        # 使用默认值填充未指定的参数
        if export_path is None:
            export_path = self.default_export_path
        if fmt is None:
            fmt = self.default_format

        # 构建服务请求
        request = GenerateScanMap.Request()
        request.map_name = map_name
        request.export_path = export_path
        request.format = fmt
        request.include_path = include_path

        # 异步调用服务并等待结果
        future = self.generate_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # 处理服务调用结果
        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(f'Successfully exported map: {result.output_file}')
                self.get_logger().info(f'Total points: {result.total_points}')
                self.get_logger().info(f'Processing time: {result.process_time:.2f}s')
            else:
                self.get_logger().error(f'Failed to export map: {result.message}')
        else:
            self.get_logger().error('Service call failed')

        return future.result()

    # 销毁节点时清理资源
    def destroy_node(self):
        super().destroy_node()


# 主函数，支持命令行参数指定地图名称、导出路径、格式和是否包含路径
def main(args=None):
    rclpy.init(args=args)
    node = MapExporter()

    import sys
    # 解析命令行参数：地图名称、导出路径、格式、是否包含路径
    if len(sys.argv) > 1:
        map_name = sys.argv[1]
        export_path = sys.argv[2] if len(sys.argv) > 2 else None
        fmt = sys.argv[3] if len(sys.argv) > 3 else None
        include_path = True
        if len(sys.argv) > 4:
            include_path = sys.argv[4].lower() == 'true'

        node.export_map(map_name, export_path, fmt, include_path)
    else:
        # 无命令行参数时，以服务模式运行等待请求
        node.get_logger().info('MapExporter running, waiting for requests...')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
