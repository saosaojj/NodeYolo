# 点云管理模块，负责接收、存储和导出3D扫描点云数据
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from agv_interfaces.msg import Scan3DData, PointCloudChunk
from agv_interfaces.srv import GenerateScanMap
import numpy as np
from collections import deque


# 点云管理节点类，接收扫描数据、重组分块数据，并提供地图生成服务
class PointCloudManager(Node):

    def __init__(self):
        super().__init__('point_cloud_manager')

        # 声明体素滤波大小和最大内存点数参数
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_memory_points', 1000000)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_memory_points = self.get_parameter('max_memory_points').value

        # 点云存储：完整点云列表、扫描路径点列表、分块缓冲区
        self.point_cloud = []
        self.path_points = []
        self.chunk_buffer = {}

        # 订阅扫描数据和点云分块话题
        self.scan_sub = self.create_subscription(Scan3DData, '/scanner/scan_data', self.scan_data_callback, 10)
        self.chunk_sub = self.create_subscription(PointCloudChunk, '/scanner/point_cloud_chunks', self.chunk_callback, 10)

        # 创建地图生成服务
        self.generate_map_srv = self.create_service(GenerateScanMap, '/scanner/generate_map', self.generate_map_callback)

        self.get_logger().info('PointCloudManager initialized')

    # 扫描数据回调，接收Scan3DData消息并存储点云和扫描路径
    def scan_data_callback(self, msg):
        # 将接收到的点云数据添加到内存中
        for i in range(msg.num_points):
            point = (msg.points_x[i], msg.points_y[i], msg.points_z[i], msg.intensities[i])
            self.point_cloud.append(point)

        # 记录扫描器位姿路径点
        self.path_points.append((msg.scanner_pose.pose.position.x,
                                 msg.scanner_pose.pose.position.y,
                                 msg.scanner_pose.pose.position.z))

        # 超过最大内存点数时，保留最新的点云数据
        if len(self.point_cloud) > self.max_memory_points:
            self.point_cloud = self.point_cloud[-self.max_memory_points:]

        self.get_logger().info(f'Received {msg.num_points} points, total: {len(self.point_cloud)}')

    # 点云分块回调，接收并重组分块数据，所有分块收齐后合并为完整点云
    def chunk_callback(self, msg):
        # 以时间戳为键缓存分块数据
        key = msg.header.stamp.sec
        if key not in self.chunk_buffer:
            self.chunk_buffer[key] = [None] * msg.total_chunks

        self.chunk_buffer[key][msg.chunk_index] = msg.data

        # 检查是否已收齐所有分块
        if all(chunk is not None for chunk in self.chunk_buffer[key]):
            # 合并所有分块数据
            all_data = []
            for chunk in self.chunk_buffer[key]:
                all_data.extend(chunk)

            # 每4个值为一组（x, y, z, intensity），解析并添加到点云中
            for i in range(0, len(all_data), 4):
                x = all_data[i]
                y = all_data[i+1]
                z = all_data[i+2]
                intensity = all_data[i+3]
                self.point_cloud.append((x, y, z, intensity))

            # 清理已处理的分块缓冲区
            del self.chunk_buffer[key]

    # 地图生成服务回调，将点云数据导出为指定格式的地图文件
    def generate_map_callback(self, request, response):
        start_time = self.get_clock().now()

        try:
            # 检查是否有可用的点云数据
            if len(self.point_cloud) == 0:
                response.success = False
                response.message = 'No point cloud data available'
                return response

            # 导出点云到文件
            output_file = self._export_point_cloud(request.export_path, request.map_name, request.format)

            # 计算处理耗时
            end_time = self.get_clock().now()
            process_time = (end_time - start_time).nanoseconds / 1e9

            response.success = True
            response.message = 'Map generated successfully'
            response.output_file = output_file
            response.total_points = len(self.point_cloud)
            response.process_time = process_time

            self.get_logger().info(f'Generated map: {output_file}, {len(self.point_cloud)} points in {process_time:.2f}s')

        except Exception as e:
            response.success = False
            response.message = f'Error generating map: {str(e)}'

        return response

    # 导出点云到文件，支持xyz、pcd、ply格式
    def _export_point_cloud(self, export_path, map_name, fmt):
        import os
        # 创建导出目录（如不存在）
        if not os.path.exists(export_path):
            os.makedirs(export_path)

        filename = f'{map_name}.{fmt}'
        filepath = os.path.join(export_path, filename)

        points_array = np.array(self.point_cloud)

        # 根据格式选择不同的导出方式
        if fmt == 'xyz':
            np.savetxt(filepath, points_array[:, :3], fmt='%.6f')
        elif fmt == 'pcd':
            self._save_pcd(filepath, points_array)
        elif fmt == 'ply':
            self._save_ply(filepath, points_array)
        else:
            # 不支持的格式默认以xyz格式保存
            np.savetxt(filepath, points_array[:, :3], fmt='%.6f')

        return filepath

    # 以PCD格式保存点云数据
    def _save_pcd(self, filepath, points):
        with open(filepath, 'w') as f:
            # 写入PCD文件头信息
            f.write('# .PCD v.7 - Point Cloud Data file format\n')
            f.write('VERSION .7\n')
            f.write('FIELDS x y z intensity\n')
            f.write('SIZE 4 4 4 4\n')
            f.write('TYPE F F F F\n')
            f.write('COUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            # 逐点写入坐标和强度值
            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n')

    # 以PLY格式保存点云数据
    def _save_ply(self, filepath, points):
        with open(filepath, 'w') as f:
            # 写入PLY文件头信息
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('property float intensity\n')
            f.write('end_header\n')
            # 逐点写入坐标和强度值
            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n')

    # 销毁节点时清理资源
    def destroy_node(self):
        super().destroy_node()


# 主函数，初始化ROS2并运行点云管理节点
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
