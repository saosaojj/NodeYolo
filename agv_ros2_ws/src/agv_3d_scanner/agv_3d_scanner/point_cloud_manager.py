import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from agv_interfaces.msg import Scan3DData, PointCloudChunk
from agv_interfaces.srv import GenerateScanMap
import numpy as np
from collections import deque


class PointCloudManager(Node):

    def __init__(self):
        super().__init__('point_cloud_manager')

        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_memory_points', 1000000)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_memory_points = self.get_parameter('max_memory_points').value

        self.point_cloud = []
        self.path_points = []
        self.chunk_buffer = {}

        self.scan_sub = self.create_subscription(Scan3DData, '/scanner/scan_data', self.scan_data_callback, 10)
        self.chunk_sub = self.create_subscription(PointCloudChunk, '/scanner/point_cloud_chunks', self.chunk_callback, 10)

        self.generate_map_srv = self.create_service(GenerateScanMap, '/scanner/generate_map', self.generate_map_callback)

        self.get_logger().info('PointCloudManager initialized')

    def scan_data_callback(self, msg):
        for i in range(msg.num_points):
            point = (msg.points_x[i], msg.points_y[i], msg.points_z[i], msg.intensities[i])
            self.point_cloud.append(point)

        self.path_points.append((msg.scanner_pose.pose.position.x,
                                 msg.scanner_pose.pose.position.y,
                                 msg.scanner_pose.pose.position.z))

        if len(self.point_cloud) > self.max_memory_points:
            self.point_cloud = self.point_cloud[-self.max_memory_points:]

        self.get_logger().info(f'Received {msg.num_points} points, total: {len(self.point_cloud)}')

    def chunk_callback(self, msg):
        key = msg.header.stamp.sec
        if key not in self.chunk_buffer:
            self.chunk_buffer[key] = [None] * msg.total_chunks

        self.chunk_buffer[key][msg.chunk_index] = msg.data

        if all(chunk is not None for chunk in self.chunk_buffer[key]):
            all_data = []
            for chunk in self.chunk_buffer[key]:
                all_data.extend(chunk)

            for i in range(0, len(all_data), 4):
                x = all_data[i]
                y = all_data[i+1]
                z = all_data[i+2]
                intensity = all_data[i+3]
                self.point_cloud.append((x, y, z, intensity))

            del self.chunk_buffer[key]

    def generate_map_callback(self, request, response):
        start_time = self.get_clock().now()

        try:
            if len(self.point_cloud) == 0:
                response.success = False
                response.message = 'No point cloud data available'
                return response

            output_file = self._export_point_cloud(request.export_path, request.map_name, request.format)

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

    def _export_point_cloud(self, export_path, map_name, fmt):
        import os
        if not os.path.exists(export_path):
            os.makedirs(export_path)

        filename = f'{map_name}.{fmt}'
        filepath = os.path.join(export_path, filename)

        points_array = np.array(self.point_cloud)

        if fmt == 'xyz':
            np.savetxt(filepath, points_array[:, :3], fmt='%.6f')
        elif fmt == 'pcd':
            self._save_pcd(filepath, points_array)
        elif fmt == 'ply':
            self._save_ply(filepath, points_array)
        else:
            np.savetxt(filepath, points_array[:, :3], fmt='%.6f')

        return filepath

    def _save_pcd(self, filepath, points):
        with open(filepath, 'w') as f:
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
            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n')

    def _save_ply(self, filepath, points):
        with open(filepath, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('property float intensity\n')
            f.write('end_header\n')
            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n')

    def destroy_node(self):
        super().destroy_node()


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
