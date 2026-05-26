import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, Image
from agv_interfaces.msg import Scan3DData, PointCloudChunk
from agv_interfaces.srv import StartScan
import numpy as np
import struct


class ScannerNode(Node):

    def __init__(self):
        super().__init__('scanner_node')

        self.declare_parameter('scan_resolution', 0.05)
        self.declare_parameter('max_points_per_scan', 10000)
        self.declare_parameter('chunk_size', 4096)
        self.declare_parameter('frame_id', 'scanner_link')

        self.scan_resolution = self.get_parameter('scan_resolution').value
        self.max_points_per_scan = self.get_parameter('max_points_per_scan').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.frame_id = self.get_parameter('frame_id').value

        self.scan_data_pub = self.create_publisher(Scan3DData, '/scanner/scan_data', 10)
        self.chunk_pub = self.create_publisher(PointCloudChunk, '/scanner/point_cloud_chunks', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/scanner/pointcloud', qos_profile_sensor_data)

        self.start_scan_srv = self.create_service(StartScan, '/scanner/start_scan', self.start_scan_callback)

        self.is_scanning = False
        self.scan_pattern = 'rectangular'
        self.current_points = []

        self.get_logger().info('ScannerNode initialized')

    def start_scan_callback(self, request, response):
        self.scan_pattern = request.scan_pattern
        self.scan_resolution = request.scan_resolution
        self.max_points_per_scan = request.max_points

        self.is_scanning = True
        self.current_points = []

        response.success = True
        response.message = f'Starting scan with pattern: {self.scan_pattern}'

        self.get_logger().info(response.message)

        self._generate_scan_data()

        return response

    def _generate_scan_data(self):
        num_points = min(self.max_points_per_scan, 5000)
        points_x = []
        points_y = []
        points_z = []
        intensities = []

        if self.scan_pattern == 'rectangular':
            for i in range(num_points):
                x = (i % 100) * self.scan_resolution - 2.5
                y = (i // 100) * self.scan_resolution - 2.5
                z = np.random.normal(0, 0.1)
                intensity = 0.5 + np.random.random() * 0.5
                points_x.append(x)
                points_y.append(y)
                points_z.append(z)
                intensities.append(intensity)
        elif self.scan_pattern == 'circular':
            for i in range(num_points):
                angle = i * 0.1
                radius = (i % 50) * self.scan_resolution
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                z = np.random.normal(0, 0.1)
                intensity = 0.5 + np.random.random() * 0.5
                points_x.append(x)
                points_y.append(y)
                points_z.append(z)
                intensities.append(intensity)
        else:
            for i in range(num_points):
                x = np.random.uniform(-5, 5)
                y = np.random.uniform(-5, 5)
                z = np.random.normal(0, 0.1)
                intensity = np.random.random()
                points_x.append(x)
                points_y.append(y)
                points_z.append(z)
                intensities.append(intensity)

        self._publish_scan_data(points_x, points_y, points_z, intensities)
        self._publish_pointcloud(points_x, points_y, points_z)
        self._publish_chunks(points_x, points_y, points_z, intensities)

        self.is_scanning = False
        self.get_logger().info(f'Scan completed, published {num_points} points')

    def _publish_scan_data(self, points_x, points_y, points_z, intensities):
        msg = Scan3DData()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.scanner_pose = PoseStamped()
        msg.scanner_pose.header = msg.header
        msg.scanner_pose.pose.position.x = 0.0
        msg.scanner_pose.pose.position.y = 0.0
        msg.scanner_pose.pose.position.z = 1.0
        msg.scanner_pose.pose.orientation.w = 1.0

        msg.points_x = points_x
        msg.points_y = points_y
        msg.points_z = points_z
        msg.intensities = intensities
        msg.num_points = len(points_x)

        self.scan_data_pub.publish(msg)

    def _publish_pointcloud(self, points_x, points_y, points_z):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = len(points_x)
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        buffer = []
        for x, y, z in zip(points_x, points_y, points_z):
            buffer.extend(struct.pack('ffff', float(x), float(y), float(z), 1.0))

        msg.data = bytes(buffer)
        self.pointcloud_pub.publish(msg)

    def _publish_chunks(self, points_x, points_y, points_z, intensities):
        num_points = len(points_x)
        total_chunks = (num_points + self.chunk_size - 1) // self.chunk_size

        for i in range(total_chunks):
            start_idx = i * self.chunk_size
            end_idx = min(start_idx + self.chunk_size, num_points)

            data = []
            for j in range(start_idx, end_idx):
                data.extend([float(points_x[j]), float(points_y[j]), float(points_z[j]), float(intensities[j])])

            chunk_msg = PointCloudChunk()
            chunk_msg.header = Header()
            chunk_msg.header.stamp = self.get_clock().now().to_msg()
            chunk_msg.header.frame_id = self.frame_id
            chunk_msg.robot_pose = PoseStamped()
            chunk_msg.robot_pose.header = chunk_msg.header
            chunk_msg.robot_pose.pose.orientation.w = 1.0
            chunk_msg.chunk_index = i
            chunk_msg.total_chunks = total_chunks
            chunk_msg.data = data

            self.chunk_pub.publish(chunk_msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    from sensor_msgs.msg import PointField
    main()
