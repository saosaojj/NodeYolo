import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from agv_interfaces.msg import Scan3DData, PointCloudChunk
from agv_interfaces.srv import StartScan
from std_srvs.srv import Trigger
import numpy as np
import struct
import os
import time
from datetime import datetime


class ScannerNode(Node):

    def __init__(self):
        super().__init__('scanner_node')

        self.declare_parameter('scan_resolution', 0.05)
        self.declare_parameter('max_points_per_scan', 10000)
        self.declare_parameter('chunk_size', 4096)
        self.declare_parameter('frame_id', 'scanner_link')

        # 增强功能8: 扫描区域定义 - 从参数定义扫描边界
        self.declare_parameter('scan_bounds.x_min', -5.0)
        self.declare_parameter('scan_bounds.x_max', 5.0)
        self.declare_parameter('scan_bounds.y_min', -5.0)
        self.declare_parameter('scan_bounds.y_max', 5.0)
        self.declare_parameter('scan_bounds.z_min', -1.0)
        self.declare_parameter('scan_bounds.z_max', 3.0)

        # 增强功能2: 点云预处理参数
        self.declare_parameter('preprocessing.voxel_leaf_size', 0.1)
        self.declare_parameter('preprocessing.outlier_mean_k', 50)
        self.declare_parameter('preprocessing.outlier_std_threshold', 1.0)
        self.declare_parameter('preprocessing.enable_voxel_downsample', True)
        self.declare_parameter('preprocessing.enable_outlier_removal', True)

        # 增强功能3: 扫描质量评估参数
        self.declare_parameter('quality.expected_point_density', 100.0)

        # 增强功能7: PLY导出路径
        self.declare_parameter('export_path', '/tmp/agv_scans')

        self.scan_resolution = self.get_parameter('scan_resolution').value
        self.max_points_per_scan = self.get_parameter('max_points_per_scan').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.frame_id = self.get_parameter('frame_id').value

        # 增强功能8: 读取扫描边界参数
        self._scan_bounds = {
            'x_min': self.get_parameter('scan_bounds.x_min').value,
            'x_max': self.get_parameter('scan_bounds.x_max').value,
            'y_min': self.get_parameter('scan_bounds.y_min').value,
            'y_max': self.get_parameter('scan_bounds.y_max').value,
            'z_min': self.get_parameter('scan_bounds.z_min').value,
            'z_max': self.get_parameter('scan_bounds.z_max').value,
        }

        # 增强功能2: 预处理参数
        self._voxel_leaf_size = self.get_parameter('preprocessing.voxel_leaf_size').value
        self._outlier_mean_k = self.get_parameter('preprocessing.outlier_mean_k').value
        self._outlier_std_threshold = self.get_parameter('preprocessing.outlier_std_threshold').value
        self._enable_voxel = self.get_parameter('preprocessing.enable_voxel_downsample').value
        self._enable_outlier = self.get_parameter('preprocessing.enable_outlier_removal').value

        # 增强功能3: 质量评估参数
        self._expected_point_density = self.get_parameter('quality.expected_point_density').value

        # 增强功能7: 导出路径
        self._export_path = self.get_parameter('export_path').value

        self.scan_data_pub = self.create_publisher(Scan3DData, '/scanner/scan_data', 10)
        self.chunk_pub = self.create_publisher(PointCloudChunk, '/scanner/point_cloud_chunks', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/scanner/pointcloud', qos_profile_sensor_data)

        # 增强功能3: 扫描质量评估发布
        self._scan_quality_pub = self.create_publisher(String, '/scanner/scan_quality', 10)

        # 增强功能6: 扫描进度发布
        self._scan_progress_pub = self.create_publisher(String, '/scanner/scan_progress', 10)

        # 增强功能5: 点云分割结果发布
        self._segmentation_pub = self.create_publisher(String, '/scanner/segmentation', 10)

        # 增强功能4: 扫描配准结果发布
        self._registration_pub = self.create_publisher(String, '/scanner/registration', 10)

        # 增强功能2: 预处理结果发布
        self._preprocessing_pub = self.create_publisher(String, '/scanner/preprocessing', 10)

        self.start_scan_srv = self.create_service(StartScan, '/scanner/start_scan', self.start_scan_callback)

        # 增强功能7: 导出服务
        self._export_srv = self.create_service(Trigger, '/scanner/export_ply', self.export_ply_callback)

        self.is_scanning = False
        self.scan_pattern = 'rectangular'
        self.current_points = []

        # 增强功能4: 扫描配准 - 保存前一次扫描用于配准
        self._previous_scan = None
        self._scan_count = 0

        # 增强功能6: 扫描进度追踪
        self._scan_start_time = 0.0
        self._scan_total_points = 0
        self._scan_generated_points = 0

        # 增强功能7: 最近一次扫描数据缓存（用于导出）
        self._last_scan_data = None

        self.get_logger().info('ScannerNode initialized')

    def start_scan_callback(self, request, response):
        self.scan_pattern = request.scan_pattern
        self.scan_resolution = request.scan_resolution
        self.max_points_per_scan = request.max_points

        self.is_scanning = True
        self.current_points = []
        self._scan_start_time = time.time()
        self._scan_total_points = min(self.max_points_per_scan, 5000)
        self._scan_generated_points = 0

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

        # 增强功能1: 多种扫描模式
        if self.scan_pattern == 'rectangular':
            points_x, points_y, points_z, intensities = self._generate_rectangular(num_points)
        elif self.scan_pattern == 'circular':
            points_x, points_y, points_z, intensities = self._generate_circular(num_points)
        elif self.scan_pattern == 'helical':
            points_x, points_y, points_z, intensities = self._generate_helical(num_points)
        elif self.scan_pattern == 'linear_sweep':
            points_x, points_y, points_z, intensities = self._generate_linear_sweep(num_points)
        else:
            points_x, points_y, points_z, intensities = self._generate_rectangular(num_points)

        # 增强功能8: 应用扫描边界裁剪
        points_x, points_y, points_z, intensities = self._apply_scan_bounds(
            points_x, points_y, points_z, intensities)

        # 增强功能6: 更新进度
        self._scan_generated_points = len(points_x)
        self._publish_scan_progress(1.0)

        # 增强功能2: 点云预处理
        points_x, points_y, points_z, intensities = self._preprocess_point_cloud(
            points_x, points_y, points_z, intensities)

        # 增强功能3: 扫描质量评估
        self._assess_scan_quality(points_x, points_y, points_z)

        # 增强功能4: 扫描配准
        self._register_scan(points_x, points_y, points_z)

        # 增强功能5: 点云分割
        self._segment_point_cloud(points_x, points_y, points_z)

        # 缓存最近扫描数据用于导出
        self._last_scan_data = {
            'points_x': points_x,
            'points_y': points_y,
            'points_z': points_z,
            'intensities': intensities,
        }

        self._publish_scan_data(points_x, points_y, points_z, intensities)
        self._publish_pointcloud(points_x, points_y, points_z)
        self._publish_chunks(points_x, points_y, points_z, intensities)

        self.is_scanning = False
        self._scan_count += 1
        self.get_logger().info(f'Scan completed, published {len(points_x)} points')

    # 增强功能1: 矩形扫描模式
    def _generate_rectangular(self, num_points):
        points_x, points_y, points_z, intensities = [], [], [], []
        side = int(np.sqrt(num_points))
        x_range = np.linspace(self._scan_bounds['x_min'], self._scan_bounds['x_max'], side)
        y_range = np.linspace(self._scan_bounds['y_min'], self._scan_bounds['y_max'], side)
        for i, x in enumerate(x_range):
            for j, y in enumerate(y_range):
                z = np.random.normal(0, 0.05)
                intensity = 0.5 + np.random.random() * 0.5
                points_x.append(float(x))
                points_y.append(float(y))
                points_z.append(float(z))
                intensities.append(float(intensity))
                self._publish_scan_progress((i * side + j) / (side * side) * 0.5)
        return points_x, points_y, points_z, intensities

    # 增强功能1: 圆形扫描模式
    def _generate_circular(self, num_points):
        points_x, points_y, points_z, intensities = [], [], [], []
        max_radius = min(
            self._scan_bounds['x_max'] - self._scan_bounds['x_min'],
            self._scan_bounds['y_max'] - self._scan_bounds['y_min']) / 2.0
        center_x = (self._scan_bounds['x_min'] + self._scan_bounds['x_max']) / 2.0
        center_y = (self._scan_bounds['y_min'] + self._scan_bounds['y_max']) / 2.0
        rings = int(np.sqrt(num_points / np.pi))
        for i in range(rings):
            r = max_radius * (i + 1) / rings
            num_in_ring = max(6, int(2 * np.pi * r / self.scan_resolution))
            for j in range(num_in_ring):
                angle = 2 * np.pi * j / num_in_ring
                x = center_x + r * np.cos(angle)
                y = center_y + r * np.sin(angle)
                z = np.random.normal(0, 0.05)
                intensity = 0.5 + np.random.random() * 0.5
                points_x.append(float(x))
                points_y.append(float(y))
                points_z.append(float(z))
                intensities.append(float(intensity))
            self._publish_scan_progress(i / rings * 0.5)
        return points_x, points_y, points_z, intensities

    # 增强功能1: 螺旋扫描模式
    def _generate_helical(self, num_points):
        points_x, points_y, points_z, intensities = [], [], [], []
        max_radius = min(
            self._scan_bounds['x_max'] - self._scan_bounds['x_min'],
            self._scan_bounds['y_max'] - self._scan_bounds['y_min']) / 2.0
        center_x = (self._scan_bounds['x_min'] + self._scan_bounds['x_max']) / 2.0
        center_y = (self._scan_bounds['y_min'] + self._scan_bounds['y_max']) / 2.0
        z_range = self._scan_bounds['z_max'] - self._scan_bounds['z_min']
        turns = 5
        for i in range(num_points):
            t = i / num_points
            angle = t * turns * 2 * np.pi
            r = max_radius * t
            x = center_x + r * np.cos(angle)
            y = center_y + r * np.sin(angle)
            z = self._scan_bounds['z_min'] + z_range * t + np.random.normal(0, 0.03)
            intensity = 0.3 + 0.7 * (1.0 - t)
            points_x.append(float(x))
            points_y.append(float(y))
            points_z.append(float(z))
            intensities.append(float(intensity))
            if i % 100 == 0:
                self._publish_scan_progress(t * 0.5)
        return points_x, points_y, points_z, intensities

    # 增强功能1: 线性扫描模式
    def _generate_linear_sweep(self, num_points):
        points_x, points_y, points_z, intensities = [], [], [], []
        x_range = self._scan_bounds['x_max'] - self._scan_bounds['x_min']
        y_range = self._scan_bounds['y_max'] - self._scan_bounds['y_min']
        num_lines = max(1, int(y_range / (self.scan_resolution * 10)))
        points_per_line = num_points // num_lines
        for i in range(num_lines):
            y = self._scan_bounds['y_min'] + y_range * i / max(1, num_lines - 1)
            for j in range(points_per_line):
                x = self._scan_bounds['x_min'] + x_range * j / max(1, points_per_line - 1)
                z = np.random.normal(0, 0.05)
                intensity = 0.5 + np.random.random() * 0.5
                points_x.append(float(x))
                points_y.append(float(y))
                points_z.append(float(z))
                intensities.append(float(intensity))
            self._publish_scan_progress(i / num_lines * 0.5)
        return points_x, points_y, points_z, intensities

    # 增强功能8: 应用扫描边界裁剪
    def _apply_scan_bounds(self, px, py, pz, pi):
        filtered_x, filtered_y, filtered_z, filtered_i = [], [], [], []
        for x, y, z, intensity in zip(px, py, pz, pi):
            if (self._scan_bounds['x_min'] <= x <= self._scan_bounds['x_max'] and
                self._scan_bounds['y_min'] <= y <= self._scan_bounds['y_max'] and
                self._scan_bounds['z_min'] <= z <= self._scan_bounds['z_max']):
                filtered_x.append(x)
                filtered_y.append(y)
                filtered_z.append(z)
                filtered_i.append(intensity)
        return filtered_x, filtered_y, filtered_z, filtered_i

    # 增强功能2: 点云预处理 - 体素下采样 + 统计离群值移除
    def _preprocess_point_cloud(self, px, py, pz, pi):
        original_count = len(px)
        if len(px) == 0:
            return px, py, pz, pi

        points = np.array(list(zip(px, py, pz)), dtype=np.float64)
        intensities = np.array(pi, dtype=np.float64)

        # 体素网格下采样
        if self._enable_voxel and len(points) > 0:
            voxel_indices = np.floor(points / self._voxel_leaf_size).astype(int)
            _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
            points = points[unique_indices]
            intensities = intensities[unique_indices]

        # 统计离群值移除
        if self._enable_outlier and len(points) > self._outlier_mean_k:
            points, intensities = self._statistical_outlier_removal(points, intensities)

        # 发布预处理信息
        preprocess_info = {
            'original_count': original_count,
            'processed_count': len(points),
            'voxel_downsampled': self._enable_voxel,
            'outlier_removed': self._enable_outlier,
            'voxel_leaf_size': self._voxel_leaf_size,
            'reduction_percent': round((1.0 - len(points) / max(original_count, 1)) * 100, 1),
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = str(preprocess_info)
        self._preprocessing_pub.publish(msg)

        return points[:, 0].tolist(), points[:, 1].tolist(), points[:, 2].tolist(), intensities.tolist()

    # 增强功能2: 统计离群值移除实现
    def _statistical_outlier_removal(self, points, intensities):
        n = len(points)
        if n <= self._outlier_mean_k:
            return points, intensities

        k = min(self._outlier_mean_k, n - 1)
        distances = np.zeros(n)

        # 随机采样计算近邻距离（避免O(n^2)复杂度）
        sample_size = min(n, 500)
        sample_indices = np.random.choice(n, sample_size, replace=False)
        sample_points = points[sample_indices]

        for i in range(n):
            dists = np.sqrt(np.sum((sample_points - points[i]) ** 2, axis=1))
            dists.sort()
            distances[i] = np.mean(dists[1:k + 1]) if k < len(dists) else np.mean(dists[1:])

        global_mean = np.mean(distances)
        global_std = np.std(distances)
        threshold = global_mean + self._outlier_std_threshold * global_std

        inlier_mask = distances <= threshold
        return points[inlier_mask], intensities[inlier_mask]

    # 增强功能3: 扫描质量评估 - 计算点密度、覆盖率、噪声水平
    def _assess_scan_quality(self, px, py, pz):
        if len(px) < 10:
            return

        points = np.array(list(zip(px, py, pz)), dtype=np.float64)
        n = len(points)

        # 点密度计算 (点/平方米)
        x_range = self._scan_bounds['x_max'] - self._scan_bounds['x_min']
        y_range = self._scan_bounds['y_max'] - self._scan_bounds['y_min']
        scan_area = x_range * y_range
        point_density = n / scan_area if scan_area > 0 else 0.0

        # 覆盖率计算 - 将扫描区域划分为网格，计算有点的网格比例
        grid_size = self.scan_resolution * 5
        nx = max(1, int(x_range / grid_size))
        ny = max(1, int(y_range / grid_size))
        occupied_cells = set()
        for x, y in zip(px, py):
            gx = int((x - self._scan_bounds['x_min']) / grid_size)
            gy = int((y - self._scan_bounds['y_min']) / grid_size)
            occupied_cells.add((gx, gy))
        total_cells = nx * ny
        coverage_percent = len(occupied_cells) / total_cells * 100.0 if total_cells > 0 else 0.0

        # 噪声水平估算 - 基于局部邻域的距离方差
        noise_level = 0.0
        if n > 50:
            sample_size = min(n, 200)
            sample_idx = np.random.choice(n, sample_size, replace=False)
            local_stds = []
            for idx in sample_idx:
                dists = np.sqrt(np.sum((points - points[idx]) ** 2, axis=1))
                neighbor_idx = np.argsort(dists)[1:min(6, n)]
                if len(neighbor_idx) > 0:
                    local_stds.append(np.std(dists[neighbor_idx]))
            if local_stds:
                noise_level = np.mean(local_stds)

        # 质量评分
        density_score = min(100.0, point_density / self._expected_point_density * 100.0)
        coverage_score = coverage_percent
        noise_score = max(0.0, 100.0 - noise_level * 1000.0)
        overall_score = density_score * 0.4 + coverage_score * 0.4 + noise_score * 0.2

        quality_data = {
            'point_density': round(point_density, 2),
            'expected_density': self._expected_point_density,
            'coverage_percent': round(coverage_percent, 1),
            'noise_level': round(noise_level, 4),
            'density_score': round(density_score, 1),
            'coverage_score': round(coverage_score, 1),
            'noise_score': round(noise_score, 1),
            'overall_score': round(overall_score, 1),
            'total_points': n,
            'scan_pattern': self.scan_pattern,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = str(quality_data)
        self._scan_quality_pub.publish(msg)

    # 增强功能4: 扫描配准 - 简化ICP配准
    def _register_scan(self, px, py, pz):
        if self._previous_scan is None:
            self._previous_scan = np.array(list(zip(px, py, pz)), dtype=np.float64)
            reg_data = {
                'status': 'first_scan',
                'scan_count': self._scan_count,
                'alignment_error': 0.0,
                'transform': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'timestamp': datetime.now().isoformat(),
            }
            msg = String()
            msg.data = str(reg_data)
            self._registration_pub.publish(msg)
            return

        current = np.array(list(zip(px, py, pz)), dtype=np.float64)
        prev = self._previous_scan

        # 简化ICP: 计算质心对齐
        current_centroid = np.mean(current, axis=0)
        prev_centroid = np.mean(prev, axis=0)

        # 平移向量
        translation = current_centroid - prev_centroid

        # 对齐当前点云到前一点云
        aligned = current - translation

        # 计算配准误差 (采样以减少计算量)
        sample_size = min(len(aligned), len(prev), 500)
        if sample_size > 10:
            curr_sample = aligned[np.random.choice(len(aligned), sample_size, replace=False)]
            prev_sample = prev[np.random.choice(len(prev), sample_size, replace=False)]

            # 计算最近邻距离
            total_dist = 0.0
            for pt in curr_sample:
                dists = np.sqrt(np.sum((prev_sample - pt) ** 2, axis=1))
                total_dist += np.min(dists)
            avg_error = total_dist / sample_size
        else:
            avg_error = float(np.linalg.norm(translation))

        reg_data = {
            'status': 'registered',
            'scan_count': self._scan_count,
            'alignment_error': round(avg_error, 4),
            'translation_x': round(translation[0], 4),
            'translation_y': round(translation[1], 4),
            'translation_z': round(translation[2], 4),
            'transform': [round(t, 4) for t in translation] + [0.0, 0.0, 0.0],
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = str(reg_data)
        self._registration_pub.publish(msg)

        self._previous_scan = current

    # 增强功能5: 点云分割 - 地面提取 + 物体聚类
    def _segment_point_cloud(self, px, py, pz):
        if len(px) < 20:
            return

        points = np.array(list(zip(px, py, pz)), dtype=np.float64)
        n = len(points)

        # 地面平面提取 - 使用RANSAC简化版
        ground_mask = np.zeros(n, dtype=bool)
        ground_z_threshold = np.percentile(points[:, 2], 30)
        ground_mask = points[:, 2] <= ground_z_threshold

        ground_count = np.sum(ground_mask)
        object_points = points[~ground_mask]
        object_count = len(object_points)

        # 物体聚类 - 简化DBSCAN
        clusters = []
        if object_count > 5:
            visited = np.zeros(object_count, dtype=bool)
            cluster_labels = np.full(object_count, -1, dtype=int)
            eps = self.scan_resolution * 10
            min_samples = 3
            cluster_id = 0

            for i in range(object_count):
                if visited[i]:
                    continue
                visited[i] = True
                neighbors = self._range_query(object_points, i, eps)
                if len(neighbors) < min_samples:
                    continue
                cluster_labels[i] = cluster_id
                seed_set = list(neighbors)
                for s in seed_set:
                    if not visited[s]:
                        visited[s] = True
                        s_neighbors = self._range_query(object_points, s, eps)
                        if len(s_neighbors) >= min_samples:
                            for sn in s_neighbors:
                                if sn not in seed_set:
                                    seed_set.append(sn)
                    if cluster_labels[s] == -1:
                        cluster_labels[s] = cluster_id
                cluster_id += 1
                if cluster_id > 20:
                    break

            for cid in range(cluster_id):
                cluster_mask = cluster_labels == cid
                cluster_points = object_points[cluster_mask]
                if len(cluster_points) > 0:
                    clusters.append({
                        'id': cid,
                        'point_count': int(len(cluster_points)),
                        'centroid': [round(float(c), 3) for c in np.mean(cluster_points, axis=0)],
                        'bounds': {
                            'x_min': round(float(np.min(cluster_points[:, 0])), 3),
                            'x_max': round(float(np.max(cluster_points[:, 0])), 3),
                            'y_min': round(float(np.min(cluster_points[:, 1])), 3),
                            'y_max': round(float(np.max(cluster_points[:, 1])), 3),
                            'z_min': round(float(np.min(cluster_points[:, 2])), 3),
                            'z_max': round(float(np.max(cluster_points[:, 2])), 3),
                        },
                    })

        seg_data = {
            'ground_points': int(ground_count),
            'object_points': object_count,
            'ground_z_threshold': round(float(ground_z_threshold), 4),
            'cluster_count': len(clusters),
            'clusters': clusters[:10],
            'total_points': n,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = str(seg_data)
        self._segmentation_pub.publish(msg)

    def _range_query(self, points, idx, eps):
        dists = np.sqrt(np.sum((points - points[idx]) ** 2, axis=1))
        return np.where(dists <= eps)[0].tolist()

    # 增强功能6: 发布扫描进度
    def _publish_scan_progress(self, progress):
        elapsed = time.time() - self._scan_start_time
        if progress > 0 and progress < 1.0:
            estimated_total = elapsed / progress
            estimated_remaining = estimated_total - elapsed
        else:
            estimated_remaining = 0.0

        progress_data = {
            'progress_percent': round(progress * 100, 1),
            'elapsed_seconds': round(elapsed, 1),
            'estimated_remaining_seconds': round(max(0, estimated_remaining), 1),
            'points_generated': self._scan_generated_points,
            'total_points_target': self._scan_total_points,
            'scan_pattern': self.scan_pattern,
            'is_scanning': self.is_scanning,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = str(progress_data)
        self._scan_progress_pub.publish(msg)

    # 增强功能7: 导出PLY格式点云
    def export_ply_callback(self, request, response):
        if self._last_scan_data is None:
            response.success = False
            response.message = 'No scan data available to export'
            return response

        try:
            os.makedirs(self._export_path, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'scan_{timestamp}.ply'
            filepath = os.path.join(self._export_path, filename)

            px = self._last_scan_data['points_x']
            py = self._last_scan_data['points_y']
            pz = self._last_scan_data['points_z']
            pi = self._last_scan_data['intensities']
            n = len(px)

            with open(filepath, 'w') as f:
                f.write('ply\n')
                f.write('format ascii 1.0\n')
                f.write(f'element vertex {n}\n')
                f.write('property float x\n')
                f.write('property float y\n')
                f.write('property float z\n')
                f.write('property float intensity\n')
                f.write('end_header\n')
                for i in range(n):
                    f.write(f'{px[i]:.6f} {py[i]:.6f} {pz[i]:.6f} {pi[i]:.6f}\n')

            response.success = True
            response.message = f'Exported {n} points to {filepath}'
            self.get_logger().info(f'PLY导出: {filepath} ({n}个点)')
        except Exception as e:
            response.success = False
            response.message = f'Export failed: {str(e)}'

        return response

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
    main()
