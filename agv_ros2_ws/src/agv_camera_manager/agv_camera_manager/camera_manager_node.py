import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from agv_interfaces.msg import CameraConfig, SystemConfig
import numpy as np
import json
import time
import threading
import glob
import subprocess
from collections import deque
from datetime import datetime


class CameraManagerNode(Node):

    def __init__(self):
        super().__init__('camera_manager')

        # 原有参数
        self.declare_parameter('camera_id', 'main_camera')
        self.declare_parameter('device_path', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('format', 'bgr8')
        self.declare_parameter('exposure_mode', 'auto')
        self.declare_parameter('exposure', 0.0)
        self.declare_parameter('gain', 0.0)
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('enabled', True)
        self.declare_parameter('use_rtsp', False)
        self.declare_parameter('rtsp_url', '')

        # 相机自动发现参数
        self.declare_parameter('auto_discovery_enabled', True)
        self.declare_parameter('discovery_interval_sec', 60.0)

        # 相机标定参数
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('undistort_enabled', False)

        # 图像预处理管道参数
        self.declare_parameter('preprocess_resize_enabled', False)
        self.declare_parameter('preprocess_resize_width', 640)
        self.declare_parameter('preprocess_resize_height', 480)
        self.declare_parameter('preprocess_denoise_enabled', False)
        self.declare_parameter('preprocess_denoise_strength', 10)
        self.declare_parameter('preprocess_color_correction_enabled', False)
        self.declare_parameter('preprocess_brightness', 0.0)
        self.declare_parameter('preprocess_contrast', 1.0)
        self.declare_parameter('preprocess_saturation', 1.0)

        # 相机健康监控参数
        self.declare_parameter('health_check_enabled', True)
        self.declare_parameter('max_frame_drop_rate', 0.3)
        self.declare_parameter('reconnect_max_attempts', 5)
        self.declare_parameter('reconnect_interval_sec', 3.0)

        # 多相机同步参数
        self.declare_parameter('sync_enabled', False)
        self.declare_parameter('sync_topic', '/camera/sync_timestamps')

        # 曝光/对焦自动调整参数
        self.declare_parameter('auto_exposure_adjust_enabled', True)
        self.declare_parameter('target_brightness', 120.0)
        self.declare_parameter('exposure_adjust_interval_sec', 5.0)
        self.declare_parameter('exposure_adjust_step', 0.05)

        self.camera_id = self.get_parameter('camera_id').get_parameter_value().string_value
        self.device_path = self.get_parameter('device_path').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.format = self.get_parameter('format').get_parameter_value().string_value
        self.exposure_mode = self.get_parameter('exposure_mode').get_parameter_value().string_value
        self.exposure = self.get_parameter('exposure').get_parameter_value().double_value
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        self.auto_exposure = self.get_parameter('auto_exposure').get_parameter_value().bool_value
        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        self.use_rtsp = self.get_parameter('use_rtsp').get_parameter_value().bool_value
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value

        # 相机自动发现初始化
        self.auto_discovery_enabled = self.get_parameter('auto_discovery_enabled').get_parameter_value().bool_value
        self.discovered_cameras = {}

        # 相机标定初始化
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.undistort_enabled = self.get_parameter('undistort_enabled').get_parameter_value().bool_value
        self.camera_matrix = None
        self.dist_coeffs = None
        self.undistort_map1 = None
        self.undistort_map2 = None
        self._load_calibration()

        # 图像预处理管道初始化
        self.preprocess_config = {
            'resize_enabled': self.get_parameter('preprocess_resize_enabled').get_parameter_value().bool_value,
            'resize_width': self.get_parameter('preprocess_resize_width').get_parameter_value().integer_value,
            'resize_height': self.get_parameter('preprocess_resize_height').get_parameter_value().integer_value,
            'denoise_enabled': self.get_parameter('preprocess_denoise_enabled').get_parameter_value().bool_value,
            'denoise_strength': self.get_parameter('preprocess_denoise_strength').get_parameter_value().integer_value,
            'color_correction_enabled': self.get_parameter('preprocess_color_correction_enabled').get_parameter_value().bool_value,
            'brightness': self.get_parameter('preprocess_brightness').get_parameter_value().double_value,
            'contrast': self.get_parameter('preprocess_contrast').get_parameter_value().double_value,
            'saturation': self.get_parameter('preprocess_saturation').get_parameter_value().double_value,
        }

        # 相机健康监控初始化
        self.health_check_enabled = self.get_parameter('health_check_enabled').get_parameter_value().bool_value
        self.max_frame_drop_rate = self.get_parameter('max_frame_drop_rate').get_parameter_value().double_value
        self.reconnect_max_attempts = self.get_parameter('reconnect_max_attempts').get_parameter_value().integer_value
        self.reconnect_interval_sec = self.get_parameter('reconnect_interval_sec').get_parameter_value().double_value
        self.frame_drop_count = 0
        self.frame_total_count = 0
        self.consecutive_failures = 0
        self.connection_stable = True
        self.last_reconnect_time = 0.0

        # 多相机同步初始化
        self.sync_enabled = self.get_parameter('sync_enabled').get_parameter_value().bool_value
        self.sync_topic = self.get_parameter('sync_topic').get_parameter_value().string_value

        # 曝光/对焦自动调整初始化
        self.auto_exposure_adjust_enabled = self.get_parameter('auto_exposure_adjust_enabled').get_parameter_value().bool_value
        self.target_brightness = self.get_parameter('target_brightness').get_parameter_value().double_value
        self.exposure_adjust_interval = self.get_parameter('exposure_adjust_interval_sec').get_parameter_value().double_value
        self.exposure_adjust_step = self.get_parameter('exposure_adjust_step').get_parameter_value().double_value
        self.last_exposure_adjust = time.time()
        self.current_brightness = 0.0

        # 相机统计信息
        self.stats_fps = 0.0
        self.stats_latency_ms = 0.0
        self.stats_dropped_frames = 0
        self.stats_frame_count = 0
        self.stats_last_time = time.time()
        self.frame_timestamps = deque(maxlen=30)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.config_subscriber = self.create_subscription(
            SystemConfig,
            '/system_config',
            self.config_callback,
            10
        )

        # 相机统计信息发布器
        self.stats_pub = self.create_publisher(String, '/camera/stats', 10)

        # 多相机同步发布器
        if self.sync_enabled:
            self.sync_pub = self.create_publisher(String, self.sync_topic, 10)

        # 发现相机发布器
        self.discovery_pub = self.create_publisher(String, '/camera/discovered', 10)

        self.cap = None
        self._init_camera()

        self._frame_interval = 1.0 / self.fps if self.fps > 0 else 0.0
        self._running = True

        self.timer = self.create_timer(self._frame_interval, self.timer_callback)

        # 相机自动发现定时器
        if self.auto_discovery_enabled:
            discovery_interval = self.get_parameter('discovery_interval_sec').get_parameter_value().double_value
            self.discovery_timer = self.create_timer(discovery_interval, self.discover_cameras_callback)

        # 统计信息定时发布
        self.stats_timer = self.create_timer(1.0, self.publish_camera_stats)

        self.get_logger().info(f'CameraManagerNode initialized at {self.fps} fps')

    def _init_camera(self):
        if self.use_rtsp and self.rtsp_url:
            self.get_logger().info(f'Opening RTSP stream: {self.rtsp_url}')
            self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        else:
            self.get_logger().info(f'Opening camera device: {self.device_path}')
            self.cap = cv2.VideoCapture(self.device_path)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            # 设置曝光参数
            if self.auto_exposure:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)

            self.cap.set(cv2.CAP_PROP_GAIN, self.gain)

            self.get_logger().info(
                f'Camera opened: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                f'{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}'
            )

            # 重新计算去畸变映射表
            if self.undistort_enabled and self.camera_matrix is not None:
                self._compute_undistort_maps()

            self.consecutive_failures = 0
            self.connection_stable = True
        else:
            self.get_logger().error('Failed to open camera')

    def _load_calibration(self):
        # 加载相机标定参数
        if not self.calibration_file:
            self.get_logger().info('未指定标定文件，跳过标定参数加载')
            return

        try:
            fs = cv2.FileStorage(self.calibration_file, cv2.FILE_STORAGE_READ)
            self.camera_matrix = fs.getNode('camera_matrix').mat()
            self.dist_coeffs = fs.getNode('distortion_coefficients').mat()
            fs.release()
            self.get_logger().info(f'相机标定参数已加载: {self.calibration_file}')

            if self.undistort_enabled and self.cap and self.cap.isOpened():
                self._compute_undistort_maps()
        except Exception as e:
            self.get_logger().error(f'加载标定参数失败: {e}')
            self.camera_matrix = None
            self.dist_coeffs = None

    def _compute_undistort_maps(self):
        # 计算去畸变映射表，避免每帧重复计算
        if self.camera_matrix is None or self.dist_coeffs is None:
            return
        try:
            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, None,
                cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs,
                                               (self.width, self.height), 1,
                                               (self.width, self.height)),
                (self.width, self.height), cv2.CV_16SC2
            )
            self.get_logger().info('去畸变映射表已计算')
        except Exception as e:
            self.get_logger().error(f'计算去畸变映射表失败: {e}')
            self.undistort_map1 = None
            self.undistort_map2 = None

    def _apply_preprocessing(self, frame):
        # 应用图像预处理管道
        # 1. 去噪
        if self.preprocess_config['denoise_enabled']:
            strength = self.preprocess_config['denoise_strength']
            frame = cv2.fastNlMeansDenoisingColored(frame, None, strength, strength, 7, 21)

        # 2. 颜色校正
        if self.preprocess_config['color_correction_enabled']:
            brightness = self.preprocess_config['brightness']
            contrast = self.preprocess_config['contrast']
            saturation = self.preprocess_config['saturation']

            # 亮度和对比度调整
            if brightness != 0.0 or contrast != 1.0:
                frame = cv2.convertScaleAbs(frame, alpha=contrast, beta=brightness)

            # 饱和度调整
            if saturation != 1.0:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
                hsv[:, :, 1] = hsv[:, :, 1] * saturation
                hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
                frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        # 3. 去畸变
        if self.undistort_enabled and self.undistort_map1 is not None and self.undistort_map2 is not None:
            frame = cv2.remap(frame, self.undistort_map1, self.undistort_map2,
                             cv2.INTER_LINEAR)

        # 4. 缩放
        if self.preprocess_config['resize_enabled']:
            target_w = self.preprocess_config['resize_width']
            target_h = self.preprocess_config['resize_height']
            frame = cv2.resize(frame, (target_w, target_h))

        return frame

    def _auto_adjust_exposure(self, frame):
        # 曝光自动调整 - 根据场景亮度自动调整曝光参数
        if not self.auto_exposure_adjust_enabled:
            return
        if self.auto_exposure:
            return

        now = time.time()
        if now - self.last_exposure_adjust < self.exposure_adjust_interval:
            return
        self.last_exposure_adjust = now

        # 计算当前帧平均亮度
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.current_brightness = np.mean(gray)

        # 根据亮度差异调整曝光
        brightness_diff = self.current_brightness - self.target_brightness
        if abs(brightness_diff) > 10:
            exposure_adjustment = -brightness_diff * self.exposure_adjust_step
            new_exposure = self.exposure + exposure_adjustment

            # 限制曝光范围
            new_exposure = max(-13.0, min(0.0, new_exposure))

            if abs(new_exposure - self.exposure) > 0.01:
                self.exposure = new_exposure
                if self.cap and self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
                    self.get_logger().debug(
                        f'自动调整曝光: {self.exposure:.2f} (亮度: {self.current_brightness:.1f})'
                    )

    def _check_camera_health(self):
        # 相机健康监控 - 检查帧丢失率和连接稳定性
        if not self.health_check_enabled:
            return

        # 计算帧丢失率
        if self.frame_total_count > 10:
            drop_rate = self.frame_drop_count / self.frame_total_count
            if drop_rate > self.max_frame_drop_rate:
                self.connection_stable = False
                self.get_logger().warn(
                    f'帧丢失率过高: {drop_rate:.2%} (阈值: {self.max_frame_drop_rate:.2%})'
                )
            else:
                self.connection_stable = True

        # 重置统计（每100帧重置一次，保持滑动窗口效果）
        if self.frame_total_count > 100:
            self.frame_drop_count = max(0, self.frame_drop_count - self.frame_total_count // 2)
            self.frame_total_count = self.frame_total_count // 2

    def _try_reconnect_with_retry(self):
        # 带重试的自动重连
        now = time.time()
        if now - self.last_reconnect_time < self.reconnect_interval_sec:
            return
        self.last_reconnect_time = now

        for attempt in range(self.reconnect_max_attempts):
            self.get_logger().info(f'尝试重连相机 (第 {attempt + 1}/{self.reconnect_max_attempts} 次)')
            if self.cap and self.cap.isOpened():
                self.cap.release()

            time.sleep(0.5)
            self._init_camera()

            if self.cap and self.cap.isOpened():
                self.get_logger().info('相机重连成功')
                self.consecutive_failures = 0
                return

        self.get_logger().error(f'相机重连失败，已尝试 {self.reconnect_max_attempts} 次')

    def discover_cameras_callback(self):
        # 相机自动发现 - 扫描可用的USB相机和RTSP流
        if not self.auto_discovery_enabled:
            return

        found_cameras = {}

        # 扫描USB视频设备
        try:
            video_devices = glob.glob('/dev/video*')
            for device_path in sorted(video_devices):
                try:
                    cap = cv2.VideoCapture(device_path)
                    if cap.isOpened():
                        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        fps = int(cap.get(cv2.CAP_PROP_FPS))
                        backend = cap.getBackendName()
                        found_cameras[device_path] = {
                            'type': 'usb',
                            'path': device_path,
                            'resolution': f'{w}x{h}',
                            'fps': fps,
                            'backend': backend,
                        }
                        cap.release()
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f'扫描USB设备失败: {e}')

        # 检查已知的RTSP流
        try:
            known_rtsp_urls = [self.rtsp_url] if self.use_rtsp and self.rtsp_url else []
            for url in known_rtsp_urls:
                try:
                    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                    if cap.isOpened():
                        found_cameras[url] = {
                            'type': 'rtsp',
                            'url': url,
                        }
                        cap.release()
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f'扫描RTSP流失败: {e}')

        # 发布发现结果
        new_cameras = set(found_cameras.keys()) - set(self.discovered_cameras.keys())
        if new_cameras:
            self.get_logger().info(f'发现新相机: {new_cameras}')

        self.discovered_cameras = found_cameras

        msg = String()
        msg.data = json.dumps(found_cameras)
        self.discovery_pub.publish(msg)

    def config_callback(self, msg):
        for cam_cfg in msg.camera_configs:
            if cam_cfg.camera_id == self.camera_id:
                self._update_config(cam_cfg)
                break

    def _update_config(self, config):
        restart_needed = False
        if config.device_path != self.device_path:
            self.device_path = config.device_path
            restart_needed = True
        if config.width != self.width:
            self.width = config.width
            restart_needed = True
        if config.height != self.height:
            self.height = config.height
            restart_needed = True
        if config.fps != self.fps:
            self.fps = config.fps
            self._frame_interval = 1.0 / self.fps if self.fps > 0 else 0.0
            self.timer.timer_period_ns = int(self._frame_interval * 1e9)
        self.format = config.format
        self.exposure_mode = config.exposure_mode
        self.exposure = config.exposure
        self.gain = config.gain
        self.auto_exposure = config.auto_exposure
        self.enabled = config.enabled

        if restart_needed and self.cap:
            self.cap.release()
            self._init_camera()

    def timer_callback(self):
        if not self.enabled:
            return

        if not self.cap or not self.cap.isOpened():
            self.get_logger().warn('Camera not open, attempting to reconnect...')
            self._try_reconnect_with_retry()
            return

        ret, frame = self.cap.read()
        self.frame_total_count += 1

        if not ret:
            self.frame_drop_count += 1
            self.consecutive_failures += 1
            self.stats_dropped_frames += 1

            if self.consecutive_failures >= 3:
                self.get_logger().warn('连续读取帧失败，尝试重连...')
                self._try_reconnect_with_retry()
            return

        self.consecutive_failures = 0

        # 曝光自动调整
        self._auto_adjust_exposure(frame)

        # 应用图像预处理管道
        frame = self._apply_preprocessing(frame)

        # 相机健康检查
        self._check_camera_health()

        try:
            capture_time = time.time()
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.format)
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.camera_id
            self.publisher.publish(img_msg)

            # 记录帧时间戳用于统计
            self.frame_timestamps.append(capture_time)
            self.stats_frame_count += 1

            # 多相机同步 - 发布时间戳
            if self.sync_enabled and self.sync_pub:
                sync_info = {
                    'camera_id': self.camera_id,
                    'timestamp': capture_time,
                    'frame_count': self.stats_frame_count,
                }
                sync_msg = String()
                sync_msg.data = json.dumps(sync_info)
                self.sync_pub.publish(sync_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to convert frame: {e}')

    def publish_camera_stats(self):
        # 发布相机统计信息 - FPS、分辨率、延迟、丢帧数
        now = time.time()
        elapsed = now - self.stats_last_time
        if elapsed > 0 and self.stats_frame_count > 0:
            self.stats_fps = self.stats_frame_count / elapsed

            # 计算延迟
            if len(self.frame_timestamps) >= 2:
                recent = list(self.frame_timestamps)
                latencies = [recent[i+1] - recent[i] for i in range(len(recent)-1)]
                avg_latency = sum(latencies) / len(latencies) * 1000 if latencies else 0
                self.stats_latency_ms = avg_latency

        self.stats_frame_count = 0
        self.stats_last_time = now

        stats = {
            'camera_id': self.camera_id,
            'fps': round(self.stats_fps, 2),
            'resolution': f'{self.width}x{self.height}',
            'latency_ms': round(self.stats_latency_ms, 2),
            'dropped_frames': self.stats_dropped_frames,
            'connection_stable': self.connection_stable,
            'frame_drop_rate': round(
                self.frame_drop_count / max(self.frame_total_count, 1), 4
            ),
            'current_brightness': round(self.current_brightness, 1),
            'exposure': self.exposure,
            'undistort_enabled': self.undistort_enabled,
        }

        msg = String()
        msg.data = json.dumps(stats)
        self.stats_pub.publish(msg)

    def _try_reconnect(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self._init_camera()

    def destroy_node(self):
        self._running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
