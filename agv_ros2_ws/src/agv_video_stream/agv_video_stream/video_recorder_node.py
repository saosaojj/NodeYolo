import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_image_camera, qos_profile_sensor_data

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs

from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import queue
import time
import os
import json
from datetime import datetime
from collections import deque


class CircularBuffer:
    def __init__(self, max_size):
        self.max_size = max_size
        self.buffer = deque(maxlen=max_size)
        self.lock = threading.Lock()

    def append(self, item):
        with self.lock:
            self.buffer.append(item)

    def get_all(self):
        with self.lock:
            return list(self.buffer)

    def get_last_n_seconds(self, seconds, fps=30):
        # 获取最近N秒的缓冲帧
        with self.lock:
            if not self.buffer:
                return []
            count = min(int(seconds * fps), len(self.buffer))
            return list(self.buffer)[-count:]

    def clear(self):
        with self.lock:
            self.buffer.clear()

    def __len__(self):
        with self.lock:
            return len(self.buffer)


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')

        # 原有参数
        self.declare_parameter('record_path', '/tmp/agv_recordings')
        self.declare_parameter('max_storage_gb', 10.0)
        self.declare_parameter('min_free_space_gb', 1.0)
        self.declare_parameter('detection_threshold', 0.7)
        self.declare_parameter('record_duration_sec', 5.0)
        self.declare_parameter('snapshot_interval_sec', 0.5)
        self.declare_parameter('trigger_classes', ['person', 'obstacle', 'forklift', 'pallet'])

        # 录制计划参数
        self.declare_parameter('schedule_enabled', False)
        self.declare_parameter('schedule_start_hour', 8)
        self.declare_parameter('schedule_end_hour', 18)
        self.declare_parameter('schedule_days', [0, 1, 2, 3, 4, 5, 6])

        # 存储管理参数
        self.declare_parameter('storage_check_interval_sec', 60.0)
        self.declare_parameter('disk_usage_threshold_percent', 90.0)

        # 事件录制缓冲参数
        self.declare_parameter('pre_event_buffer_sec', 3.0)
        self.declare_parameter('post_event_buffer_sec', 5.0)

        # 录制健康检查参数
        self.declare_parameter('health_check_interval_sec', 300.0)

        self.record_path = self.get_parameter('record_path').value
        self.max_storage_gb = self.get_parameter('max_storage_gb').value
        self.min_free_space_gb = self.get_parameter('min_free_space_gb').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.record_duration_sec = self.get_parameter('record_duration_sec').value
        self.snapshot_interval_sec = self.get_parameter('snapshot_interval_sec').value
        self.trigger_classes = self.get_parameter('trigger_classes').value

        # 录制计划初始化
        self.schedule_enabled = self.get_parameter('schedule_enabled').value
        self.schedule_start_hour = self.get_parameter('schedule_start_hour').value
        self.schedule_end_hour = self.get_parameter('schedule_end_hour').value
        self.schedule_days = self.get_parameter('schedule_days').value

        # 存储管理初始化
        self.storage_check_interval = self.get_parameter('storage_check_interval_sec').value
        self.disk_usage_threshold = self.get_parameter('disk_usage_threshold_percent').value
        self.last_storage_check = time.time()

        # 事件录制缓冲初始化
        self.pre_event_buffer_sec = self.get_parameter('pre_event_buffer_sec').value
        self.post_event_buffer_sec = self.get_parameter('post_event_buffer_sec').value
        self.event_recording_active = False
        self.event_recording_end_time = None
        self.event_detection_count = 0

        # 录制健康检查初始化
        self.health_check_interval = self.get_parameter('health_check_interval_sec').value
        self.last_health_check = time.time()
        self.recording_health_status = 'unknown'

        # 录制元数据
        self.recording_metadata = {}

        os.makedirs(self.record_path, exist_ok=True)
        os.makedirs(os.path.join(self.record_path, 'snapshots'), exist_ok=True)
        os.makedirs(os.path.join(self.record_path, 'videos'), exist_ok=True)
        os.makedirs(os.path.join(self.record_path, 'metadata'), exist_ok=True)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_yolo = None
        self.is_recording = False
        self.record_thread = None
        self.snapshot_thread = None
        self.frame_buffer = CircularBuffer(int(30 * max(self.record_duration_sec, self.pre_event_buffer_sec + self.post_event_buffer_sec + 5)))

        self.recording_writer = None
        self.recording_start_time = None
        self.current_recording_path = None

        self.status_pub = self.create_publisher(std_msgs.String, 'recording_status', 10)
        self.snapshot_pub = self.create_publisher(sensor_msgs.Image, 'latest_snapshot', 10)

        self.image_sub = self.create_subscription(
            sensor_msgs.Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_image_camera
        )

        try:
            self.yolo_sub = self.create_subscription(
                'YoloResult',
                '/yolo_result',
                self.yolo_callback,
                qos_profile_sensor_data
            )
        except Exception:
            self.get_logger().warn('YoloResult topic not available')
            self.yolo_sub = None

        self.start_recording_srv = self.create_service(
            std_srvs.Trigger,
            '/video/start_recording',
            self.start_recording_callback
        )

        self.stop_recording_srv = self.create_service(
            std_srvs.Trigger,
            '/video/stop_recording',
            self.stop_recording_callback
        )

        self.snapshot_srv = self.create_service(
            std_srvs.Trigger,
            '/video/snapshot',
            self.snapshot_callback
        )

        self.set_detection_record_srv = self.create_service(
            'SetModel',
            '/video/set_detection_record',
            self.set_detection_record_callback
        )

        # 录制计划配置服务
        self.set_schedule_srv = self.create_service(
            'SetModel',
            '/video/set_schedule',
            self.set_schedule_callback
        )

        self.last_snapshot_time = time.time()
        self.snapshot_thread = threading.Thread(target=self.snapshot_worker)
        self.snapshot_thread.daemon = True
        self.snapshot_thread.start()

        # 存储管理定时检查
        self.storage_timer = self.create_timer(self.storage_check_interval, self.storage_check_callback)

        # 录制健康检查定时器
        self.health_timer = self.create_timer(self.health_check_interval, self.health_check_callback)

        # 录制计划定时检查
        self.schedule_timer = self.create_timer(60.0, self.schedule_check_callback)

        self.get_logger().info('VideoRecorderNode initialized')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image.copy()
            self.frame_buffer.append({
                'frame': cv_image,
                'timestamp': time.time(),
                'header': msg.header
            })

            if self.is_recording:
                self.write_frame(cv_image)

                # 事件录制后缓冲检查
                if self.event_recording_active and self.event_recording_end_time:
                    if time.time() >= self.event_recording_end_time:
                        self.get_logger().info('事件录制后缓冲结束，停止录制')
                        self.stop_recording()
                        self.event_recording_active = False
                        self.event_recording_end_time = None
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def yolo_callback(self, msg):
        self.latest_yolo = msg
        if self.trigger_classes and self.check_important_detection(msg):
            self.event_detection_count += 1
            self.trigger_detection_recording()

    def check_important_detection(self, yolo_msg):
        try:
            class_names = yolo_msg.class_names
            scores = yolo_msg.scores

            for i, class_name in enumerate(class_names):
                if class_name in self.trigger_classes:
                    if i < len(scores) and scores[i] >= self.detection_threshold:
                        return True
            return False
        except Exception:
            return False

    def trigger_detection_recording(self):
        # 事件触发录制，支持前后缓冲
        if not self.is_recording:
            self.get_logger().info('重要检测触发，启动事件录制')
            pre_frames = self.frame_buffer.get_last_n_seconds(self.pre_event_buffer_sec)
            if pre_frames:
                self.start_event_recording_from_buffer(pre_frames)
        else:
            # 已在录制中，延长后缓冲时间
            if self.event_recording_active:
                self.event_recording_end_time = time.time() + self.post_event_buffer_sec
                self.get_logger().info('检测到新事件，延长录制后缓冲')

    def start_event_recording_from_buffer(self, buffered_frames):
        # 事件录制启动，包含前缓冲帧
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.current_recording_path = os.path.join(
            self.record_path, 'videos', f'event_{timestamp}.mp4'
        )

        if not buffered_frames:
            return

        frame_shape = buffered_frames[0]['frame'].shape
        height, width = frame_shape[:2]

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.recording_writer = cv2.VideoWriter(
            self.current_recording_path,
            fourcc,
            30.0,
            (width, height)
        )

        # 写入前缓冲帧
        for frame_data in buffered_frames:
            self.write_frame_with_overlay(frame_data['frame'])

        self.recording_start_time = time.time()
        self.is_recording = True
        self.event_recording_active = True
        self.event_detection_count = 0
        self.event_recording_end_time = time.time() + self.post_event_buffer_sec

        # 保存录制元数据
        self.recording_metadata = {
            'type': 'event',
            'start_time': datetime.now().isoformat(),
            'pre_buffer_sec': self.pre_event_buffer_sec,
            'post_buffer_sec': self.post_event_buffer_sec,
        }

        self.publish_status('recording')

    def start_recording_from_buffer(self, buffered_frames):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.current_recording_path = os.path.join(
            self.record_path, 'videos', f'event_{timestamp}.mp4'
        )

        if not buffered_frames:
            return

        frame_shape = buffered_frames[0]['frame'].shape
        height, width = frame_shape[:2]

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.recording_writer = cv2.VideoWriter(
            self.current_recording_path,
            fourcc,
            30.0,
            (width, height)
        )

        for frame_data in buffered_frames:
            self.write_frame_with_overlay(frame_data['frame'])

        self.recording_start_time = time.time()
        self.is_recording = True

        self.recording_metadata = {
            'type': 'event',
            'start_time': datetime.now().isoformat(),
        }

        self.publish_status('recording')

    def start_recording_callback(self, request, response):
        if not self.is_recording:
            self.start_continuous_recording()
            response.success = True
            response.message = 'Recording started'
        else:
            response.success = True
            response.message = 'Already recording'
        return response

    def start_continuous_recording(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.current_recording_path = os.path.join(
            self.record_path, 'videos', f'continuous_{timestamp}.mp4'
        )

        if self.latest_frame is not None:
            height, width = self.latest_frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.recording_writer = cv2.VideoWriter(
                self.current_recording_path,
                fourcc,
                30.0,
                (width, height)
            )

        self.recording_start_time = time.time()
        self.is_recording = True
        self.record_thread = threading.Thread(target=self.recording_worker)
        self.record_thread.daemon = True
        self.record_thread.start()

        self.recording_metadata = {
            'type': 'continuous',
            'start_time': datetime.now().isoformat(),
        }

        self.publish_status('recording')
        self.get_logger().info('Continuous recording started')

    def recording_worker(self):
        while self.is_recording:
            if self.latest_frame is not None and self.recording_writer:
                self.write_frame_with_overlay(self.latest_frame)
            time.sleep(1.0 / 30.0)

    def stop_recording_callback(self, request, response):
        if self.is_recording:
            self.stop_recording()
            response.success = True
            response.message = 'Recording stopped'
        else:
            response.success = True
            response.message = 'Not recording'
        return response

    def stop_recording(self):
        duration = time.time() - self.recording_start_time if self.recording_start_time else 0.0
        self.is_recording = False
        self.event_recording_active = False
        self.event_recording_end_time = None

        if self.recording_writer:
            self.recording_writer.release()
            self.recording_writer = None

        # 保存录制元数据
        self.recording_metadata['end_time'] = datetime.now().isoformat()
        self.recording_metadata['duration_sec'] = round(duration, 2)
        self.recording_metadata['detection_count'] = self.event_detection_count
        self.recording_metadata['file_path'] = self.current_recording_path
        self._save_recording_metadata()

        self.publish_status('stopped')
        self.get_logger().info(f'Recording saved to {self.current_recording_path}')

    def _save_recording_metadata(self):
        # 保存录制元数据到JSON文件
        if not self.current_recording_path:
            return
        try:
            base_name = os.path.splitext(os.path.basename(self.current_recording_path))[0]
            metadata_path = os.path.join(self.record_path, 'metadata', f'{base_name}.json')
            with open(metadata_path, 'w') as f:
                json.dump(self.recording_metadata, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f'录制元数据已保存: {metadata_path}')
        except Exception as e:
            self.get_logger().error(f'保存录制元数据失败: {e}')

    def write_frame(self, frame):
        if self.recording_writer:
            self.recording_writer.write(frame)

    def write_frame_with_overlay(self, frame):
        overlay = frame.copy()

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(overlay, timestamp, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if self.latest_yolo and self.check_important_detection(self.latest_yolo):
            cv2.putText(overlay, 'IMPORTANT DETECTION', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self.write_frame(overlay)

    def snapshot_callback(self, request, response):
        if self.latest_frame is not None:
            self.take_snapshot(self.latest_frame)
            response.success = True
            response.message = 'Snapshot taken'
        else:
            response.success = False
            response.message = 'No frame available'
        return response

    def snapshot_worker(self):
        while True:
            if (time.time() - self.last_snapshot_time) >= self.snapshot_interval_sec:
                if self.latest_frame is not None:
                    self.take_snapshot(self.latest_frame)
                    self.last_snapshot_time = time.time()
            time.sleep(0.1)

    def take_snapshot(self, frame):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.record_path, 'snapshots', f'{timestamp}.jpg')

        overlay = frame.copy()
        timestamp_text = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(overlay, timestamp_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imwrite(filename, overlay)

        try:
            snapshot_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            self.snapshot_pub.publish(snapshot_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing snapshot: {e}')

        self.get_logger().debug(f'Snapshot saved: {filename}')

    def set_detection_record_callback(self, request, response):
        self.trigger_classes = request.name.split(',')
        response.success = True
        response.message = f'Detection classes set to: {self.trigger_classes}'
        self.get_logger().info(f'Detection classes updated: {self.trigger_classes}')
        return response

    def set_schedule_callback(self, request, response):
        # 设置录制计划（通过name字段传递JSON编码的计划配置）
        try:
            config = json.loads(request.name)
            if 'enabled' in config:
                self.schedule_enabled = bool(config['enabled'])
            if 'start_hour' in config:
                self.schedule_start_hour = int(config['start_hour'])
            if 'end_hour' in config:
                self.schedule_end_hour = int(config['end_hour'])
            if 'days' in config:
                self.schedule_days = [int(d) for d in config['days']]

            response.success = True
            schedule_info = {
                'enabled': self.schedule_enabled,
                'start_hour': self.schedule_start_hour,
                'end_hour': self.schedule_end_hour,
                'days': self.schedule_days,
            }
            response.message = json.dumps(schedule_info)
            self.get_logger().info(f'录制计划已更新: {schedule_info}')
        except Exception as e:
            response.success = False
            response.message = f'Invalid schedule config: {e}'
            self.get_logger().error(f'录制计划配置失败: {e}')
        return response

    def schedule_check_callback(self):
        # 定时检查录制计划，在计划时间窗口内自动开始/停止录制
        if not self.schedule_enabled:
            return

        now = datetime.now()
        current_hour = now.hour
        current_day = now.weekday()

        in_schedule = (
            current_day in self.schedule_days and
            self.schedule_start_hour <= current_hour < self.schedule_end_hour
        )

        if in_schedule and not self.is_recording:
            self.get_logger().info('录制计划时间到达，自动开始录制')
            self.start_continuous_recording()
        elif not in_schedule and self.is_recording and not self.event_recording_active:
            self.get_logger().info('录制计划时间结束，自动停止录制')
            self.stop_recording()

    def storage_check_callback(self):
        # 存储管理定时检查 - 磁盘使用率超过阈值时自动清理
        try:
            stat = os.statvfs(self.record_path)
            total_gb = (stat.f_blocks * stat.f_frsize) / (1024 ** 3)
            free_gb = (stat.f_bavail * stat.f_frsize) / (1024 ** 3)
            used_percent = ((total_gb - free_gb) / total_gb * 100) if total_gb > 0 else 0

            if used_percent > self.disk_usage_threshold:
                self.get_logger().warn(
                    f'磁盘使用率 {used_percent:.1f}% 超过阈值 {self.disk_usage_threshold}%，开始清理'
                )
                self.cleanup_old_recordings()

            if free_gb < self.min_free_space_gb:
                self.get_logger().warn(
                    f'剩余空间 {free_gb:.2f}GB 低于最小阈值 {self.min_free_space_gb}GB，开始清理'
                )
                self.cleanup_old_recordings()
        except Exception as e:
            self.get_logger().error(f'存储检查失败: {e}')

    def check_storage_and_cleanup(self):
        try:
            stat = os.statvfs(self.record_path)
            free_gb = (stat.f_bavail * stat.f_frsize) / (1024 ** 3)

            if free_gb < self.min_free_space_gb:
                self.cleanup_old_recordings()
        except Exception as e:
            self.get_logger().error(f'Error checking storage: {e}')

    def cleanup_old_recordings(self):
        # 清理旧录制文件，同时清理对应的元数据
        videos_dir = os.path.join(self.record_path, 'videos')
        snapshots_dir = os.path.join(self.record_path, 'snapshots')
        metadata_dir = os.path.join(self.record_path, 'metadata')

        for directory in [videos_dir, snapshots_dir, metadata_dir]:
            if not os.path.exists(directory):
                continue

            files = []
            for f in os.listdir(directory):
                filepath = os.path.join(directory, f)
                if os.path.isfile(filepath):
                    files.append((filepath, os.path.getsize(filepath)))

            files.sort(key=lambda x: os.path.getctime(x[0]))

            total_size = sum(f[1] for f in files)
            max_bytes = self.max_storage_gb * (1024 ** 3)

            while total_size > max_bytes and files:
                oldest_file = files.pop(0)
                try:
                    os.remove(oldest_file[0])
                    total_size -= oldest_file[1]
                    self.get_logger().info(f'Cleaned up old file: {oldest_file[0]}')
                except Exception as e:
                    self.get_logger().error(f'Error removing file: {e}')

    def health_check_callback(self):
        # 录制健康检查 - 验证录制文件完整性
        if not self.is_recording or not self.current_recording_path:
            self.recording_health_status = 'idle'
            return

        try:
            # 检查录制文件是否存在且正在增长
            if os.path.exists(self.current_recording_path):
                file_size = os.path.getsize(self.current_recording_path)
                if file_size == 0:
                    self.recording_health_status = 'warning_empty'
                    self.get_logger().warn('录制文件为空，可能存在写入问题')
                else:
                    self.recording_health_status = 'healthy'
            else:
                self.recording_health_status = 'error_missing'
                self.get_logger().error('录制文件不存在')

            # 检查录制写入器是否正常
            if self.recording_writer is None:
                self.recording_health_status = 'error_no_writer'
                self.get_logger().error('录制写入器未初始化')

            # 检查录制时长是否异常
            if self.recording_start_time:
                duration = time.time() - self.recording_start_time
                if duration > 3600 and self.current_recording_path:
                    # 超过1小时的录制，检查文件大小是否合理
                    if os.path.exists(self.current_recording_path):
                        expected_min_size = duration * 1000  # 粗略估计
                        actual_size = os.path.getsize(self.current_recording_path)
                        if actual_size < expected_min_size * 0.1:
                            self.recording_health_status = 'warning_small'
                            self.get_logger().warn(
                                f'录制文件大小异常偏小: {actual_size} bytes (录制 {duration:.0f} 秒)'
                            )
        except Exception as e:
            self.recording_health_status = 'error'
            self.get_logger().error(f'录制健康检查失败: {e}')

    def publish_status(self, status):
        msg = std_msgs.String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.is_recording = False
        if self.record_thread:
            self.record_thread.join(timeout=2)
        if self.recording_writer:
            self.recording_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
