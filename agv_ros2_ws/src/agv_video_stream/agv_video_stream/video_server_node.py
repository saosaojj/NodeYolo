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
import subprocess
import json
from collections import deque
from datetime import datetime


class VideoServerNode(Node):
    def __init__(self):
        super().__init__('video_server_node')

        # 原有参数
        self.declare_parameter('port', 8554)
        self.declare_parameter('stream_quality', 85)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        self.declare_parameter('rtsp_path', 'stream')

        # 自适应码率参数
        self.declare_parameter('adaptive_bitrate_enabled', True)
        self.declare_parameter('bitrate_min', 500000)
        self.declare_parameter('bitrate_max', 4000000)
        self.declare_parameter('bitrate_default', 2000000)
        self.declare_parameter('bitrate_adjust_interval_sec', 2.0)

        # 多客户端管理参数
        self.declare_parameter('max_clients', 10)

        # ROI流参数
        self.declare_parameter('roi_enabled', False)
        self.declare_parameter('roi_x', 0)
        self.declare_parameter('roi_y', 0)
        self.declare_parameter('roi_width', 640)
        self.declare_parameter('roi_height', 480)

        # 叠加层配置参数
        self.declare_parameter('overlay_color_r', 0)
        self.declare_parameter('overlay_color_g', 255)
        self.declare_parameter('overlay_color_b', 0)
        self.declare_parameter('overlay_font_scale', 0.5)
        self.declare_parameter('overlay_thickness', 2)
        self.declare_parameter('overlay_box_style', 'rectangle')

        self.port = self.get_parameter('port').value
        self.stream_quality = self.get_parameter('stream_quality').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.resolution_width = self.get_parameter('resolution_width').value
        self.resolution_height = self.get_parameter('resolution_height').value
        self.rtsp_path = self.get_parameter('rtsp_path').value

        # 自适应码率初始化
        self.adaptive_bitrate_enabled = self.get_parameter('adaptive_bitrate_enabled').value
        self.bitrate_min = self.get_parameter('bitrate_min').value
        self.bitrate_max = self.get_parameter('bitrate_max').value
        self.current_bitrate = self.get_parameter('bitrate_default').value
        self.bitrate_adjust_interval = self.get_parameter('bitrate_adjust_interval_sec').value
        self.last_bitrate_adjust_time = time.time()
        self.frame_send_times = deque(maxlen=60)
        self.network_quality_score = 1.0

        # 多客户端管理初始化
        self.max_clients = self.get_parameter('max_clients').value
        self.connected_clients = {}
        self.client_lock = threading.Lock()

        # 流统计信息
        self.stream_fps = 0.0
        self.frames_sent_count = 0
        self.stats_last_time = time.time()
        self.stats_frame_count = 0

        # ROI流初始化
        self.roi_enabled = self.get_parameter('roi_enabled').value
        self.roi_x = self.get_parameter('roi_x').value
        self.roi_y = self.get_parameter('roi_y').value
        self.roi_width = self.get_parameter('roi_width').value
        self.roi_height = self.get_parameter('roi_height').value

        # 叠加层配置初始化
        self.overlay_config = {
            'color': (
                self.get_parameter('overlay_color_b').value,
                self.get_parameter('overlay_color_g').value,
                self.get_parameter('overlay_color_r').value,
            ),
            'font_scale': self.get_parameter('overlay_font_scale').value,
            'thickness': self.get_parameter('overlay_thickness').value,
            'box_style': self.get_parameter('overlay_box_style').value,
        }

        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=2)
        self.latest_frame = None
        self.latest_yolo = None
        self.is_streaming = False
        self.stream_thread = None
        self.rtsp_server = None

        self.status_pub = self.create_publisher(std_msgs.String, 'rtsp_stream_status', 10)

        # 流统计信息发布器
        self.stream_stats_pub = self.create_publisher(std_msgs.String, 'stream_stats', 10)

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
            self.get_logger().warn('YoloResult topic not available, skipping YOLO subscription')
            self.yolo_sub = None

        self.start_stream_srv = self.create_service(
            std_srvs.Trigger,
            '/video/start_stream',
            self.start_stream_callback
        )

        self.stop_stream_srv = self.create_service(
            std_srvs.Trigger,
            '/video/stop_stream',
            self.stop_stream_callback
        )

        self.set_quality_srv = self.create_service(
            'SetConfidence',
            '/video/set_quality',
            self.set_quality_callback
        )

        # 快照服务 - 捕获单帧并返回
        self.snapshot_srv = self.create_service(
            std_srvs.Trigger,
            '/video/snapshot',
            self.snapshot_callback
        )

        # 叠加层配置服务
        self.set_overlay_srv = self.create_service(
            'SetConfidence',
            '/video/set_overlay',
            self.set_overlay_callback
        )

        # ROI配置服务
        self.set_roi_srv = self.create_service(
            'SetConfidence',
            '/video/set_roi',
            self.set_roi_callback
        )

        # 统计信息定时发布
        self.stats_timer = self.create_timer(1.0, self.publish_stream_stats)

        self.get_logger().info(f'VideoServerNode initialized on port {self.port}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def yolo_callback(self, msg):
        self.latest_yolo = msg

    def overlay_yolo_detections(self, frame):
        if self.latest_yolo is None:
            return frame

        try:
            class_names = self.latest_yolo.class_names
            boxes = self.latest_yolo.boxes
            scores = self.latest_yolo.scores

            color = self.overlay_config['color']
            font_scale = self.overlay_config['font_scale']
            thickness = self.overlay_config['thickness']
            box_style = self.overlay_config['box_style']

            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = [int(b) for b in box]
                class_id = i if i < len(class_names) else 0
                score = scores[i] if i < len(scores) else 0.0
                label = f'{class_names[class_id]}: {score:.2f}'

                # 根据配置的边界框样式绘制
                if box_style == 'rectangle':
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                elif box_style == 'rounded':
                    self._draw_rounded_rect(frame, (x1, y1), (x2, y2), color, thickness, radius=8)
                elif box_style == 'dashed':
                    self._draw_dashed_rect(frame, (x1, y1), (x2, y2), color, thickness)

                cv2.putText(frame, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
        except Exception as e:
            self.get_logger().warn(f'Error overlaying YOLO detections: {e}')

        return frame

    def _draw_rounded_rect(self, frame, pt1, pt2, color, thickness, radius=8):
        # 绘制圆角矩形
        x1, y1 = pt1
        x2, y2 = pt2
        cv2.line(frame, (x1 + radius, y1), (x2 - radius, y1), color, thickness)
        cv2.line(frame, (x1 + radius, y2), (x2 - radius, y2), color, thickness)
        cv2.line(frame, (x1, y1 + radius), (x1, y2 - radius), color, thickness)
        cv2.line(frame, (x2, y1 + radius), (x2, y2 - radius), color, thickness)
        cv2.ellipse(frame, (x1 + radius, y1 + radius), (radius, radius), 180, 0, 90, color, thickness)
        cv2.ellipse(frame, (x2 - radius, y1 + radius), (radius, radius), 270, 0, 90, color, thickness)
        cv2.ellipse(frame, (x1 + radius, y2 - radius), (radius, radius), 90, 0, 90, color, thickness)
        cv2.ellipse(frame, (x2 - radius, y2 - radius), (radius, radius), 0, 0, 90, color, thickness)

    def _draw_dashed_rect(self, frame, pt1, pt2, color, thickness, dash_length=10):
        # 绘制虚线矩形
        x1, y1 = pt1
        x2, y2 = pt2
        for start, end in [
            ((x1, y1), (x2, y1)),
            ((x2, y1), (x2, y2)),
            ((x2, y2), (x1, y2)),
            ((x1, y2), (x1, y1)),
        ]:
            self._draw_dashed_line(frame, start, end, color, thickness, dash_length)

    def _draw_dashed_line(self, frame, pt1, pt2, color, thickness, dash_length=10):
        x1, y1 = pt1
        x2, y2 = pt2
        dist = max(abs(x2 - x1), abs(y2 - y1))
        if dist == 0:
            return
        dashes = dist // (dash_length * 2)
        if dashes == 0:
            cv2.line(frame, pt1, pt2, color, thickness)
            return
        for i in range(dashes + 1):
            sx = int(x1 + (x2 - x1) * (2 * i * dash_length) / dist)
            sy = int(y1 + (y2 - y1) * (2 * i * dash_length) / dist)
            ex = int(x1 + (x2 - x1) * min((2 * i + 1) * dash_length, dist) / dist)
            ey = int(y1 + (y2 - y1) * min((2 * i + 1) * dash_length, dist) / dist)
            cv2.line(frame, (sx, sy), (ex, ey), color, thickness)

    def apply_roi(self, frame):
        # 应用感兴趣区域裁剪
        if not self.roi_enabled:
            return frame
        h, w = frame.shape[:2]
        x1 = max(0, min(self.roi_x, w))
        y1 = max(0, min(self.roi_y, h))
        x2 = max(0, min(self.roi_x + self.roi_width, w))
        y2 = max(0, min(self.roi_y + self.roi_height, h))
        if x2 <= x1 or y2 <= y1:
            return frame
        return frame[y1:y2, x1:x2]

    def adjust_bitrate_adaptive(self):
        # 自适应码率调整 - 根据网络质量评分调整码率
        if not self.adaptive_bitrate_enabled:
            return

        now = time.time()
        if now - self.last_bitrate_adjust_time < self.bitrate_adjust_interval:
            return
        self.last_bitrate_adjust_time = now

        # 根据帧发送延迟评估网络质量
        if len(self.frame_send_times) >= 5:
            recent_times = list(self.frame_send_times)[-10:]
            if len(recent_times) >= 2:
                intervals = [recent_times[i+1] - recent_times[i] for i in range(len(recent_times)-1)]
                avg_interval = sum(intervals) / len(intervals)
                target_interval = 1.0 / self.frame_rate
                # 网络质量评分：实际间隔与目标间隔的比值
                if avg_interval > 0:
                    ratio = target_interval / avg_interval
                    self.network_quality_score = max(0.1, min(1.0, ratio))
                else:
                    self.network_quality_score = 1.0

                # 根据网络质量调整码率
                target_bitrate = int(self.bitrate_max * self.network_quality_score)
                target_bitrate = max(self.bitrate_min, min(self.bitrate_max, target_bitrate))

                # 平滑调整，避免码率跳变
                adjustment = 0.3
                self.current_bitrate = int(
                    self.current_bitrate * (1 - adjustment) + target_bitrate * adjustment
                )
                self.current_bitrate = max(self.bitrate_min, min(self.bitrate_max, self.current_bitrate))

    def register_client(self, client_id):
        # 注册新客户端连接
        with self.client_lock:
            if len(self.connected_clients) >= self.max_clients:
                self.get_logger().warn(f'达到最大客户端数限制: {self.max_clients}，拒绝客户端 {client_id}')
                return False
            self.connected_clients[client_id] = {
                'connect_time': time.time(),
                'last_active': time.time(),
            }
            self.get_logger().info(f'客户端 {client_id} 已连接，当前客户端数: {len(self.connected_clients)}')
            return True

    def unregister_client(self, client_id):
        # 注销客户端连接
        with self.client_lock:
            if client_id in self.connected_clients:
                del self.connected_clients[client_id]
                self.get_logger().info(f'客户端 {client_id} 已断开，当前客户端数: {len(self.connected_clients)}')

    def update_client_activity(self, client_id):
        # 更新客户端活跃时间
        with self.client_lock:
            if client_id in self.connected_clients:
                self.connected_clients[client_id]['last_active'] = time.time()

    def cleanup_inactive_clients(self, timeout=120.0):
        # 清理不活跃的客户端
        now = time.time()
        with self.client_lock:
            inactive = [cid for cid, info in self.connected_clients.items()
                       if now - info['last_active'] > timeout]
            for cid in inactive:
                del self.connected_clients[cid]
                self.get_logger().info(f'清理不活跃客户端: {cid}')

    def publish_stream_stats(self):
        # 发布流统计信息
        now = time.time()
        elapsed = now - self.stats_last_time
        if elapsed > 0:
            self.stream_fps = self.stats_frame_count / elapsed
        self.stats_frame_count = 0
        self.stats_last_time = now

        with self.client_lock:
            client_count = len(self.connected_clients)

        stats = {
            'fps': round(self.stream_fps, 2),
            'resolution': f'{self.resolution_width}x{self.resolution_height}',
            'client_count': client_count,
            'bitrate': self.current_bitrate,
            'network_quality': round(self.network_quality_score, 3),
            'roi_enabled': self.roi_enabled,
            'streaming': self.is_streaming,
        }

        msg = std_msgs.String()
        msg.data = json.dumps(stats)
        self.stream_stats_pub.publish(msg)

    def snapshot_callback(self, request, response):
        # 快照服务 - 捕获当前帧并保存
        if self.latest_frame is None:
            response.success = False
            response.message = 'No frame available'
            return response

        try:
            frame = self.latest_frame.copy()
            frame = self.overlay_yolo_detections(frame)

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'/tmp/snapshot_{timestamp}.jpg'
            cv2.imwrite(filename, frame)

            response.success = True
            response.message = filename
            self.get_logger().info(f'快照已保存: {filename}')
        except Exception as e:
            response.success = False
            response.message = f'Snapshot failed: {e}'
            self.get_logger().error(f'快照捕获失败: {e}')

        return response

    def set_overlay_callback(self, request, response):
        # 设置叠加层配置（通过confidence字段传递JSON编码的配置）
        try:
            config = json.loads(str(request.confidence))
            if 'color_r' in config:
                self.overlay_config['color'] = (
                    int(config.get('color_b', self.overlay_config['color'][0])),
                    int(config.get('color_g', self.overlay_config['color'][1])),
                    int(config.get('color_r', self.overlay_config['color'][2])),
                )
            if 'font_scale' in config:
                self.overlay_config['font_scale'] = float(config['font_scale'])
            if 'thickness' in config:
                self.overlay_config['thickness'] = int(config['thickness'])
            if 'box_style' in config:
                self.overlay_config['box_style'] = config['box_style']

            response.success = True
            response.message = json.dumps(self.overlay_config, default=str)
            self.get_logger().info(f'叠加层配置已更新: {self.overlay_config}')
        except Exception as e:
            response.success = False
            response.message = f'Invalid overlay config: {e}'
            self.get_logger().error(f'叠加层配置更新失败: {e}')

        return response

    def set_roi_callback(self, request, response):
        # 设置ROI区域（通过confidence字段传递JSON编码的ROI参数）
        try:
            config = json.loads(str(request.confidence))
            if 'enabled' in config:
                self.roi_enabled = bool(config['enabled'])
            if 'x' in config:
                self.roi_x = int(config['x'])
            if 'y' in config:
                self.roi_y = int(config['y'])
            if 'width' in config:
                self.roi_width = int(config['width'])
            if 'height' in config:
                self.roi_height = int(config['height'])

            response.success = True
            roi_info = {
                'enabled': self.roi_enabled,
                'x': self.roi_x, 'y': self.roi_y,
                'width': self.roi_width, 'height': self.roi_height,
            }
            response.message = json.dumps(roi_info)
            self.get_logger().info(f'ROI配置已更新: {roi_info}')
        except Exception as e:
            response.success = False
            response.message = f'Invalid ROI config: {e}'
            self.get_logger().error(f'ROI配置更新失败: {e}')

        return response

    def stream_worker(self):
        rtsp_url = f'rtsp://localhost:{self.port}/{self.rtsp_path}'
        ffmpeg_cmd = [
            'ffmpeg',
            '-re',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.resolution_width}x{self.resolution_height}',
            '-r', str(self.frame_rate),
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-b:v', str(self.current_bitrate),
            '-pix_fmt', 'yuv420p',
            '-y',
            rtsp_url
        ]

        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdout=subprocess.PIPE
            )

            self.get_logger().info(f'RTSP stream started at {rtsp_url}')
            self.publish_status('streaming')

            while self.is_streaming:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                    frame = self.overlay_yolo_detections(frame)

                    # 应用ROI裁剪
                    frame = self.apply_roi(frame)

                    # 调整到目标分辨率
                    frame = cv2.resize(frame, (self.resolution_width, self.resolution_height))

                    # 自适应码率调整
                    self.adjust_bitrate_adaptive()

                    try:
                        self.ffmpeg_process.stdin.write(frame.tobytes())
                        self.frame_send_times.append(time.time())
                        self.stats_frame_count += 1
                        self.frames_sent_count += 1
                    except Exception:
                        break
                time.sleep(1.0 / self.frame_rate)

                # 定期清理不活跃客户端
                if self.frames_sent_count % 300 == 0:
                    self.cleanup_inactive_clients()

            if self.ffmpeg_process:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait(timeout=5)

        except Exception as e:
            self.get_logger().error(f'Stream worker error: {e}')
            self.publish_status('error')
        finally:
            self.publish_status('stopped')

    def start_stream_callback(self, request, response):
        if not self.is_streaming:
            self.is_streaming = True
            self.stream_thread = threading.Thread(target=self.stream_worker)
            self.stream_thread.daemon = True
            self.stream_thread.start()
            response.success = True
            response.message = 'Stream started'
            self.get_logger().info('Stream started')
        else:
            response.success = True
            response.message = 'Stream already running'

        return response

    def stop_stream_callback(self, request, response):
        if self.is_streaming:
            self.is_streaming = False
            if self.stream_thread:
                self.stream_thread.join(timeout=2)
            if hasattr(self, 'ffmpeg_process') and self.ffmpeg_process:
                self.ffmpeg_process.terminate()
            response.success = True
            response.message = 'Stream stopped'
            self.get_logger().info('Stream stopped')
        else:
            response.success = True
            response.message = 'Stream not running'

        return response

    def set_quality_callback(self, request, response):
        self.stream_quality = int(request.confidence)
        # 根据质量设置调整码率范围
        quality_ratio = self.stream_quality / 100.0
        self.current_bitrate = int(self.bitrate_min +
            (self.bitrate_max - self.bitrate_min) * quality_ratio)
        response.success = True
        response.message = f'Quality set to {self.stream_quality}, bitrate: {self.current_bitrate}'
        self.get_logger().info(f'Quality set to {self.stream_quality}, bitrate: {self.current_bitrate}')
        return response

    def publish_status(self, status):
        msg = std_msgs.String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.is_streaming = False
        if self.stream_thread:
            self.stream_thread.join(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
