import threading
import time
import json
from collections import deque

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from ultralytics import YOLO

from agv_interfaces.msg import YoloDetection, YoloResult
from agv_interfaces.srv import SetConfidence, SetModel


class CentroidTracker:
    # 简单质心跟踪器 - 跨帧分配跟踪ID

    def __init__(self, max_disappeared=30, max_distance=50):
        self.next_id = 0
        self.objects = {}
        self.disappeared = {}
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def register(self, centroid):
        self.objects[self.next_id] = centroid
        self.disappeared[self.next_id] = 0
        track_id = self.next_id
        self.next_id += 1
        return track_id

    def deregister(self, track_id):
        del self.objects[track_id]
        del self.disappeared[track_id]

    def update(self, centroids):
        results = {}

        if len(centroids) == 0:
            for track_id in list(self.disappeared.keys()):
                self.disappeared[track_id] += 1
                if self.disappeared[track_id] > self.max_disappeared:
                    self.deregister(track_id)
            return results

        if len(self.objects) == 0:
            for centroid in centroids:
                track_id = self.register(centroid)
                results[track_id] = centroid
        else:
            object_ids = list(self.objects.keys())
            object_centroids = list(self.objects.values())

            # 计算距离矩阵
            dist_matrix = np.zeros((len(object_centroids), len(centroids)))
            for i, oc in enumerate(object_centroids):
                for j, c in enumerate(centroids):
                    dist_matrix[i, j] = np.linalg.norm(np.array(oc) - np.array(c))

            # 贪心匹配
            used_rows = set()
            used_cols = set()

            for _ in range(min(len(object_centroids), len(centroids))):
                min_val = float('inf')
                min_row = -1
                min_col = -1
                for i in range(len(object_centroids)):
                    if i in used_rows:
                        continue
                    for j in range(len(centroids)):
                        if j in used_cols:
                            continue
                        if dist_matrix[i, j] < min_val:
                            min_val = dist_matrix[i, j]
                            min_row = i
                            min_col = j

                if min_val > self.max_distance:
                    break

                track_id = object_ids[min_row]
                self.objects[track_id] = centroids[min_col]
                self.disappeared[track_id] = 0
                results[track_id] = centroids[min_col]
                used_rows.add(min_row)
                used_cols.add(min_col)

            # 未匹配的已有对象
            unused_rows = set(range(len(object_centroids))) - used_rows
            for row in unused_rows:
                track_id = object_ids[row]
                self.disappeared[track_id] += 1
                if self.disappeared[track_id] > self.max_disappeared:
                    self.deregister(track_id)

            # 未匹配的新检测
            unused_cols = set(range(len(centroids))) - used_cols
            for col in unused_cols:
                track_id = self.register(centroids[col])
                results[track_id] = centroids[col]

        return results


class ConfidenceSmoother:
    # 置信度时间平滑器 - 减少检测闪烁

    def __init__(self, smoothing_factor=0.3, history_length=5):
        self.smoothing_factor = smoothing_factor
        self.history_length = history_length
        self.confidence_history = {}

    def smooth(self, track_key, raw_confidence):
        if track_key not in self.confidence_history:
            self.confidence_history[track_key] = deque(maxlen=self.history_length)

        self.confidence_history[track_key].append(raw_confidence)

        # 指数移动平均
        history = list(self.confidence_history[track_key])
        smoothed = history[0]
        for conf in history[1:]:
            smoothed = self.smoothing_factor * conf + (1 - self.smoothing_factor) * smoothed

        return smoothed

    def cleanup(self, active_keys):
        # 清理不再活跃的跟踪键
        stale_keys = set(self.confidence_history.keys()) - set(active_keys)
        for key in stale_keys:
            del self.confidence_history[key]


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # 原有参数
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('classes_filter', [])
        self.declare_parameter('frame_skip', 1)
        self.declare_parameter('max_inference_queue', 2)
        self.declare_parameter('warmup_iterations', 3)

        # 多模型支持参数
        self.declare_parameter('model_paths', [])
        self.declare_parameter('active_model', 'yolov8n.pt')

        # 检测跟踪参数
        self.declare_parameter('tracking_enabled', True)
        self.declare_parameter('tracking_max_disappeared', 30)
        self.declare_parameter('tracking_max_distance', 50)

        # 检测过滤参数
        self.declare_parameter('filter_min_area', 0)
        self.declare_parameter('filter_max_area', 999999)
        self.declare_parameter('filter_min_aspect_ratio', 0.0)
        self.declare_parameter('filter_max_aspect_ratio', 999.0)
        self.declare_parameter('filter_movement_direction', '')
        self.declare_parameter('filter_movement_threshold', 5.0)

        # 推理性能分析参数
        self.declare_parameter('profiling_enabled', True)

        # 检测区域配置参数
        self.declare_parameter('detection_zones', '')
        self.declare_parameter('zone_mode', 'include')

        # 置信度平滑参数
        self.declare_parameter('confidence_smoothing_enabled', True)
        self.declare_parameter('confidence_smoothing_factor', 0.3)
        self.declare_parameter('confidence_smoothing_history', 5)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.classes_filter = self.get_parameter('classes_filter').get_parameter_value().string_array_value
        self._frame_skip = self.get_parameter('frame_skip').get_parameter_value().integer_value
        self._max_inference_queue = self.get_parameter('max_inference_queue').get_parameter_value().integer_value
        self._warmup_iterations = self.get_parameter('warmup_iterations').get_parameter_value().integer_value

        # 多模型支持初始化
        model_paths = list(self.get_parameter('model_paths').get_parameter_value().string_array_value)
        active_model = self.get_parameter('active_model').get_parameter_value().string_value
        self.loaded_models = {}
        self.model_names_map = {}

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model_name = model_path
        self.loaded_models[model_path] = self.model
        self.model_names_map[model_path] = self.model.names
        self.get_logger().info('YOLO model loaded successfully')

        # 预加载其他模型
        for mp in model_paths:
            if mp and mp != model_path:
                try:
                    self.loaded_models[mp] = YOLO(mp)
                    self.model_names_map[mp] = self.loaded_models[mp].names
                    self.get_logger().info(f'预加载模型: {mp}')
                except Exception as e:
                    self.get_logger().warn(f'预加载模型失败 {mp}: {e}')

        # 模型预热
        self._warmup_model()

        # 检测跟踪初始化
        self.tracking_enabled = self.get_parameter('tracking_enabled').get_parameter_value().bool_value
        tracking_max_disappeared = self.get_parameter('tracking_max_disappeared').get_parameter_value().integer_value
        tracking_max_distance = self.get_parameter('tracking_max_distance').get_parameter_value().integer_value
        self.tracker = CentroidTracker(
            max_disappeared=tracking_max_disappeared,
            max_distance=tracking_max_distance
        )
        self.track_id_map = {}

        # 检测过滤初始化
        self.filter_min_area = self.get_parameter('filter_min_area').get_parameter_value().integer_value
        self.filter_max_area = self.get_parameter('filter_max_area').get_parameter_value().integer_value
        self.filter_min_aspect_ratio = self.get_parameter('filter_min_aspect_ratio').get_parameter_value().double_value
        self.filter_max_aspect_ratio = self.get_parameter('filter_max_aspect_ratio').get_parameter_value().double_value
        self.filter_movement_direction = self.get_parameter('filter_movement_direction').get_parameter_value().string_value
        self.filter_movement_threshold = self.get_parameter('filter_movement_threshold').get_parameter_value().double_value
        self.prev_centroids = {}

        # 推理性能分析初始化
        self.profiling_enabled = self.get_parameter('profiling_enabled').get_parameter_value().bool_value
        self.preprocess_times = deque(maxlen=30)
        self.inference_times_prof = deque(maxlen=30)
        self.postprocess_times = deque(maxlen=30)

        # 检测区域配置初始化
        detection_zones_str = self.get_parameter('detection_zones').get_parameter_value().string_value
        self.zone_mode = self.get_parameter('zone_mode').get_parameter_value().string_value
        self.detection_zones = []
        if detection_zones_str:
            try:
                self.detection_zones = json.loads(detection_zones_str)
            except Exception:
                self.get_logger().warn('检测区域配置解析失败')

        # 置信度平滑初始化
        self.confidence_smoothing_enabled = self.get_parameter('confidence_smoothing_enabled').get_parameter_value().bool_value
        smoothing_factor = self.get_parameter('confidence_smoothing_factor').get_parameter_value().double_value
        smoothing_history = self.get_parameter('confidence_smoothing_history').get_parameter_value().integer_value
        self.confidence_smoother = ConfidenceSmoother(
            smoothing_factor=smoothing_factor,
            history_length=smoothing_history
        )

        self.bridge = CvBridge()
        self._model_lock = threading.Lock()

        self.result_pub = self.create_publisher(YoloResult, 'yolo_result', 10)
        self.annotated_pub = self.create_publisher(Image, 'annotated_image', 10)

        # 推理性能分析发布器
        self.profiling_pub = self.create_publisher(String, '/yolo/profiling', 10)

        self.sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.set_model_srv = self.create_service(
            SetModel,
            '/yolo/set_model',
            self.set_model_callback
        )

        self.set_confidence_srv = self.create_service(
            SetConfidence,
            '/yolo/set_confidence',
            self.set_confidence_callback
        )

        # 模型导出服务
        self.export_model_srv = self.create_service(
            SetModel,
            '/yolo/export_model',
            self.export_model_callback
        )

        # 检测区域配置服务
        self.set_zones_srv = self.create_service(
            SetModel,
            '/yolo/set_zones',
            self.set_zones_callback
        )

        # 检测过滤配置服务
        self.set_filter_srv = self.create_service(
            SetConfidence,
            '/yolo/set_filter',
            self.set_filter_callback
        )

        self._frame_interval = 1.0 / publish_rate if publish_rate > 0 else 0.0
        self._last_frame_time = self.get_clock().now()

        self._frame_counter = 0
        self._inference_times = deque(maxlen=30)
        self._adaptive_skip = self._frame_skip

        self._inference_lock = threading.Lock()
        self._inference_queue = deque(maxlen=self._max_inference_queue)
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._inference_thread_active = True
        self._inference_condition = threading.Condition()
        self._inference_thread.start()

        # 性能分析定时发布
        if self.profiling_enabled:
            self.profiling_timer = self.create_timer(2.0, self.publish_profiling_stats)

        self.get_logger().info('YoloDetectorNode initialized')

    def _warmup_model(self):
        # 模型预热 - 运行虚拟推理以预热模型
        self.get_logger().info(f'Warming up model with {self._warmup_iterations} iteration(s)')
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        for i in range(self._warmup_iterations):
            with self._model_lock:
                self.model.predict(source=dummy, conf=self.confidence_threshold,
                                   iou=self.iou_threshold, device=self.device, verbose=False)
        self.get_logger().info('Model warmup complete')

    def image_callback(self, msg):
        self._frame_counter += 1
        if self._frame_counter % self._adaptive_skip != 0:
            return

        now = self.get_clock().now()
        if self._frame_interval > 0.0:
            elapsed = (now - self._last_frame_time).nanoseconds / 1e9
            if elapsed < self._frame_interval:
                return
        self._last_frame_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        with self._inference_condition:
            if len(self._inference_queue) >= self._max_inference_queue:
                self._inference_queue.popleft()
            self._inference_queue.append((cv_image, msg.header))
            self._inference_condition.notify()

    def _inference_loop(self):
        while self._inference_thread_active:
            with self._inference_condition:
                while not self._inference_queue and self._inference_thread_active:
                    self._inference_condition.wait(timeout=0.1)
                if not self._inference_thread_active:
                    break
                if not self._inference_queue:
                    continue
                item = self._inference_queue.popleft()

            cv_image, header = item

            with self._model_lock:
                model = self.model
                confidence = self.confidence_threshold
                iou = self.iou_threshold
                device = self.device

            # 推理性能分析 - 预处理计时
            preprocess_start = time.time()

            inference_start = time.time()
            results = model.predict(
                source=cv_image,
                conf=confidence,
                iou=iou,
                device=device,
                verbose=False
            )
            inference_end = time.time()
            inference_time = (inference_end - inference_start) * 1000.0
            self._inference_times.append(inference_time)

            if self.profiling_enabled:
                self.inference_times_prof.append(inference_time)

            # 推理性能分析 - 后处理计时
            postprocess_start = time.time()

            self._update_adaptive_skip()

            if not results:
                continue

            result = results[0]
            detections = []
            centroids = []
            detection_keys = []

            if result.boxes is not None:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = model.names.get(cls_id, str(cls_id))

                    if self.classes_filter and class_name not in self.classes_filter:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    img_h, img_w = cv_image.shape[:2]

                    # 检测区域过滤
                    if not self._check_detection_zone(x1, y1, x2, y2, img_w, img_h):
                        continue

                    # 检测大小和宽高比过滤
                    box_w = x2 - x1
                    box_h = y2 - y1
                    box_area = box_w * box_h
                    aspect_ratio = box_w / max(box_h, 1)

                    if box_area < self.filter_min_area or box_area > self.filter_max_area:
                        continue
                    if aspect_ratio < self.filter_min_aspect_ratio or aspect_ratio > self.filter_max_aspect_ratio:
                        continue

                    x_center = float((x1 + x2) / 2.0 / img_w)
                    y_center = float((y1 + y2) / 2.0 / img_h)
                    width = float((x2 - x1) / img_w)
                    height = float((y2 - y1) / img_h)

                    # 运动方向过滤
                    centroid_px = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    if not self._check_movement_direction(centroid_px, class_name, cls_id):
                        continue

                    centroids.append(centroid_px)
                    detection_keys.append(f'{class_name}_{cls_id}')

                    # 置信度平滑
                    smooth_key = f'{class_name}_{cls_id}_{len(detections)}'
                    if self.confidence_smoothing_enabled:
                        conf = self.confidence_smoother.smooth(smooth_key, conf)

                    det = YoloDetection()
                    det.class_name = class_name
                    det.confidence = conf
                    det.class_id = cls_id
                    det.x_center = x_center
                    det.y_center = y_center
                    det.width = width
                    det.height = height
                    detections.append(det)

            # 目标跟踪
            track_results = {}
            if self.tracking_enabled and centroids:
                track_results = self.tracker.update(centroids)

                # 将跟踪ID分配给检测
                centroid_to_track = {}
                for track_id, centroid in track_results.items():
                    centroid_tuple = tuple(centroid) if isinstance(centroid, (list, np.ndarray)) else centroid
                    centroid_to_track[centroid_tuple] = track_id

                for i, det in enumerate(detections):
                    if i < len(centroids):
                        c = centroids[i]
                        c_tuple = tuple(c) if isinstance(c, (list, np.ndarray)) else c
                        if c_tuple in centroid_to_track:
                            det.track_id = centroid_to_track[c_tuple]

            # 清理置信度平滑器中不再活跃的键
            if self.confidence_smoothing_enabled:
                self.confidence_smoother.cleanup(set(detection_keys))

            # 更新前帧质心用于运动方向过滤
            self.prev_centroids = {}
            for i, det in enumerate(detections):
                if i < len(centroids):
                    key = f'{det.class_name}_{det.class_id}'
                    self.prev_centroids[key] = centroids[i]

            yolo_result = YoloResult()
            yolo_result.header = header
            yolo_result.model_name = self.model_name
            yolo_result.inference_time = inference_time
            yolo_result.detections = detections

            self.result_pub.publish(yolo_result)

            # 推理性能分析 - 后处理计时
            if self.profiling_enabled:
                postprocess_time = (time.time() - postprocess_start) * 1000.0
                preprocess_time = (postprocess_start - inference_end) * 1000.0
                self.preprocess_times.append(preprocess_time)
                self.postprocess_times.append(postprocess_time)

            annotated = result.plot()
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                annotated_msg.header = header
                self.annotated_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Annotated image conversion failed: {e}')

    def _check_detection_zone(self, x1, y1, x2, y2, img_w, img_h):
        # 检测区域过滤 - 判断检测框中心是否在有效区域内
        if not self.detection_zones:
            return True

        cx = (x1 + x2) / 2.0 / img_w
        cy = (y1 + y2) / 2.0 / img_h

        in_zone = False
        for zone in self.detection_zones:
            zx1 = zone.get('x1', 0.0)
            zy1 = zone.get('y1', 0.0)
            zx2 = zone.get('x2', 1.0)
            zy2 = zone.get('y2', 1.0)
            if zx1 <= cx <= zx2 and zy1 <= cy <= zy2:
                in_zone = True
                break

        if self.zone_mode == 'include':
            return in_zone
        else:
            return not in_zone

    def _check_movement_direction(self, centroid, class_name, cls_id):
        # 运动方向过滤 - 根据配置的方向过滤检测
        if not self.filter_movement_direction:
            return True

        key = f'{class_name}_{cls_id}'
        if key not in self.prev_centroids:
            return True

        prev = self.prev_centroids[key]
        dx = centroid[0] - prev[0]
        dy = centroid[1] - prev[1]
        distance = (dx**2 + dy**2) ** 0.5

        if distance < self.filter_movement_threshold:
            return True

        direction = self.filter_movement_direction.lower()
        if direction == 'left' and dx < -self.filter_movement_threshold:
            return True
        elif direction == 'right' and dx > self.filter_movement_threshold:
            return True
        elif direction == 'up' and dy < -self.filter_movement_threshold:
            return True
        elif direction == 'down' and dy > self.filter_movement_threshold:
            return True
        elif direction == 'horizontal' and abs(dx) > self.filter_movement_threshold:
            return True
        elif direction == 'vertical' and abs(dy) > self.filter_movement_threshold:
            return True

        return False

    def publish_profiling_stats(self):
        # 发布推理性能分析统计
        if not self.profiling_enabled:
            return

        stats = {
            'model_name': self.model_name,
            'inference_avg_ms': round(sum(self.inference_times_prof) / max(len(self.inference_times_prof), 1), 2),
            'inference_max_ms': round(max(self.inference_times_prof) if self.inference_times_prof else 0, 2),
            'inference_min_ms': round(min(self.inference_times_prof) if self.inference_times_prof else 0, 2),
            'preprocess_avg_ms': round(sum(self.preprocess_times) / max(len(self.preprocess_times), 1), 2),
            'postprocess_avg_ms': round(sum(self.postprocess_times) / max(len(self.postprocess_times), 1), 2),
            'adaptive_skip': self._adaptive_skip,
            'device': self.device,
            'tracking_enabled': self.tracking_enabled,
            'active_tracks': len(self.tracker.objects) if self.tracking_enabled else 0,
        }

        msg = String()
        msg.data = json.dumps(stats)
        self.profiling_pub.publish(msg)

    def _update_adaptive_skip(self):
        if len(self._inference_times) < 5:
            return
        avg_inference = sum(self._inference_times) / len(self._inference_times)
        target_frame_time = self._frame_interval * 1000.0 if self._frame_interval > 0 else 33.33
        if avg_inference > target_frame_time * 1.5:
            self._adaptive_skip = min(self._adaptive_skip + 1, max(self._frame_skip * 4, 8))
        elif avg_inference < target_frame_time * 0.7 and self._adaptive_skip > self._frame_skip:
            self._adaptive_skip = max(self._frame_skip, self._adaptive_skip - 1)

    def set_model_callback(self, request, response):
        model_path = request.model_path
        self.get_logger().info(f'Switching model to: {model_path}')
        try:
            # 检查模型是否已预加载
            if model_path in self.loaded_models:
                with self._model_lock:
                    self.model = self.loaded_models[model_path]
                    self.model_name = model_path
                self.get_logger().info(f'切换到已预加载的模型: {model_path}')
            else:
                new_model = YOLO(model_path)
                with self._model_lock:
                    self.model = new_model
                    self.model_name = model_path
                    self.loaded_models[model_path] = new_model
                    self.model_names_map[model_path] = new_model.names
                self.get_logger().info(f'加载新模型: {model_path}')

            self._warmup_model()
            response.success = True
            response.message = f'Model switched to {model_path}'
            self.get_logger().info(f'Model switched successfully to {model_path}')
        except Exception as e:
            response.success = False
            response.message = f'Failed to load model: {e}'
            self.get_logger().error(f'Failed to switch model: {e}')
        return response

    def set_confidence_callback(self, request, response):
        confidence = request.confidence
        if 0.0 <= confidence <= 1.0:
            with self._model_lock:
                self.confidence_threshold = confidence
            response.success = True
            response.message = f'Confidence threshold set to {confidence}'
            self.get_logger().info(f'Confidence threshold set to {confidence}')
        else:
            response.success = False
            response.message = 'Confidence must be between 0.0 and 1.0'
            self.get_logger().warn(f'Invalid confidence value: {confidence}')
        return response

    def export_model_callback(self, request, response):
        # 模型导出服务 - 导出当前模型到ONNX/TensorRT格式
        model_path = request.model_path
        self.get_logger().info(f'导出模型: {model_path}')

        try:
            with self._model_lock:
                export_format = 'onnx'
                if model_path.endswith('.engine') or model_path.endswith('.trt'):
                    export_format = 'engine'
                elif model_path.endswith('.onnx'):
                    export_format = 'onnx'

                export_path = self.model.export(format=export_format)
                response.success = True
                response.message = f'Model exported to: {export_path}'
                self.get_logger().info(f'模型导出成功: {export_path}')
        except Exception as e:
            response.success = False
            response.message = f'Export failed: {e}'
            self.get_logger().error(f'模型导出失败: {e}')

        return response

    def set_zones_callback(self, request, response):
        # 检测区域配置服务
        try:
            config = json.loads(request.model_path)
            if 'zones' in config:
                self.detection_zones = config['zones']
            if 'mode' in config:
                self.zone_mode = config['mode']

            response.success = True
            zone_info = {
                'zones': self.detection_zones,
                'mode': self.zone_mode,
            }
            response.message = json.dumps(zone_info)
            self.get_logger().info(f'检测区域配置已更新: {zone_info}')
        except Exception as e:
            response.success = False
            response.message = f'Invalid zone config: {e}'
            self.get_logger().error(f'检测区域配置失败: {e}')
        return response

    def set_filter_callback(self, request, response):
        # 检测过滤配置服务
        try:
            config = json.loads(str(request.confidence))
            if 'min_area' in config:
                self.filter_min_area = int(config['min_area'])
            if 'max_area' in config:
                self.filter_max_area = int(config['max_area'])
            if 'min_aspect_ratio' in config:
                self.filter_min_aspect_ratio = float(config['min_aspect_ratio'])
            if 'max_aspect_ratio' in config:
                self.filter_max_aspect_ratio = float(config['max_aspect_ratio'])
            if 'movement_direction' in config:
                self.filter_movement_direction = config['movement_direction']
            if 'movement_threshold' in config:
                self.filter_movement_threshold = float(config['movement_threshold'])

            response.success = True
            filter_info = {
                'min_area': self.filter_min_area,
                'max_area': self.filter_max_area,
                'min_aspect_ratio': self.filter_min_aspect_ratio,
                'max_aspect_ratio': self.filter_max_aspect_ratio,
                'movement_direction': self.filter_movement_direction,
                'movement_threshold': self.filter_movement_threshold,
            }
            response.message = json.dumps(filter_info)
            self.get_logger().info(f'检测过滤配置已更新: {filter_info}')
        except Exception as e:
            response.success = False
            response.message = f'Invalid filter config: {e}'
            self.get_logger().error(f'检测过滤配置失败: {e}')
        return response

    def destroy(self):
        self._inference_thread_active = False
        with self._inference_condition:
            self._inference_condition.notify_all()
        self._inference_thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
