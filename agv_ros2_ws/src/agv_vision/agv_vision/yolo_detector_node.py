# YOLO目标检测节点模块，订阅相机图像进行实时推理并发布检测结果和标注图像
import threading
import time
from collections import deque

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from ultralytics import YOLO

from agv_interfaces.msg import YoloDetection, YoloResult
from agv_interfaces.srv import SetConfidence, SetModel


# YOLO目标检测ROS2节点，支持模型热切换、置信度调节和自适应帧跳过
class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # 声明ROS2参数：模型路径、置信度阈值、IOU阈值、推理设备、发布频率等
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

        # 获取参数值
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

        # 加载YOLO模型
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model_name = model_path
        self.get_logger().info('YOLO model loaded successfully')

        # 模型预热，避免首次推理延迟
        self._warmup_model()

        self.bridge = CvBridge()
        self._model_lock = threading.Lock()

        # 创建检测结果和标注图像的话题发布者
        self.result_pub = self.create_publisher(YoloResult, 'yolo_result', 10)
        self.annotated_pub = self.create_publisher(Image, 'annotated_image', 10)

        # 订阅相机图像话题
        self.sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # 创建模型切换和置信度调节服务
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

        # 帧率控制和自适应跳帧相关变量
        self._frame_interval = 1.0 / publish_rate if publish_rate > 0 else 0.0
        self._last_frame_time = self.get_clock().now()

        self._frame_counter = 0
        self._inference_times = deque(maxlen=30)
        self._adaptive_skip = self._frame_skip

        # 初始化推理队列和独立推理线程，实现采集与推理解耦
        self._inference_lock = threading.Lock()
        self._inference_queue = deque(maxlen=self._max_inference_queue)
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._inference_thread_active = True
        self._inference_condition = threading.Condition()
        self._inference_thread.start()

        self.get_logger().info('YoloDetectorNode initialized')

    # 模型预热，使用空白图像进行若干次推理以消除首次延迟
    def _warmup_model(self):
        self.get_logger().info(f'Warming up model with {self._warmup_iterations} iteration(s)')
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        for i in range(self._warmup_iterations):
            with self._model_lock:
                self.model.predict(source=dummy, conf=self.confidence_threshold,
                                   iou=self.iou_threshold, device=self.device, verbose=False)
        self.get_logger().info('Model warmup complete')

    # 图像回调，按帧跳过策略和频率控制将帧送入推理队列
    def image_callback(self, msg):
        self._frame_counter += 1
        # 自适应跳帧：跳过部分帧以匹配推理速度
        if self._frame_counter % self._adaptive_skip != 0:
            return

        # 帧率控制：确保不超过配置的发布频率
        now = self.get_clock().now()
        if self._frame_interval > 0.0:
            elapsed = (now - self._last_frame_time).nanoseconds / 1e9
            if elapsed < self._frame_interval:
                return
        self._last_frame_time = now

        # 将图像转换为OpenCV格式并加入推理队列
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        with self._inference_condition:
            # 队列满时丢弃最旧的帧，保持推理实时性
            if len(self._inference_queue) >= self._max_inference_queue:
                self._inference_queue.popleft()
            self._inference_queue.append((cv_image, msg.header))
            self._inference_condition.notify()

    # 推理工作线程主循环，从队列取帧执行YOLO推理并发布结果
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

            # 获取当前模型和参数（加锁保护，防止热切换时冲突）
            with self._model_lock:
                model = self.model
                confidence = self.confidence_threshold
                iou = self.iou_threshold
                device = self.device

            # 执行YOLO推理
            start_time = time.time()
            results = model.predict(
                source=cv_image,
                conf=confidence,
                iou=iou,
                device=device,
                verbose=False
            )
            inference_time = (time.time() - start_time) * 1000.0
            self._inference_times.append(inference_time)

            # 根据推理耗时自适应调整跳帧数
            self._update_adaptive_skip()

            if not results:
                continue

            result = results[0]
            detections = []

            # 解析检测结果，提取类别、置信度和归一化边界框
            if result.boxes is not None:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = model.names.get(cls_id, str(cls_id))

                    # 类别过滤：仅保留指定类别的检测结果
                    if self.classes_filter and class_name not in self.classes_filter:
                        continue

                    # 计算归一化的中心坐标和宽高
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    img_h, img_w = cv_image.shape[:2]
                    x_center = float((x1 + x2) / 2.0 / img_w)
                    y_center = float((y1 + y2) / 2.0 / img_h)
                    width = float((x2 - x1) / img_w)
                    height = float((y2 - y1) / img_h)

                    # 构造检测结果消息
                    det = YoloDetection()
                    det.class_name = class_name
                    det.confidence = conf
                    det.class_id = cls_id
                    det.x_center = x_center
                    det.y_center = y_center
                    det.width = width
                    det.height = height
                    detections.append(det)

            # 发布YOLO检测结果消息
            yolo_result = YoloResult()
            yolo_result.header = header
            yolo_result.model_name = self.model_name
            yolo_result.inference_time = inference_time
            yolo_result.detections = detections

            self.result_pub.publish(yolo_result)

            # 发布带检测标注的图像
            annotated = result.plot()
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                annotated_msg.header = header
                self.annotated_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Annotated image conversion failed: {e}')

    # 根据平均推理耗时自适应调整跳帧数，平衡推理精度和实时性
    def _update_adaptive_skip(self):
        if len(self._inference_times) < 5:
            return
        avg_inference = sum(self._inference_times) / len(self._inference_times)
        target_frame_time = self._frame_interval * 1000.0 if self._frame_interval > 0 else 33.33
        # 推理耗时过长时增加跳帧数
        if avg_inference > target_frame_time * 1.5:
            self._adaptive_skip = min(self._adaptive_skip + 1, max(self._frame_skip * 4, 8))
        # 推理耗时充裕时减少跳帧数
        elif avg_inference < target_frame_time * 0.7 and self._adaptive_skip > self._frame_skip:
            self._adaptive_skip = max(self._frame_skip, self._adaptive_skip - 1)

    # 模型切换服务回调，支持运行时热切换YOLO模型
    def set_model_callback(self, request, response):
        model_path = request.model_path
        self.get_logger().info(f'Switching model to: {model_path}')
        try:
            new_model = YOLO(model_path)
            with self._model_lock:
                self.model = new_model
                self.model_name = model_path
            self._warmup_model()
            response.success = True
            response.message = f'Model switched to {model_path}'
            self.get_logger().info(f'Model switched successfully to {model_path}')
        except Exception as e:
            response.success = False
            response.message = f'Failed to load model: {e}'
            self.get_logger().error(f'Failed to switch model: {e}')
        return response

    # 置信度阈值设置服务回调
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

    # 销毁节点时停止推理线程并等待其结束
    def destroy(self):
        self._inference_thread_active = False
        with self._inference_condition:
            self._inference_condition.notify_all()
        self._inference_thread.join(timeout=5.0)
        super().destroy_node()


# 节点入口函数，初始化ROS2并启动YOLO检测节点
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
