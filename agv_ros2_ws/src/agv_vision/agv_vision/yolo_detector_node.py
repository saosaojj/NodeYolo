import threading

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


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('classes_filter', [])

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.classes_filter = self.get_parameter('classes_filter').get_parameter_value().string_array_value

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model_name = model_path
        self.get_logger().info('YOLO model loaded successfully')

        self.bridge = CvBridge()
        self._model_lock = threading.Lock()

        self.result_pub = self.create_publisher(YoloResult, 'yolo_result', 10)
        self.annotated_pub = self.create_publisher(Image, 'annotated_image', 10)

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

        self._frame_interval = 1.0 / publish_rate if publish_rate > 0 else 0.0
        self._last_frame_time = self.get_clock().now()

        self.get_logger().info('YoloDetectorNode initialized')

    def image_callback(self, msg):
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

        with self._model_lock:
            model = self.model
            confidence = self.confidence_threshold
            iou = self.iou_threshold
            device = self.device

        results = model.predict(
            source=cv_image,
            conf=confidence,
            iou=iou,
            device=device,
            verbose=False
        )

        if not results:
            return

        result = results[0]
        detections = []

        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = model.names.get(cls_id, str(cls_id))

                if self.classes_filter and class_name not in self.classes_filter:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                img_h, img_w = cv_image.shape[:2]
                x_center = float((x1 + x2) / 2.0 / img_w)
                y_center = float((y1 + y2) / 2.0 / img_h)
                width = float((x2 - x1) / img_w)
                height = float((y2 - y1) / img_h)

                det = YoloDetection()
                det.class_name = class_name
                det.confidence = conf
                det.class_id = cls_id
                det.x_center = x_center
                det.y_center = y_center
                det.width = width
                det.height = height
                detections.append(det)

        yolo_result = YoloResult()
        yolo_result.header = msg.header
        yolo_result.model_name = self.model_name
        yolo_result.inference_time = float(result.speed.get('inference', 0.0)) if result.speed else 0.0
        yolo_result.detections = detections

        self.result_pub.publish(yolo_result)

        annotated = result.plot()
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Annotated image conversion failed: {e}')

    def set_model_callback(self, request, response):
        model_path = request.model_path
        self.get_logger().info(f'Switching model to: {model_path}')
        try:
            new_model = YOLO(model_path)
            with self._model_lock:
                self.model = new_model
                self.model_name = model_path
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


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
