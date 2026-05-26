import time

import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera')

        self.declare_parameter('camera_device', 0)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('use_rtsp', False)
        self.declare_parameter('rtsp_url', '')

        camera_device = self.get_parameter('camera_device').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        use_rtsp = self.get_parameter('use_rtsp').get_parameter_value().bool_value
        rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)

        if use_rtsp and rtsp_url:
            self.get_logger().info(f'Opening RTSP stream: {rtsp_url}')
            self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        else:
            self.get_logger().info(f'Opening camera device: {camera_device}')
            self.cap = cv2.VideoCapture(camera_device)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            self.get_logger().info(
                f'Camera opened: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                f'{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}'
            )
        else:
            self.get_logger().error('Failed to open camera')

        self._frame_interval = 1.0 / self.fps if self.fps > 0 else 0.0
        self._running = True

        self.timer = self.create_timer(self._frame_interval, self.timer_callback)
        self.get_logger().info(f'CameraNode initialized at {self.fps} fps')

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().warn('Camera not open, attempting to reconnect...')
            self._try_reconnect()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame, attempting to reconnect...')
            self._try_reconnect()
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera'
            self.publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert frame: {e}')

    def _try_reconnect(self):
        if self.cap.isOpened():
            self.cap.release()

        use_rtsp = self.get_parameter('use_rtsp').get_parameter_value().bool_value
        rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        camera_device = self.get_parameter('camera_device').get_parameter_value().integer_value

        if use_rtsp and rtsp_url:
            self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        else:
            self.cap = cv2.VideoCapture(camera_device)

        if self.cap.isOpened():
            self.get_logger().info('Camera reconnected successfully')
        else:
            self.get_logger().error('Camera reconnection failed')

    def destroy_node(self):
        self._running = False
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
