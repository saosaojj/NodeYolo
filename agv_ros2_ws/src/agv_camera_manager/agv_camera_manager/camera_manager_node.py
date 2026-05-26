# 摄像头管理节点模块，负责摄像头设备的初始化、图像采集和配置更新
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from agv_interfaces.msg import CameraConfig, SystemConfig


# 摄像头管理节点类，管理摄像头设备并定时发布图像帧
class CameraManagerNode(Node):

    def __init__(self):
        super().__init__('camera_manager')

        # 声明摄像头相关参数
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

        # 获取参数值
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

        # OpenCV与ROS图像消息转换桥接器
        self.bridge = CvBridge()
        # 创建图像发布者
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        # 订阅系统配置话题，用于动态更新摄像头参数
        self.config_subscriber = self.create_subscription(
            SystemConfig,
            '/system_config',
            self.config_callback,
            10
        )

        self.cap = None
        # 初始化摄像头设备
        self._init_camera()

        # 计算帧间隔并创建定时器
        self._frame_interval = 1.0 / self.fps if self.fps > 0 else 0.0
        self._running = True

        self.timer = self.create_timer(self._frame_interval, self.timer_callback)
        self.get_logger().info(f'CameraManagerNode initialized at {self.fps} fps')

    # 初始化摄像头设备，支持本地设备和RTSP流
    def _init_camera(self):
        if self.use_rtsp and self.rtsp_url:
            # 使用RTSP流作为视频源
            self.get_logger().info(f'Opening RTSP stream: {self.rtsp_url}')
            self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        else:
            # 使用本地摄像头设备
            self.get_logger().info(f'Opening camera device: {self.device_path}')
            self.cap = cv2.VideoCapture(self.device_path)

        if self.cap.isOpened():
            # 设置摄像头分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.get_logger().info(
                f'Camera opened: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                f'{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}'
            )
        else:
            self.get_logger().error('Failed to open camera')

    # 系统配置回调，匹配当前摄像头ID并更新配置
    def config_callback(self, msg):
        for cam_cfg in msg.camera_configs:
            if cam_cfg.camera_id == self.camera_id:
                self._update_config(cam_cfg)
                break

    # 根据新的配置更新摄像头参数，部分参数变更需要重启摄像头
    def _update_config(self, config):
        restart_needed = False
        # 设备路径变更需要重启
        if config.device_path != self.device_path:
            self.device_path = config.device_path
            restart_needed = True
        # 分辨率变更需要重启
        if config.width != self.width:
            self.width = config.width
            restart_needed = True
        if config.height != self.height:
            self.height = config.height
            restart_needed = True
        # 帧率变更需要更新定时器周期
        if config.fps != self.fps:
            self.fps = config.fps
            self._frame_interval = 1.0 / self.fps if self.fps > 0 else 0.0
            self.timer.timer_period_ns = int(self._frame_interval * 1e9)
        # 更新其他无需重启的参数
        self.format = config.format
        self.exposure_mode = config.exposure_mode
        self.exposure = config.exposure
        self.gain = config.gain
        self.auto_exposure = config.auto_exposure
        self.enabled = config.enabled

        # 需要重启时，释放旧设备并重新初始化
        if restart_needed and self.cap:
            self.cap.release()
            self._init_camera()

    # 定时器回调，采集图像帧并发布
    def timer_callback(self):
        # 摄像头未启用时跳过采集
        if not self.enabled:
            return

        # 摄像头未打开时尝试重连
        if not self.cap or not self.cap.isOpened():
            self.get_logger().warn('Camera not open, attempting to reconnect...')
            self._try_reconnect()
            return

        # 读取一帧图像
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame, attempting to reconnect...')
            self._try_reconnect()
            return

        # 将OpenCV图像转换为ROS消息并发布
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.format)
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.camera_id
            self.publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert frame: {e}')

    # 尝试重新连接摄像头
    def _try_reconnect(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self._init_camera()

    # 销毁节点时释放摄像头资源
    def destroy_node(self):
        self._running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


# 主函数，初始化ROS2并运行摄像头管理节点
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
