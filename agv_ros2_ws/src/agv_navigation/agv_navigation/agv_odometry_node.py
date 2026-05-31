import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from agv_interfaces.msg import MotorState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class AgvOdometryNode(Node):

    def __init__(self):
        super().__init__('agv_odometry')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('tf_publish_rate', 20.0)
        self.declare_parameter('covariance_linear', 0.01)
        self.declare_parameter('covariance_angular', 0.005)
        self.declare_parameter('covariance_linear_angular', 0.001)
        self.declare_parameter('use_encoder_odometry', False)
        self.declare_parameter('track_width', 0.4)

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        self._tf_publish_rate = self.get_parameter('tf_publish_rate').value
        self._cov_linear = self.get_parameter('covariance_linear').value
        self._cov_angular = self.get_parameter('covariance_angular').value
        self._cov_linear_angular = self.get_parameter('covariance_linear_angular').value
        self._use_encoder_odometry = self.get_parameter('use_encoder_odometry').value
        self._track_width = self.get_parameter('track_width').value

        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.theta = self.get_parameter('initial_theta').value

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.prev_time = self.get_clock().now()

        # 编码器里程计数据
        self._encoder_left_speed = 0.0
        self._encoder_right_speed = 0.0
        self._encoder_received = False

        self._drift_correction_x = 0.0
        self._drift_correction_y = 0.0
        self._drift_correction_theta = 0.0
        self._last_drift_correction_time = self.get_clock().now()

        self._tf_counter = 0
        self._tf_publish_interval = max(1, int(publish_rate / self._tf_publish_rate))

        self.cmd_vel_out_sub = self.create_subscription(
            Twist, 'cmd_vel_out', self.cmd_vel_out_callback, 10)

        # 订阅电机状态，获取编码器轮速
        self.motor_state_sub = self.create_subscription(
            MotorState, 'motor_state', self.motor_state_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        publish_period = 1.0 / publish_rate
        self.publish_timer = self.create_timer(publish_period, self.publish_loop)

    def cmd_vel_out_callback(self, msg):
        if not self._use_encoder_odometry:
            self.linear_vel = msg.linear.x
            self.angular_vel = msg.angular.z

    def motor_state_callback(self, msg):
        """电机状态回调，处理轮式编码器数据"""
        self._encoder_left_speed = msg.left_wheel_speed
        self._encoder_right_speed = msg.right_wheel_speed
        self._encoder_received = True

        if self._use_encoder_odometry:
            # 使用差速驱动模型从编码器轮速计算线速度和角速度
            self.linear_vel = (self._encoder_left_speed + self._encoder_right_speed) / 2.0
            if self._track_width > 0:
                self.angular_vel = (self._encoder_right_speed - self._encoder_left_speed) / self._track_width
            else:
                self.angular_vel = 0.0

    def apply_drift_correction(self, correction_x, correction_y, correction_theta):
        self._drift_correction_x += correction_x
        self._drift_correction_y += correction_y
        self._drift_correction_theta += correction_theta
        self._last_drift_correction_time = self.get_clock().now()

    def _compute_covariance(self, dt):
        linear_factor = abs(self.linear_vel) * dt
        angular_factor = abs(self.angular_vel) * dt

        cov_x = self._cov_linear * (1.0 + linear_factor)
        cov_y = self._cov_linear * 0.5 * (1.0 + linear_factor)
        cov_theta = self._cov_angular * (1.0 + angular_factor)
        cov_xy = self._cov_linear_angular * linear_factor * angular_factor

        covariance = [0.0] * 36
        covariance[0] = cov_x
        covariance[1] = cov_xy
        covariance[6] = cov_xy
        covariance[7] = cov_y
        covariance[35] = cov_theta
        return covariance

    def _compute_twist_covariance(self, dt):
        twist_covariance = [0.0] * 36
        twist_covariance[0] = self._cov_linear * 2.0
        twist_covariance[35] = self._cov_angular * 2.0
        return twist_covariance

    def publish_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        if dt <= 0.0:
            dt = 1e-6

        self.theta += self.angular_vel * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt

        effective_x = self.x + self._drift_correction_x
        effective_y = self.y + self._drift_correction_y
        effective_theta = self.theta + self._drift_correction_theta
        effective_theta = math.atan2(math.sin(effective_theta), math.cos(effective_theta))

        self._tf_counter += 1
        if self._tf_counter >= self._tf_publish_interval:
            self._tf_counter = 0
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = effective_x
            t.transform.translation.y = effective_y
            t.transform.translation.z = 0.0
            q = self._yaw_to_quaternion(effective_theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = effective_x
        odom.pose.pose.position.y = effective_y
        odom.pose.pose.position.z = 0.0
        q = self._yaw_to_quaternion(effective_theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance = self._compute_covariance(dt)
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel
        odom.twist.covariance = self._compute_twist_covariance(dt)
        self.odom_pub.publish(odom)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        half = yaw / 2.0
        return [0.0, 0.0, math.sin(half), math.cos(half)]


def main(args=None):
    rclpy.init(args=args)
    node = AgvOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
