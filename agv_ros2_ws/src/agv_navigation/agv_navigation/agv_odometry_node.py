# AGV里程计节点，根据速度指令计算机器人的位姿估计
# 发布里程计消息和TF变换，支持漂移校正和协方差计算
# 注意：AGV状态由agv_controller_node统一发布，本节点不再重复发布agv_status
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


# AGV里程计节点类，根据轮式运动学模型推算机器人位姿
class AgvOdometryNode(Node):

    # 初始化里程计节点，声明参数并创建发布者和订阅者
    def __init__(self):
        super().__init__('agv_odometry')

        # 声明里程计参数：发布频率、坐标系、初始位姿、协方差等
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

        # 读取参数值
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        self._tf_publish_rate = self.get_parameter('tf_publish_rate').value
        self._cov_linear = self.get_parameter('covariance_linear').value
        self._cov_angular = self.get_parameter('covariance_angular').value
        self._cov_linear_angular = self.get_parameter('covariance_linear_angular').value

        # 初始化位姿
        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.theta = self.get_parameter('initial_theta').value

        # 当前速度
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.prev_time = self.get_clock().now()

        # 漂移校正量，用于外部定位系统修正累积误差
        self._drift_correction_x = 0.0
        self._drift_correction_y = 0.0
        self._drift_correction_theta = 0.0
        self._last_drift_correction_time = self.get_clock().now()

        # TF发布计数器，控制TF发布频率低于里程计发布频率
        self._tf_counter = 0
        self._tf_publish_interval = max(1, int(publish_rate / self._tf_publish_rate))

        # 订阅速度指令输出话题
        self.cmd_vel_out_sub = self.create_subscription(
            Twist, 'cmd_vel_out', self.cmd_vel_out_callback, 10)

        # 发布里程计消息
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF广播器，发布odom到base_link的变换
        self.tf_broadcaster = TransformBroadcaster(self)

        # 创建发布定时器
        publish_period = 1.0 / publish_rate
        self.publish_timer = self.create_timer(publish_period, self.publish_loop)

    # 速度指令回调，更新当前线速度和角速度
    def cmd_vel_out_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    # 应用漂移校正，外部定位系统可调用此方法修正累积误差
    def apply_drift_correction(self, correction_x, correction_y, correction_theta):
        self._drift_correction_x += correction_x
        self._drift_correction_y += correction_y
        self._drift_correction_theta += correction_theta
        self._last_drift_correction_time = self.get_clock().now()

    # 计算位姿协方差矩阵，考虑速度对不确定性的影响
    def _compute_covariance(self, dt):
        linear_factor = abs(self.linear_vel) * dt
        angular_factor = abs(self.angular_vel) * dt

        cov_x = self._cov_linear * (1.0 + linear_factor)
        cov_y = self._cov_linear * 0.5 * (1.0 + linear_factor)
        cov_theta = self._cov_angular * (1.0 + angular_factor)
        cov_xy = self._cov_linear_angular * linear_factor * angular_factor

        # 构建6x6协方差矩阵（展平为一维数组）
        covariance = [0.0] * 36
        covariance[0] = cov_x
        covariance[1] = cov_xy
        covariance[6] = cov_xy
        covariance[7] = cov_y
        covariance[35] = cov_theta
        return covariance

    # 计算速度协方差矩阵
    def _compute_twist_covariance(self, dt):
        twist_covariance = [0.0] * 36
        twist_covariance[0] = self._cov_linear * 2.0
        twist_covariance[35] = self._cov_angular * 2.0
        return twist_covariance

    # 发布循环主函数，更新位姿并发布里程计、TF和状态
    def publish_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        if dt <= 0.0:
            dt = 1e-6

        # 根据运动学模型更新位姿
        self.theta += self.angular_vel * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt

        # 应用漂移校正得到有效位姿
        effective_x = self.x + self._drift_correction_x
        effective_y = self.y + self._drift_correction_y
        effective_theta = self.theta + self._drift_correction_theta
        effective_theta = math.atan2(math.sin(effective_theta), math.cos(effective_theta))

        # 按指定频率发布TF变换
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

        # 构建并发布里程计消息
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

    # 将偏航角转换为四元数表示
    @staticmethod
    def _yaw_to_quaternion(yaw):
        half = yaw / 2.0
        return [0.0, 0.0, math.sin(half), math.cos(half)]


# 节点主入口函数
def main(args=None):
    rclpy.init(args=args)
    node = AgvOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
