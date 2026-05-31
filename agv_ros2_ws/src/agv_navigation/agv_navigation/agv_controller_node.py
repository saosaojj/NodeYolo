import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from agv_interfaces.msg import AGVStatus, MotorState
from agv_interfaces.srv import ControlAGV


class AgvControllerNode(Node):

    def __init__(self):
        super().__init__('agv_controller')

        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('kd_linear', 0.1)
        self.declare_parameter('kd_angular', 0.3)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_acceleration', 2.0)
        self.declare_parameter('max_jerk', 10.0)
        self.declare_parameter('cmd_vel_timeout', 1.0)
        self.declare_parameter('velocity_ramp_rate', 1.0)
        self.declare_parameter('anti_windup_limit', 5.0)
        self.declare_parameter('use_motor_feedback', False)
        self.declare_parameter('low_battery_threshold', 20.0)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kd_angular = self.get_parameter('kd_angular').value
        control_rate = self.get_parameter('control_rate').value
        self._max_acceleration = self.get_parameter('max_acceleration').value
        self._max_jerk = self.get_parameter('max_jerk').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self._velocity_ramp_rate = self.get_parameter('velocity_ramp_rate').value
        self._anti_windup_limit = self.get_parameter('anti_windup_limit').value
        self._use_motor_feedback = self.get_parameter('use_motor_feedback').value
        self._low_battery_threshold = self.get_parameter('low_battery_threshold').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self._integral_linear_error = 0.0
        self._integral_angular_error = 0.0
        self._prev_desired_linear = 0.0
        self._prev_desired_angular = 0.0
        self._prev_prev_desired_linear = 0.0
        self._prev_prev_desired_angular = 0.0

        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.has_target = False

        self.mode = 'idle'
        self.emergency_stop_flag = False
        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0
        self._smoothed_linear = 0.0
        self._smoothed_angular = 0.0
        self.battery_level = 100.0
        self.active_alarms = []

        self._last_cmd_vel_time = self.get_clock().now()

        # 电机反馈数据
        self._motor_linear_vel = 0.0
        self._motor_angular_vel = 0.0
        self._motor_state_received = False

        # 看门狗超时日志控制
        self._watchdog_logged = False

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, 'target_pose', self.target_pose_callback, 10)

        # 订阅电机状态，获取实际轮速
        self.motor_state_sub = self.create_subscription(
            MotorState, 'motor_state', self.motor_state_callback, 10)

        self.cmd_vel_out_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.agv_status_pub = self.create_publisher(AGVStatus, 'agv_status', 10)

        self.control_srv = self.create_service(
            ControlAGV, 'control_agv', self.control_callback)

        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

    def cmd_vel_callback(self, msg):
        if self.mode == 'manual' and not self.emergency_stop_flag:
            self.cmd_vel_linear = msg.linear.x
            self.cmd_vel_angular = msg.angular.z
            self._last_cmd_vel_time = self.get_clock().now()
            self._watchdog_logged = False

    def target_pose_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        q = msg.pose.pose.orientation if hasattr(msg.pose, 'pose') else msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.target_theta = math.atan2(siny_cosp, cosy_cosp)
        self.has_target = True
        if self.mode == 'idle':
            self.mode = 'auto'
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self._integral_linear_error = 0.0
        self._integral_angular_error = 0.0

    def motor_state_callback(self, msg):
        """电机状态回调，从实际轮速计算线速度和角速度"""
        left_speed = msg.left_wheel_speed
        right_speed = msg.right_wheel_speed
        self._motor_linear_vel = (left_speed + right_speed) / 2.0
        if self.wheel_base > 0:
            self._motor_angular_vel = (right_speed - left_speed) / self.wheel_base
        else:
            self._motor_angular_vel = 0.0
        self._motor_state_received = True

    def control_callback(self, request, response):
        cmd = request.command.lower()
        if cmd == 'start':
            self.mode = 'auto'
            self.emergency_stop_flag = False
            response.success = True
            response.message = 'AGV started in auto mode'
        elif cmd == 'stop':
            self.mode = 'idle'
            self.has_target = False
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            self._smoothed_linear = 0.0
            self._smoothed_angular = 0.0
            self._integral_linear_error = 0.0
            self._integral_angular_error = 0.0
            response.success = True
            response.message = 'AGV stopped'
        elif cmd == 'pause':
            self.mode = 'idle'
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            self._smoothed_linear = 0.0
            self._smoothed_angular = 0.0
            response.success = True
            response.message = 'AGV paused'
        elif cmd == 'resume':
            self.mode = 'auto'
            self.emergency_stop_flag = False
            response.success = True
            response.message = 'AGV resumed'
        elif cmd == 'charge':
            self.mode = 'charging'
            self.has_target = False
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            self._smoothed_linear = 0.0
            self._smoothed_angular = 0.0
            response.success = True
            response.message = 'AGV in charging mode'
        elif cmd == 'emergency_stop':
            self.emergency_stop_flag = True
            self.mode = 'error'
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            self._smoothed_linear = 0.0
            self._smoothed_angular = 0.0
            self.has_target = False
            if 'emergency_stop' not in self.active_alarms:
                self.active_alarms.append('emergency_stop')
            response.success = True
            response.message = 'Emergency stop activated'
        else:
            response.success = False
            response.message = f'Unknown command: {cmd}'
        return response

    def _apply_velocity_smoothing(self, desired_linear, desired_angular, dt):
        max_delta_linear = self._velocity_ramp_rate * dt
        max_delta_angular = self._velocity_ramp_rate * dt

        delta_linear = desired_linear - self._smoothed_linear
        delta_angular = desired_angular - self._smoothed_angular

        delta_linear = max(-max_delta_linear, min(max_delta_linear, delta_linear))
        delta_angular = max(-max_delta_angular, min(max_delta_angular, delta_angular))

        new_linear = self._smoothed_linear + delta_linear
        new_angular = self._smoothed_angular + delta_angular

        prev_accel_linear = self._smoothed_linear - self._prev_desired_linear
        prev_accel_angular = self._smoothed_angular - self._prev_desired_angular
        new_accel_linear = new_linear - self._smoothed_linear
        new_accel_angular = new_angular - self._smoothed_angular

        jerk_linear = (new_accel_linear - prev_accel_linear) / dt if dt > 0 else 0.0
        jerk_angular = (new_accel_angular - prev_accel_angular) / dt if dt > 0 else 0.0

        if abs(jerk_linear) > self._max_jerk:
            sign = 1.0 if jerk_linear > 0 else -1.0
            new_accel_linear = prev_accel_linear + sign * self._max_jerk * dt
            new_linear = self._smoothed_linear + new_accel_linear

        if abs(jerk_angular) > self._max_jerk:
            sign = 1.0 if jerk_angular > 0 else -1.0
            new_accel_angular = prev_accel_angular + sign * self._max_jerk * dt
            new_angular = self._smoothed_angular + new_accel_angular

        accel_linear = (new_linear - self._smoothed_linear) / dt if dt > 0 else 0.0
        accel_angular = (new_angular - self._smoothed_angular) / dt if dt > 0 else 0.0

        if abs(accel_linear) > self._max_acceleration:
            sign = 1.0 if accel_linear > 0 else -1.0
            new_linear = self._smoothed_linear + sign * self._max_acceleration * dt
        if abs(accel_angular) > self._max_acceleration:
            sign = 1.0 if accel_angular > 0 else -1.0
            new_angular = self._smoothed_angular + sign * self._max_acceleration * dt

        self._prev_prev_desired_linear = self._prev_desired_linear
        self._prev_prev_desired_angular = self._prev_desired_angular
        self._prev_desired_linear = self._smoothed_linear
        self._prev_desired_angular = self._smoothed_angular
        self._smoothed_linear = new_linear
        self._smoothed_angular = new_angular

        return new_linear, new_angular

    def control_loop(self):
        dt = 1.0 / self.get_parameter('control_rate').value

        if self.emergency_stop_flag:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            self._smoothed_linear = 0.0
            self._smoothed_angular = 0.0
        elif self.mode == 'auto' and self.has_target:
            self._compute_pd_control()
        elif self.mode == 'manual':
            elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
            if elapsed > self._cmd_vel_timeout:
                self.cmd_vel_linear = 0.0
                self.cmd_vel_angular = 0.0
                if not self._watchdog_logged:
                    self.get_logger().warn(
                        f'手动模式下未在 {self._cmd_vel_timeout:.1f} 秒内收到速度指令，已发送零速度指令')
                    self._watchdog_logged = True
        else:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0

        smoothed_linear, smoothed_angular = self._apply_velocity_smoothing(
            self.cmd_vel_linear, self.cmd_vel_angular, dt)

        cmd_out = Twist()
        cmd_out.linear.x = max(-self.max_linear_vel,
                               min(self.max_linear_vel, smoothed_linear))
        cmd_out.angular.z = max(-self.max_angular_vel,
                                min(self.max_angular_vel, smoothed_angular))
        self.cmd_vel_out_pub.publish(cmd_out)

        # 根据是否使用电机反馈选择速度来源
        if self._use_motor_feedback and self._motor_state_received:
            self.current_linear_vel = self._motor_linear_vel
            self.current_angular_vel = self._motor_angular_vel
        else:
            self.current_linear_vel = cmd_out.linear.x
            self.current_angular_vel = cmd_out.angular.z

        self.current_theta += self.current_angular_vel * dt
        self.current_theta = math.atan2(math.sin(self.current_theta),
                                        math.cos(self.current_theta))
        self.current_x += self.current_linear_vel * math.cos(self.current_theta) * dt
        self.current_y += self.current_linear_vel * math.sin(self.current_theta) * dt

        # 低电量自动切换充电模式
        if self.battery_level < self._low_battery_threshold and self.mode not in ('charging', 'error'):
            self.get_logger().warn(
                f'电量低于阈值 {self._low_battery_threshold:.1f}%，自动切换到充电模式')
            self.mode = 'charging'
            self.has_target = False
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            self._smoothed_linear = 0.0
            self._smoothed_angular = 0.0

        status = AGVStatus()
        status.x = self.current_x
        status.y = self.current_y
        status.theta = self.current_theta
        status.linear_velocity = self.current_linear_vel
        status.angular_velocity = self.current_angular_vel
        status.battery_level = self.battery_level
        status.mode = self.mode
        status.emergency_stop = self.emergency_stop_flag
        status.active_alarms = self.active_alarms
        status.timestamp = self.get_clock().now().to_msg()
        self.agv_status_pub.publish(status)

    def _compute_pd_control(self):
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        linear_error = distance
        angular_error = angle_error

        d_linear = (linear_error - self.prev_linear_error)
        d_angular = (angular_error - self.prev_angular_error)

        self._integral_linear_error += linear_error
        self._integral_angular_error += angular_error

        self._integral_linear_error = max(-self._anti_windup_limit,
                                          min(self._anti_windup_limit, self._integral_linear_error))
        self._integral_angular_error = max(-self._anti_windup_limit,
                                           min(self._anti_windup_limit, self._integral_angular_error))

        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error

        if abs(angle_error) > math.pi / 4:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = self.kp_angular * angular_error + self.kd_angular * d_angular
        else:
            self.cmd_vel_linear = self.kp_linear * linear_error + self.kd_linear * d_linear
            self.cmd_vel_angular = self.kp_angular * angular_error + self.kd_angular * d_angular

        if distance < 0.05:
            final_angle_error = self.target_theta - self.current_theta
            final_angle_error = math.atan2(math.sin(final_angle_error),
                                           math.cos(final_angle_error))
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = self.kp_angular * final_angle_error
            if abs(final_angle_error) < 0.05:
                self.has_target = False
                self.mode = 'idle'
                self.cmd_vel_linear = 0.0
                self.cmd_vel_angular = 0.0
                self._integral_linear_error = 0.0
                self._integral_angular_error = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = AgvControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
