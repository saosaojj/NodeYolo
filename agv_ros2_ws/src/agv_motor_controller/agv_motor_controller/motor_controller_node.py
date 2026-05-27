import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from agv_interfaces.msg import AGVStatus, MotorState, DriveCommand
from agv_interfaces.srv import SetSpeed, SetSteering


class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller')

        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_steering_angle', 0.7854)
        self.declare_parameter('min_steering_angle', -0.7854)
        self.declare_parameter('wheel_diameter', 0.2)
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('track_width', 0.4)
        self.declare_parameter('speed_kp', 2.0)
        self.declare_parameter('speed_ki', 0.1)
        self.declare_parameter('speed_kd', 0.05)
        self.declare_parameter('steering_kp', 3.0)
        self.declare_parameter('steering_ki', 0.0)
        self.declare_parameter('steering_kd', 0.2)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('acceleration_limit', 2.0)
        self.declare_parameter('deceleration_limit', 3.0)
        self.declare_parameter('steering_rate_limit', 1.5)
        self.declare_parameter('speed_deadband', 0.01)
        self.declare_parameter('steering_deadband', 0.01)
        self.declare_parameter('simulate', False)

        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_steering_angle = self.get_parameter('min_steering_angle').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.speed_kp = self.get_parameter('speed_kp').value
        self.speed_ki = self.get_parameter('speed_ki').value
        self.speed_kd = self.get_parameter('speed_kd').value
        self.steering_kp = self.get_parameter('steering_kp').value
        self.steering_ki = self.get_parameter('steering_ki').value
        self.steering_kd = self.get_parameter('steering_kd').value
        self.control_rate = self.get_parameter('control_rate').value
        self.acceleration_limit = self.get_parameter('acceleration_limit').value
        self.deceleration_limit = self.get_parameter('deceleration_limit').value
        self.steering_rate_limit = self.get_parameter('steering_rate_limit').value
        self.speed_deadband = self.get_parameter('speed_deadband').value
        self.steering_deadband = self.get_parameter('steering_deadband').value
        self.simulate = self.get_parameter('simulate').value

        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.target_speed = 0.0
        self.target_steering_angle = 0.0

        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.current_steering_angle = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        self.speed_integral = 0.0
        self.speed_prev_error = 0.0
        self.steering_integral = 0.0
        self.steering_prev_error = 0.0

        self.prev_speed_command = 0.0
        self.prev_steering_command = 0.0

        self.brake_active = False
        self.mode = 'idle'
        self.emergency_stop = False

        self._last_cmd_vel_time = self.get_clock().now()

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_out', self.cmd_vel_callback, 10)

        self.agv_status_sub = self.create_subscription(
            AGVStatus, 'agv_status', self.agv_status_callback, 10)

        self.motor_state_pub = self.create_publisher(
            MotorState, 'motor_state', 10)

        self.drive_command_pub = self.create_publisher(
            DriveCommand, 'drive_command', 10)

        self.set_speed_srv = self.create_service(
            SetSpeed, 'set_speed', self.set_speed_callback)

        self.set_steering_srv = self.create_service(
            SetSteering, 'set_steering', self.set_steering_callback)

        self.emergency_brake_srv = self.create_service(
            Trigger, 'emergency_brake', self.emergency_brake_callback)

        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info(
            f'电机控制器已启动, 仿真模式: {self.simulate}, 控制频率: {self.control_rate}Hz')

    def cmd_vel_callback(self, msg):
        if self.emergency_stop:
            return
        self.target_linear_vel = msg.linear.x
        self.target_angular_vel = msg.angular.z
        self._last_cmd_vel_time = self.get_clock().now()
        if self.mode == 'idle':
            self.mode = 'running'

    def agv_status_callback(self, msg):
        if msg.emergency_stop and not self.emergency_stop:
            self.emergency_stop = True
            self.brake_active = True
            self.get_logger().warn('收到紧急停车状态，激活制动')
        self.current_linear_vel = msg.linear_velocity
        self.current_angular_vel = msg.angular_velocity

    def compute_ackermann(self, linear_vel, angular_vel):
        if abs(linear_vel) < 1e-6 and abs(angular_vel) < 1e-6:
            return 0.0, 0.0, 0.0

        if abs(angular_vel) < 1e-6:
            left_speed = linear_vel
            right_speed = linear_vel
            steering_angle = 0.0
            return left_speed, right_speed, steering_angle

        turn_radius = linear_vel / angular_vel

        steering_angle = math.atan(self.wheel_base / turn_radius)

        steering_angle = max(self.min_steering_angle,
                             min(self.max_steering_angle, steering_angle))

        inner_radius = turn_radius - self.track_width / 2.0
        outer_radius = turn_radius + self.track_width / 2.0

        if turn_radius > 0:
            left_speed = angular_vel * inner_radius
            right_speed = angular_vel * outer_radius
        else:
            left_speed = angular_vel * outer_radius
            right_speed = angular_vel * inner_radius

        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

        return left_speed, right_speed, steering_angle

    def apply_speed_pid(self, target_speed, current_speed, dt):
        error = target_speed - current_speed

        if abs(error) < self.speed_deadband:
            return self.prev_speed_command

        self.speed_integral += error * dt
        self.speed_integral = max(-5.0, min(5.0, self.speed_integral))

        derivative = (error - self.speed_prev_error) / dt if dt > 0 else 0.0

        output = (self.speed_kp * error
                  + self.speed_ki * self.speed_integral
                  + self.speed_kd * derivative)

        self.speed_prev_error = error

        return output

    def apply_steering_pid(self, target_angle, current_angle, dt):
        error = target_angle - current_angle
        error = math.atan2(math.sin(error), math.cos(error))

        if abs(error) < self.steering_deadband:
            return self.prev_steering_command

        self.steering_integral += error * dt
        self.steering_integral = max(-2.0, min(2.0, self.steering_integral))

        derivative = (error - self.steering_prev_error) / dt if dt > 0 else 0.0

        output = (self.steering_kp * error
                  + self.steering_ki * self.steering_integral
                  + self.steering_kd * derivative)

        self.steering_prev_error = error

        return output

    def limit_acceleration(self, target, current, dt, limit):
        delta = target - current
        max_delta = limit * dt

        if delta > max_delta:
            return current + max_delta
        elif delta < -max_delta:
            return current - max_delta

        return target

    def control_loop(self):
        dt = 1.0 / self.control_rate

        cmd_vel_timeout = 1.0
        elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > cmd_vel_timeout and not self.emergency_stop:
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            if self.mode == 'running':
                self.mode = 'idle'

        if self.emergency_stop or self.brake_active:
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.target_speed = 0.0
            self.target_steering_angle = 0.0
            self.speed_integral = 0.0
            self.speed_prev_error = 0.0
            self.steering_integral = 0.0
            self.steering_prev_error = 0.0

        left_speed, right_speed, steering_angle = self.compute_ackermann(
            self.target_linear_vel, self.target_angular_vel)

        self.target_speed = (left_speed + right_speed) / 2.0
        self.target_steering_angle = steering_angle

        current_avg_speed = (self.current_left_speed + self.current_right_speed) / 2.0

        speed_command = self.apply_speed_pid(
            self.target_speed, current_avg_speed, dt)

        if self.target_speed > current_avg_speed:
            speed_command = self.limit_acceleration(
                speed_command, self.prev_speed_command, dt, self.acceleration_limit)
        else:
            speed_command = self.limit_acceleration(
                speed_command, self.prev_speed_command, dt, self.deceleration_limit)

        steering_command = self.apply_steering_pid(
            self.target_steering_angle, self.current_steering_angle, dt)

        steering_command = self.limit_acceleration(
            steering_command, self.prev_steering_command, dt, self.steering_rate_limit)

        self.prev_speed_command = speed_command
        self.prev_steering_command = steering_command

        if self.simulate:
            self.current_left_speed += speed_command * dt
            self.current_right_speed += speed_command * dt
            self.current_steering_angle = steering_command

            self.current_left_speed = max(-self.max_speed,
                                          min(self.max_speed, self.current_left_speed))
            self.current_right_speed = max(-self.max_speed,
                                           min(self.max_speed, self.current_right_speed))
            self.current_steering_angle = max(self.min_steering_angle,
                                              min(self.max_steering_angle,
                                                  self.current_steering_angle))
        else:
            self.current_left_speed = left_speed
            self.current_right_speed = right_speed
            self.current_steering_angle = steering_angle

        motor_state = MotorState()
        motor_state.left_wheel_speed = self.current_left_speed
        motor_state.right_wheel_speed = self.current_right_speed
        motor_state.steering_angle = self.current_steering_angle
        motor_state.target_speed = self.target_speed
        motor_state.target_steering_angle = self.target_steering_angle
        motor_state.left_motor_current = 0.0
        motor_state.right_motor_current = 0.0
        motor_state.mode = self.mode
        motor_state.brake_active = self.brake_active
        motor_state.timestamp = self.get_clock().now().to_msg()
        self.motor_state_pub.publish(motor_state)

        drive_cmd = DriveCommand()
        drive_cmd.left_motor_value = speed_command
        drive_cmd.right_motor_value = speed_command
        drive_cmd.steering_servo_value = steering_command
        drive_cmd.mode = self.mode
        drive_cmd.timestamp = self.get_clock().now().to_msg()
        self.drive_command_pub.publish(drive_cmd)

        if (self.brake_active
                and abs(self.current_left_speed) < self.speed_deadband
                and abs(self.current_right_speed) < self.speed_deadband):
            self.brake_active = False

    def set_speed_callback(self, request, response):
        if self.emergency_stop:
            response.success = False
            response.message = '紧急停车状态，无法设置速度'
            response.actual_speed = 0.0
            return response

        requested_speed = request.speed
        if abs(requested_speed) > self.max_speed:
            requested_speed = self.max_speed if requested_speed > 0 else -self.max_speed
            self.get_logger().warn(
                f'请求速度超出限制，已截断至 {requested_speed}')

        self.target_linear_vel = requested_speed
        self.target_angular_vel = 0.0
        self._last_cmd_vel_time = self.get_clock().now()

        if self.mode == 'idle':
            self.mode = 'running'

        response.success = True
        response.message = f'目标速度已设置为 {requested_speed}'
        response.actual_speed = requested_speed
        self.get_logger().info(f'通过服务设置目标速度: {requested_speed}')
        return response

    def set_steering_callback(self, request, response):
        if self.emergency_stop:
            response.success = False
            response.message = '紧急停车状态，无法设置转向角'
            response.actual_angle = 0.0
            return response

        requested_angle = request.angle
        if requested_angle > self.max_steering_angle:
            requested_angle = self.max_steering_angle
            self.get_logger().warn(
                f'请求转向角超出最大限制，已截断至 {requested_angle}')
        elif requested_angle < self.min_steering_angle:
            requested_angle = self.min_steering_angle
            self.get_logger().warn(
                f'请求转向角超出最小限制，已截断至 {requested_angle}')

        self.target_steering_angle = requested_angle
        self._last_cmd_vel_time = self.get_clock().now()

        response.success = True
        response.message = f'目标转向角已设置为 {requested_angle}'
        response.actual_angle = requested_angle
        self.get_logger().info(f'通过服务设置目标转向角: {requested_angle}')
        return response

    def emergency_brake_callback(self, request, response):
        self.emergency_stop = True
        self.brake_active = True
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.target_speed = 0.0
        self.target_steering_angle = 0.0
        self.speed_integral = 0.0
        self.speed_prev_error = 0.0
        self.steering_integral = 0.0
        self.steering_prev_error = 0.0
        self.prev_speed_command = 0.0
        self.prev_steering_command = 0.0
        self.mode = 'error'

        response.success = True
        response.message = '紧急制动已激活'
        self.get_logger().warn('紧急制动已激活！')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
