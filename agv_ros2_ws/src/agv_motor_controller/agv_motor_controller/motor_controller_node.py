import math
import json
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from agv_interfaces.msg import AGVStatus, MotorState, DriveCommand
from agv_interfaces.srv import SetSpeed, SetSteering


class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller')

        # 声明原有参数
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

        # 声明新增参数：驱动类型
        self.declare_parameter('drive_type', 'ackermann')
        # 声明新增参数：编码器
        self.declare_parameter('encoder_ticks_per_revolution', 1000)
        self.declare_parameter('gear_ratio', 20.0)
        self.declare_parameter('use_encoder_feedback', False)
        # 声明新增参数：电机故障检测
        self.declare_parameter('max_motor_current', 10.0)
        self.declare_parameter('stall_speed_threshold', 0.05)
        self.declare_parameter('stall_time_threshold', 2.0)
        # 声明新增参数：里程计
        self.declare_parameter('publish_motor_odom', True)
        self.declare_parameter('motor_odom_frame', 'motor_odom')
        # 声明新增参数：偏航校正
        self.declare_parameter('yaw_correction_gain', 0.5)
        # 声明新增参数：仿真模型改进
        self.declare_parameter('motor_time_constant', 0.1)
        self.declare_parameter('friction_coefficient', 0.1)

        # 读取原有参数
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

        # 读取新增参数
        self.drive_type = self.get_parameter('drive_type').value
        self.encoder_ticks_per_revolution = self.get_parameter('encoder_ticks_per_revolution').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.use_encoder_feedback = self.get_parameter('use_encoder_feedback').value
        self.max_motor_current = self.get_parameter('max_motor_current').value
        self.stall_speed_threshold = self.get_parameter('stall_speed_threshold').value
        self.stall_time_threshold = self.get_parameter('stall_time_threshold').value
        self.publish_motor_odom = self.get_parameter('publish_motor_odom').value
        self.motor_odom_frame = self.get_parameter('motor_odom_frame').value
        self.yaw_correction_gain = self.get_parameter('yaw_correction_gain').value
        self.motor_time_constant = self.get_parameter('motor_time_constant').value
        self.friction_coefficient = self.get_parameter('friction_coefficient').value

        # 验证驱动类型参数
        if self.drive_type not in ('ackermann', 'differential'):
            self.get_logger().warn(
                f'不支持的驱动类型: {self.drive_type}, 回退到ackermann')
            self.drive_type = 'ackermann'

        # 目标速度状态
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.target_speed = 0.0
        self.target_steering_angle = 0.0

        # 当前速度状态
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.current_steering_angle = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        # 速度PID状态（阿克曼模式使用）
        self.speed_integral = 0.0
        self.speed_prev_error = 0.0
        self.steering_integral = 0.0
        self.steering_prev_error = 0.0

        # 差速驱动左右轮独立PID状态
        self.left_speed_integral = 0.0
        self.left_speed_prev_error = 0.0
        self.right_speed_integral = 0.0
        self.right_speed_prev_error = 0.0

        # 上一帧指令
        self.prev_speed_command = 0.0
        self.prev_steering_command = 0.0
        self.prev_left_command = 0.0
        self.prev_right_command = 0.0

        # 制动与模式状态
        self.brake_active = False
        self.mode = 'idle'
        self.emergency_stop = False
        self._external_emergency = False

        # 编码器反馈状态
        self._encoder_left_speed = 0.0
        self._encoder_right_speed = 0.0
        self._encoder_feedback_received = False

        # 电机电流状态
        self._left_motor_current = 0.0
        self._right_motor_current = 0.0
        self._prev_left_speed_for_accel = 0.0
        self._prev_right_speed_for_accel = 0.0

        # 堵转检测计时
        self._left_stall_start = None
        self._right_stall_start = None

        # 故障状态
        self._fault_active = False

        # 里程计状态
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0

        # 指令超时计时
        self._last_cmd_vel_time = self.get_clock().now()

        # 订阅
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_out', self.cmd_vel_callback, 10)

        self.agv_status_sub = self.create_subscription(
            AGVStatus, 'agv_status', self.agv_status_callback, 10)

        self.encoder_feedback_sub = self.create_subscription(
            Float32MultiArray, 'encoder_feedback', self.encoder_feedback_callback, 10)

        # 发布
        self.motor_state_pub = self.create_publisher(
            MotorState, 'motor_state', 10)

        self.drive_command_pub = self.create_publisher(
            DriveCommand, 'drive_command', 10)

        self.encoder_data_pub = self.create_publisher(
            Float32MultiArray, 'encoder_data', 10)

        self.motor_fault_pub = self.create_publisher(
            String, 'motor_fault', 10)

        self.motor_odom_pub = self.create_publisher(
            Odometry, 'motor_odom', 10)

        # 服务
        self.set_speed_srv = self.create_service(
            SetSpeed, 'set_speed', self.set_speed_callback)

        self.set_steering_srv = self.create_service(
            SetSteering, 'set_steering', self.set_steering_callback)

        self.emergency_brake_srv = self.create_service(
            Trigger, 'emergency_brake', self.emergency_brake_callback)

        self.recover_brake_srv = self.create_service(
            Trigger, 'recover_brake', self.recover_brake_callback)

        # 控制循环定时器
        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info(
            f'电机控制器已启动, 驱动类型: {self.drive_type}, '
            f'仿真模式: {self.simulate}, 控制频率: {self.control_rate}Hz')

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
            self._external_emergency = True
            self.brake_active = True
            self.get_logger().warn('收到紧急停车状态，激活制动')
        elif not msg.emergency_stop and self._external_emergency:
            self._external_emergency = False
        self.current_linear_vel = msg.linear_velocity
        self.current_angular_vel = msg.angular_velocity

    def encoder_feedback_callback(self, msg):
        if len(msg.data) >= 2:
            self._encoder_left_speed = msg.data[0]
            self._encoder_right_speed = msg.data[1]
            self._encoder_feedback_received = True

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

    def compute_differential(self, linear_vel, angular_vel):
        # 差速驱动运动学：根据线速度和角速度计算左右轮速度
        left_speed = linear_vel - (angular_vel * self.track_width / 2.0)
        right_speed = linear_vel + (angular_vel * self.track_width / 2.0)
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))
        return left_speed, right_speed

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

    def apply_wheel_pid(self, target_speed, current_speed, dt,
                        integral, prev_error, prev_command):
        # 独立轮速PID控制器，用于差速驱动模式
        error = target_speed - current_speed

        if abs(error) < self.speed_deadband:
            return prev_command, integral, prev_error

        integral += error * dt
        integral = max(-5.0, min(5.0, integral))

        derivative = (error - prev_error) / dt if dt > 0 else 0.0

        output = (self.speed_kp * error
                  + self.speed_ki * integral
                  + self.speed_kd * derivative)

        prev_error = error

        return output, integral, prev_error

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

    def simulate_motor_current(self, left_speed, right_speed, dt):
        # 平滑电机电流仿真：电流 = 基础电流 + 负载因子 * |加速度| + 噪声
        base_current = 0.5
        load_factor = 3.0

        left_accel = (left_speed - self._prev_left_speed_for_accel) / dt if dt > 0 else 0.0
        right_accel = (right_speed - self._prev_right_speed_for_accel) / dt if dt > 0 else 0.0

        self._left_motor_current = (base_current
                                    + load_factor * abs(left_accel)
                                    + random.uniform(-0.1, 0.1))
        self._right_motor_current = (base_current
                                     + load_factor * abs(right_accel)
                                     + random.uniform(-0.1, 0.1))

        self._left_motor_current = max(0.0, self._left_motor_current)
        self._right_motor_current = max(0.0, self._right_motor_current)

        self._prev_left_speed_for_accel = left_speed
        self._prev_right_speed_for_accel = right_speed

    def simulate_encoder_ticks(self, left_speed, right_speed, dt):
        # 根据轮速仿真编码器脉冲数
        wheel_circumference = math.pi * self.wheel_diameter
        left_ticks = (left_speed / wheel_circumference
                      * self.encoder_ticks_per_revolution
                      * self.gear_ratio * dt)
        right_ticks = (right_speed / wheel_circumference
                       * self.encoder_ticks_per_revolution
                       * self.gear_ratio * dt)
        return left_ticks, right_ticks

    def check_motor_faults(self):
        now = self.get_clock().now()

        # 过流检测：电机电流超过最大允许值
        if self._left_motor_current > self.max_motor_current:
            self._publish_fault('overcurrent', 'left', self._left_motor_current)
            self._activate_fault_mode()
            return
        if self._right_motor_current > self.max_motor_current:
            self._publish_fault('overcurrent', 'right', self._right_motor_current)
            self._activate_fault_mode()
            return

        # 确定实际速度来源
        if self.use_encoder_feedback and self._encoder_feedback_received:
            left_actual = abs(self._encoder_left_speed)
            right_actual = abs(self._encoder_right_speed)
        else:
            left_actual = abs(self.current_left_speed)
            right_actual = abs(self.current_right_speed)

        # 堵转检测：有指令但实际速度低于阈值持续一定时间
        left_commanded = abs(self.target_speed) > self.stall_speed_threshold
        right_commanded = abs(self.target_speed) > self.stall_speed_threshold

        # 左轮堵转检测
        if left_commanded and left_actual < self.stall_speed_threshold:
            if self._left_stall_start is None:
                self._left_stall_start = now
            else:
                stall_duration = (now - self._left_stall_start).nanoseconds / 1e9
                if stall_duration > self.stall_time_threshold:
                    self._publish_fault('stall', 'left', stall_duration)
                    self._activate_fault_mode()
                    return
        else:
            self._left_stall_start = None

        # 右轮堵转检测
        if right_commanded and right_actual < self.stall_speed_threshold:
            if self._right_stall_start is None:
                self._right_stall_start = now
            else:
                stall_duration = (now - self._right_stall_start).nanoseconds / 1e9
                if stall_duration > self.stall_time_threshold:
                    self._publish_fault('stall', 'right', stall_duration)
                    self._activate_fault_mode()
                    return
        else:
            self._right_stall_start = None

        # 编码器故障检测：有指令但编码器反馈突然归零
        if self.use_encoder_feedback and self._encoder_feedback_received:
            if (abs(self.target_speed) > self.stall_speed_threshold
                    and abs(self._encoder_left_speed) < 1e-6
                    and abs(self._encoder_right_speed) < 1e-6):
                self._publish_fault('encoder_fault', 'left', 0.0)
                self._activate_fault_mode()

    def _publish_fault(self, fault_type, motor, value):
        # 发布电机故障消息（JSON格式）
        fault_msg = String()
        fault_data = {
            'type': fault_type,
            'motor': motor,
            'value': round(value, 4)
        }
        fault_msg.data = json.dumps(fault_data)
        self.motor_fault_pub.publish(fault_msg)
        self.get_logger().error(
            f'电机故障检测: {fault_type}, 电机: {motor}, 值: {value}')

    def _activate_fault_mode(self):
        # 激活故障模式：设置错误状态并激活制动
        self._fault_active = True
        self.emergency_stop = True
        self.brake_active = True
        self.mode = 'error'

    def compute_motor_odom(self, left_speed, right_speed, dt):
        # 差速驱动里程计模型
        linear = (left_speed + right_speed) / 2.0
        angular = (right_speed - left_speed) / self.track_width

        self._odom_x += linear * math.cos(self._odom_theta) * dt
        self._odom_y += linear * math.sin(self._odom_theta) * dt
        self._odom_theta += angular * dt

        # 将角度归一化到 [-pi, pi]
        self._odom_theta = math.atan2(math.sin(self._odom_theta),
                                      math.cos(self._odom_theta))

    def _euler_to_quaternion(self, yaw):
        # 欧拉角转四元数（仅绕z轴旋转）
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def control_loop(self):
        dt = 1.0 / self.control_rate

        # 指令超时检测
        cmd_vel_timeout = 1.0
        elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > cmd_vel_timeout and not self.emergency_stop:
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            if self.mode == 'running':
                self.mode = 'idle'

        # 紧急停车或制动状态处理
        if self.emergency_stop or self.brake_active:
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.target_speed = 0.0
            self.target_steering_angle = 0.0
            self.speed_integral = 0.0
            self.speed_prev_error = 0.0
            self.steering_integral = 0.0
            self.steering_prev_error = 0.0
            self.left_speed_integral = 0.0
            self.left_speed_prev_error = 0.0
            self.right_speed_integral = 0.0
            self.right_speed_prev_error = 0.0

        # 根据驱动类型计算目标轮速和转向角
        if self.drive_type == 'differential':
            left_speed, right_speed = self.compute_differential(
                self.target_linear_vel, self.target_angular_vel)
            steering_angle = 0.0

            # 偏航校正：目标角速度为0但实际存在角速度偏差时进行修正
            if (abs(self.target_angular_vel) < 1e-6
                    and abs(self.current_angular_vel) > 1e-6):
                angular_error = self.current_angular_vel
                left_speed += self.yaw_correction_gain * angular_error
                right_speed -= self.yaw_correction_gain * angular_error
                left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
                right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

            self.target_speed = (left_speed + right_speed) / 2.0
            self.target_steering_angle = 0.0
        else:
            left_speed, right_speed, steering_angle = self.compute_ackermann(
                self.target_linear_vel, self.target_angular_vel)
            self.target_speed = (left_speed + right_speed) / 2.0
            self.target_steering_angle = steering_angle

        # 确定当前实际速度来源（编码器反馈或内部估计）
        if self.use_encoder_feedback and self._encoder_feedback_received:
            current_left = self._encoder_left_speed
            current_right = self._encoder_right_speed
        else:
            current_left = self.current_left_speed
            current_right = self.current_right_speed

        current_avg_speed = (current_left + current_right) / 2.0

        # PID控制与指令生成
        if self.drive_type == 'differential':
            # 差速模式：左右轮独立PID控制
            left_command, self.left_speed_integral, self.left_speed_prev_error = \
                self.apply_wheel_pid(left_speed, current_left, dt,
                                     self.left_speed_integral,
                                     self.left_speed_prev_error,
                                     self.prev_left_command)
            right_command, self.right_speed_integral, self.right_speed_prev_error = \
                self.apply_wheel_pid(right_speed, current_right, dt,
                                     self.right_speed_integral,
                                     self.right_speed_prev_error,
                                     self.prev_right_command)

            # 加速度限制
            if left_speed > current_left:
                left_command = self.limit_acceleration(
                    left_command, self.prev_left_command, dt, self.acceleration_limit)
            else:
                left_command = self.limit_acceleration(
                    left_command, self.prev_left_command, dt, self.deceleration_limit)

            if right_speed > current_right:
                right_command = self.limit_acceleration(
                    right_command, self.prev_right_command, dt, self.acceleration_limit)
            else:
                right_command = self.limit_acceleration(
                    right_command, self.prev_right_command, dt, self.deceleration_limit)

            self.prev_left_command = left_command
            self.prev_right_command = right_command

            speed_command = (left_command + right_command) / 2.0
            steering_command = 0.0
        else:
            # 阿克曼模式：使用原有平均速度PID和转向PID
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

            left_command = speed_command
            right_command = speed_command

        # 仿真模式更新
        if self.simulate:
            # 一阶响应系数
            alpha = 1.0 - math.exp(-dt / self.motor_time_constant)

            if self.drive_type == 'differential':
                # 差速模式：左右轮独立一阶响应仿真
                self.current_left_speed += (left_speed - self.current_left_speed) * alpha
                self.current_right_speed += (right_speed - self.current_right_speed) * alpha

                # 摩擦模型
                self.current_left_speed *= (1.0 - self.friction_coefficient * dt)
                self.current_right_speed *= (1.0 - self.friction_coefficient * dt)

                self.current_left_speed = max(-self.max_speed,
                                              min(self.max_speed, self.current_left_speed))
                self.current_right_speed = max(-self.max_speed,
                                               min(self.max_speed, self.current_right_speed))
                self.current_steering_angle = 0.0
            else:
                # 阿克曼模式：一阶响应仿真
                target_avg = (left_speed + right_speed) / 2.0
                current_avg = (self.current_left_speed + self.current_right_speed) / 2.0
                new_avg = current_avg + (target_avg - current_avg) * alpha

                # 摩擦模型
                new_avg *= (1.0 - self.friction_coefficient * dt)

                self.current_left_speed = new_avg
                self.current_right_speed = new_avg

                # 转向角一阶响应
                self.current_steering_angle += (steering_angle - self.current_steering_angle) * alpha

                self.current_left_speed = max(-self.max_speed,
                                              min(self.max_speed, self.current_left_speed))
                self.current_right_speed = max(-self.max_speed,
                                               min(self.max_speed, self.current_right_speed))
                self.current_steering_angle = max(self.min_steering_angle,
                                                  min(self.max_steering_angle,
                                                      self.current_steering_angle))

            # 仿真电机电流
            self.simulate_motor_current(
                self.current_left_speed, self.current_right_speed, dt)

            # 仿真编码器数据并发布
            left_ticks, right_ticks = self.simulate_encoder_ticks(
                self.current_left_speed, self.current_right_speed, dt)
            encoder_msg = Float32MultiArray()
            encoder_msg.data = [left_ticks, right_ticks, dt]
            self.encoder_data_pub.publish(encoder_msg)
        else:
            # 非仿真模式：使用编码器反馈或开环估计
            if self.use_encoder_feedback and self._encoder_feedback_received:
                self.current_left_speed = self._encoder_left_speed
                self.current_right_speed = self._encoder_right_speed
            else:
                self.current_left_speed = left_speed
                self.current_right_speed = right_speed
            self.current_steering_angle = steering_angle

            # 非仿真模式下也计算电机电流（基于实际速度变化）
            self.simulate_motor_current(
                self.current_left_speed, self.current_right_speed, dt)

        # 电机故障检测
        self.check_motor_faults()

        # 计算并发布里程计
        if self.publish_motor_odom:
            self.compute_motor_odom(
                self.current_left_speed, self.current_right_speed, dt)

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.motor_odom_frame
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self._odom_x
            odom_msg.pose.pose.position.y = self._odom_y
            odom_msg.pose.pose.position.z = 0.0
            qx, qy, qz, qw = self._euler_to_quaternion(self._odom_theta)
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            linear_vel = (self.current_left_speed + self.current_right_speed) / 2.0
            angular_vel = (self.current_right_speed - self.current_left_speed) / self.track_width
            odom_msg.twist.twist.linear.x = linear_vel
            odom_msg.twist.twist.angular.z = angular_vel
            self.motor_odom_pub.publish(odom_msg)

        # 发布电机状态
        motor_state = MotorState()
        motor_state.left_wheel_speed = self.current_left_speed
        motor_state.right_wheel_speed = self.current_right_speed
        motor_state.steering_angle = self.current_steering_angle
        motor_state.target_speed = self.target_speed
        motor_state.target_steering_angle = self.target_steering_angle
        motor_state.left_motor_current = self._left_motor_current
        motor_state.right_motor_current = self._right_motor_current
        motor_state.mode = self.mode
        motor_state.brake_active = self.brake_active
        motor_state.timestamp = self.get_clock().now().to_msg()
        self.motor_state_pub.publish(motor_state)

        # 发布驱动指令
        drive_cmd = DriveCommand()
        drive_cmd.left_motor_value = left_command
        drive_cmd.right_motor_value = right_command
        drive_cmd.steering_servo_value = steering_command
        drive_cmd.mode = self.mode
        drive_cmd.timestamp = self.get_clock().now().to_msg()
        self.drive_command_pub.publish(drive_cmd)

        # 制动释放检测
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
        self.left_speed_integral = 0.0
        self.left_speed_prev_error = 0.0
        self.right_speed_integral = 0.0
        self.right_speed_prev_error = 0.0
        self.prev_speed_command = 0.0
        self.prev_steering_command = 0.0
        self.prev_left_command = 0.0
        self.prev_right_command = 0.0
        self.mode = 'error'

        response.success = True
        response.message = '紧急制动已激活'
        self.get_logger().warn('紧急制动已激活！')
        return response

    def recover_brake_callback(self, request, response):
        # 检查外部紧急条件是否已解除
        if self._external_emergency:
            response.success = False
            response.message = '外部紧急条件尚未解除，无法恢复'
            return response

        # 检查故障是否仍然存在（过流等）
        if self._fault_active:
            if (self._left_motor_current > self.max_motor_current
                    or self._right_motor_current > self.max_motor_current):
                response.success = False
                response.message = '过流故障尚未解除，无法恢复'
                return response

        # 重置紧急停车标志
        self.emergency_stop = False
        self.brake_active = False
        self.mode = 'idle'
        self._fault_active = False

        # 清除所有PID积分和误差项
        self.speed_integral = 0.0
        self.speed_prev_error = 0.0
        self.steering_integral = 0.0
        self.steering_prev_error = 0.0
        self.left_speed_integral = 0.0
        self.left_speed_prev_error = 0.0
        self.right_speed_integral = 0.0
        self.right_speed_prev_error = 0.0
        self.prev_speed_command = 0.0
        self.prev_steering_command = 0.0
        self.prev_left_command = 0.0
        self.prev_right_command = 0.0

        # 重置目标速度
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.target_speed = 0.0
        self.target_steering_angle = 0.0

        # 重置堵转计时
        self._left_stall_start = None
        self._right_stall_start = None

        response.success = True
        response.message = '紧急制动已恢复，控制器已重置为空闲模式'
        self.get_logger().info('紧急制动已恢复，控制器已重置')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
