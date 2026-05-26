# AGV控制器节点，实现PD位置控制、速度平滑和运动模式管理
# 支持自动导航、手动控制、急停、暂停等多种运行模式
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from agv_interfaces.msg import AGVStatus
from agv_interfaces.srv import ControlAGV


# AGV控制器节点类，负责运动控制、状态管理和速度平滑
class AgvControllerNode(Node):

    # 初始化AGV控制器节点，声明参数并创建订阅、发布和服务
    def __init__(self):
        super().__init__('agv_controller')

        # 声明控制器参数：轮距、速度限制、PD增益、控制频率等
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

        # 读取控制器参数
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

        # 当前位姿状态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        # 当前速度状态
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        # PD控制误差记录
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        # 积分误差（带抗饱和限制）
        self._integral_linear_error = 0.0
        self._integral_angular_error = 0.0
        # 速度平滑用的历史值
        self._prev_desired_linear = 0.0
        self._prev_desired_angular = 0.0
        self._prev_prev_desired_linear = 0.0
        self._prev_prev_desired_angular = 0.0

        # 目标位姿
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.has_target = False

        # 运行模式和状态标志
        self.mode = 'idle'
        self.emergency_stop_flag = False
        # 期望速度指令
        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0
        # 平滑后的速度
        self._smoothed_linear = 0.0
        self._smoothed_angular = 0.0
        # 电池和报警状态
        self.battery_level = 100.0
        self.active_alarms = []

        # 记录最后一次速度指令时间，用于超时检测
        self._last_cmd_vel_time = self.get_clock().now()

        # 订阅速度指令话题（手动模式使用）
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # 订阅目标位姿话题（自动模式使用）
        self.target_pose_sub = self.create_subscription(
            PoseStamped, 'target_pose', self.target_pose_callback, 10)

        # 发布平滑后的速度指令
        self.cmd_vel_out_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        # 发布AGV状态信息
        self.agv_status_pub = self.create_publisher(AGVStatus, 'agv_status', 10)

        # 创建AGV控制服务，支持启动/停止/暂停/急停等命令
        self.control_srv = self.create_service(
            ControlAGV, 'control_agv', self.control_callback)

        # 创建控制循环定时器
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

    # 速度指令回调，手动模式下接收外部速度指令
    def cmd_vel_callback(self, msg):
        if self.mode == 'manual' and not self.emergency_stop_flag:
            self.cmd_vel_linear = msg.linear.x
            self.cmd_vel_angular = msg.angular.z
            self._last_cmd_vel_time = self.get_clock().now()

    # 目标位姿回调，接收导航目标并切换到自动模式
    def target_pose_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        # 从四元数提取偏航角
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.target_theta = math.atan2(siny_cosp, cosy_cosp)
        self.has_target = True
        # 如果当前空闲，自动切换到自动模式
        if self.mode == 'idle':
            self.mode = 'auto'
        # 重置PD控制误差
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self._integral_linear_error = 0.0
        self._integral_angular_error = 0.0

    # AGV控制服务回调，处理启动/停止/暂停/急停等控制命令
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

    # 速度平滑处理，限制加速度和加加速度（jerk）以实现平滑运动
    def _apply_velocity_smoothing(self, desired_linear, desired_angular, dt):
        # 计算最大允许速度变化量
        max_delta_linear = self._velocity_ramp_rate * dt
        max_delta_angular = self._velocity_ramp_rate * dt

        delta_linear = desired_linear - self._smoothed_linear
        delta_angular = desired_angular - self._smoothed_angular

        # 限制速度变化率
        delta_linear = max(-max_delta_linear, min(max_delta_linear, delta_linear))
        delta_angular = max(-max_delta_angular, min(max_delta_angular, delta_angular))

        new_linear = self._smoothed_linear + delta_linear
        new_angular = self._smoothed_angular + delta_angular

        # 计算加速度和加加速度（jerk）
        prev_accel_linear = self._smoothed_linear - self._prev_desired_linear
        prev_accel_angular = self._smoothed_angular - self._prev_desired_angular
        new_accel_linear = new_linear - self._smoothed_linear
        new_accel_angular = new_angular - self._smoothed_angular

        jerk_linear = (new_accel_linear - prev_accel_linear) / dt if dt > 0 else 0.0
        jerk_angular = (new_accel_angular - prev_accel_angular) / dt if dt > 0 else 0.0

        # 限制加加速度（jerk）
        if abs(jerk_linear) > self._max_jerk:
            sign = 1.0 if jerk_linear > 0 else -1.0
            new_accel_linear = prev_accel_linear + sign * self._max_jerk * dt
            new_linear = self._smoothed_linear + new_accel_linear

        if abs(jerk_angular) > self._max_jerk:
            sign = 1.0 if jerk_angular > 0 else -1.0
            new_accel_angular = prev_accel_angular + sign * self._max_jerk * dt
            new_angular = self._smoothed_angular + new_accel_angular

        # 限制最大加速度
        accel_linear = (new_linear - self._smoothed_linear) / dt if dt > 0 else 0.0
        accel_angular = (new_angular - self._smoothed_angular) / dt if dt > 0 else 0.0

        if abs(accel_linear) > self._max_acceleration:
            sign = 1.0 if accel_linear > 0 else -1.0
            new_linear = self._smoothed_linear + sign * self._max_acceleration * dt
        if abs(accel_angular) > self._max_acceleration:
            sign = 1.0 if accel_angular > 0 else -1.0
            new_angular = self._smoothed_angular + sign * self._max_acceleration * dt

        # 更新历史值
        self._prev_prev_desired_linear = self._prev_desired_linear
        self._prev_prev_desired_angular = self._prev_desired_angular
        self._prev_desired_linear = self._smoothed_linear
        self._prev_desired_angular = self._smoothed_angular
        self._smoothed_linear = new_linear
        self._smoothed_angular = new_angular

        return new_linear, new_angular

    # 控制循环主函数，根据当前模式执行控制逻辑
    def control_loop(self):
        dt = 1.0 / self.get_parameter('control_rate').value

        # 急停状态下速度归零
        if self.emergency_stop_flag:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            self._smoothed_linear = 0.0
            self._smoothed_angular = 0.0
        # 自动模式下执行PD控制
        elif self.mode == 'auto' and self.has_target:
            self._compute_pd_control()
        # 手动模式下检测速度指令超时
        elif self.mode == 'manual':
            elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
            if elapsed > self._cmd_vel_timeout:
                self.cmd_vel_linear = 0.0
                self.cmd_vel_angular = 0.0
        else:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0

        # 对速度指令进行平滑处理
        smoothed_linear, smoothed_angular = self._apply_velocity_smoothing(
            self.cmd_vel_linear, self.cmd_vel_angular, dt)

        # 限速并发布速度指令
        cmd_out = Twist()
        cmd_out.linear.x = max(-self.max_linear_vel,
                               min(self.max_linear_vel, smoothed_linear))
        cmd_out.angular.z = max(-self.max_angular_vel,
                                min(self.max_angular_vel, smoothed_angular))
        self.cmd_vel_out_pub.publish(cmd_out)

        # 更新当前速度
        self.current_linear_vel = cmd_out.linear.x
        self.current_angular_vel = cmd_out.angular.z

        # 根据速度更新位姿估计（简单运动学模型）
        self.current_theta += self.current_angular_vel * dt
        self.current_theta = math.atan2(math.sin(self.current_theta),
                                        math.cos(self.current_theta))
        self.current_x += self.current_linear_vel * math.cos(self.current_theta) * dt
        self.current_y += self.current_linear_vel * math.sin(self.current_theta) * dt

        # 发布AGV状态
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

    # PD控制计算，根据目标位姿计算线速度和角速度指令
    def _compute_pd_control(self):
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        # 计算目标方向角和角度误差
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        linear_error = distance
        angular_error = angle_error

        # 计算微分项
        d_linear = (linear_error - self.prev_linear_error)
        d_angular = (angular_error - self.prev_angular_error)

        # 累加积分项
        self._integral_linear_error += linear_error
        self._integral_angular_error += angular_error

        # 积分抗饱和限制
        self._integral_linear_error = max(-self._anti_windup_limit,
                                          min(self._anti_windup_limit, self._integral_linear_error))
        self._integral_angular_error = max(-self._anti_windup_limit,
                                           min(self._anti_windup_limit, self._integral_angular_error))

        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error

        # 角度误差较大时只旋转不平移
        if abs(angle_error) > math.pi / 4:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = self.kp_angular * angular_error + self.kd_angular * d_angular
        else:
            self.cmd_vel_linear = self.kp_linear * linear_error + self.kd_linear * d_linear
            self.cmd_vel_angular = self.kp_angular * angular_error + self.kd_angular * d_angular

        # 到达目标位置后进行最终角度调整
        if distance < 0.05:
            final_angle_error = self.target_theta - self.current_theta
            final_angle_error = math.atan2(math.sin(final_angle_error),
                                           math.cos(final_angle_error))
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = self.kp_angular * final_angle_error
            # 角度也到位则完成任务
            if abs(final_angle_error) < 0.05:
                self.has_target = False
                self.mode = 'idle'
                self.cmd_vel_linear = 0.0
                self.cmd_vel_angular = 0.0
                self._integral_linear_error = 0.0
                self._integral_angular_error = 0.0


# 节点主入口函数
def main(args=None):
    rclpy.init(args=args)
    node = AgvControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
