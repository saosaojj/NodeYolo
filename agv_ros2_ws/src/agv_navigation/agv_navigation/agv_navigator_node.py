# AGV导航节点，实现点到点导航和巡逻功能
# 提供NavigateTo和Patrol两个动作服务器，支持障碍物检测和避障
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from agv_interfaces.msg import AGVStatus
from agv_interfaces.action import NavigateTo, Patrol


# AGV导航节点类，管理导航动作、障碍物检测和速度避障
class AgvNavigatorNode(Node):

    # 初始化导航节点，声明参数并创建动作服务器和订阅
    def __init__(self):
        super().__init__('agv_navigator')

        # 声明导航参数：目标容差、障碍物阈值、超时等
        self.declare_parameter('goal_tolerance_xy', 0.1)
        self.declare_parameter('goal_tolerance_theta', 0.1)
        self.declare_parameter('obstacle_distance_threshold', 0.3)
        self.declare_parameter('goal_timeout', 300.0)
        self.declare_parameter('obstacle_replan_threshold', 0.2)
        self.declare_parameter('velocity_obstacle_horizon', 2.0)

        # 读取参数值
        self.goal_tolerance_xy = self.get_parameter('goal_tolerance_xy').value
        self.goal_tolerance_theta = self.get_parameter('goal_tolerance_theta').value
        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').value
        self._goal_timeout = self.get_parameter('goal_timeout').value
        self._obstacle_replan_threshold = self.get_parameter('obstacle_replan_threshold').value
        self._velocity_obstacle_horizon = self.get_parameter('velocity_obstacle_horizon').value

        # 当前位姿和速度状态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.current_mode = 'idle'
        self.emergency_stop = False
        # 障碍物检测状态
        self.obstacle_detected = False
        self.obstacle_angle = 0.0
        self.obstacle_distance = float('inf')
        self._prev_obstacle_state = False
        # 当前活跃的目标句柄
        self._active_goal_handle = None

        # 激光扫描缓存，用于减少重复处理
        self._scan_cache = []
        self._scan_cache_time = 0.0
        self._scan_cache_ttl = 0.5

        # 订阅AGV状态话题
        self.agv_status_sub = self.create_subscription(
            AGVStatus, 'agv_status', self.agv_status_callback, 10)
        # 订阅激光扫描话题
        self.laser_scan_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_scan_callback, 10)

        # 发布速度指令
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 创建NavigateTo动作服务器，处理点到点导航请求
        self.navigate_action = ActionServer(
            self, NavigateTo, 'navigate_to',
            execute_callback=self.navigate_to_execute,
            goal_callback=self.navigate_goal_callback,
            cancel_callback=self.navigate_cancel_callback)

        # 创建Patrol动作服务器，处理巡逻请求
        self.patrol_action = ActionServer(
            self, Patrol, 'patrol',
            execute_callback=self.patrol_execute,
            goal_callback=self.patrol_goal_callback,
            cancel_callback=self.patrol_cancel_callback)

    # AGV状态回调，更新当前位姿、速度和模式
    def agv_status_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        self.current_mode = msg.mode
        self.emergency_stop = msg.emergency_stop
        self.current_linear_vel = msg.linear_velocity
        self.current_angular_vel = msg.angular_velocity

    # 激光扫描回调，检测前方障碍物的距离和角度
    def laser_scan_callback(self, msg):
        self.obstacle_detected = False
        self.obstacle_angle = 0.0
        self.obstacle_distance = float('inf')
        if len(msg.ranges) == 0:
            return

        now = time.time()
        self._scan_cache = list(msg.ranges)
        self._scan_cache_time = now

        # 遍历扫描点，查找最近的障碍物
        angle = msg.angle_min
        angle_increment = msg.angle_increment
        min_dist = float('inf')
        min_angle = 0.0
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and r < self.obstacle_distance_threshold:
                self.obstacle_detected = True
                if r < min_dist:
                    min_dist = r
                    min_angle = angle
            angle += angle_increment
        if self.obstacle_detected:
            self.obstacle_angle = min_angle
            self.obstacle_distance = min_dist

    # 计算速度避障指令，根据障碍物位置和距离生成避障速度
    def _compute_velocity_obstacle_avoidance(self):
        if not self.obstacle_detected:
            return 0.0, 0.0

        avoidance_angle = self.obstacle_angle
        # 避障强度随障碍物距离减小而增大
        avoidance_strength = max(0.0, 1.0 - self.obstacle_distance / self.obstacle_distance_threshold)

        # 根据障碍物角度决定避障旋转方向
        if abs(avoidance_angle) < 0.1:
            avoidance_angular = 0.8 * (1.0 if avoidance_angle >= 0 else -1.0)
        else:
            avoidance_angular = -0.8 * math.copysign(1.0, avoidance_angle)

        avoidance_angular *= avoidance_strength
        # 障碍物越近线速度越低
        avoidance_linear = 0.1 * (1.0 - avoidance_strength)

        return avoidance_linear, avoidance_angular

    # 判断是否需要重新规划路径
    def _should_replan(self):
        obstacle_changed = self.obstacle_detected != self._prev_obstacle_state
        if obstacle_changed and self.obstacle_detected:
            self._prev_obstacle_state = self.obstacle_detected
            return True
        if self.obstacle_detected and self.obstacle_distance < self._obstacle_replan_threshold:
            return True
        self._prev_obstacle_state = self.obstacle_detected
        return False

    # 导航目标接收回调，接受新的导航请求
    def navigate_goal_callback(self, goal_request):
        self.get_logger().info('Received navigate_to goal request')
        if self._active_goal_handle is not None:
            self.get_logger().warn('A goal is already active, preempting')
        return GoalResponse.ACCEPT

    # 导航取消回调，接受取消请求
    def navigate_cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for navigate_to')
        return CancelResponse.ACCEPT

    # 巡逻目标接收回调
    def patrol_goal_callback(self, goal_request):
        self.get_logger().info('Received patrol goal request')
        return GoalResponse.ACCEPT

    # 巡逻取消回调
    def patrol_cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for patrol')
        return CancelResponse.ACCEPT

    # 点到点导航动作执行函数，循环控制AGV向目标移动
    async def navigate_to_execute(self, goal_handle):
        self._active_goal_handle = goal_handle
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        target_theta = goal_handle.request.target_theta
        tol_xy = goal_handle.request.tolerance_xy if goal_handle.request.tolerance_xy > 0 else self.goal_tolerance_xy
        tol_theta = goal_handle.request.tolerance_theta if goal_handle.request.tolerance_theta > 0 else self.goal_tolerance_theta

        self.get_logger().info(
            f'Navigating to ({target_x}, {target_y}, {target_theta})')

        result = NavigateTo.Result()
        feedback = NavigateTo.Feedback()
        start_time = time.time()

        while True:
            # 检查取消请求
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Navigation canceled'
                self._active_goal_handle = None
                return result

            # 检查急停状态
            if self.emergency_stop:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = 'Emergency stop activated'
                self._active_goal_handle = None
                return result

            # 检查导航超时
            elapsed = time.time() - start_time
            if elapsed > self._goal_timeout:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = f'Goal timeout after {self._goal_timeout:.0f}s'
                self._active_goal_handle = None
                return result

            # 计算到目标的距离和角度
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            # 发布导航反馈
            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_theta = self.current_theta
            feedback.distance_remaining = distance
            if abs(self.current_linear_vel) > 0.01:
                feedback.estimated_time = distance / abs(self.current_linear_vel)
            else:
                feedback.estimated_time = float('inf')
            goal_handle.publish_feedback(feedback)

            # 到达目标位置后进行最终角度调整
            if distance < tol_xy:
                final_angle_error = target_theta - self.current_theta
                final_angle_error = math.atan2(math.sin(final_angle_error),
                                               math.cos(final_angle_error))
                if abs(final_angle_error) < tol_theta:
                    self._stop_robot()
                    goal_handle.succeed()
                    result.success = True
                    result.message = 'Goal reached'
                    result.final_x = self.current_x
                    result.final_y = self.current_y
                    result.final_theta = self.current_theta
                    self._active_goal_handle = None
                    return result
                else:
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 1.0 * final_angle_error
                    self.cmd_vel_pub.publish(cmd)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue

            # 检测到障碍物时执行避障
            if self.obstacle_detected:
                avoid_linear, avoid_angular = self._compute_velocity_obstacle_avoidance()
                cmd = Twist()
                cmd.linear.x = avoid_linear
                cmd.angular.z = avoid_angular
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            # 正常导航：根据角度误差决定运动策略
            cmd = Twist()
            if abs(angle_error) > math.pi / 6:
                # 角度偏差大时慢速旋转
                cmd.linear.x = 0.1
                cmd.angular.z = 1.5 * angle_error
            else:
                # 角度偏差小时接近目标
                cmd.linear.x = min(0.8, 0.5 * distance)
                cmd.angular.z = 1.5 * angle_error
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    # 巡逻动作执行函数，依次导航到每个路径点并循环
    async def patrol_execute(self, goal_handle):
        waypoints_x = goal_handle.request.waypoints_x
        waypoints_y = goal_handle.request.waypoints_y
        waypoints_theta = goal_handle.request.waypoints_theta
        total_loops = goal_handle.request.loops

        result = Patrol.Result()
        feedback = Patrol.Feedback()
        completed_loops = 0

        for loop in range(total_loops):
            # 检查取消请求
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Patrol canceled'
                result.completed_loops = completed_loops
                return result

            # 检查急停状态
            if self.emergency_stop:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = 'Emergency stop during patrol'
                result.completed_loops = completed_loops
                return result

            # 依次导航到每个路径点
            for i in range(len(waypoints_x)):
                if goal_handle.is_cancel_requested:
                    self._stop_robot()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Patrol canceled'
                    result.completed_loops = completed_loops
                    return result

                # 发布巡逻反馈
                feedback.current_x = self.current_x
                feedback.current_y = self.current_y
                feedback.current_waypoint_index = i
                feedback.completed_loops = completed_loops
                goal_handle.publish_feedback(feedback)

                # 构建导航目标
                nav_goal = NavigateTo.Goal()
                nav_goal.target_x = waypoints_x[i]
                nav_goal.target_y = waypoints_y[i]
                nav_goal.target_theta = waypoints_theta[i] if i < len(waypoints_theta) else 0.0
                nav_goal.tolerance_xy = self.goal_tolerance_xy
                nav_goal.tolerance_theta = self.goal_tolerance_theta

                # 执行单点导航
                reached = await self._navigate_to_waypoint(
                    nav_goal.target_x, nav_goal.target_y, nav_goal.target_theta,
                    nav_goal.tolerance_xy, nav_goal.tolerance_theta, goal_handle)

                if not reached:
                    self._stop_robot()
                    goal_handle.abort()
                    result.success = False
                    result.message = f'Failed to reach waypoint {i}'
                    result.completed_loops = completed_loops
                    return result

            completed_loops += 1

        # 巡逻完成
        self._stop_robot()
        goal_handle.succeed()
        result.success = True
        result.message = 'Patrol completed'
        result.completed_loops = completed_loops
        return result

    # 导航到单个路径点的内部方法
    async def _navigate_to_waypoint(self, target_x, target_y, target_theta,
                                     tol_xy, tol_theta, parent_goal_handle):
        start_time = time.time()
        while True:
            # 检查父级取消请求
            if parent_goal_handle.is_cancel_requested:
                return False

            # 检查急停
            if self.emergency_stop:
                return False

            # 检查超时
            elapsed = time.time() - start_time
            if elapsed > self._goal_timeout:
                self.get_logger().warn(f'Waypoint navigation timeout after {self._goal_timeout:.0f}s')
                return False

            # 计算到目标的距离和角度
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            # 到达目标位置后进行最终角度调整
            if distance < tol_xy:
                final_angle_error = target_theta - self.current_theta
                final_angle_error = math.atan2(math.sin(final_angle_error),
                                               math.cos(final_angle_error))
                if abs(final_angle_error) < tol_theta:
                    self._stop_robot()
                    return True
                else:
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 1.0 * final_angle_error
                    self.cmd_vel_pub.publish(cmd)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue

            # 检测到障碍物时执行避障
            if self.obstacle_detected:
                avoid_linear, avoid_angular = self._compute_velocity_obstacle_avoidance()
                cmd = Twist()
                cmd.linear.x = avoid_linear
                cmd.angular.z = avoid_angular
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            # 正常导航控制
            cmd = Twist()
            if abs(angle_error) > math.pi / 6:
                cmd.linear.x = 0.1
                cmd.angular.z = 1.5 * angle_error
            else:
                cmd.linear.x = min(0.8, 0.5 * distance)
                cmd.angular.z = 1.5 * angle_error
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    # 停止机器人运动，发布零速度指令
    def _stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


# 节点主入口函数
def main(args=None):
    rclpy.init(args=args)
    node = AgvNavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
