import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
from agv_interfaces.msg import AGVStatus
from agv_interfaces.action import NavigateTo, Patrol


class AgvNavigatorNode(Node):

    # 导航状态枚举
    NAV_IDLE = 'IDLE'
    NAV_PLANNING = 'PLANNING'
    NAV_FOLLOWING_PATH = 'FOLLOWING_PATH'
    NAV_DIRECT_NAV = 'DIRECT_NAV'
    NAV_AVOIDING_OBSTACLE = 'AVOIDING_OBSTACLE'
    NAV_GOAL_REACHED = 'GOAL_REACHED'
    NAV_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('agv_navigator')

        # 声明原有参数
        self.declare_parameter('goal_tolerance_xy', 0.1)
        self.declare_parameter('goal_tolerance_theta', 0.1)
        self.declare_parameter('obstacle_distance_threshold', 0.3)
        self.declare_parameter('goal_timeout', 300.0)
        self.declare_parameter('obstacle_replan_threshold', 0.2)
        self.declare_parameter('velocity_obstacle_horizon', 2.0)

        # 声明Pure Pursuit参数
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('lookahead_gain', 0.3)
        self.declare_parameter('min_lookahead_distance', 0.2)

        # 声明速度参数
        self.declare_parameter('max_navigation_speed', 0.8)
        self.declare_parameter('approach_speed', 0.3)
        self.declare_parameter('curvature_speed_factor', 2.0)

        # 声明重规划参数
        self.declare_parameter('replan_cooldown', 3.0)
        self.declare_parameter('max_replan_attempts', 3)

        # 读取原有参数
        self.goal_tolerance_xy = self.get_parameter('goal_tolerance_xy').value
        self.goal_tolerance_theta = self.get_parameter('goal_tolerance_theta').value
        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').value
        self._goal_timeout = self.get_parameter('goal_timeout').value
        self._obstacle_replan_threshold = self.get_parameter('obstacle_replan_threshold').value
        self._velocity_obstacle_horizon = self.get_parameter('velocity_obstacle_horizon').value

        # 读取Pure Pursuit参数
        self._lookahead_distance = self.get_parameter('lookahead_distance').value
        self._lookahead_gain = self.get_parameter('lookahead_gain').value
        self._min_lookahead_distance = self.get_parameter('min_lookahead_distance').value

        # 读取速度参数
        self._max_navigation_speed = self.get_parameter('max_navigation_speed').value
        self._approach_speed = self.get_parameter('approach_speed').value
        self._curvature_speed_factor = self.get_parameter('curvature_speed_factor').value

        # 读取重规划参数
        self._replan_cooldown = self.get_parameter('replan_cooldown').value
        self._max_replan_attempts = self.get_parameter('max_replan_attempts').value

        # 当前位姿与速度
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.current_mode = 'idle'
        self.emergency_stop = False
        self.obstacle_detected = False
        self.obstacle_angle = 0.0
        self.obstacle_distance = float('inf')
        self._prev_obstacle_state = False
        self._active_goal_handle = None

        # 里程计数据标志（优先使用里程计）
        self._odom_received = False

        # 激光扫描缓存
        self._scan_cache = []
        self._scan_cache_time = 0.0
        self._scan_cache_ttl = 0.5

        # 路径数据
        self.planned_path = None
        self.path_received = False

        # 导航状态机
        self.nav_state = self.NAV_IDLE
        self._prev_nav_state = self.NAV_IDLE

        # 路径进度追踪
        self._current_waypoint_index = 0
        self._total_waypoints = 0

        # 重规划控制
        self._last_replan_time = 0.0
        self._replan_attempts = 0

        # 订阅AGVStatus（保留作为备用位置源）
        self.agv_status_sub = self.create_subscription(
            AGVStatus, 'agv_status', self.agv_status_callback, 10)

        # 订阅里程计（更可靠的位置源）
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.laser_scan_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_scan_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.target_pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)

        # 导航状态发布
        self.nav_state_pub = self.create_publisher(String, 'nav_state', 10)

        # 路径进度发布
        self.path_progress_pub = self.create_publisher(Float32, 'path_progress', 10)

        self.planned_path_sub = self.create_subscription(
            Path, 'planned_path', self.planned_path_callback, 10)

        self.path_plan_client = self.create_client(Trigger, '/path_plan')

        self.navigate_action = ActionServer(
            self, NavigateTo, 'navigate_to',
            execute_callback=self.navigate_to_execute,
            goal_callback=self.navigate_goal_callback,
            cancel_callback=self.navigate_cancel_callback)

        self.patrol_action = ActionServer(
            self, Patrol, 'patrol',
            execute_callback=self.patrol_execute,
            goal_callback=self.patrol_goal_callback,
            cancel_callback=self.patrol_cancel_callback)

    def nav_state_callback(self, new_state):
        """导航状态机转换，记录日志并发布状态"""
        if new_state != self.nav_state:
            self._prev_nav_state = self.nav_state
            self.nav_state = new_state
            self.get_logger().info(
                f'导航状态转换: {self._prev_nav_state} -> {self.nav_state}')
            state_msg = String()
            state_msg.data = self.nav_state
            self.nav_state_pub.publish(state_msg)

    def odom_callback(self, msg):
        """里程计回调，优先使用里程计数据更新位姿"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z
        self._odom_received = True

    def agv_status_callback(self, msg):
        """AGV状态回调，仅在未收到里程计时使用"""
        if not self._odom_received:
            self.current_x = msg.x
            self.current_y = msg.y
            self.current_theta = msg.theta
            self.current_linear_vel = msg.linear_velocity
            self.current_angular_vel = msg.angular_velocity
        self.current_mode = msg.mode
        self.emergency_stop = msg.emergency_stop

    def laser_scan_callback(self, msg):
        self.obstacle_detected = False
        self.obstacle_angle = 0.0
        self.obstacle_distance = float('inf')
        if len(msg.ranges) == 0:
            return

        now = time.time()
        self._scan_cache = list(msg.ranges)
        self._scan_cache_time = now

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

    def planned_path_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('收到空路径')
            return
        self.planned_path = msg
        self.path_received = True
        self._total_waypoints = len(msg.poses)
        self._current_waypoint_index = 0
        self._replan_attempts = 0
        self.get_logger().info(f'收到规划路径，包含 {len(msg.poses)} 个路径点')

    def _compute_velocity_obstacle_avoidance(self):
        if not self.obstacle_detected:
            return 0.0, 0.0

        avoidance_angle = self.obstacle_angle
        avoidance_strength = max(0.0, 1.0 - self.obstacle_distance / self.obstacle_distance_threshold)

        if abs(avoidance_angle) < 0.1:
            avoidance_angular = 0.8 * (1.0 if avoidance_angle >= 0 else -1.0)
        else:
            avoidance_angular = -0.8 * math.copysign(1.0, avoidance_angle)

        avoidance_angular *= avoidance_strength
        avoidance_linear = 0.1 * (1.0 - avoidance_strength)

        return avoidance_linear, avoidance_angular

    def _should_replan(self):
        obstacle_changed = self.obstacle_detected != self._prev_obstacle_state
        if obstacle_changed and self.obstacle_detected:
            self._prev_obstacle_state = self.obstacle_detected
            return True
        if self.obstacle_detected and self.obstacle_distance < self._obstacle_replan_threshold:
            return True
        self._prev_obstacle_state = self.obstacle_detected
        return False

    def _request_replan(self, target_x, target_y, target_theta):
        """请求重新规划路径，带冷却时间控制"""
        now = time.time()
        if now - self._last_replan_time < self._replan_cooldown:
            self.get_logger().debug('重规划冷却中，跳过本次请求')
            return False

        if self._replan_attempts >= self._max_replan_attempts:
            self.get_logger().warn(
                f'已达到最大重规划次数 {self._max_replan_attempts}，切换到直接导航模式')
            return False

        self._last_replan_time = now
        self._replan_attempts += 1
        self.get_logger().info(
            f'请求重规划（第 {self._replan_attempts}/{self._max_replan_attempts} 次）')

        # 发布目标位姿触发重规划
        target_pose_msg = PoseStamped()
        target_pose_msg.header.stamp = self.get_clock().now().to_msg()
        target_pose_msg.header.frame_id = 'map'
        target_pose_msg.pose.position.x = target_x
        target_pose_msg.pose.position.y = target_y
        target_pose_msg.pose.position.z = 0.0
        target_pose_msg.pose.orientation.z = math.sin(target_theta / 2.0)
        target_pose_msg.pose.orientation.w = math.cos(target_theta / 2.0)
        self.target_pose_pub.publish(target_pose_msg)

        return True

    def find_lookahead_point(self, path_poses):
        """在路径上寻找前视点，基于动态前视距离"""
        current_speed = abs(self.current_linear_vel)
        lookahead_dist = self._min_lookahead_distance + self._lookahead_gain * current_speed
        lookahead_dist = max(lookahead_dist, self._min_lookahead_distance)

        # 从当前路径点索引开始搜索
        start_idx = max(0, self._current_waypoint_index)
        best_point = None
        best_idx = start_idx

        for i in range(start_idx, len(path_poses)):
            wp_x = path_poses[i].pose.position.x
            wp_y = path_poses[i].pose.position.y
            dx = wp_x - self.current_x
            dy = wp_y - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist >= lookahead_dist:
                best_point = (wp_x, wp_y)
                best_idx = i
                break

        # 如果没有找到足够远的点，使用最后一个路径点
        if best_point is None and len(path_poses) > 0:
            last = path_poses[-1]
            best_point = (last.pose.position.x, last.pose.position.y)
            best_idx = len(path_poses) - 1

        return best_point, best_idx

    def compute_curvature(self, path_poses, lookahead_idx):
        """计算前视点处的路径曲率"""
        if lookahead_idx < 1 or lookahead_idx >= len(path_poses):
            return 0.0

        p1 = path_poses[lookahead_idx - 1].pose.position
        p2 = path_poses[lookahead_idx].pose.position

        if lookahead_idx + 1 < len(path_poses):
            p3 = path_poses[lookahead_idx + 1].pose.position
        else:
            return 0.0

        # 使用三点计算曲率：Menger曲率
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        x3, y3 = p3.x, p3.y

        area = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
        d12 = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        d23 = math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)
        d13 = math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)

        if area < 1e-6 or d12 < 1e-6 or d23 < 1e-6 or d13 < 1e-6:
            return 0.0

        curvature = 2.0 * area / (d12 * d23 * d13)
        return curvature

    def compute_pure_pursuit(self, lookahead_point, curvature):
        """使用Pure Pursuit几何关系计算速度指令"""
        if lookahead_point is None:
            return 0.0, 0.0

        lx, ly = lookahead_point
        dx = lx - self.current_x
        dy = ly - self.current_y

        # 将目标点转换到车辆坐标系
        cos_theta = math.cos(self.current_theta)
        sin_theta = math.sin(self.current_theta)
        local_x = dx * cos_theta + dy * sin_theta
        local_y = -dx * sin_theta + dy * cos_theta

        # 计算到前视点的距离
        L = math.sqrt(local_x * local_x + local_y * local_y)
        if L < 1e-6:
            return 0.0, 0.0

        # Pure Pursuit核心公式：角速度 = 2 * v * local_y / L^2
        # 基于曲率计算目标速度
        target_speed = self._max_navigation_speed / (1.0 + curvature * self._curvature_speed_factor)

        # 计算到最终目标的距离，接近时减速
        if self._active_goal_handle is not None:
            goal_dx = self._active_goal_handle.request.target_x - self.current_x
            goal_dy = self._active_goal_handle.request.target_y - self.current_y
            goal_dist = math.sqrt(goal_dx * goal_dx + goal_dy * goal_dy)
            if goal_dist < 0.5:
                target_speed = min(target_speed, self._approach_speed)

        target_speed = max(0.05, min(self._max_navigation_speed, target_speed))

        angular_vel = 2.0 * target_speed * local_y / (L * L)
        angular_vel = max(-1.5, min(1.5, angular_vel))

        return target_speed, angular_vel

    def _publish_path_progress(self):
        """发布路径进度信息"""
        if self._total_waypoints > 0:
            progress = (self._current_waypoint_index / self._total_waypoints) * 100.0
            progress_msg = Float32()
            progress_msg.data = progress
            self.path_progress_pub.publish(progress_msg)

    def navigate_goal_callback(self, goal_request):
        self.get_logger().info('收到导航目标请求')
        if self._active_goal_handle is not None:
            self.get_logger().warn('已有活跃目标，抢占')
        return GoalResponse.ACCEPT

    def navigate_cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消导航请求')
        return CancelResponse.ACCEPT

    def patrol_goal_callback(self, goal_request):
        self.get_logger().info('收到巡逻目标请求')
        return GoalResponse.ACCEPT

    def patrol_cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消巡逻请求')
        return CancelResponse.ACCEPT

    async def navigate_to_execute(self, goal_handle):
        self._active_goal_handle = goal_handle
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        target_theta = goal_handle.request.target_theta
        tol_xy = goal_handle.request.tolerance_xy if goal_handle.request.tolerance_xy > 0 else self.goal_tolerance_xy
        tol_theta = goal_handle.request.tolerance_theta if goal_handle.request.tolerance_theta > 0 else self.goal_tolerance_theta

        self.get_logger().info(
            f'导航到 ({target_x}, {target_y}, {target_theta})')

        # 状态：规划中
        self.nav_state_callback(self.NAV_PLANNING)

        # 发布目标位姿给路径规划器
        target_pose_msg = PoseStamped()
        target_pose_msg.header.stamp = self.get_clock().now().to_msg()
        target_pose_msg.header.frame_id = 'map'
        target_pose_msg.pose.position.x = target_x
        target_pose_msg.pose.position.y = target_y
        target_pose_msg.pose.position.z = 0.0
        target_pose_msg.pose.orientation.z = math.sin(target_theta / 2.0)
        target_pose_msg.pose.orientation.w = math.cos(target_theta / 2.0)
        self.target_pose_pub.publish(target_pose_msg)
        self.get_logger().info('已发布目标位姿到 target_pose 话题，等待路径规划器响应')

        # 等待路径规划器发布规划路径，最多等待5秒
        self.planned_path = None
        self.path_received = False
        self._replan_attempts = 0
        self._last_replan_time = 0.0
        wait_start = time.time()
        path_wait_timeout = 5.0

        while not self.path_received and (time.time() - wait_start) < path_wait_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.path_received and self.planned_path is not None:
            self.get_logger().info(
                f'收到规划路径，共 {len(self.planned_path.poses)} 个路径点，开始沿路径导航')
            # 状态：沿路径导航
            self.nav_state_callback(self.NAV_FOLLOWING_PATH)
            result = await self.follow_planned_path(goal_handle, self.planned_path)
            self._active_goal_handle = None
            return result
        else:
            self.get_logger().warn(
                f'未在 {path_wait_timeout:.1f} 秒内收到规划路径，回退到直接导航模式')

        # 状态：直接导航
        self.nav_state_callback(self.NAV_DIRECT_NAV)

        # 直接导航模式（回退方案）
        result = NavigateTo.Result()
        feedback = NavigateTo.Feedback()
        start_time = time.time()

        while True:
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Navigation canceled'
                self.nav_state_callback(self.NAV_IDLE)
                self._active_goal_handle = None
                return result

            if self.emergency_stop:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = 'Emergency stop activated'
                self.nav_state_callback(self.NAV_ERROR)
                self._active_goal_handle = None
                return result

            elapsed = time.time() - start_time
            if elapsed > self._goal_timeout:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = f'Goal timeout after {self._goal_timeout:.0f}s'
                self.nav_state_callback(self.NAV_ERROR)
                self._active_goal_handle = None
                return result

            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_theta = self.current_theta
            feedback.distance_remaining = distance
            if abs(self.current_linear_vel) > 0.01:
                feedback.estimated_time = distance / abs(self.current_linear_vel)
            else:
                feedback.estimated_time = float('inf')
            goal_handle.publish_feedback(feedback)

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
                    self.nav_state_callback(self.NAV_GOAL_REACHED)
                    self._active_goal_handle = None
                    return result
                else:
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 1.0 * final_angle_error
                    self.cmd_vel_pub.publish(cmd)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue

            if self.obstacle_detected:
                self.nav_state_callback(self.NAV_AVOIDING_OBSTACLE)
                avoid_linear, avoid_angular = self._compute_velocity_obstacle_avoidance()
                cmd = Twist()
                cmd.linear.x = avoid_linear
                cmd.angular.z = avoid_angular
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            self.nav_state_callback(self.NAV_DIRECT_NAV)
            cmd = Twist()
            if abs(angle_error) > math.pi / 6:
                cmd.linear.x = 0.1
                cmd.angular.z = 1.5 * angle_error
            else:
                cmd.linear.x = min(self._max_navigation_speed, 0.5 * distance)
                cmd.angular.z = 1.5 * angle_error
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    async def follow_planned_path(self, goal_handle, path):
        """使用Pure Pursuit算法沿规划路径导航"""
        result = NavigateTo.Result()
        feedback = NavigateTo.Feedback()
        start_time = time.time()
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        target_theta = goal_handle.request.target_theta
        tol_xy = goal_handle.request.tolerance_xy if goal_handle.request.tolerance_xy > 0 else self.goal_tolerance_xy
        tol_theta = goal_handle.request.tolerance_theta if goal_handle.request.tolerance_theta > 0 else self.goal_tolerance_theta

        path_poses = path.poses
        self._total_waypoints = len(path_poses)
        self._current_waypoint_index = 0

        while self._current_waypoint_index < len(path_poses):
            # 检查取消请求
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = '沿路径导航被取消'
                self.nav_state_callback(self.NAV_IDLE)
                return result

            # 检查紧急停车
            if self.emergency_stop:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = '紧急停车已激活'
                self.nav_state_callback(self.NAV_ERROR)
                return result

            # 检查超时
            elapsed = time.time() - start_time
            if elapsed > self._goal_timeout:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = f'导航超时，已用时 {self._goal_timeout:.0f} 秒'
                self.nav_state_callback(self.NAV_ERROR)
                return result

            # 检查是否到达最终目标
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance_to_goal = math.sqrt(dx * dx + dy * dy)

            if distance_to_goal < tol_xy:
                final_angle_error = target_theta - self.current_theta
                final_angle_error = math.atan2(math.sin(final_angle_error),
                                               math.cos(final_angle_error))
                if abs(final_angle_error) < tol_theta:
                    self._stop_robot()
                    goal_handle.succeed()
                    result.success = True
                    result.message = '沿规划路径到达目标'
                    result.final_x = self.current_x
                    result.final_y = self.current_y
                    result.final_theta = self.current_theta
                    self.nav_state_callback(self.NAV_GOAL_REACHED)
                    return result
                else:
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 1.0 * final_angle_error
                    self.cmd_vel_pub.publish(cmd)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue

            # 障碍物检测与重规划
            if self.obstacle_detected:
                self.nav_state_callback(self.NAV_AVOIDING_OBSTACLE)

                # 尝试请求重规划
                replan_requested = self._request_replan(target_x, target_y, target_theta)

                if replan_requested:
                    # 等待新路径
                    self.path_received = False
                    replan_wait_start = time.time()
                    replan_wait_timeout = 3.0
                    while not self.path_received and (time.time() - replan_wait_start) < replan_wait_timeout:
                        rclpy.spin_once(self, timeout_sec=0.1)

                    if self.path_received and self.planned_path is not None:
                        self.get_logger().info('重规划成功，切换到新路径')
                        self.nav_state_callback(self.NAV_FOLLOWING_PATH)
                        path_poses = self.planned_path.poses
                        self._total_waypoints = len(path_poses)
                        self._current_waypoint_index = 0
                        continue

                # 重规划失败次数已达上限，切换到直接导航模式
                if self._replan_attempts >= self._max_replan_attempts:
                    self.get_logger().warn('重规划次数已达上限，切换到直接导航模式')
                    self.nav_state_callback(self.NAV_DIRECT_NAV)
                    direct_result = await self._direct_navigate_to_goal(
                        goal_handle, target_x, target_y, target_theta, tol_xy, tol_theta, start_time)
                    return direct_result

                # 执行避障动作
                avoid_linear, avoid_angular = self._compute_velocity_obstacle_avoidance()
                cmd = Twist()
                cmd.linear.x = avoid_linear
                cmd.angular.z = avoid_angular
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            self.nav_state_callback(self.NAV_FOLLOWING_PATH)

            # 更新当前路径点索引（跳过已经过的路径点）
            while self._current_waypoint_index < len(path_poses):
                wp = path_poses[self._current_waypoint_index]
                wp_x = wp.pose.position.x
                wp_y = wp.pose.position.y
                wp_dx = wp_x - self.current_x
                wp_dy = wp_y - self.current_y
                wp_dist = math.sqrt(wp_dx * wp_dx + wp_dy * wp_dy)
                if wp_dist < 0.2 and self._current_waypoint_index < len(path_poses) - 1:
                    self._current_waypoint_index += 1
                else:
                    break

            # Pure Pursuit：寻找前视点
            lookahead_point, lookahead_idx = self.find_lookahead_point(path_poses)

            if lookahead_point is None:
                self._current_waypoint_index += 1
                continue

            # 计算曲率
            curvature = self.compute_curvature(path_poses, lookahead_idx)

            # Pure Pursuit：计算速度指令
            linear_vel, angular_vel = self.compute_pure_pursuit(lookahead_point, curvature)

            # 发布速度指令
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)

            # 更新路径点索引
            self._current_waypoint_index = max(self._current_waypoint_index, lookahead_idx)

            # 发布反馈
            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_theta = self.current_theta
            feedback.distance_remaining = distance_to_goal
            if abs(self.current_linear_vel) > 0.01:
                feedback.estimated_time = distance_to_goal / abs(self.current_linear_vel)
            else:
                feedback.estimated_time = float('inf')
            goal_handle.publish_feedback(feedback)

            # 发布路径进度
            self._publish_path_progress()

            rclpy.spin_once(self, timeout_sec=0.05)

        # 所有路径点已遍历但未到达目标（理论上不应发生）
        self._stop_robot()
        goal_handle.abort()
        result.success = False
        result.message = '路径点已遍历但未到达目标'
        self.nav_state_callback(self.NAV_ERROR)
        return result

    async def _direct_navigate_to_goal(self, goal_handle, target_x, target_y,
                                        target_theta, tol_xy, tol_theta, start_time):
        """直接导航到目标（重规划失败后的回退方案）"""
        result = NavigateTo.Result()
        feedback = NavigateTo.Feedback()

        while True:
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = '直接导航被取消'
                self.nav_state_callback(self.NAV_IDLE)
                return result

            if self.emergency_stop:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = '紧急停车已激活'
                self.nav_state_callback(self.NAV_ERROR)
                return result

            elapsed = time.time() - start_time
            if elapsed > self._goal_timeout:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = f'导航超时，已用时 {self._goal_timeout:.0f} 秒'
                self.nav_state_callback(self.NAV_ERROR)
                return result

            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_theta = self.current_theta
            feedback.distance_remaining = distance
            if abs(self.current_linear_vel) > 0.01:
                feedback.estimated_time = distance / abs(self.current_linear_vel)
            else:
                feedback.estimated_time = float('inf')
            goal_handle.publish_feedback(feedback)

            if distance < tol_xy:
                final_angle_error = target_theta - self.current_theta
                final_angle_error = math.atan2(math.sin(final_angle_error),
                                               math.cos(final_angle_error))
                if abs(final_angle_error) < tol_theta:
                    self._stop_robot()
                    goal_handle.succeed()
                    result.success = True
                    result.message = '直接导航到达目标'
                    result.final_x = self.current_x
                    result.final_y = self.current_y
                    result.final_theta = self.current_theta
                    self.nav_state_callback(self.NAV_GOAL_REACHED)
                    return result
                else:
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 1.0 * final_angle_error
                    self.cmd_vel_pub.publish(cmd)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue

            if self.obstacle_detected:
                avoid_linear, avoid_angular = self._compute_velocity_obstacle_avoidance()
                cmd = Twist()
                cmd.linear.x = avoid_linear
                cmd.angular.z = avoid_angular
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            cmd = Twist()
            if abs(angle_error) > math.pi / 6:
                cmd.linear.x = 0.1
                cmd.angular.z = 1.5 * angle_error
            else:
                cmd.linear.x = min(self._max_navigation_speed, 0.5 * distance)
                cmd.angular.z = 1.5 * angle_error
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    async def patrol_execute(self, goal_handle):
        waypoints_x = goal_handle.request.waypoints_x
        waypoints_y = goal_handle.request.waypoints_y
        waypoints_theta = goal_handle.request.waypoints_theta
        total_loops = goal_handle.request.loops

        result = Patrol.Result()
        feedback = Patrol.Feedback()
        completed_loops = 0

        for loop in range(total_loops):
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Patrol canceled'
                result.completed_loops = completed_loops
                self.nav_state_callback(self.NAV_IDLE)
                return result

            if self.emergency_stop:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = 'Emergency stop during patrol'
                result.completed_loops = completed_loops
                self.nav_state_callback(self.NAV_ERROR)
                return result

            for i in range(len(waypoints_x)):
                if goal_handle.is_cancel_requested:
                    self._stop_robot()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Patrol canceled'
                    result.completed_loops = completed_loops
                    self.nav_state_callback(self.NAV_IDLE)
                    return result

                feedback.current_x = self.current_x
                feedback.current_y = self.current_y
                feedback.current_waypoint_index = i
                feedback.completed_loops = completed_loops
                goal_handle.publish_feedback(feedback)

                nav_goal = NavigateTo.Goal()
                nav_goal.target_x = waypoints_x[i]
                nav_goal.target_y = waypoints_y[i]
                nav_goal.target_theta = waypoints_theta[i] if i < len(waypoints_theta) else 0.0
                nav_goal.tolerance_xy = self.goal_tolerance_xy
                nav_goal.tolerance_theta = self.goal_tolerance_theta

                reached = await self._navigate_to_waypoint(
                    nav_goal.target_x, nav_goal.target_y, nav_goal.target_theta,
                    nav_goal.tolerance_xy, nav_goal.tolerance_theta, goal_handle)

                if not reached:
                    self._stop_robot()
                    goal_handle.abort()
                    result.success = False
                    result.message = f'Failed to reach waypoint {i}'
                    result.completed_loops = completed_loops
                    self.nav_state_callback(self.NAV_ERROR)
                    return result

            completed_loops += 1

        self._stop_robot()
        goal_handle.succeed()
        result.success = True
        result.message = 'Patrol completed'
        result.completed_loops = completed_loops
        self.nav_state_callback(self.NAV_GOAL_REACHED)
        return result

    async def _navigate_to_waypoint(self, target_x, target_y, target_theta,
                                     tol_xy, tol_theta, parent_goal_handle):
        start_time = time.time()
        self.nav_state_callback(self.NAV_DIRECT_NAV)
        while True:
            if parent_goal_handle.is_cancel_requested:
                return False

            if self.emergency_stop:
                return False

            elapsed = time.time() - start_time
            if elapsed > self._goal_timeout:
                self.get_logger().warn(f'航点导航超时，已用时 {self._goal_timeout:.0f} 秒')
                return False

            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

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

            if self.obstacle_detected:
                avoid_linear, avoid_angular = self._compute_velocity_obstacle_avoidance()
                cmd = Twist()
                cmd.linear.x = avoid_linear
                cmd.angular.z = avoid_angular
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            cmd = Twist()
            if abs(angle_error) > math.pi / 6:
                cmd.linear.x = 0.1
                cmd.angular.z = 1.5 * angle_error
            else:
                cmd.linear.x = min(self._max_navigation_speed, 0.5 * distance)
                cmd.angular.z = 1.5 * angle_error
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AgvNavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
