import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from agv_interfaces.msg import AGVStatus
from agv_interfaces.action import NavigateTo, Patrol


class AgvNavigatorNode(Node):

    def __init__(self):
        super().__init__('agv_navigator')

        self.declare_parameter('goal_tolerance_xy', 0.1)
        self.declare_parameter('goal_tolerance_theta', 0.1)
        self.declare_parameter('obstacle_distance_threshold', 0.3)
        self.declare_parameter('goal_timeout', 300.0)
        self.declare_parameter('obstacle_replan_threshold', 0.2)
        self.declare_parameter('velocity_obstacle_horizon', 2.0)

        self.goal_tolerance_xy = self.get_parameter('goal_tolerance_xy').value
        self.goal_tolerance_theta = self.get_parameter('goal_tolerance_theta').value
        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').value
        self._goal_timeout = self.get_parameter('goal_timeout').value
        self._obstacle_replan_threshold = self.get_parameter('obstacle_replan_threshold').value
        self._velocity_obstacle_horizon = self.get_parameter('velocity_obstacle_horizon').value

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

        self._scan_cache = []
        self._scan_cache_time = 0.0
        self._scan_cache_ttl = 0.5

        self.agv_status_sub = self.create_subscription(
            AGVStatus, 'agv_status', self.agv_status_callback, 10)
        self.laser_scan_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_scan_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

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

    def agv_status_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        self.current_mode = msg.mode
        self.emergency_stop = msg.emergency_stop
        self.current_linear_vel = msg.linear_velocity
        self.current_angular_vel = msg.angular_velocity

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

    def navigate_goal_callback(self, goal_request):
        self.get_logger().info('Received navigate_to goal request')
        if self._active_goal_handle is not None:
            self.get_logger().warn('A goal is already active, preempting')
        return GoalResponse.ACCEPT

    def navigate_cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for navigate_to')
        return CancelResponse.ACCEPT

    def patrol_goal_callback(self, goal_request):
        self.get_logger().info('Received patrol goal request')
        return GoalResponse.ACCEPT

    def patrol_cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for patrol')
        return CancelResponse.ACCEPT

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
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Navigation canceled'
                self._active_goal_handle = None
                return result

            if self.emergency_stop:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = 'Emergency stop activated'
                self._active_goal_handle = None
                return result

            elapsed = time.time() - start_time
            if elapsed > self._goal_timeout:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = f'Goal timeout after {self._goal_timeout:.0f}s'
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
                cmd.linear.x = min(0.8, 0.5 * distance)
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
                return result

            if self.emergency_stop:
                self._stop_robot()
                goal_handle.abort()
                result.success = False
                result.message = 'Emergency stop during patrol'
                result.completed_loops = completed_loops
                return result

            for i in range(len(waypoints_x)):
                if goal_handle.is_cancel_requested:
                    self._stop_robot()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Patrol canceled'
                    result.completed_loops = completed_loops
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
                    return result

            completed_loops += 1

        self._stop_robot()
        goal_handle.succeed()
        result.success = True
        result.message = 'Patrol completed'
        result.completed_loops = completed_loops
        return result

    async def _navigate_to_waypoint(self, target_x, target_y, target_theta,
                                     tol_xy, tol_theta, parent_goal_handle):
        start_time = time.time()
        while True:
            if parent_goal_handle.is_cancel_requested:
                return False

            if self.emergency_stop:
                return False

            elapsed = time.time() - start_time
            if elapsed > self._goal_timeout:
                self.get_logger().warn(f'Waypoint navigation timeout after {self._goal_timeout:.0f}s')
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
                cmd.linear.x = min(0.8, 0.5 * distance)
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
