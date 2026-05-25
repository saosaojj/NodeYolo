import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from agv_interfaces.msg import AGVStatus
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

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kd_angular = self.get_parameter('kd_angular').value
        control_rate = self.get_parameter('control_rate').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.has_target = False

        self.mode = 'idle'
        self.emergency_stop_flag = False
        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0
        self.battery_level = 100.0
        self.active_alarms = []

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, 'target_pose', self.target_pose_callback, 10)

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

    def target_pose_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.target_theta = math.atan2(siny_cosp, cosy_cosp)
        self.has_target = True
        if self.mode == 'idle':
            self.mode = 'auto'
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

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
            response.success = True
            response.message = 'AGV stopped'
        elif cmd == 'pause':
            self.mode = 'idle'
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
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
            response.success = True
            response.message = 'AGV in charging mode'
        elif cmd == 'emergency_stop':
            self.emergency_stop_flag = True
            self.mode = 'error'
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            self.has_target = False
            if 'emergency_stop' not in self.active_alarms:
                self.active_alarms.append('emergency_stop')
            response.success = True
            response.message = 'Emergency stop activated'
        else:
            response.success = False
            response.message = f'Unknown command: {cmd}'
        return response

    def control_loop(self):
        if self.emergency_stop_flag:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
        elif self.mode == 'auto' and self.has_target:
            self._compute_pd_control()
        elif self.mode == 'manual':
            pass
        else:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0

        cmd_out = Twist()
        cmd_out.linear.x = max(-self.max_linear_vel,
                               min(self.max_linear_vel, self.cmd_vel_linear))
        cmd_out.angular.z = max(-self.max_angular_vel,
                                min(self.max_angular_vel, self.cmd_vel_angular))
        self.cmd_vel_out_pub.publish(cmd_out)

        self.current_linear_vel = cmd_out.linear.x
        self.current_angular_vel = cmd_out.angular.z

        dt = 1.0 / self.get_parameter('control_rate').value
        self.current_theta += self.current_angular_vel * dt
        self.current_theta = math.atan2(math.sin(self.current_theta),
                                        math.cos(self.current_theta))
        self.current_x += self.current_linear_vel * math.cos(self.current_theta) * dt
        self.current_y += self.current_linear_vel * math.sin(self.current_theta) * dt

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


def main(args=None):
    rclpy.init(args=args)
    node = AgvControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
