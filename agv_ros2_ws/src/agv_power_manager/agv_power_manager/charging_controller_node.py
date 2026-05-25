import enum
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from agv_interfaces.msg import BatteryState, AGVStatus
from agv_interfaces.srv import SetCharging

import rclpy


class ChargingState(enum.Enum):
    IDLE = 0
    SEEKING_CHARGER = 1
    DOCKING = 2
    CHARGING = 3
    CHARGING_COMPLETE = 4


class ChargingControllerNode(Node):

    def __init__(self):
        super().__init__('charging_controller')

        self.declare_parameter('charge_threshold_low', 20.0)
        self.declare_parameter('charge_threshold_high', 90.0)
        self.declare_parameter('charge_station_x', 0.0)
        self.declare_parameter('charge_station_y', 0.0)
        self.declare_parameter('charge_station_theta', 0.0)
        self.declare_parameter('auto_charge_enabled', True)
        self.declare_parameter('docking_approach_speed', 0.05)
        self.declare_parameter('docking_approach_angular', 0.1)

        self.charge_threshold_low = self.get_parameter('charge_threshold_low').value
        self.charge_threshold_high = self.get_parameter('charge_threshold_high').value
        self.charge_station_x = self.get_parameter('charge_station_x').value
        self.charge_station_y = self.get_parameter('charge_station_y').value
        self.charge_station_theta = self.get_parameter('charge_station_theta').value
        self.auto_charge_enabled = self.get_parameter('auto_charge_enabled').value
        self.docking_approach_speed = self.get_parameter('docking_approach_speed').value
        self.docking_approach_angular = self.get_parameter('docking_approach_angular').value

        self.state = ChargingState.IDLE
        self.battery_charge_level = 100.0
        self.charge_contact_active = False
        self.agv_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.agv_status = 'idle'
        self.docking_retry_count = 0
        self.max_docking_retries = 3

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.charging_command_pub = self.create_publisher(String, 'charging_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        self.battery_state_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_state_callback, qos_profile)
        self.agv_status_sub = self.create_subscription(
            AGVStatus, 'agv_status', self.agv_status_callback, qos_profile)
        self.charge_contact_sub = self.create_subscription(
            Bool, 'charge_contact', self.charge_contact_callback, qos_profile)

        self.set_charging_srv = self.create_service(
            SetCharging, 'set_charging', self.set_charging_callback)

        self.state_timer = self.create_timer(0.5, self.state_machine_callback)

    def battery_state_callback(self, msg):
        self.battery_charge_level = msg.charge_level

    def agv_status_callback(self, msg):
        self.agv_status = msg.status
        if hasattr(msg, 'position') and msg.position is not None:
            self.agv_position['x'] = msg.position.x
            self.agv_position['y'] = msg.position.y
            self.agv_position['theta'] = msg.position.theta

    def charge_contact_callback(self, msg):
        self.charge_contact_active = msg.data

    def set_charging_callback(self, request, response):
        command = request.command.lower()

        if command == 'start_charging':
            if self.state == ChargingState.IDLE:
                self.transition_to(ChargingState.SEEKING_CHARGER)
                response.success = True
                response.message = 'Charging sequence initiated'
            else:
                response.success = False
                response.message = f'Cannot start charging in state: {self.state.name}'
        elif command == 'stop_charging':
            if self.state in (ChargingState.CHARGING, ChargingState.DOCKING, ChargingState.SEEKING_CHARGER):
                self.transition_to(ChargingState.IDLE)
                self.publish_charging_command('stop_charging')
                response.success = True
                response.message = 'Charging stopped'
            else:
                response.success = False
                response.message = f'Not in a charging state: {self.state.name}'
        elif command == 'force_dock':
            if self.state == ChargingState.IDLE:
                self.transition_to(ChargingState.DOCKING)
                response.success = True
                response.message = 'Force docking initiated'
            else:
                response.success = False
                response.message = f'Cannot force dock in state: {self.state.name}'
        else:
            response.success = False
            response.message = f'Unknown command: {command}'

        response.charge_level = self.battery_charge_level
        return response

    def transition_to(self, new_state):
        self.get_logger().info(
            f'Charging state transition: {self.state.name} -> {new_state.name}')
        self.state = new_state

    def publish_charging_command(self, command):
        msg = String()
        msg.data = command
        self.charging_command_pub.publish(msg)

    def calculate_distance_to_station(self):
        dx = self.charge_station_x - self.agv_position['x']
        dy = self.charge_station_y - self.agv_position['y']
        return (dx ** 2 + dy ** 2) ** 0.5

    def calculate_angle_to_station(self):
        dx = self.charge_station_x - self.agv_position['x']
        dy = self.charge_station_y - self.agv_position['y']
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.agv_position['theta']
        while angle_diff > math.pi:
            angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2.0 * math.pi
        return angle_diff

    def state_machine_callback(self):
        if self.state == ChargingState.IDLE:
            self.handle_idle()
        elif self.state == ChargingState.SEEKING_CHARGER:
            self.handle_seeking_charger()
        elif self.state == ChargingState.DOCKING:
            self.handle_docking()
        elif self.state == ChargingState.CHARGING:
            self.handle_charging()
        elif self.state == ChargingState.CHARGING_COMPLETE:
            self.handle_charging_complete()

    def handle_idle(self):
        if self.auto_charge_enabled and self.battery_charge_level <= self.charge_threshold_low:
            self.get_logger().info(
                f'Battery low ({self.battery_charge_level:.1f}%), auto-seeking charger')
            self.transition_to(ChargingState.SEEKING_CHARGER)
            self.publish_charging_command('seek_charger')

    def handle_seeking_charger(self):
        distance = self.calculate_distance_to_station()
        if distance < 0.5:
            self.get_logger().info('Near charging station, switching to docking')
            self.transition_to(ChargingState.DOCKING)
            self.docking_retry_count = 0

    def handle_docking(self):
        if self.charge_contact_active:
            self.get_logger().info('Charge contact detected, switching to charging')
            self.transition_to(ChargingState.CHARGING)
            self.publish_charging_command('start_charging')
            self.stop_motion()
            return

        angle_diff = self.calculate_angle_to_station()
        twist = Twist()
        twist.linear.x = self.docking_approach_speed
        twist.angular.z = max(-self.docking_approach_angular,
                              min(self.docking_approach_angular, angle_diff * 0.5))
        self.cmd_vel_pub.publish(twist)

    def handle_charging(self):
        if not self.charge_contact_active:
            self.get_logger().warn('Charge contact lost during charging')
            self.transition_to(ChargingState.SEEKING_CHARGER)
            self.publish_charging_command('seek_charger')
            return

        if self.battery_charge_level >= self.charge_threshold_high:
            self.get_logger().info(
                f'Battery charged to {self.battery_charge_level:.1f}%, charging complete')
            self.transition_to(ChargingState.CHARGING_COMPLETE)
            self.publish_charging_command('charging_complete')

    def handle_charging_complete(self):
        self.stop_motion()
        self.transition_to(ChargingState.IDLE)

    def stop_motion(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ChargingControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
