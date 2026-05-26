# 电源管理节点，根据电池电量自动切换功耗模式（性能/均衡/节能/临界），并发布电源状态信息
import json
from collections import deque

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from agv_interfaces.msg import BatteryState
from agv_interfaces.srv import SetModel

import rclpy


# 功耗策略配置字典，定义各模式下的摄像头帧率、YOLO频率、导航速率等参数
POWER_STRATEGIES = {
    'performance': {
        'camera_fps': 30,
        'yolo_frequency': 1,
        'navigation_rate': 10.0,
        'io_enabled': True,
        'comms_rate': 10.0,
    },
    'balanced': {
        'camera_fps': 15,
        'yolo_frequency': 2,
        'navigation_rate': 5.0,
        'io_enabled': True,
        'comms_rate': 5.0,
    },
    'power_save': {
        'camera_fps': 5,
        'yolo_frequency': 5,
        'navigation_rate': 2.0,
        'io_enabled': False,
        'comms_rate': 2.0,
    },
    'critical': {
        'camera_fps': 0,
        'yolo_frequency': 0,
        'navigation_rate': 0.0,
        'io_enabled': False,
        'comms_rate': 1.0,
    },
}


# PowerManagerNode: 电源管理ROS2节点
# 根据电池电量自动切换功耗模式，支持手动/自动模式切换，发布电源状态信息
class PowerManagerNode(Node):

    def __init__(self):
        super().__init__('power_manager')

        # 声明电量阈值参数
        self.declare_parameter('power_save_threshold', 15.0)
        self.declare_parameter('critical_threshold', 5.0)
        self.declare_parameter('performance_mode', 'balanced')
        # 声明各模式对应的电量阈值
        self.declare_parameter('mode_thresholds.performance', 80.0)
        self.declare_parameter('mode_thresholds.balanced', 30.0)
        self.declare_parameter('mode_thresholds.power_save', 15.0)
        self.declare_parameter('mode_thresholds.critical', 5.0)

        # 获取电量阈值参数
        self.power_save_threshold = self.get_parameter('power_save_threshold').value
        self.critical_threshold = self.get_parameter('critical_threshold').value
        self.current_mode = self.get_parameter('performance_mode').value
        # 获取各模式电量阈值
        self.mode_thresholds = {
            'performance': self.get_parameter('mode_thresholds.performance').value,
            'balanced': self.get_parameter('mode_thresholds.balanced').value,
            'power_save': self.get_parameter('mode_thresholds.power_save').value,
            'critical': self.get_parameter('mode_thresholds.critical').value,
        }

        # 电池状态数据
        self.battery_charge_level = 100.0
        self.battery_voltage = 48.0
        self.battery_current = 0.0
        self.battery_temperature = 25.0
        self.battery_health = 100.0
        self.charging_state = 'idle'
        self.estimated_time_remaining = 0.0

        # 功耗历史记录，用于计算平均功耗
        self.power_history = deque(maxlen=100)
        # 自动模式切换开关
        self.auto_mode_enabled = True

        # 使用BEST_EFFORT策略接收传感器数据
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建发布者：功耗模式和电源状态
        self.power_mode_pub = self.create_publisher(String, 'power_mode', 10)
        self.power_status_pub = self.create_publisher(String, 'power_status', 10)

        # 创建订阅者：电池状态
        self.battery_state_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_state_callback, qos_profile)

        # 创建服务：设置功耗模式和获取电源状态
        self.set_power_mode_srv = self.create_service(
            SetModel, 'set_power_mode', self.set_power_mode_callback)
        self.get_power_status_srv = self.create_service(
            Trigger, 'get_power_status', self.get_power_status_callback)

        # 创建定时器：模式检查和状态发布
        self.mode_timer = self.create_timer(2.0, self.mode_check_callback)
        self.status_timer = self.create_timer(5.0, self.publish_power_status)

        # 发布初始模式
        self.publish_current_mode()

    # 电池状态回调，更新电池信息并记录功耗
    def battery_state_callback(self, msg):
        self.battery_charge_level = msg.charge_level
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        self.battery_temperature = msg.temperature
        self.battery_health = msg.health_percent
        self.charging_state = msg.charging_state
        self.estimated_time_remaining = msg.estimated_time_remaining

        # 计算瞬时功耗并记录到历史
        power_watts = abs(msg.voltage * msg.current)
        self.power_history.append(power_watts)

    # 根据电量百分比判断应处于的功耗模式
    def determine_mode_from_battery(self, charge_level):
        if charge_level <= self.mode_thresholds['critical']:
            return 'critical'
        elif charge_level <= self.mode_thresholds['power_save']:
            return 'power_save'
        elif charge_level <= self.mode_thresholds['balanced']:
            return 'balanced'
        else:
            return 'performance'

    # 切换功耗模式，发布模式变更通知
    def set_power_mode(self, mode):
        if mode not in POWER_STRATEGIES:
            return False, f'Unknown power mode: {mode}'

        if mode == self.current_mode:
            return True, f'Already in {mode} mode'

        old_mode = self.current_mode
        self.current_mode = mode
        self.get_logger().info(f'Power mode changed: {old_mode} -> {mode}')

        self.publish_current_mode()

        # 发布模式变更告警
        alert_msg = String()
        alert_msg.data = f'POWER MODE CHANGE: {old_mode} -> {mode}'
        self.power_mode_pub.publish(alert_msg)

        return True, f'Power mode set to {mode}'

    # 设置功耗模式服务回调，支持auto/manual/具体模式名
    def set_power_mode_callback(self, request, response):
        mode = request.model_path.lower()

        if mode == 'auto':
            self.auto_mode_enabled = True
            response.success = True
            response.message = 'Auto power mode enabled'
            return response

        if mode == 'manual':
            self.auto_mode_enabled = False
            response.success = True
            response.message = 'Auto power mode disabled, manual control active'
            return response

        # 手动设置具体模式时关闭自动切换
        self.auto_mode_enabled = False
        success, message = self.set_power_mode(mode)
        response.success = success
        response.message = message
        return response

    # 获取电源状态服务回调，返回包含电池和功耗信息的JSON
    def get_power_status_callback(self, request, response):
        avg_power = 0.0
        if len(self.power_history) > 0:
            avg_power = sum(self.power_history) / len(self.power_history)

        strategy = POWER_STRATEGIES.get(self.current_mode, POWER_STRATEGIES['balanced'])

        status = {
            'current_mode': self.current_mode,
            'auto_mode_enabled': self.auto_mode_enabled,
            'battery_charge_level': self.battery_charge_level,
            'battery_voltage': self.battery_voltage,
            'battery_current': self.battery_current,
            'battery_temperature': self.battery_temperature,
            'battery_health': self.battery_health,
            'charging_state': self.charging_state,
            'estimated_time_remaining': self.estimated_time_remaining,
            'avg_power_consumption_w': avg_power,
            'power_history_size': len(self.power_history),
            'strategy': strategy,
        }

        response.success = True
        response.message = json.dumps(status)
        return response

    # 模式检查定时回调，自动模式下根据电量切换功耗模式
    def mode_check_callback(self):
        if not self.auto_mode_enabled:
            return

        target_mode = self.determine_mode_from_battery(self.battery_charge_level)

        if target_mode != self.current_mode:
            self.set_power_mode(target_mode)

    # 发布当前功耗模式
    def publish_current_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.power_mode_pub.publish(msg)

    # 发布电源状态信息，包含电池数据和平均功耗
    def publish_power_status(self):
        avg_power = 0.0
        if len(self.power_history) > 0:
            avg_power = sum(self.power_history) / len(self.power_history)

        strategy = POWER_STRATEGIES.get(self.current_mode, POWER_STRATEGIES['balanced'])

        status = {
            'mode': self.current_mode,
            'auto_mode': self.auto_mode_enabled,
            'charge_level': self.battery_charge_level,
            'voltage': self.battery_voltage,
            'current': self.battery_current,
            'temperature': self.battery_temperature,
            'health': self.battery_health,
            'charging_state': self.charging_state,
            'estimated_time_min': self.estimated_time_remaining,
            'avg_power_w': avg_power,
            'strategy': strategy,
        }

        msg = String()
        msg.data = json.dumps(status)
        self.power_status_pub.publish(msg)


# 节点入口函数
def main(args=None):
    rclpy.init(args=args)
    node = PowerManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
