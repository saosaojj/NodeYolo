import json
import time
from collections import deque
from datetime import datetime

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from agv_interfaces.msg import BatteryState
from agv_interfaces.srv import SetModel

import rclpy


# 增强功能4: 详细电源模式配置 - 定义各子系统的具体设置
POWER_STRATEGIES = {
    'performance': {
        'camera_fps': 30,
        'yolo_frequency': 1,
        'navigation_rate': 10.0,
        'io_enabled': True,
        'comms_rate': 10.0,
        'motor_max_speed': 1.5,
        'lidar_rate': 10.0,
        'display_brightness': 100,
        'wifi_power_save': False,
        'cpu_governor': 'performance',
        'gpu_clock_mhz': 1000,
    },
    'balanced': {
        'camera_fps': 15,
        'yolo_frequency': 2,
        'navigation_rate': 5.0,
        'io_enabled': True,
        'comms_rate': 5.0,
        'motor_max_speed': 1.0,
        'lidar_rate': 5.0,
        'display_brightness': 70,
        'wifi_power_save': False,
        'cpu_governor': 'ondemand',
        'gpu_clock_mhz': 700,
    },
    'power_save': {
        'camera_fps': 5,
        'yolo_frequency': 5,
        'navigation_rate': 2.0,
        'io_enabled': False,
        'comms_rate': 2.0,
        'motor_max_speed': 0.5,
        'lidar_rate': 2.0,
        'display_brightness': 30,
        'wifi_power_save': True,
        'cpu_governor': 'powersave',
        'gpu_clock_mhz': 400,
    },
    'ultra_eco': {
        'camera_fps': 0,
        'yolo_frequency': 0,
        'navigation_rate': 0.0,
        'io_enabled': False,
        'comms_rate': 1.0,
        'motor_max_speed': 0.0,
        'lidar_rate': 0.0,
        'display_brightness': 0,
        'wifi_power_save': True,
        'cpu_governor': 'powersave',
        'gpu_clock_mhz': 200,
    },
    'critical': {
        'camera_fps': 0,
        'yolo_frequency': 0,
        'navigation_rate': 0.0,
        'io_enabled': False,
        'comms_rate': 1.0,
        'motor_max_speed': 0.0,
        'lidar_rate': 0.0,
        'display_brightness': 0,
        'wifi_power_save': True,
        'cpu_governor': 'powersave',
        'gpu_clock_mhz': 0,
    },
}

# 增强功能1: 各子系统的典型功耗 (瓦特)
SUBSYSTEM_POWER_RATINGS = {
    'navigation': 8.0,
    'vision': 15.0,
    'motors': 40.0,
    'comms': 3.0,
    'io': 2.0,
    'lidar': 5.0,
    'display': 4.0,
    'computing': 12.0,
}


class PowerManagerNode(Node):

    def __init__(self):
        super().__init__('power_manager')

        self.declare_parameter('power_save_threshold', 15.0)
        self.declare_parameter('critical_threshold', 5.0)
        self.declare_parameter('performance_mode', 'balanced')
        self.declare_parameter('mode_thresholds.performance', 80.0)
        self.declare_parameter('mode_thresholds.balanced', 30.0)
        self.declare_parameter('mode_thresholds.power_save', 15.0)
        self.declare_parameter('mode_thresholds.critical', 5.0)

        # 增强功能5: 紧急电源储备 - 维持最低电池储备用于紧急停止
        self.declare_parameter('emergency_reserve_percent', 5.0)
        self.declare_parameter('emergency_reserve_enabled', True)

        # 增强功能3: 充电调度管理参数
        self.declare_parameter('charging_schedule_enabled', False)
        self.declare_parameter('charging_preferred_start_hour', 22)
        self.declare_parameter('charging_preferred_end_hour', 6)
        self.declare_parameter('charging_target_level', 95.0)
        self.declare_parameter('task_schedule', [])

        self.power_save_threshold = self.get_parameter('power_save_threshold').value
        self.critical_threshold = self.get_parameter('critical_threshold').value
        self.current_mode = self.get_parameter('performance_mode').value
        self.mode_thresholds = {
            'performance': self.get_parameter('mode_thresholds.performance').value,
            'balanced': self.get_parameter('mode_thresholds.balanced').value,
            'power_save': self.get_parameter('mode_thresholds.power_save').value,
            'critical': self.get_parameter('mode_thresholds.critical').value,
        }

        self.battery_charge_level = 100.0
        self.battery_voltage = 48.0
        self.battery_current = 0.0
        self.battery_temperature = 25.0
        self.battery_health = 100.0
        self.charging_state = 'idle'
        self.estimated_time_remaining = 0.0

        self.power_history = deque(maxlen=100)
        self.auto_mode_enabled = True

        # 增强功能1: 功耗分析 - 跟踪各子系统功耗
        self._subsystem_power = {
            'navigation': 0.0,
            'vision': 0.0,
            'motors': 0.0,
            'comms': 0.0,
            'io': 0.0,
            'lidar': 0.0,
            'display': 0.0,
            'computing': 0.0,
        }
        self._subsystem_power_history = {
            subsystem: deque(maxlen=100) for subsystem in SUBSYSTEM_POWER_RATINGS
        }

        # 增强功能2: 预测性电池管理 - 基于当前消耗率估算剩余运行时间
        self._consumption_rate_history = deque(maxlen=50)
        self._predicted_runtime_minutes = 0.0
        self._predicted_runtime_at_current_rate = 0.0
        self._battery_depletion_time = None

        # 增强功能3: 充电调度管理
        self._charging_schedule_enabled = self.get_parameter('charging_schedule_enabled').value
        self._charging_preferred_start_hour = self.get_parameter('charging_preferred_start_hour').value
        self._charging_preferred_end_hour = self.get_parameter('charging_preferred_end_hour').value
        self._charging_target_level = self.get_parameter('charging_target_level').value
        self._task_schedule = self.get_parameter('task_schedule').value
        self._charging_recommendation = 'none'

        # 增强功能5: 紧急电源储备
        self._emergency_reserve_percent = self.get_parameter('emergency_reserve_percent').value
        self._emergency_reserve_enabled = self.get_parameter('emergency_reserve_enabled').value
        self._emergency_reserve_active = False
        self._emergency_power_consumption = 0.0

        # 增强功能6: 电源事件通知
        self._last_event_time = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.power_mode_pub = self.create_publisher(String, 'power_mode', 10)
        self.power_status_pub = self.create_publisher(String, 'power_status', 10)

        # 增强功能1: 子系统功耗发布
        self._subsystem_power_pub = self.create_publisher(
            String, 'subsystem_power', 10)

        # 增强功能2: 电池预测发布
        self._battery_prediction_pub = self.create_publisher(
            String, 'battery_prediction', 10)

        # 增强功能3: 充电调度发布
        self._charging_schedule_pub = self.create_publisher(
            String, 'charging_schedule', 10)

        # 增强功能5: 紧急储备发布
        self._emergency_reserve_pub = self.create_publisher(
            String, 'emergency_reserve', 10)

        # 增强功能6: 电源事件通知发布
        self._power_event_pub = self.create_publisher(
            String, 'power_events', 10)

        self.battery_state_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_state_callback, qos_profile)

        self.set_power_mode_srv = self.create_service(
            SetModel, 'set_power_mode', self.set_power_mode_callback)
        self.get_power_status_srv = self.create_service(
            Trigger, 'get_power_status', self.get_power_status_callback)

        self.mode_timer = self.create_timer(2.0, self.mode_check_callback)
        self.status_timer = self.create_timer(5.0, self.publish_power_status)

        # 增强功能1: 子系统功耗估算定时器
        self._subsystem_timer = self.create_timer(5.0, self.estimate_subsystem_power)

        # 增强功能2: 电池预测定时器
        self._prediction_timer = self.create_timer(10.0, self.update_battery_prediction)

        # 增强功能3: 充电调度定时器
        self._charging_timer = self.create_timer(30.0, self.evaluate_charging_schedule)

        # 增强功能5: 紧急储备检查定时器
        self._emergency_timer = self.create_timer(5.0, self.check_emergency_reserve)

        self.publish_current_mode()

    def battery_state_callback(self, msg):
        old_charging_state = self.charging_state
        old_charge_level = self.battery_charge_level

        self.battery_charge_level = msg.charge_level
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        self.battery_temperature = msg.temperature
        self.battery_health = msg.health_percent
        self.charging_state = msg.charging_state
        self.estimated_time_remaining = msg.estimated_time_remaining

        power_watts = abs(msg.voltage * msg.current)
        self.power_history.append(power_watts)

        # 增强功能2: 记录消耗率
        if msg.current < -0.5:
            self._consumption_rate_history.append(abs(msg.current))

        # 增强功能6: 发布电源事件
        if old_charging_state != self.charging_state:
            if self.charging_state == 'charging':
                self._publish_power_event('charging_started',
                    f'开始充电, 电量: {msg.charge_level:.1f}%, 电压: {msg.voltage:.2f}V')
            elif self.charging_state == 'discharging':
                self._publish_power_event('charging_stopped',
                    f'停止充电, 电量: {msg.charge_level:.1f}%')

        if old_charge_level > 20.0 and msg.charge_level <= 20.0:
            self._publish_power_event('low_battery', f'低电量警告: {msg.charge_level:.1f}%')
        if old_charge_level > 10.0 and msg.charge_level <= 10.0:
            self._publish_power_event('critical_battery', f'严重低电量: {msg.charge_level:.1f}%')

    def determine_mode_from_battery(self, charge_level):
        # 增强功能5: 紧急储备检查 - 如果电量低于紧急储备，强制进入critical模式
        if self._emergency_reserve_enabled and charge_level <= self._emergency_reserve_percent:
            return 'critical'

        if charge_level <= self.mode_thresholds['critical']:
            return 'critical'
        elif charge_level <= self.mode_thresholds['power_save']:
            return 'power_save'
        elif charge_level <= self.mode_thresholds['balanced']:
            return 'balanced'
        else:
            return 'performance'

    def set_power_mode(self, mode):
        if mode not in POWER_STRATEGIES:
            return False, f'Unknown power mode: {mode}'

        if mode == self.current_mode:
            return True, f'Already in {mode} mode'

        old_mode = self.current_mode
        self.current_mode = mode
        self.get_logger().info(f'Power mode changed: {old_mode} -> {mode}')

        self.publish_current_mode()

        alert_msg = String()
        alert_msg.data = f'POWER MODE CHANGE: {old_mode} -> {mode}'
        self.power_mode_pub.publish(alert_msg)

        # 增强功能6: 发布模式变更事件
        self._publish_power_event('mode_change',
            f'电源模式变更: {old_mode} -> {mode}, 电量: {self.battery_charge_level:.1f}%')

        return True, f'Power mode set to {mode}'

    def set_power_mode_callback(self, request, response):
        mode = request.model_path.lower()

        if mode == 'auto':
            self.auto_mode_enabled = True
            response.success = True
            response.message = 'Auto power mode enabled'
            self._publish_power_event('auto_mode_enabled', '自动电源模式已启用')
            return response

        if mode == 'manual':
            self.auto_mode_enabled = False
            response.success = True
            response.message = 'Auto power mode disabled, manual control active'
            self._publish_power_event('manual_mode_enabled', '手动电源模式已启用')
            return response

        self.auto_mode_enabled = False
        success, message = self.set_power_mode(mode)
        response.success = success
        response.message = message
        return response

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
            'subsystem_power': self._subsystem_power,
            'predicted_runtime_min': self._predicted_runtime_minutes,
            'emergency_reserve_active': self._emergency_reserve_active,
            'charging_recommendation': self._charging_recommendation,
        }

        response.success = True
        response.message = json.dumps(status)
        return response

    def mode_check_callback(self):
        if not self.auto_mode_enabled:
            return

        target_mode = self.determine_mode_from_battery(self.battery_charge_level)

        if target_mode != self.current_mode:
            self.set_power_mode(target_mode)

    def publish_current_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.power_mode_pub.publish(msg)

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

    # 增强功能1: 功耗分析 - 估算各子系统功耗
    def estimate_subsystem_power(self):
        strategy = POWER_STRATEGIES.get(self.current_mode, POWER_STRATEGIES['balanced'])
        total_power = 0.0

        # 导航子系统功耗
        nav_rate = strategy.get('navigation_rate', 5.0)
        nav_power = SUBSYSTEM_POWER_RATINGS['navigation'] * (nav_rate / 10.0)
        self._subsystem_power['navigation'] = round(nav_power, 2)
        total_power += nav_power

        # 视觉子系统功耗
        cam_fps = strategy.get('camera_fps', 15)
        yolo_freq = strategy.get('yolo_frequency', 2)
        vision_util = (cam_fps / 30.0) * 0.6 + (1.0 / max(yolo_freq, 1)) * 0.4 if yolo_freq > 0 else (cam_fps / 30.0)
        vision_power = SUBSYSTEM_POWER_RATINGS['vision'] * min(vision_util, 1.0)
        self._subsystem_power['vision'] = round(vision_power, 2)
        total_power += vision_power

        # 电机功耗
        motor_speed = strategy.get('motor_max_speed', 1.0)
        motor_power = SUBSYSTEM_POWER_RATINGS['motors'] * (motor_speed / 1.5)
        self._subsystem_power['motors'] = round(motor_power, 2)
        total_power += motor_power

        # 通信功耗
        comms_rate = strategy.get('comms_rate', 5.0)
        comms_power = SUBSYSTEM_POWER_RATINGS['comms'] * (comms_rate / 10.0)
        self._subsystem_power['comms'] = round(comms_power, 2)
        total_power += comms_power

        # IO功耗
        io_enabled = strategy.get('io_enabled', True)
        io_power = SUBSYSTEM_POWER_RATINGS['io'] if io_enabled else 0.0
        self._subsystem_power['io'] = round(io_power, 2)
        total_power += io_power

        # 激光雷达功耗
        lidar_rate = strategy.get('lidar_rate', 5.0)
        lidar_power = SUBSYSTEM_POWER_RATINGS['lidar'] * (lidar_rate / 10.0)
        self._subsystem_power['lidar'] = round(lidar_power, 2)
        total_power += lidar_power

        # 显示功耗
        brightness = strategy.get('display_brightness', 70)
        display_power = SUBSYSTEM_POWER_RATINGS['display'] * (brightness / 100.0)
        self._subsystem_power['display'] = round(display_power, 2)
        total_power += display_power

        # 计算功耗
        gpu_clock = strategy.get('gpu_clock_mhz', 700)
        computing_power = SUBSYSTEM_POWER_RATINGS['computing'] * (gpu_clock / 1000.0)
        self._subsystem_power['computing'] = round(computing_power, 2)
        total_power += computing_power

        # 记录历史
        for subsystem in self._subsystem_power:
            self._subsystem_power_history[subsystem].append(self._subsystem_power[subsystem])

        # 发布子系统功耗
        power_data = {
            'subsystems': self._subsystem_power,
            'total_estimated_w': round(total_power, 2),
            'mode': self.current_mode,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(power_data, ensure_ascii=False)
        self._subsystem_power_pub.publish(msg)

    # 增强功能2: 预测性电池管理 - 估算剩余运行时间
    def update_battery_prediction(self):
        if len(self._consumption_rate_history) == 0:
            return

        avg_consumption_rate = sum(self._consumption_rate_history) / len(self._consumption_rate_history)

        # 基于当前消耗率预测剩余运行时间
        if avg_consumption_rate > 0:
            # 假设48V系统，50Ah容量
            battery_capacity_wh = 48.0 * 50.0
            remaining_wh = battery_capacity_wh * (self.battery_charge_level / 100.0)

            # 增强功能5: 考虑紧急储备
            if self._emergency_reserve_enabled:
                reserve_wh = battery_capacity_wh * (self._emergency_reserve_percent / 100.0)
                usable_wh = remaining_wh - reserve_wh
                usable_wh = max(0.0, usable_wh)
            else:
                usable_wh = remaining_wh

            current_power_w = self.battery_voltage * avg_consumption_rate
            if current_power_w > 0:
                self._predicted_runtime_at_current_rate = (usable_wh / current_power_w) * 60.0
            else:
                self._predicted_runtime_at_current_rate = float('inf')

            # 基于当前模式的预测
            strategy = POWER_STRATEGIES.get(self.current_mode, POWER_STRATEGIES['balanced'])
            estimated_mode_power = sum([
                SUBSYSTEM_POWER_RATINGS['navigation'] * (strategy.get('navigation_rate', 5.0) / 10.0),
                SUBSYSTEM_POWER_RATINGS['vision'] * (strategy.get('camera_fps', 15) / 30.0),
                SUBSYSTEM_POWER_RATINGS['motors'] * (strategy.get('motor_max_speed', 1.0) / 1.5),
                SUBSYSTEM_POWER_RATINGS['comms'] * (strategy.get('comms_rate', 5.0) / 10.0),
                SUBSYSTEM_POWER_RATINGS['lidar'] * (strategy.get('lidar_rate', 5.0) / 10.0),
            ])

            if estimated_mode_power > 0:
                self._predicted_runtime_minutes = (usable_wh / estimated_mode_power) * 60.0
            else:
                self._predicted_runtime_minutes = float('inf')

            # 预测电池耗尽时间
            if current_power_w > 0:
                hours_to_depletion = usable_wh / current_power_w
                self._battery_depletion_time = time.time() + hours_to_depletion * 3600
            else:
                self._battery_depletion_time = None

        # 发布预测数据
        prediction_data = {
            'predicted_runtime_min': round(self._predicted_runtime_minutes, 1),
            'predicted_runtime_at_current_rate_min': round(self._predicted_runtime_at_current_rate, 1),
            'avg_consumption_rate_a': round(avg_consumption_rate, 2),
            'charge_level': round(self.battery_charge_level, 1),
            'mode': self.current_mode,
            'depletion_time': datetime.fromtimestamp(self._battery_depletion_time).isoformat() if self._battery_depletion_time else None,
            'emergency_reserve_active': self._emergency_reserve_active,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(prediction_data, ensure_ascii=False)
        self._battery_prediction_pub.publish(msg)

    # 增强功能3: 充电调度管理 - 基于任务调度优化充电时间
    def evaluate_charging_schedule(self):
        if not self._charging_schedule_enabled:
            self._charging_recommendation = 'none'
            return

        current_hour = datetime.now().hour
        is_preferred_time = (
            self._charging_preferred_start_hour <= current_hour or
            current_hour < self._charging_preferred_end_hour
        )

        # 检查是否有即将到来的任务
        has_upcoming_task = False
        task_start_time = None
        for task in self._task_schedule:
            if isinstance(task, dict):
                task_hour = task.get('start_hour', -1)
                if task_hour > current_hour and (task_start_time is None or task_hour < task_start_time):
                    has_upcoming_task = True
                    task_start_time = task_hour

        # 生成充电建议
        if self.charging_state == 'charging':
            if self.battery_charge_level >= self._charging_target_level:
                self._charging_recommendation = 'stop_charging_target_reached'
            elif not is_preferred_time and self.battery_charge_level > 30.0:
                self._charging_recommendation = 'consider_stopping_non_preferred_time'
            else:
                self._charging_recommendation = 'continue_charging'
        elif self.charging_state == 'discharging':
            if self.battery_charge_level <= 20.0:
                self._charging_recommendation = 'charge_immediately_low_battery'
            elif is_preferred_time and self.battery_charge_level < 80.0:
                self._charging_recommendation = 'charge_preferred_time'
            elif has_upcoming_task and self.battery_charge_level < 60.0:
                self._charging_recommendation = 'charge_before_task'
            elif self.battery_charge_level < 40.0:
                self._charging_recommendation = 'consider_charging'
            else:
                self._charging_recommendation = 'no_action_needed'
        else:
            self._charging_recommendation = 'idle'

        # 发布充电调度
        schedule_data = {
            'recommendation': self._charging_recommendation,
            'is_preferred_charging_time': is_preferred_time,
            'current_hour': current_hour,
            'preferred_start_hour': self._charging_preferred_start_hour,
            'preferred_end_hour': self._charging_preferred_end_hour,
            'target_level': self._charging_target_level,
            'current_level': round(self.battery_charge_level, 1),
            'has_upcoming_task': has_upcoming_task,
            'charging_state': self.charging_state,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(schedule_data, ensure_ascii=False)
        self._charging_schedule_pub.publish(msg)

    # 增强功能5: 紧急电源储备检查
    def check_emergency_reserve(self):
        if not self._emergency_reserve_enabled:
            self._emergency_reserve_active = False
            return

        was_active = self._emergency_reserve_active

        if self.battery_charge_level <= self._emergency_reserve_percent:
            self._emergency_reserve_active = True

            # 计算紧急停止所需功耗
            self._emergency_power_consumption = (
                SUBSYSTEM_POWER_RATINGS['comms'] * 0.3 +
                SUBSYSTEM_POWER_RATINGS['motors'] * 0.1
            )

            if not was_active:
                self._publish_power_event('emergency_reserve_activated',
                    f'紧急电源储备已激活! 电量: {self.battery_charge_level:.1f}% '
                    f'(储备阈值: {self._emergency_reserve_percent}%)')

                # 强制进入critical模式
                if self.current_mode != 'critical':
                    self.set_power_mode('critical')

            # 发布紧急储备状态
            reserve_data = {
                'active': True,
                'current_level': round(self.battery_charge_level, 2),
                'reserve_threshold': self._emergency_reserve_percent,
                'emergency_power_consumption_w': round(self._emergency_power_consumption, 2),
                'estimated_emergency_runtime_min': 0.0,
                'timestamp': datetime.now().isoformat(),
            }

            # 计算紧急运行时间
            battery_capacity_wh = 48.0 * 50.0
            remaining_wh = battery_capacity_wh * (self.battery_charge_level / 100.0)
            if self._emergency_power_consumption > 0:
                reserve_data['estimated_emergency_runtime_min'] = round(
                    (remaining_wh / self._emergency_power_consumption) * 60.0, 1)

            msg = String()
            msg.data = json.dumps(reserve_data, ensure_ascii=False)
            self._emergency_reserve_pub.publish(msg)
        else:
            if was_active:
                self._publish_power_event('emergency_reserve_deactivated',
                    f'紧急电源储备已解除, 电量恢复: {self.battery_charge_level:.1f}%')
            self._emergency_reserve_active = False

    # 增强功能6: 电源事件通知
    def _publish_power_event(self, event_type, description):
        event_data = {
            'event_type': event_type,
            'description': description,
            'timestamp': datetime.now().isoformat(),
            'charge_level': round(self.battery_charge_level, 1),
            'mode': self.current_mode,
            'charging_state': self.charging_state,
        }
        msg = String()
        msg.data = json.dumps(event_data, ensure_ascii=False)
        self._power_event_pub.publish(msg)
        self.get_logger().info(f'[电源事件] {event_type}: {description}')


def main(args=None):
    rclpy.init(args=args)
    node = PowerManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
