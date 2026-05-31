import math
import time
import json
from collections import deque
from datetime import datetime

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, String
from agv_interfaces.msg import BatteryState

import rclpy


class BatteryMonitorNode(Node):

    def __init__(self):
        super().__init__('battery_monitor')

        self.declare_parameter('poll_rate', 1.0)
        self.declare_parameter('battery_capacity_ah', 50.0)
        self.declare_parameter('nominal_voltage', 48.0)
        self.declare_parameter('min_voltage', 39.0)
        self.declare_parameter('max_voltage', 54.6)
        self.declare_parameter('num_cells', 13)
        self.declare_parameter('simulate', True)
        self.declare_parameter('alert_thresholds.low_battery', 20.0)
        self.declare_parameter('alert_thresholds.critical_battery', 10.0)
        self.declare_parameter('alert_thresholds.over_temperature', 45.0)
        self.declare_parameter('alert_thresholds.under_voltage', 40.0)

        # 增强功能1: 电池健康评估参数
        self.declare_parameter('health_assessment.max_cycles', 2000)
        self.declare_parameter('health_assessment.cycle_degradation_rate', 0.05)
        self.declare_parameter('health_assessment.capacity_fade_threshold', 80.0)

        # 增强功能2: 电池单体均衡监控参数
        self.declare_parameter('cell_balance.imbalance_threshold_mv', 50)
        self.declare_parameter('cell_balance.critical_imbalance_mv', 200)

        # 增强功能3: 温度充电控制参数
        self.declare_parameter('charging_temp.min_temp_c', 0.0)
        self.declare_parameter('charging_temp.max_temp_c', 45.0)
        self.declare_parameter('charging_temp.reduced_rate_temp_c', 40.0)
        self.declare_parameter('charging_temp.cold_reduce_factor', 0.3)
        self.declare_parameter('charging_temp.hot_reduce_factor', 0.5)

        # 增强功能4: 电池故障检测参数
        self.declare_parameter('fault_detection.overvoltage', 55.0)
        self.declare_parameter('fault_detection.undervoltage', 36.0)
        self.declare_parameter('fault_detection.overcurrent_charge', 20.0)
        self.declare_parameter('fault_detection.overcurrent_discharge', 30.0)
        self.declare_parameter('fault_detection.over_temperature', 55.0)
        self.declare_parameter('fault_detection.rapid_voltage_change_vps', 2.0)

        # 增强功能5: 充电效率跟踪参数
        self.declare_parameter('efficiency_tracking.window_seconds', 300)

        # 增强功能6: 电池统计发布参数
        self.declare_parameter('stats_publish_rate', 30.0)

        self.poll_rate = self.get_parameter('poll_rate').value
        self.battery_capacity_ah = self.get_parameter('battery_capacity_ah').value
        self.nominal_voltage = self.get_parameter('nominal_voltage').value
        self.min_voltage = self.get_parameter('min_voltage').value
        self.max_voltage = self.get_parameter('max_voltage').value
        self.num_cells = self.get_parameter('num_cells').value
        self.simulate = self.get_parameter('simulate').value
        self.low_battery_threshold = self.get_parameter('alert_thresholds.low_battery').value
        self.critical_battery_threshold = self.get_parameter('alert_thresholds.critical_battery').value
        self.over_temperature_threshold = self.get_parameter('alert_thresholds.over_temperature').value
        self.under_voltage_threshold = self.get_parameter('alert_thresholds.under_voltage').value

        self.ema_alpha = 0.1
        self.voltage_ema = self.nominal_voltage
        self.current_ema = 0.0
        self.temp_ema = 25.0
        self.ema_initialized = False

        self.raw_voltage = self.nominal_voltage
        self.raw_current = 0.0
        self.raw_temp = 25.0

        self.charge_cycles = 0
        self.battery_type = 'LiFePO4'
        self.charging_state = 'discharging'
        self.sim_time = 0.0
        self.sim_charge_level = 75.0
        self.last_update_time = time.time()

        # 增强功能1: 电池健康评估 - 跟踪充电循环和容量退化
        self._max_cycles = self.get_parameter('health_assessment.max_cycles').value
        self._cycle_degradation_rate = self.get_parameter('health_assessment.cycle_degradation_rate').value
        self._capacity_fade_threshold = self.get_parameter('health_assessment.capacity_fade_threshold').value
        self._cycle_tracking = {
            'last_charge_level': 75.0,
            'cycle_progress': 0.0,
            'partial_cycles': 0.0,
            'design_capacity_ah': self.battery_capacity_ah,
            'actual_capacity_ah': self.battery_capacity_ah,
            'capacity_health_percent': 100.0,
        }

        # 增强功能2: 电池单体均衡监控 - 检测不均衡单体(模拟)
        self._cell_imbalance_threshold_mv = self.get_parameter('cell_balance.imbalance_threshold_mv').value
        self._cell_critical_imbalance_mv = self.get_parameter('cell_balance.critical_imbalance_mv').value
        self._cell_voltages = [self.nominal_voltage / self.num_cells] * self.num_cells
        self._cell_imbalance_detected = False
        self._cell_critical_imbalance = False
        self._max_cell_deviation_mv = 0

        # 增强功能3: 温度充电控制 - 极端温度下降低充电速率
        self._charging_min_temp = self.get_parameter('charging_temp.min_temp_c').value
        self._charging_max_temp = self.get_parameter('charging_temp.max_temp_c').value
        self._charging_reduced_rate_temp = self.get_parameter('charging_temp.reduced_rate_temp_c').value
        self._cold_reduce_factor = self.get_parameter('charging_temp.cold_reduce_factor').value
        self._hot_reduce_factor = self.get_parameter('charging_temp.hot_reduce_factor').value
        self._charge_rate_modifier = 1.0
        self._temperature_charging_limited = False

        # 增强功能4: 电池故障检测 - 检测过压、欠压、过流、过热
        self._fault_overvoltage = self.get_parameter('fault_detection.overvoltage').value
        self._fault_undervoltage = self.get_parameter('fault_detection.undervoltage').value
        self._fault_overcurrent_charge = self.get_parameter('fault_detection.overcurrent_charge').value
        self._fault_overcurrent_discharge = self.get_parameter('fault_detection.overcurrent_discharge').value
        self._fault_over_temperature = self.get_parameter('fault_detection.over_temperature').value
        self._fault_rapid_voltage_change = self.get_parameter('fault_detection.rapid_voltage_change_vps').value
        self._active_faults = set()
        self._last_fault_check_voltage = self.nominal_voltage
        self._last_fault_check_time = time.time()

        # 增强功能5: 充电效率跟踪 - 测量实际与理论充电速率
        self._efficiency_window = self.get_parameter('efficiency_tracking.window_seconds').value
        self._charging_efficiency = 1.0
        self._charge_energy_added_wh = 0.0
        self._charge_energy_theoretical_wh = 0.0
        self._efficiency_tracking_start = None
        self._efficiency_history = deque(maxlen=50)

        # 增强功能6: 电池统计
        self._stats_publish_rate = self.get_parameter('stats_publish_rate').value
        self._battery_stats = {
            'total_energy_consumed_wh': 0.0,
            'total_energy_charged_wh': 0.0,
            'total_discharge_time_seconds': 0.0,
            'total_charge_time_seconds': 0.0,
            'max_temperature': 25.0,
            'min_temperature': 25.0,
            'max_discharge_current': 0.0,
            'max_charge_current': 0.0,
        }
        self._last_stats_update = time.time()

        self.voltage_lut = [
            (0.0, self.min_voltage),
            (10.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.1),
            (20.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.25),
            (30.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.35),
            (40.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.45),
            (50.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.55),
            (60.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.65),
            (70.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.75),
            (80.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.85),
            (90.0, self.min_voltage + (self.nominal_voltage - self.min_voltage) * 0.93),
            (100.0, self.max_voltage),
        ]

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.battery_state_pub = self.create_publisher(BatteryState, 'battery_state', qos_profile)
        self.battery_alert_pub = self.create_publisher(String, 'battery_alert', 10)

        # 增强功能1: 电池健康发布
        self._battery_health_pub = self.create_publisher(String, 'battery_health', 10)

        # 增强功能2: 单体均衡发布
        self._cell_balance_pub = self.create_publisher(String, 'battery_cell_balance', 10)

        # 增强功能3: 温度充电控制发布
        self._charging_temp_pub = self.create_publisher(String, 'battery_charging_temp', 10)

        # 增强功能4: 故障检测发布
        self._battery_fault_pub = self.create_publisher(String, 'battery_fault', 10)

        # 增强功能5: 充电效率发布
        self._charging_efficiency_pub = self.create_publisher(String, 'battery_charging_efficiency', 10)

        # 增强功能6: 电池统计发布
        self._battery_stats_pub = self.create_publisher(String, 'battery_stats', 10)

        self.voltage_sub = self.create_subscription(
            Float64, 'battery_voltage_raw', self.voltage_callback, qos_profile)
        self.current_sub = self.create_subscription(
            Float64, 'battery_current_raw', self.current_callback, qos_profile)
        self.temp_sub = self.create_subscription(
            Float64, 'battery_temp_raw', self.temp_callback, qos_profile)

        self.timer = self.create_timer(1.0 / self.poll_rate, self.timer_callback)

        # 增强功能6: 统计发布定时器
        self._stats_timer = self.create_timer(
            1.0 / self._stats_publish_rate, self.publish_battery_stats)

        # 增强功能2: 单体均衡检查定时器
        self._cell_timer = self.create_timer(5.0, self.check_cell_balance)

        # 增强功能5: 充电效率计算定时器
        self._efficiency_timer = self.create_timer(10.0, self.compute_charging_efficiency)

        self.alert_states = {
            'low_battery': False,
            'critical_battery': False,
            'over_temperature': False,
            'under_voltage': False,
        }

    def voltage_callback(self, msg):
        self.raw_voltage = msg.data

    def current_callback(self, msg):
        self.raw_current = msg.data

    def temp_callback(self, msg):
        self.raw_temp = msg.data

    def update_ema(self, raw_value, current_ema):
        if not self.ema_initialized:
            return raw_value
        return self.ema_alpha * raw_value + (1.0 - self.ema_alpha) * current_ema

    def voltage_to_charge_level(self, voltage):
        if voltage <= self.voltage_lut[0][1]:
            return 0.0
        if voltage >= self.voltage_lut[-1][1]:
            return 100.0
        for i in range(len(self.voltage_lut) - 1):
            v_low = self.voltage_lut[i][1]
            v_high = self.voltage_lut[i + 1][1]
            pct_low = self.voltage_lut[i][0]
            pct_high = self.voltage_lut[i + 1][0]
            if v_low <= voltage <= v_high:
                ratio = (voltage - v_low) / (v_high - v_low) if v_high != v_low else 0.0
                return pct_low + ratio * (pct_high - pct_low)
        return 0.0

    def calculate_health_percent(self):
        health = 100.0 - (self.charge_cycles * 0.05)
        return max(0.0, min(100.0, health))

    # 增强功能1: 增强的健康评估 - 考虑容量退化
    def calculate_enhanced_health(self):
        cycle_health = 100.0 - (self.charge_cycles * self._cycle_degradation_rate)
        capacity_health = self._cycle_tracking['capacity_health_percent']
        enhanced_health = cycle_health * 0.6 + capacity_health * 0.4
        return max(0.0, min(100.0, enhanced_health))

    def calculate_estimated_time_remaining(self, charge_level, discharge_rate):
        if discharge_rate <= 0.0:
            return float('inf')
        remaining_ah = self.battery_capacity_ah * (charge_level / 100.0)
        hours_remaining = remaining_ah / abs(discharge_rate)
        return hours_remaining * 60.0

    # 增强功能1: 跟踪充电循环
    def _track_charge_cycles(self, charge_level):
        old_level = self._cycle_tracking['last_charge_level']
        self._cycle_tracking['last_charge_level'] = charge_level

        if old_level < 100.0 and charge_level >= 100.0:
            self.charge_cycles += 1
            self._cycle_tracking['cycle_progress'] = 0.0
            self.get_logger().info(f'完成充电循环 #{self.charge_cycles}')

            # 更新容量健康
            degradation = self.charge_cycles * self._cycle_degradation_rate
            self._cycle_tracking['capacity_health_percent'] = max(0.0, 100.0 - degradation)
            self._cycle_tracking['actual_capacity_ah'] = (
                self._cycle_tracking['design_capacity_ah'] *
                self._cycle_tracking['capacity_health_percent'] / 100.0)

            if self._cycle_tracking['capacity_health_percent'] < self._capacity_fade_threshold:
                self.get_logger().warn(
                    f'电池容量退化至 {self._cycle_tracking["capacity_health_percent"]:.1f}% '
                    f'(阈值: {self._capacity_fade_threshold}%)')
        elif old_level > charge_level:
            delta = old_level - charge_level
            self._cycle_tracking['cycle_progress'] += delta / 100.0
            self._cycle_tracking['partial_cycles'] += delta / 100.0

            if self._cycle_tracking['partial_cycles'] >= 1.0:
                self.charge_cycles += 1
                self._cycle_tracking['partial_cycles'] -= 1.0
                self.get_logger().info(f'累计部分充电循环完成 #{self.charge_cycles}')

    # 增强功能2: 电池单体均衡监控 - 模拟检测不均衡单体
    def check_cell_balance(self):
        avg_cell_voltage = self.voltage_ema / self.num_cells

        # 模拟单体电压偏差
        for i in range(self.num_cells):
            deviation = math.sin(self.sim_time * 0.01 + i * 0.5) * 0.02
            if i == 0:
                deviation += 0.01 * math.sin(self.sim_time * 0.005)
            if i == self.num_cells - 1:
                deviation -= 0.015 * math.cos(self.sim_time * 0.003)
            self._cell_voltages[i] = avg_cell_voltage + deviation

        max_cell = max(self._cell_voltages)
        min_cell = min(self._cell_voltages)
        deviation_mv = (max_cell - min_cell) * 1000.0
        self._max_cell_deviation_mv = deviation_mv

        old_imbalance = self._cell_imbalance_detected
        old_critical = self._cell_critical_imbalance

        self._cell_imbalance_detected = deviation_mv > self._cell_imbalance_threshold_mv
        self._cell_critical_imbalance = deviation_mv > self._cell_critical_imbalance_mv

        if self._cell_critical_imbalance and not old_critical:
            self.get_logger().error(
                f'严重单体不均衡! 偏差: {deviation_mv:.0f}mV (阈值: {self._cell_critical_imbalance_mv}mV)')
        elif self._cell_imbalance_detected and not old_imbalance:
            self.get_logger().warn(
                f'检测到单体不均衡: 偏差 {deviation_mv:.0f}mV (阈值: {self._cell_imbalance_threshold_mv}mV)')

        # 发布单体均衡数据
        balance_data = {
            'cell_voltages': [round(v, 4) for v in self._cell_voltages],
            'avg_cell_voltage': round(avg_cell_voltage, 4),
            'max_cell_voltage': round(max_cell, 4),
            'min_cell_voltage': round(min_cell, 4),
            'deviation_mv': round(deviation_mv, 1),
            'imbalance_detected': self._cell_imbalance_detected,
            'critical_imbalance': self._cell_critical_imbalance,
            'imbalance_threshold_mv': self._cell_imbalance_threshold_mv,
            'critical_threshold_mv': self._cell_critical_imbalance_mv,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(balance_data, ensure_ascii=False)
        self._cell_balance_pub.publish(msg)

    # 增强功能3: 温度充电控制 - 极端温度下降低充电速率
    def _compute_charge_rate_modifier(self, temperature):
        modifier = 1.0
        self._temperature_charging_limited = False

        if temperature < self._charging_min_temp:
            modifier = 0.0
            self._temperature_charging_limited = True
            self.get_logger().error(
                f'温度过低禁止充电: {temperature:.1f}C (最低: {self._charging_min_temp}C)')
        elif temperature < self._charging_min_temp + 5.0:
            modifier = self._cold_reduce_factor
            self._temperature_charging_limited = True
        elif temperature > self._charging_max_temp:
            modifier = 0.0
            self._temperature_charging_limited = True
            self.get_logger().error(
                f'温度过高禁止充电: {temperature:.1f}C (最高: {self._charging_max_temp}C)')
        elif temperature > self._charging_reduced_rate_temp:
            modifier = self._hot_reduce_factor
            self._temperature_charging_limited = True

        self._charge_rate_modifier = modifier
        return modifier

    # 增强功能4: 电池故障检测
    def _detect_faults(self, voltage, current, temperature):
        new_faults = set()
        now = time.time()

        # 过压检测
        if voltage > self._fault_overvoltage:
            new_faults.add('overvoltage')
            if 'overvoltage' not in self._active_faults:
                self.get_logger().error(f'过压故障! 电压: {voltage:.2f}V (阈值: {self._fault_overvoltage}V)')

        # 欠压检测
        if voltage < self._fault_undervoltage:
            new_faults.add('undervoltage')
            if 'undervoltage' not in self._active_faults:
                self.get_logger().error(f'欠压故障! 电压: {voltage:.2f}V (阈值: {self._fault_undervoltage}V)')

        # 过流检测
        if current > self._fault_overcurrent_charge:
            new_faults.add('overcurrent_charge')
            if 'overcurrent_charge' not in self._active_faults:
                self.get_logger().error(f'充电过流! 电流: {current:.2f}A (阈值: {self._fault_overcurrent_charge}A)')

        if current < -self._fault_overcurrent_discharge:
            new_faults.add('overcurrent_discharge')
            if 'overcurrent_discharge' not in self._active_faults:
                self.get_logger().error(f'放电过流! 电流: {current:.2f}A (阈值: {self._fault_overcurrent_discharge}A)')

        # 过热检测
        if temperature > self._fault_over_temperature:
            new_faults.add('over_temperature')
            if 'over_temperature' not in self._active_faults:
                self.get_logger().error(f'过热故障! 温度: {temperature:.1f}C (阈值: {self._fault_over_temperature}C)')

        # 电压快速变化检测
        dt = now - self._last_fault_check_time
        if dt > 0:
            dv = abs(voltage - self._last_fault_check_voltage)
            dv_per_sec = dv / dt
            if dv_per_sec > self._fault_rapid_voltage_change:
                new_faults.add('rapid_voltage_change')
                if 'rapid_voltage_change' not in self._active_faults:
                    self.get_logger().error(
                        f'电压快速变化! 变化率: {dv_per_sec:.2f}V/s (阈值: {self._fault_rapid_voltage_change}V/s)')

        self._last_fault_check_voltage = voltage
        self._last_fault_check_time = now

        # 发布故障状态
        cleared_faults = self._active_faults - new_faults
        for fault in cleared_faults:
            self.get_logger().info(f'故障已清除: {fault}')

        self._active_faults = new_faults

        if self._active_faults:
            fault_data = {
                'active_faults': list(self._active_faults),
                'voltage': round(voltage, 2),
                'current': round(current, 2),
                'temperature': round(temperature, 1),
                'timestamp': datetime.now().isoformat(),
            }
            msg = String()
            msg.data = json.dumps(fault_data, ensure_ascii=False)
            self._battery_fault_pub.publish(msg)

    # 增强功能5: 充电效率跟踪
    def compute_charging_efficiency(self):
        if self.charging_state != 'charging':
            if self._efficiency_tracking_start is not None:
                self._efficiency_tracking_start = None
            return

        now = time.time()
        if self._efficiency_tracking_start is None:
            self._efficiency_tracking_start = now
            self._charge_energy_added_wh = 0.0
            self._charge_energy_theoretical_wh = 0.0
            return

        elapsed = now - self._efficiency_tracking_start
        if elapsed < self._efficiency_window:
            return

        # 实际充入能量
        actual_wh = self._charge_energy_added_wh
        # 理论充入能量 (基于电流和电压)
        theoretical_wh = self._charge_energy_theoretical_wh

        if theoretical_wh > 0:
            self._charging_efficiency = min(1.0, actual_wh / theoretical_wh)
        else:
            self._charging_efficiency = 1.0

        self._efficiency_history.append(self._charging_efficiency)

        # 发布效率数据
        avg_efficiency = (
            sum(self._efficiency_history) / len(self._efficiency_history)
            if self._efficiency_history else self._charging_efficiency)

        efficiency_data = {
            'current_efficiency': round(self._charging_efficiency * 100, 1),
            'average_efficiency': round(avg_efficiency * 100, 1),
            'energy_added_wh': round(actual_wh, 1),
            'energy_theoretical_wh': round(theoretical_wh, 1),
            'charge_rate_modifier': round(self._charge_rate_modifier, 2),
            'temperature_limited': self._temperature_charging_limited,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(efficiency_data, ensure_ascii=False)
        self._charging_efficiency_pub.publish(msg)

        # 重置跟踪
        self._efficiency_tracking_start = now
        self._charge_energy_added_wh = 0.0
        self._charge_energy_theoretical_wh = 0.0

    def generate_simulated_data(self):
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        self.sim_time += dt

        # 增强功能3: 应用温度充电控制
        charge_rate_modifier = self._compute_charge_rate_modifier(self.temp_ema)

        discharge_rate = 2.0 + 0.5 * math.sin(self.sim_time * 0.01)
        self.sim_charge_level -= (discharge_rate * dt) / (self.battery_capacity_ah * 36.0)

        if self.sim_charge_level <= 5.0:
            self.charging_state = 'charging'
            charge_rate = 10.0 * charge_rate_modifier
            self.sim_charge_level += (charge_rate * dt) / (self.battery_capacity_ah * 36.0)
            if self.sim_charge_level >= 95.0:
                self.charging_state = 'discharging'
        elif self.sim_charge_level >= 95.0 and self.charging_state == 'charging':
            self.charging_state = 'discharging'

        self.sim_charge_level = max(0.0, min(100.0, self.sim_charge_level))

        voltage = self.min_voltage + (self.max_voltage - self.min_voltage) * (self.sim_charge_level / 100.0)
        voltage += 0.1 * math.sin(self.sim_time * 0.5)
        voltage += 0.05 * math.sin(self.sim_time * 2.0)

        if self.charging_state == 'charging':
            current = 10.0 * charge_rate_modifier + 0.5 * math.sin(self.sim_time * 0.3)
        else:
            current = -(discharge_rate + 0.2 * math.sin(self.sim_time * 0.7))

        temperature = 25.0 + 5.0 * math.sin(self.sim_time * 0.02) + abs(current) * 0.1

        return voltage, current, temperature

    def check_alerts(self, charge_level, voltage, temperature):
        alerts = []

        if charge_level <= self.critical_battery_threshold and not self.alert_states['critical_battery']:
            self.alert_states['critical_battery'] = True
            alerts.append(f'CRITICAL BATTERY: {charge_level:.1f}%')
        elif charge_level > self.critical_battery_threshold:
            self.alert_states['critical_battery'] = False

        if charge_level <= self.low_battery_threshold and not self.alert_states['low_battery']:
            self.alert_states['low_battery'] = True
            alerts.append(f'LOW BATTERY: {charge_level:.1f}%')
        elif charge_level > self.low_battery_threshold:
            self.alert_states['low_battery'] = False

        if temperature >= self.over_temperature_threshold and not self.alert_states['over_temperature']:
            self.alert_states['over_temperature'] = True
            alerts.append(f'OVER TEMPERATURE: {temperature:.1f}C')
        elif temperature < self.over_temperature_threshold - 2.0:
            self.alert_states['over_temperature'] = False

        if voltage <= self.under_voltage_threshold and not self.alert_states['under_voltage']:
            self.alert_states['under_voltage'] = True
            alerts.append(f'UNDER VOLTAGE: {voltage:.2f}V')
        elif voltage > self.under_voltage_threshold + 1.0:
            self.alert_states['under_voltage'] = False

        return alerts

    def timer_callback(self):
        if self.simulate:
            voltage, current, temperature = self.generate_simulated_data()
        else:
            voltage = self.raw_voltage
            current = self.raw_current
            temperature = self.raw_temp

        self.voltage_ema = self.update_ema(voltage, self.voltage_ema)
        self.current_ema = self.update_ema(current, self.current_ema)
        self.temp_ema = self.update_ema(temperature, self.temp_ema)
        self.ema_initialized = True

        charge_level = self.voltage_to_charge_level(self.voltage_ema)
        health_percent = self.calculate_enhanced_health()

        if self.current_ema > 0.5:
            self.charging_state = 'charging'
            charge_rate = self.current_ema * self._charge_rate_modifier
            discharge_rate = 0.0
        elif self.current_ema < -0.5:
            self.charging_state = 'discharging'
            charge_rate = 0.0
            discharge_rate = abs(self.current_ema)
        else:
            self.charging_state = 'idle'
            charge_rate = 0.0
            discharge_rate = 0.0

        estimated_time = self.calculate_estimated_time_remaining(charge_level, discharge_rate)

        # 增强功能1: 跟踪充电循环
        self._track_charge_cycles(charge_level)

        # 增强功能4: 故障检测
        self._detect_faults(self.voltage_ema, self.current_ema, self.temp_ema)

        # 增强功能5: 跟踪充放电能量
        now = time.time()
        dt = now - self._last_stats_update
        self._last_stats_update = now
        power_w = self.voltage_ema * abs(self.current_ema) * dt / 3600.0
        if self.charging_state == 'charging':
            self._charge_energy_added_wh += power_w
            self._charge_energy_theoretical_wh += self.voltage_ema * self.current_ema * dt / 3600.0
            self._battery_stats['total_energy_charged_wh'] += power_w
            self._battery_stats['total_charge_time_seconds'] += dt
        elif self.charging_state == 'discharging':
            self._battery_stats['total_energy_consumed_wh'] += power_w
            self._battery_stats['total_discharge_time_seconds'] += dt

        # 更新统计极值
        if self.temp_ema > self._battery_stats['max_temperature']:
            self._battery_stats['max_temperature'] = round(self.temp_ema, 1)
        if self.temp_ema < self._battery_stats['min_temperature']:
            self._battery_stats['min_temperature'] = round(self.temp_ema, 1)
        if self.current_ema > self._battery_stats['max_charge_current']:
            self._battery_stats['max_charge_current'] = round(self.current_ema, 2)
        if self.current_ema < -self._battery_stats['max_discharge_current']:
            self._battery_stats['max_discharge_current'] = round(abs(self.current_ema), 2)

        msg = BatteryState()
        msg.voltage = self.voltage_ema
        msg.current = self.current_ema
        msg.charge_level = charge_level
        msg.temperature = self.temp_ema
        msg.health_percent = health_percent
        msg.charging_state = self.charging_state
        msg.charge_rate = charge_rate
        msg.discharge_rate = discharge_rate
        msg.estimated_time_remaining = estimated_time
        msg.charge_cycles = self.charge_cycles
        msg.battery_type = self.battery_type
        msg.timestamp = self.get_clock().now().to_msg()

        self.battery_state_pub.publish(msg)

        alerts = self.check_alerts(charge_level, self.voltage_ema, self.temp_ema)
        for alert_text in alerts:
            alert_msg = String()
            alert_msg.data = alert_text
            self.battery_alert_pub.publish(alert_msg)

        # 增强功能1: 发布电池健康评估
        self._publish_battery_health(charge_level, health_percent)

        # 增强功能3: 发布温度充电控制状态
        self._publish_charging_temp_status()

    # 增强功能1: 发布电池健康评估
    def _publish_battery_health(self, charge_level, health_percent):
        health_data = {
            'health_percent': round(health_percent, 1),
            'charge_cycles': self.charge_cycles,
            'partial_cycles': round(self._cycle_tracking['partial_cycles'], 2),
            'design_capacity_ah': self._cycle_tracking['design_capacity_ah'],
            'actual_capacity_ah': round(self._cycle_tracking['actual_capacity_ah'], 1),
            'capacity_health_percent': round(self._cycle_tracking['capacity_health_percent'], 1),
            'capacity_fade_threshold': self._capacity_fade_threshold,
            'capacity_faded': self._cycle_tracking['capacity_health_percent'] < self._capacity_fade_threshold,
            'charge_level': round(charge_level, 1),
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(health_data, ensure_ascii=False)
        self._battery_health_pub.publish(msg)

    # 增强功能3: 发布温度充电控制状态
    def _publish_charging_temp_status(self):
        temp_data = {
            'temperature': round(self.temp_ema, 1),
            'charge_rate_modifier': round(self._charge_rate_modifier, 2),
            'temperature_limited': self._temperature_charging_limited,
            'min_charging_temp': self._charging_min_temp,
            'max_charging_temp': self._charging_max_temp,
            'reduced_rate_temp': self._charging_reduced_rate_temp,
            'charging_state': self.charging_state,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(temp_data, ensure_ascii=False)
        self._charging_temp_pub.publish(msg)

    # 增强功能6: 发布电池统计
    def publish_battery_stats(self):
        stats_data = {
            'charge_cycles': self.charge_cycles,
            'total_energy_consumed_wh': round(self._battery_stats['total_energy_consumed_wh'], 1),
            'total_energy_charged_wh': round(self._battery_stats['total_energy_charged_wh'], 1),
            'total_discharge_time_seconds': round(self._battery_stats['total_discharge_time_seconds'], 0),
            'total_charge_time_seconds': round(self._battery_stats['total_charge_time_seconds'], 0),
            'max_temperature': self._battery_stats['max_temperature'],
            'min_temperature': self._battery_stats['min_temperature'],
            'max_charge_current': self._battery_stats['max_charge_current'],
            'max_discharge_current': self._battery_stats['max_discharge_current'],
            'health_percent': round(self.calculate_enhanced_health(), 1),
            'actual_capacity_ah': round(self._cycle_tracking['actual_capacity_ah'], 1),
            'charging_efficiency': round(self._charging_efficiency * 100, 1),
            'cell_imbalance_detected': self._cell_imbalance_detected,
            'cell_critical_imbalance': self._cell_critical_imbalance,
            'max_cell_deviation_mv': round(self._max_cell_deviation_mv, 1),
            'active_faults': list(self._active_faults),
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(stats_data, ensure_ascii=False)
        self._battery_stats_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
