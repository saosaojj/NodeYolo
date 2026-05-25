import math
import time

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

        self.voltage_sub = self.create_subscription(
            Float64, 'battery_voltage_raw', self.voltage_callback, qos_profile)
        self.current_sub = self.create_subscription(
            Float64, 'battery_current_raw', self.current_callback, qos_profile)
        self.temp_sub = self.create_subscription(
            Float64, 'battery_temp_raw', self.temp_callback, qos_profile)

        self.timer = self.create_timer(1.0 / self.poll_rate, self.timer_callback)

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

    def calculate_estimated_time_remaining(self, charge_level, discharge_rate):
        if discharge_rate <= 0.0:
            return float('inf')
        remaining_ah = self.battery_capacity_ah * (charge_level / 100.0)
        hours_remaining = remaining_ah / abs(discharge_rate)
        return hours_remaining * 60.0

    def generate_simulated_data(self):
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        self.sim_time += dt

        discharge_rate = 2.0 + 0.5 * math.sin(self.sim_time * 0.01)
        self.sim_charge_level -= (discharge_rate * dt) / (self.battery_capacity_ah * 36.0)

        if self.sim_charge_level <= 5.0:
            self.charging_state = 'charging'
            charge_rate = 10.0
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
            current = 10.0 + 0.5 * math.sin(self.sim_time * 0.3)
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
        health_percent = self.calculate_health_percent()

        if self.current_ema > 0.5:
            self.charging_state = 'charging'
            charge_rate = self.current_ema
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


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
