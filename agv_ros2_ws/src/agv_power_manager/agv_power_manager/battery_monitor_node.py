# 电池监控节点，监测电池电压、电流、温度等状态，支持模拟数据和真实传感器数据
import math
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, String
from agv_interfaces.msg import BatteryState

import rclpy


# BatteryMonitorNode: 电池状态监控ROS2节点
# 支持模拟模式和真实传感器模式，使用指数移动平均(EMA)滤波原始数据
# 通过电压-电量查找表估算剩余电量，并提供低电量、过温等告警功能
class BatteryMonitorNode(Node):

    def __init__(self):
        super().__init__('battery_monitor')

        # 声明轮询参数
        self.declare_parameter('poll_rate', 2.0)
        # 声明电池参数
        self.declare_parameter('battery_capacity_ah', 50.0)
        self.declare_parameter('nominal_voltage', 48.0)
        self.declare_parameter('min_voltage', 39.0)
        self.declare_parameter('max_voltage', 54.6)
        self.declare_parameter('num_cells', 13)
        # 声明是否使用模拟数据
        self.declare_parameter('simulate', True)
        # 声明告警阈值参数
        self.declare_parameter('alert_thresholds.low_battery', 20.0)
        self.declare_parameter('alert_thresholds.critical_battery', 10.0)
        self.declare_parameter('alert_thresholds.over_temperature', 45.0)
        self.declare_parameter('alert_thresholds.under_voltage', 40.0)

        # 获取轮询参数
        self.poll_rate = self.get_parameter('poll_rate').value
        # 获取电池参数
        self.battery_capacity_ah = self.get_parameter('battery_capacity_ah').value
        self.nominal_voltage = self.get_parameter('nominal_voltage').value
        self.min_voltage = self.get_parameter('min_voltage').value
        self.max_voltage = self.get_parameter('max_voltage').value
        self.num_cells = self.get_parameter('num_cells').value
        self.simulate = self.get_parameter('simulate').value
        # 获取告警阈值
        self.low_battery_threshold = self.get_parameter('alert_thresholds.low_battery').value
        self.critical_battery_threshold = self.get_parameter('alert_thresholds.critical_battery').value
        self.over_temperature_threshold = self.get_parameter('alert_thresholds.over_temperature').value
        self.under_voltage_threshold = self.get_parameter('alert_thresholds.under_voltage').value

        # EMA滤波参数，alpha越小滤波越平滑
        self.ema_alpha = 0.1
        self.voltage_ema = self.nominal_voltage
        self.current_ema = 0.0
        self.temp_ema = 25.0
        self.ema_initialized = False

        # 原始传感器数据
        self.raw_voltage = self.nominal_voltage
        self.raw_current = 0.0
        self.raw_temp = 25.0

        # 电池状态信息
        self.charge_cycles = 0
        self.battery_type = 'LiFePO4'
        self.charging_state = 'discharging'
        # 模拟数据相关状态
        self.sim_time = 0.0
        self.sim_charge_level = 75.0
        self.last_update_time = time.time()

        # 电压-电量查找表，用于通过电压估算剩余电量百分比
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

        # 使用BEST_EFFORT策略发布传感器数据
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建发布者：电池状态和告警
        self.battery_state_pub = self.create_publisher(BatteryState, 'battery_state', qos_profile)
        self.battery_alert_pub = self.create_publisher(String, 'battery_alert', 10)

        # 创建订阅者：接收原始电压、电流、温度数据
        self.voltage_sub = self.create_subscription(
            Float64, 'battery_voltage_raw', self.voltage_callback, qos_profile)
        self.current_sub = self.create_subscription(
            Float64, 'battery_current_raw', self.current_callback, qos_profile)
        self.temp_sub = self.create_subscription(
            Float64, 'battery_temp_raw', self.temp_callback, qos_profile)

        # 创建定时器，按指定频率更新电池状态
        self.timer = self.create_timer(1.0 / self.poll_rate, self.timer_callback)

        # 告警状态字典，防止重复触发同一告警
        self.alert_states = {
            'low_battery': False,
            'critical_battery': False,
            'over_temperature': False,
            'under_voltage': False,
        }

    # 电压原始数据回调
    def voltage_callback(self, msg):
        self.raw_voltage = msg.data

    # 电流原始数据回调
    def current_callback(self, msg):
        self.raw_current = msg.data

    # 温度原始数据回调
    def temp_callback(self, msg):
        self.raw_temp = msg.data

    # EMA滤波更新，首次调用直接使用原始值
    def update_ema(self, raw_value, current_ema):
        if not self.ema_initialized:
            return raw_value
        return self.ema_alpha * raw_value + (1.0 - self.ema_alpha) * current_ema

    # 通过电压值在查找表中线性插值估算剩余电量百分比
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

    # 计算电池健康度，基于充电循环次数估算衰减
    def calculate_health_percent(self):
        health = 100.0 - (self.charge_cycles * 0.05)
        return max(0.0, min(100.0, health))

    # 计算预计剩余使用时间（分钟），基于当前放电速率
    def calculate_estimated_time_remaining(self, charge_level, discharge_rate):
        if discharge_rate <= 0.0:
            return float('inf')
        remaining_ah = self.battery_capacity_ah * (charge_level / 100.0)
        hours_remaining = remaining_ah / abs(discharge_rate)
        return hours_remaining * 60.0

    # 生成模拟电池数据，模拟放电和充电过程
    def generate_simulated_data(self):
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        self.sim_time += dt

        # 模拟放电速率，带有正弦波动
        discharge_rate = 2.0 + 0.5 * math.sin(self.sim_time * 0.01)
        self.sim_charge_level -= (discharge_rate * dt) / (self.battery_capacity_ah * 36.0)

        # 电量低于5%时切换为充电状态
        if self.sim_charge_level <= 5.0:
            self.charging_state = 'charging'
            charge_rate = 10.0
            self.sim_charge_level += (charge_rate * dt) / (self.battery_capacity_ah * 36.0)
            if self.sim_charge_level >= 95.0:
                self.charging_state = 'discharging'
        elif self.sim_charge_level >= 95.0 and self.charging_state == 'charging':
            self.charging_state = 'discharging'

        self.sim_charge_level = max(0.0, min(100.0, self.sim_charge_level))

        # 根据电量百分比计算电压，添加噪声模拟真实波动
        voltage = self.min_voltage + (self.max_voltage - self.min_voltage) * (self.sim_charge_level / 100.0)
        voltage += 0.1 * math.sin(self.sim_time * 0.5)
        voltage += 0.05 * math.sin(self.sim_time * 2.0)

        # 根据充放电状态模拟电流
        if self.charging_state == 'charging':
            current = 10.0 + 0.5 * math.sin(self.sim_time * 0.3)
        else:
            current = -(discharge_rate + 0.2 * math.sin(self.sim_time * 0.7))

        # 模拟温度，与电流大小和环境温度相关
        temperature = 25.0 + 5.0 * math.sin(self.sim_time * 0.02) + abs(current) * 0.1

        return voltage, current, temperature

    # 检查告警条件，返回触发的告警消息列表
    def check_alerts(self, charge_level, voltage, temperature):
        alerts = []

        # 严重低电量告警
        if charge_level <= self.critical_battery_threshold and not self.alert_states['critical_battery']:
            self.alert_states['critical_battery'] = True
            alerts.append(f'CRITICAL BATTERY: {charge_level:.1f}%')
        elif charge_level > self.critical_battery_threshold:
            self.alert_states['critical_battery'] = False

        # 低电量告警
        if charge_level <= self.low_battery_threshold and not self.alert_states['low_battery']:
            self.alert_states['low_battery'] = True
            alerts.append(f'LOW BATTERY: {charge_level:.1f}%')
        elif charge_level > self.low_battery_threshold:
            self.alert_states['low_battery'] = False

        # 过温告警
        if temperature >= self.over_temperature_threshold and not self.alert_states['over_temperature']:
            self.alert_states['over_temperature'] = True
            alerts.append(f'OVER TEMPERATURE: {temperature:.1f}C')
        elif temperature < self.over_temperature_threshold - 2.0:
            self.alert_states['over_temperature'] = False

        # 欠压告警
        if voltage <= self.under_voltage_threshold and not self.alert_states['under_voltage']:
            self.alert_states['under_voltage'] = True
            alerts.append(f'UNDER VOLTAGE: {voltage:.2f}V')
        elif voltage > self.under_voltage_threshold + 1.0:
            self.alert_states['under_voltage'] = False

        return alerts

    # 定时器回调，更新电池状态并发布
    def timer_callback(self):
        # 根据模式获取数据源
        if self.simulate:
            voltage, current, temperature = self.generate_simulated_data()
        else:
            voltage = self.raw_voltage
            current = self.raw_current
            temperature = self.raw_temp

        # 对原始数据进行EMA滤波
        self.voltage_ema = self.update_ema(voltage, self.voltage_ema)
        self.current_ema = self.update_ema(current, self.current_ema)
        self.temp_ema = self.update_ema(temperature, self.temp_ema)
        self.ema_initialized = True

        # 根据滤波后电压估算电量百分比
        charge_level = self.voltage_to_charge_level(self.voltage_ema)
        health_percent = self.calculate_health_percent()

        # 根据电流方向判断充放电状态
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

        # 计算预计剩余使用时间
        estimated_time = self.calculate_estimated_time_remaining(charge_level, discharge_rate)

        # 构建并发布电池状态消息
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

        # 检查并发布告警
        alerts = self.check_alerts(charge_level, self.voltage_ema, self.temp_ema)
        for alert_text in alerts:
            alert_msg = String()
            alert_msg.data = alert_text
            self.battery_alert_pub.publish(alert_msg)


# 节点入口函数
def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
