import copy
import json
import logging
import os
import threading
import time
import yaml
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from agv_interfaces.msg import PlcData, PlcConfig
from agv_interfaces.srv import ReadPlc, WritePlc, SetConfig, GetConfig, SaveConfig, LoadConfig
from std_msgs.msg import String, Int32
from pymodbus.client import ModbusTcpClient


@dataclass
class PlcDevice:
    name: str = ''
    ip: str = ''
    port: int = 502
    slave_id: int = 1
    connected: bool = False
    client: ModbusTcpClient = field(default=None, repr=False)
    coil_read_start: int = 0
    coil_read_count: int = 16
    register_read_start: int = 0
    register_read_count: int = 16
    timeout: float = 5.0
    retry_backoff: float = 1.0
    max_retry_backoff: float = 30.0
    last_successful_poll: float = 0.0
    reconnect_lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    is_master: bool = True
    slave_control_map: Dict[str, Dict] = field(default_factory=dict)
    # 新增：上次读取的线圈值缓存
    cached_coil_values: List[int] = field(default_factory=list)
    # 新增：上次读取的寄存器值缓存
    cached_register_values: List[int] = field(default_factory=list)
    # 新增：缓存时间戳
    cache_timestamp: float = 0.0
    # 新增：上次发布的线圈值（用于变更检测）
    last_published_coils: List[int] = field(default_factory=list)
    # 新增：上次发布的寄存器值（用于变更检测）
    last_published_registers: List[int] = field(default_factory=list)


@dataclass
class PlcCommStats:
    """PLC通信统计数据"""
    total_reads: int = 0
    successful_reads: int = 0
    failed_reads: int = 0
    total_writes: int = 0
    successful_writes: int = 0
    failed_writes: int = 0
    total_read_latency_ms: float = 0.0
    total_write_latency_ms: float = 0.0
    last_read_latency_ms: float = 0.0
    last_write_latency_ms: float = 0.0
    consecutive_errors: int = 0
    last_error_time: float = 0.0
    last_error_msg: str = ''

    @property
    def read_success_rate(self):
        return self.successful_reads / max(self.total_reads, 1)

    @property
    def write_success_rate(self):
        return self.successful_writes / max(self.total_writes, 1)

    @property
    def avg_read_latency_ms(self):
        return self.total_read_latency_ms / max(self.successful_reads, 1)

    @property
    def avg_write_latency_ms(self):
        return self.total_write_latency_ms / max(self.successful_writes, 1)


class PlcManagerNode(Node):

    def __init__(self):
        super().__init__('plc_manager_node')

        self.declare_parameter('plc_config_file', '')
        self.declare_parameter('poll_rate_ms', 100)
        self.declare_parameter('default_timeout', 5.0)
        self.declare_parameter('max_retry_backoff', 30.0)
        self.declare_parameter('health_check_interval', 10.0)
        self.declare_parameter('auto_slave_control', True)
        # 新增参数：数据变更检测死带（寄存器值变化小于此值不发布）
        self.declare_parameter('analog_deadband', 1)
        # 新增参数：数据日志记录间隔（秒）
        self.declare_parameter('data_log_interval', 60.0)
        # 新增参数：数据日志文件路径
        self.declare_parameter('data_log_path', '/tmp/plc_data_log.csv')
        # 新增参数：缓存过期时间（秒），超过此时间视为过期数据
        self.declare_parameter('cache_stale_timeout', 5.0)
        # 新增参数：批量读取最大寄存器数量
        self.declare_parameter('batch_read_max_registers', 125)

        self.devices: Dict[str, PlcDevice] = {}
        self.publishers: Dict[str, object] = {}
        self._device_timeouts: Dict[str, float] = {}
        self._config_lock = threading.Lock()

        # 新增：通信统计数据 {device_name: PlcCommStats}
        self._comm_stats: Dict[str, PlcCommStats] = {}
        # 新增：寄存器映射配置 {device_name: {register_name: {address, type, ...}}}
        self._register_maps: Dict[str, Dict] = {}
        # 新增：告警条件定义 {alarm_name: {device, register, condition, ...}}
        self._alarm_conditions: Dict[str, Dict] = {}
        # 新增：告警状态跟踪 {alarm_name: {active: bool, last_trigger_time: float}}
        self._alarm_states: Dict[str, Dict] = {}
        # 新增：数据日志最后记录时间
        self._last_data_log_time: float = 0.0
        # 新增：批量读请求队列 {device_name: [(start_addr, count), ...]}
        self._batch_read_requests: Dict[str, List] = {}

        self._default_timeout = self.get_parameter('default_timeout').value
        self._max_retry_backoff = self.get_parameter('max_retry_backoff').value
        self._auto_slave_control = self.get_parameter('auto_slave_control').value
        self._analog_deadband = self.get_parameter('analog_deadband').value
        self._data_log_interval = self.get_parameter('data_log_interval').value
        self._data_log_path = self.get_parameter('data_log_path').value
        self._cache_stale_timeout = self.get_parameter('cache_stale_timeout').value
        self._batch_read_max = self.get_parameter('batch_read_max_registers').value

        self._current_config = PlcConfig()

        config_file = self.get_parameter('plc_config_file').value
        if config_file:
            self._load_config(config_file)

        # 原有定时器
        poll_rate = self.get_parameter('poll_rate_ms').value
        self.poll_timer = self.create_timer(poll_rate / 1000.0, self.poll_all)

        health_interval = self.get_parameter('health_check_interval').value
        self._health_timer = self.create_timer(health_interval, self._health_check)

        if self._auto_slave_control:
            self._slave_timer = self.create_timer(0.2, self._slave_control_loop)

        # 新增定时器：数据日志记录
        self._data_log_timer = self.create_timer(self._data_log_interval, self._data_log_callback)

        # 原有服务
        self.read_plc_srv = self.create_service(ReadPlc, 'read_plc', self.read_plc_callback)
        self.write_plc_srv = self.create_service(WritePlc, 'write_plc', self.write_plc_callback)
        self.set_config_srv = self.create_service(SetConfig, 'set_plc_config', self.set_config_callback)
        self.get_config_srv = self.create_service(GetConfig, 'get_plc_config', self.get_config_callback)
        self.save_config_srv = self.create_service(SaveConfig, 'save_plc_config', self.save_config_callback)
        self.load_config_srv = self.create_service(LoadConfig, 'load_plc_config', self.load_config_callback)

        # 新增服务：获取通信统计
        self.get_comm_stats_srv = self.create_service(GetConfig, 'get_plc_comm_stats', self.get_comm_stats_callback)
        # 新增服务：获取缓存数据
        self.get_cached_data_srv = self.create_service(GetConfig, 'get_plc_cached_data', self.get_cached_data_callback)
        # 新增服务：批量读取
        self.batch_read_srv = self.create_service(ReadPlc, 'batch_read_plc', self.batch_read_callback)
        # 新增服务：批量写入
        self.batch_write_srv = self.create_service(WritePlc, 'batch_write_plc', self.batch_write_callback)
        # 新增服务：配置告警条件
        self.config_alarm_srv = self.create_service(SetConfig, 'config_plc_alarm', self.config_alarm_callback)
        # 新增服务：获取告警状态
        self.get_alarm_status_srv = self.create_service(GetConfig, 'get_plc_alarm_status', self.get_alarm_status_callback)

        # 原有发布者
        self.config_pub = self.create_publisher(PlcConfig, 'plc_config', 10)

        # 新增发布者：PLC告警事件
        self.plc_alarm_pub = self.create_publisher(String, 'plc_alarm', 10)
        # 新增发布者：PLC通信统计
        self.plc_stats_pub = self.create_publisher(String, 'plc_comm_stats', 10)
        # 新增发布者：PLC数据变更
        self.plc_data_changed_pub = self.create_publisher(String, 'plc_data_changed', 10)
        # 新增发布者：缓存过期通知
        self.plc_cache_stale_pub = self.create_publisher(String, 'plc_cache_stale', 10)

        self.get_logger().info(
            f'PlcManagerNode已启动，共 {len(self.devices)} 个设备，'
            f'支持数据缓存、通信统计、变更检测、寄存器映射、告警管理')

    # ==================== 配置加载（增强） ====================

    def _load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            params = config.get('plc_manager', {}).get('ros__parameters', {})
            devices = params.get('devices', {})

            for name, dev_conf in devices.items():
                self.add_plc(
                    device_name=name,
                    ip=dev_conf.get('ip', '127.0.0.1'),
                    port=dev_conf.get('port', 502),
                    slave_id=dev_conf.get('slave_id', 1),
                    coil_read_start=dev_conf.get('coil_read_start', 0),
                    coil_read_count=dev_conf.get('coil_read_count', 16),
                    register_read_start=dev_conf.get('register_read_start', 0),
                    register_read_count=dev_conf.get('register_read_count', 16),
                    timeout=dev_conf.get('timeout', self._default_timeout),
                    is_master=dev_conf.get('is_master', True),
                    slave_control_map=dev_conf.get('slave_control_map', {}),
                )

            # 加载寄存器映射配置
            register_maps = params.get('register_maps', {})
            for device_name, reg_map in register_maps.items():
                self._register_maps[device_name] = reg_map
                self.get_logger().info(f'已加载设备 {device_name} 的寄存器映射: {len(reg_map)} 个寄存器')

            # 加载告警条件配置
            alarms = params.get('alarms', {})
            for alarm_name, alarm_conf in alarms.items():
                self._alarm_conditions[alarm_name] = alarm_conf
                self._alarm_states[alarm_name] = {
                    'active': False,
                    'last_trigger_time': 0.0,
                    'last_clear_time': 0.0
                }
                self.get_logger().info(f'已加载告警条件: {alarm_name}')

        except Exception as e:
            self.get_logger().error(f'加载配置文件 {config_file} 失败: {e}')

    # ==================== 设备管理（增强） ====================

    def add_plc(self, device_name, ip, port=502, slave_id=1,
                coil_read_start=0, coil_read_count=16,
                register_read_start=0, register_read_count=16,
                timeout=None, is_master=True, slave_control_map=None):
        with self._config_lock:
            if device_name in self.devices:
                self.get_logger().warn(f'设备 {device_name} 已存在，替换')
                self.remove_plc(device_name, skip_log=True)

            device_timeout = timeout if timeout is not None else self._default_timeout

            device = PlcDevice(
                name=device_name,
                ip=ip,
                port=port,
                slave_id=slave_id,
                coil_read_start=coil_read_start,
                coil_read_count=coil_read_count,
                register_read_start=register_read_start,
                register_read_count=register_read_count,
                timeout=device_timeout,
                max_retry_backoff=self._max_retry_backoff,
                is_master=is_master,
                slave_control_map=slave_control_map if slave_control_map else {},
            )

            device.client = ModbusTcpClient(host=ip, port=port, timeout=device_timeout)
            result = device.client.connect()
            device.connected = result

            if result:
                device.last_successful_poll = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(f'已连接到 {device_name} ({ip}:{port}, master={is_master})')
            else:
                self.get_logger().warn(f'连接 {device_name} ({ip}:{port}) 失败')

            self.devices[device_name] = device
            self.publishers[device_name] = self.create_publisher(
                PlcData, f'plc_status/{device_name}', 10)

            # 初始化通信统计
            self._comm_stats[device_name] = PlcCommStats()
            # 初始化批量读请求
            self._batch_read_requests[device_name] = []

            self._update_current_config()

    def remove_plc(self, device_name, skip_log=False):
        with self._config_lock:
            if device_name not in self.devices:
                if not skip_log:
                    self.get_logger().warn(f'设备 {device_name} 未找到')
                return

            device = self.devices[device_name]
            if device.client:
                device.client.close()
            device.connected = False

            del self.devices[device_name]
            del self.publishers[device_name]
            self._comm_stats.pop(device_name, None)
            self._register_maps.pop(device_name, None)
            self._batch_read_requests.pop(device_name, None)

            if not skip_log:
                self.get_logger().info(f'已移除设备 {device_name}')

            self._update_current_config()

    def _update_current_config(self):
        self._current_config = PlcConfig()
        self._current_config.device_names = list(self.devices.keys())
        self._current_config.ips = [d.ip for d in self.devices.values()]
        self._current_config.ports = [d.port for d in self.devices.values()]
        self._current_config.slave_ids = [d.slave_id for d in self.devices.values()]
        self._current_config.coil_read_starts = [d.coil_read_start for d in self.devices.values()]
        self._current_config.coil_read_counts = [d.coil_read_count for d in self.devices.values()]
        self._current_config.register_read_starts = [d.register_read_start for d in self.devices.values()]
        self._current_config.register_read_counts = [d.register_read_count for d in self.devices.values()]
        self._current_config.is_masters = [d.is_master for d in self.devices.values()]
        self.config_pub.publish(self._current_config)

    # ==================== 重连（增强：指数退避） ====================

    def _reconnect_device(self, device):
        with device.reconnect_lock:
            if device.connected:
                return True
            try:
                device.client.close()
            except Exception:
                pass
            result = device.client.connect()
            device.connected = result
            if result:
                device.retry_backoff = 1.0
                device.last_successful_poll = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(f'已重连到 {device.name} ({device.ip}:{device.port})')
                # 重连成功，重置连续错误计数
                stats = self._comm_stats.get(device.name)
                if stats:
                    stats.consecutive_errors = 0
            else:
                device.retry_backoff = min(device.retry_backoff * 2.0, device.max_retry_backoff)
                self.get_logger().warn(
                    f'重连 {device.name} 失败，下次退避 {device.retry_backoff:.1f}s')
            return result

    # ==================== 健康检查（增强） ====================

    def _health_check(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for name, device in self.devices.items():
            if not device.connected:
                self._reconnect_device(device)
            elif device.last_successful_poll > 0:
                elapsed = now - device.last_successful_poll
                if elapsed > device.timeout * 3:
                    self.get_logger().warn(
                        f'健康检查: 设备 {name} 已 {elapsed:.1f}s 无成功轮询')
                    device.connected = False
                    self._reconnect_device(device)

            # 检查缓存是否过期
            if device.cache_timestamp > 0:
                cache_age = now - device.cache_timestamp
                if cache_age > self._cache_stale_timeout:
                    stale_info = {
                        'device': name,
                        'cache_age': cache_age,
                        'threshold': self._cache_stale_timeout,
                        'timestamp': now
                    }
                    msg = String()
                    msg.data = json.dumps(stale_info, ensure_ascii=False)
                    self.plc_cache_stale_pub.publish(msg)
                    self.get_logger().warn(
                        f'缓存过期: 设备 {name} 缓存已 {cache_age:.1f}s 未更新')

        # 定期发布通信统计
        self._publish_comm_stats()

    # ==================== 数据变更检测 ====================

    def _detect_data_change(self, device, new_coils, new_registers):
        """检测数据变更，仅当值实际改变时返回True（模拟量使用死带）"""
        coil_changed = False
        register_changed = False
        changed_details = []

        # 线圈变更检测（精确匹配）
        if new_coils != device.last_published_coils:
            for i, (old, new) in enumerate(
                    zip(device.last_published_coils, new_coils)):
                if old != new:
                    changed_details.append({
                        'type': 'coil',
                        'address': device.coil_read_start + i,
                        'old_value': old,
                        'new_value': new
                    })
            coil_changed = True

        # 寄存器变更检测（使用死带）
        if new_registers != device.last_published_registers:
            for i, (old, new) in enumerate(
                    zip(device.last_published_registers, new_registers)):
                if abs(new - old) > self._analog_deadband:
                    changed_details.append({
                        'type': 'register',
                        'address': device.register_read_start + i,
                        'old_value': old,
                        'new_value': new
                    })
            register_changed = True

        return coil_changed or register_changed, changed_details

    # ==================== 轮询（增强：缓存、变更检测、告警） ====================

    def poll_all(self):
        for name, device in self.devices.items():
            plc_data = PlcData()
            plc_data.device_name = name
            plc_data.ip_address = device.ip
            plc_data.connected = device.connected
            plc_data.timestamp = self.get_clock().now().to_msg()

            if not device.connected:
                self._reconnect_device(device)
                if not device.connected:
                    plc_data.coil_values = []
                    plc_data.register_values = []
                    if name in self.publishers:
                        self.publishers[name].publish(plc_data)
                    continue

            # 读取线圈
            read_start = time.time()
            coil_result = device.client.read_coils(
                address=device.coil_read_start,
                count=device.coil_read_count,
                slave=device.slave_id,
            )
            read_latency = (time.time() - read_start) * 1000.0

            stats = self._comm_stats.get(name)
            if stats:
                stats.total_reads += 1
                stats.total_read_latency_ms += read_latency
                stats.last_read_latency_ms = read_latency

            if not coil_result.isError():
                coil_bits = coil_result.bits[:device.coil_read_count]
                new_coils = [int(b) for b in coil_bits]
                plc_data.coil_values = new_coils

                # 更新缓存
                device.cached_coil_values = new_coils[:]
                device.cache_timestamp = self.get_clock().now().nanoseconds / 1e9

                if stats:
                    stats.successful_reads += 1
                    stats.consecutive_errors = 0
            else:
                device.connected = False
                plc_data.coil_values = []
                if stats:
                    stats.failed_reads += 1
                    stats.consecutive_errors += 1
                    stats.last_error_time = time.time()
                    stats.last_error_msg = str(coil_result)

            # 读取寄存器
            read_start = time.time()
            register_result = device.client.read_holding_registers(
                address=device.register_read_start,
                count=device.register_read_count,
                slave=device.slave_id,
            )
            read_latency = (time.time() - read_start) * 1000.0

            if stats:
                stats.total_reads += 1
                stats.total_read_latency_ms += read_latency
                stats.last_read_latency_ms = read_latency

            if not register_result.isError():
                register_values = register_result.registers[:device.register_read_count]
                new_registers = [int(v) for v in register_values]
                plc_data.register_values = new_registers

                # 更新缓存
                device.cached_register_values = new_registers[:]
                device.cache_timestamp = self.get_clock().now().nanoseconds / 1e9
                device.last_successful_poll = self.get_clock().now().nanoseconds / 1e9

                if stats:
                    stats.successful_reads += 1
                    stats.consecutive_errors = 0
            else:
                device.connected = False
                plc_data.register_values = []
                if stats:
                    stats.failed_reads += 1
                    stats.consecutive_errors += 1
                    stats.last_error_time = time.time()
                    stats.last_error_msg = str(register_result)

            # 数据变更检测
            has_change, changed_details = self._detect_data_change(
                device, plc_data.coil_values, plc_data.register_values)

            if has_change and changed_details:
                change_info = {
                    'device': name,
                    'timestamp': self.get_clock().now().nanoseconds / 1e9,
                    'changes': changed_details
                }
                msg = String()
                msg.data = json.dumps(change_info, ensure_ascii=False, default=str)
                self.plc_data_changed_pub.publish(msg)

                # 更新上次发布值
                if plc_data.coil_values:
                    device.last_published_coils = plc_data.coil_values[:]
                if plc_data.register_values:
                    device.last_published_registers = plc_data.register_values[:]

                # 检查告警条件
                self._check_alarms(name, plc_data)

            if name in self.publishers:
                self.publishers[name].publish(plc_data)

    # ==================== 告警管理 ====================

    def _check_alarms(self, device_name, plc_data):
        """检查PLC数据是否触发告警条件"""
        for alarm_name, alarm_conf in self._alarm_conditions.items():
            if alarm_conf.get('device') != device_name:
                continue

            alarm_type = alarm_conf.get('type', 'register')
            condition = alarm_conf.get('condition', 'above')
            threshold = alarm_conf.get('threshold', 0)
            address = alarm_conf.get('address', 0)

            triggered = False
            current_value = None

            if alarm_type == 'register':
                reg_start = self.devices[device_name].register_read_start
                reg_index = address - reg_start
                if 0 <= reg_index < len(plc_data.register_values):
                    current_value = plc_data.register_values[reg_index]
                    if condition == 'above' and current_value > threshold:
                        triggered = True
                    elif condition == 'below' and current_value < threshold:
                        triggered = True
                    elif condition == 'equal' and current_value == threshold:
                        triggered = True
                    elif condition == 'not_equal' and current_value != threshold:
                        triggered = True
                    elif condition == 'range':
                        low = alarm_conf.get('low', 0)
                        high = alarm_conf.get('high', 0)
                        if current_value < low or current_value > high:
                            triggered = True

            elif alarm_type == 'coil':
                coil_start = self.devices[device_name].coil_read_start
                coil_index = address - coil_start
                if 0 <= coil_index < len(plc_data.coil_values):
                    current_value = plc_data.coil_values[coil_index]
                    expected = alarm_conf.get('expected', 1)
                    if condition == 'equal' and current_value == expected:
                        triggered = True
                    elif condition == 'not_equal' and current_value != expected:
                        triggered = True

            # 更新告警状态
            alarm_state = self._alarm_states.get(alarm_name, {
                'active': False, 'last_trigger_time': 0.0, 'last_clear_time': 0.0
            })

            if triggered and not alarm_state.get('active', False):
                # 新告警触发
                alarm_state['active'] = True
                alarm_state['last_trigger_time'] = time.time()
                self._alarm_states[alarm_name] = alarm_state

                alarm_data = {
                    'alarm_name': alarm_name,
                    'device': device_name,
                    'type': alarm_type,
                    'condition': condition,
                    'address': address,
                    'current_value': current_value,
                    'threshold': threshold,
                    'severity': alarm_conf.get('severity', 'warning'),
                    'message': alarm_conf.get('message', f'告警 {alarm_name} 已触发'),
                    'timestamp': time.time()
                }
                msg = String()
                msg.data = json.dumps(alarm_data, ensure_ascii=False, default=str)
                self.plc_alarm_pub.publish(msg)
                self.get_logger().warn(
                    f'PLC告警: {alarm_name} - 设备 {device_name}, '
                    f'地址 {address}, 当前值 {current_value}, 条件 {condition} {threshold}')

            elif not triggered and alarm_state.get('active', False):
                # 告警清除
                alarm_state['active'] = False
                alarm_state['last_clear_time'] = time.time()
                self._alarm_states[alarm_name] = alarm_state

                clear_data = {
                    'alarm_name': alarm_name,
                    'device': device_name,
                    'action': 'cleared',
                    'timestamp': time.time()
                }
                msg = String()
                msg.data = json.dumps(clear_data, ensure_ascii=False, default=str)
                self.plc_alarm_pub.publish(msg)
                self.get_logger().info(f'PLC告警已清除: {alarm_name}')

    def config_alarm_callback(self, request, response):
        """配置PLC告警条件服务回调"""
        try:
            data = json.loads(request.config_value)
            alarm_name = request.config_key

            if 'device' not in data:
                response.success = False
                response.message = '告警配置缺少 device 字段'
                return response

            self._alarm_conditions[alarm_name] = data
            if alarm_name not in self._alarm_states:
                self._alarm_states[alarm_name] = {
                    'active': False,
                    'last_trigger_time': 0.0,
                    'last_clear_time': 0.0
                }

            response.success = True
            response.message = f'告警条件 {alarm_name} 已配置'
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'JSON解析失败: {e}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def get_alarm_status_callback(self, request, response):
        """获取告警状态服务回调"""
        try:
            alarm_name = request.config_key
            if alarm_name == 'all':
                status = {}
                for name, state in self._alarm_states.items():
                    status[name] = {
                        'active': state.get('active', False),
                        'last_trigger_time': state.get('last_trigger_time', 0),
                        'last_clear_time': state.get('last_clear_time', 0),
                        'condition': self._alarm_conditions.get(name, {})
                    }
                response.config_value = json.dumps(status, ensure_ascii=False, default=str)
            elif alarm_name in self._alarm_states:
                state = self._alarm_states[alarm_name]
                status = {
                    'active': state.get('active', False),
                    'last_trigger_time': state.get('last_trigger_time', 0),
                    'last_clear_time': state.get('last_clear_time', 0),
                    'condition': self._alarm_conditions.get(alarm_name, {})
                }
                response.config_value = json.dumps(status, ensure_ascii=False, default=str)
            else:
                response.config_value = ''
                response.success = False
                response.message = f'告警 {alarm_name} 不存在'
                return response
            response.success = True
            response.message = 'Success'
        except Exception as e:
            response.success = False
            response.config_value = ''
            response.message = str(e)
        return response

    # ==================== 通信统计 ====================

    def _publish_comm_stats(self):
        """发布PLC通信统计数据"""
        stats_report = {}
        for name, stats in self._comm_stats.items():
            stats_report[name] = {
                'read_success_rate': stats.read_success_rate,
                'write_success_rate': stats.write_success_rate,
                'avg_read_latency_ms': stats.avg_read_latency_ms,
                'avg_write_latency_ms': stats.avg_write_latency_ms,
                'last_read_latency_ms': stats.last_read_latency_ms,
                'last_write_latency_ms': stats.last_write_latency_ms,
                'total_reads': stats.total_reads,
                'successful_reads': stats.successful_reads,
                'failed_reads': stats.failed_reads,
                'total_writes': stats.total_writes,
                'successful_writes': stats.successful_writes,
                'failed_writes': stats.failed_writes,
                'consecutive_errors': stats.consecutive_errors,
            }
        msg = String()
        msg.data = json.dumps(stats_report, ensure_ascii=False, default=str)
        self.plc_stats_pub.publish(msg)

    def get_comm_stats_callback(self, request, response):
        """获取通信统计服务回调"""
        try:
            device_name = request.config_key
            if device_name == 'all':
                stats_report = {}
                for name, stats in self._comm_stats.items():
                    stats_report[name] = {
                        'read_success_rate': stats.read_success_rate,
                        'write_success_rate': stats.write_success_rate,
                        'avg_read_latency_ms': stats.avg_read_latency_ms,
                        'avg_write_latency_ms': stats.avg_write_latency_ms,
                        'total_reads': stats.total_reads,
                        'total_writes': stats.total_writes,
                        'consecutive_errors': stats.consecutive_errors,
                    }
                response.config_value = json.dumps(stats_report, ensure_ascii=False, default=str)
            elif device_name in self._comm_stats:
                stats = self._comm_stats[device_name]
                response.config_value = json.dumps({
                    'read_success_rate': stats.read_success_rate,
                    'write_success_rate': stats.write_success_rate,
                    'avg_read_latency_ms': stats.avg_read_latency_ms,
                    'avg_write_latency_ms': stats.avg_write_latency_ms,
                    'total_reads': stats.total_reads,
                    'total_writes': stats.total_writes,
                    'consecutive_errors': stats.consecutive_errors,
                }, ensure_ascii=False, default=str)
            else:
                response.config_value = ''
                response.success = False
                response.message = f'设备 {device_name} 无统计数据'
                return response
            response.success = True
            response.message = 'Success'
        except Exception as e:
            response.success = False
            response.config_value = ''
            response.message = str(e)
        return response

    # ==================== 缓存数据访问 ====================

    def get_cached_data_callback(self, request, response):
        """获取缓存数据服务回调"""
        try:
            device_name = request.config_key
            now = self.get_clock().now().nanoseconds / 1e9

            if device_name == 'all':
                cache_report = {}
                for name, device in self.devices.items():
                    cache_age = now - device.cache_timestamp if device.cache_timestamp > 0 else -1
                    cache_report[name] = {
                        'coil_values': device.cached_coil_values,
                        'register_values': device.cached_register_values,
                        'cache_age_seconds': cache_age,
                        'is_stale': cache_age > self._cache_stale_timeout if cache_age >= 0 else True,
                        'connected': device.connected
                    }
                response.config_value = json.dumps(cache_report, ensure_ascii=False, default=str)
            elif device_name in self.devices:
                device = self.devices[device_name]
                cache_age = now - device.cache_timestamp if device.cache_timestamp > 0 else -1
                response.config_value = json.dumps({
                    'coil_values': device.cached_coil_values,
                    'register_values': device.cached_register_values,
                    'cache_age_seconds': cache_age,
                    'is_stale': cache_age > self._cache_stale_timeout if cache_age >= 0 else True,
                    'connected': device.connected
                }, ensure_ascii=False, default=str)
            else:
                response.config_value = ''
                response.success = False
                response.message = f'设备 {device_name} 不存在'
                return response
            response.success = True
            response.message = 'Success'
        except Exception as e:
            response.success = False
            response.config_value = ''
            response.message = str(e)
        return response

    # ==================== 批量读写优化 ====================

    def batch_read_callback(self, request, response):
        """批量读取寄存器，合并多个地址范围到最少Modbus请求"""
        try:
            device = self._find_device(request.device_name, request.ip_address)
            if device is None:
                response.success = False
                response.message = f'设备未找到: {request.device_name or request.ip_address}'
                response.values = []
                return response

            if not device.connected:
                self._reconnect_device(device)
                if not device.connected:
                    response.success = False
                    response.message = 'PLC未连接'
                    response.values = []
                    return response

            # 合并连续或重叠的地址范围
            start_addr = request.start_address
            quantity = request.quantity

            # 如果请求超过最大批量大小，分批读取
            all_values = []
            current_addr = start_addr
            remaining = quantity

            while remaining > 0:
                batch_size = min(remaining, self._batch_read_max)
                read_start_time = time.time()

                read_result = device.client.read_holding_registers(
                    address=current_addr,
                    count=batch_size,
                    slave=device.slave_id,
                )

                read_latency = (time.time() - read_start_time) * 1000.0
                stats = self._comm_stats.get(device.name)
                if stats:
                    stats.total_reads += 1
                    stats.total_read_latency_ms += read_latency
                    stats.last_read_latency_ms = read_latency

                if not read_result.isError():
                    all_values.extend([int(v) for v in read_result.registers[:batch_size]])
                    if stats:
                        stats.successful_reads += 1
                        stats.consecutive_errors = 0
                else:
                    if stats:
                        stats.failed_reads += 1
                        stats.consecutive_errors += 1
                        stats.last_error_time = time.time()
                        stats.last_error_msg = str(read_result)
                    device.connected = False
                    response.success = False
                    response.message = f'批量读取失败: {read_result}'
                    response.values = all_values
                    return response

                current_addr += batch_size
                remaining -= batch_size

            response.success = True
            response.message = f'批量读取成功: {quantity} 个寄存器'
            response.values = all_values
            device.last_successful_poll = self.get_clock().now().nanoseconds / 1e9

        except Exception as e:
            response.success = False
            response.message = f'批量读取异常: {str(e)}'
            response.values = []
        return response

    def batch_write_callback(self, request, response):
        """批量写入寄存器"""
        try:
            device = self._find_device(request.device_name, request.ip_address)
            if device is None:
                response.success = False
                response.message = f'设备未找到: {request.device_name or request.ip_address}'
                return response

            if not device.connected:
                self._reconnect_device(device)
                if not device.connected:
                    response.success = False
                    response.message = 'PLC未连接'
                    return response

            write_start_time = time.time()

            # 分批写入
            values = request.values
            current_addr = request.start_address
            remaining = len(values)
            offset = 0

            while remaining > 0:
                batch_size = min(remaining, self._batch_read_max)
                batch_values = values[offset:offset + batch_size]

                write_result = device.client.write_registers(
                    address=current_addr,
                    values=batch_values,
                    slave=device.slave_id,
                )

                write_latency = (time.time() - write_start_time) * 1000.0
                stats = self._comm_stats.get(device.name)
                if stats:
                    stats.total_writes += 1
                    stats.total_write_latency_ms += write_latency
                    stats.last_write_latency_ms = write_latency

                if not write_result.isError():
                    if stats:
                        stats.successful_writes += 1
                        stats.consecutive_errors = 0
                else:
                    if stats:
                        stats.failed_writes += 1
                        stats.consecutive_errors += 1
                        stats.last_error_time = time.time()
                        stats.last_error_msg = str(write_result)
                    device.connected = False
                    response.success = False
                    response.message = f'批量写入失败: {write_result}'
                    return response

                current_addr += batch_size
                offset += batch_size
                remaining -= batch_size

            response.success = True
            response.message = f'批量写入成功: {len(values)} 个寄存器'
            device.last_successful_poll = self.get_clock().now().nanoseconds / 1e9

        except Exception as e:
            response.success = False
            response.message = f'批量写入异常: {str(e)}'
        return response

    # ==================== 数据日志 ====================

    def _data_log_callback(self):
        """定期记录关键寄存器值到日志文件"""
        try:
            now = time.time()
            if now - self._last_data_log_time < self._data_log_interval:
                return
            self._last_data_log_time = now

            # 确保日志目录存在
            log_dir = os.path.dirname(self._data_log_path)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir, exist_ok=True)

            is_new_file = not os.path.exists(self._data_log_path)

            with open(self._data_log_path, 'a') as f:
                if is_new_file:
                    f.write('timestamp,device,connected,coil_values,register_values\n')

                for name, device in self.devices.items():
                    coil_str = ';'.join(str(v) for v in device.cached_coil_values)
                    reg_str = ';'.join(str(v) for v in device.cached_register_values)
                    f.write(f'{now:.3f},{name},{device.connected},{coil_str},{reg_str}\n')

        except Exception as e:
            self.get_logger().error(f'数据日志记录失败: {e}')

    # ==================== 从控循环 ====================

    def _slave_control_loop(self):
        for name, device in self.devices.items():
            if device.is_master and device.connected and device.slave_control_map:
                self._execute_slave_control(device)

    def _execute_slave_control(self, device):
        try:
            pass
        except Exception as e:
            self.get_logger().debug(f'从控 {device.name}: {e}')

    # ==================== 设备查找 ====================

    def _find_device(self, device_name, ip_address):
        if device_name and device_name in self.devices:
            return self.devices[device_name]
        if ip_address:
            for dev in self.devices.values():
                if dev.ip == ip_address:
                    return dev
        return None

    # ==================== 原有服务回调（增强统计） ====================

    def read_plc_callback(self, request, response):
        device = self._find_device(request.device_name, request.ip_address)

        if device is None:
            response.success = False
            response.message = f'设备未找到: {request.device_name or request.ip_address}'
            response.values = []
            return response

        if not device.connected:
            self._reconnect_device(device)
            if not device.connected:
                response.success = False
                response.message = 'PLC未连接'
                response.values = []
                return response

        read_start_time = time.time()
        read_result = device.client.read_holding_registers(
            address=request.start_address,
            count=request.quantity,
            slave=device.slave_id,
        )
        read_latency = (time.time() - read_start_time) * 1000.0

        stats = self._comm_stats.get(device.name)
        if stats:
            stats.total_reads += 1
            stats.total_read_latency_ms += read_latency
            stats.last_read_latency_ms = read_latency

        if not read_result.isError():
            response.success = True
            response.message = '读取成功'
            response.values = [int(v) for v in read_result.registers[:request.quantity]]
            device.last_successful_poll = self.get_clock().now().nanoseconds / 1e9
            if stats:
                stats.successful_reads += 1
                stats.consecutive_errors = 0
        else:
            response.success = False
            response.message = f'读取失败: {read_result}'
            response.values = []
            device.connected = False
            if stats:
                stats.failed_reads += 1
                stats.consecutive_errors += 1
                stats.last_error_time = time.time()
                stats.last_error_msg = str(read_result)

        return response

    def write_plc_callback(self, request, response):
        device = self._find_device(request.device_name, request.ip_address)

        if device is None:
            response.success = False
            response.message = f'设备未找到: {request.device_name or request.ip_address}'
            return response

        if not device.connected:
            self._reconnect_device(device)
            if not device.connected:
                response.success = False
                response.message = 'PLC未连接'
                return response

        write_start_time = time.time()
        write_result = device.client.write_registers(
            address=request.start_address,
            values=request.values,
            slave=device.slave_id,
        )
        write_latency = (time.time() - write_start_time) * 1000.0

        stats = self._comm_stats.get(device.name)
        if stats:
            stats.total_writes += 1
            stats.total_write_latency_ms += write_latency
            stats.last_write_latency_ms = write_latency

        if not write_result.isError():
            response.success = True
            response.message = '写入成功'
            device.last_successful_poll = self.get_clock().now().nanoseconds / 1e9
            if stats:
                stats.successful_writes += 1
                stats.consecutive_errors = 0
        else:
            response.success = False
            response.message = f'写入失败: {write_result}'
            device.connected = False
            if stats:
                stats.failed_writes += 1
                stats.consecutive_errors += 1
                stats.last_error_time = time.time()
                stats.last_error_msg = str(write_result)

        return response

    def set_config_callback(self, request, response):
        try:
            with self._config_lock:
                config_data = request.config
                if not config_data:
                    response.success = False
                    response.message = '未提供配置'
                    return response

                device_names = getattr(config_data, 'device_names', [])
                ips = getattr(config_data, 'ips', [])
                ports = getattr(config_data, 'ports', [])
                slave_ids = getattr(config_data, 'slave_ids', [])
                coil_read_starts = getattr(config_data, 'coil_read_starts', [])
                coil_read_counts = getattr(config_data, 'coil_read_counts', [])
                register_read_starts = getattr(config_data, 'register_read_starts', [])
                register_read_counts = getattr(config_data, 'register_read_counts', [])
                is_masters = getattr(config_data, 'is_masters', [])

                if len(device_names) != len(ips):
                    response.success = False
                    response.message = '设备名称和IP数量不匹配'
                    return response

                for name in list(self.devices.keys()):
                    if name not in device_names:
                        self.remove_plc(name, skip_log=True)

                for i, name in enumerate(device_names):
                    self.add_plc(
                        device_name=name,
                        ip=ips[i] if i < len(ips) else '127.0.0.1',
                        port=ports[i] if i < len(ports) else 502,
                        slave_id=slave_ids[i] if i < len(slave_ids) else 1,
                        coil_read_start=coil_read_starts[i] if i < len(coil_read_starts) else 0,
                        coil_read_count=coil_read_counts[i] if i < len(coil_read_counts) else 16,
                        register_read_start=register_read_starts[i] if i < len(register_read_starts) else 0,
                        register_read_count=register_read_counts[i] if i < len(register_read_counts) else 16,
                        is_master=is_masters[i] if i < len(is_masters) else True,
                    )

                response.success = True
                response.message = f'配置已应用, {len(device_names)} 个设备'
        except Exception as e:
            response.success = False
            response.message = f'错误: {str(e)}'
        return response

    def get_config_callback(self, request, response):
        try:
            response.config = self._current_config
            response.success = True
            response.message = '配置已获取'
        except Exception as e:
            response.success = False
            response.message = f'错误: {str(e)}'
        return response

    def save_config_callback(self, request, response):
        try:
            save_path = request.path or '/tmp/plc_config.yaml'
            config_dict = {'plc_manager': {'ros__parameters': {'devices': {}}}}
            for name, dev in self.devices.items():
                config_dict['plc_manager']['ros__parameters']['devices'][name] = {
                    'ip': dev.ip,
                    'port': dev.port,
                    'slave_id': dev.slave_id,
                    'coil_read_start': dev.coil_read_start,
                    'coil_read_count': dev.coil_read_count,
                    'register_read_start': dev.register_read_start,
                    'register_read_count': dev.register_read_count,
                    'timeout': dev.timeout,
                    'is_master': dev.is_master,
                }
            # 保存寄存器映射
            if self._register_maps:
                config_dict['plc_manager']['ros__parameters']['register_maps'] = self._register_maps
            # 保存告警条件
            if self._alarm_conditions:
                config_dict['plc_manager']['ros__parameters']['alarms'] = self._alarm_conditions

            with open(save_path, 'w') as f:
                yaml.dump(config_dict, f)
            response.success = True
            response.message = f'配置已保存到 {save_path}'
        except Exception as e:
            response.success = False
            response.message = f'错误: {str(e)}'
        return response

    def load_config_callback(self, request, response):
        try:
            load_path = request.path or '/tmp/plc_config.yaml'
            self._load_config(load_path)
            response.success = True
            response.message = f'配置已从 {load_path} 加载'
        except Exception as e:
            response.success = False
            response.message = f'错误: {str(e)}'
        return response

    # ==================== 清理 ====================

    def destroy(self):
        for name, device in self.devices.items():
            if device.client:
                device.client.close()
            device.connected = False
        self.devices.clear()
        self.publishers.clear()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlcManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
