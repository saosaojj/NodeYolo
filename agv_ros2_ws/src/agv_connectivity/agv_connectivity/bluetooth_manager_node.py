import subprocess
import re
import json
import time
from collections import deque
from datetime import datetime

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy

from agv_interfaces.msg import BluetoothDevice
from agv_interfaces.srv import ConnectBluetooth
from std_msgs.msg import String
from std_srvs.srv import Trigger


# 增强功能3: 设备类型识别映射表
DEVICE_TYPE_PATTERNS = {
    'sensor': ['sensor', 'temp', 'humidity', 'pressure', 'imu', 'accel', 'gyro', 'environmental'],
    'remote': ['remote', 'controller', 'gamepad', 'joystick', 'rc-'],
    'audio': ['speaker', 'headphone', 'audio', 'headset', 'earbuds'],
    'input': ['keyboard', 'mouse', 'trackpad', 'touchpad'],
    'health': ['heart', 'pulse', 'blood', 'fitness', 'health', 'watch', 'band'],
    'beacon': ['beacon', 'tag', 'tracker', 'ibeacon'],
    'phone': ['phone', 'mobile', 'iphone', 'galaxy'],
    'display': ['display', 'tv', 'monitor', 'screen', 'projector'],
}


class BluetoothManagerNode(Node):

    def __init__(self):
        super().__init__('bluetooth_manager_node')

        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('adapter', 'hci0')
        self.declare_parameter('auto_connect_paired', False)

        # 增强功能2: 蓝牙信号监控参数
        self.declare_parameter('rssi_check_interval', 5.0)
        self.declare_parameter('rssi_disconnect_threshold', -90)
        self.declare_parameter('rssi_warning_threshold', -75)
        self.declare_parameter('rssi_history_size', 100)

        # 增强功能4: 连接统计参数
        self.declare_parameter('stats_publish_rate', 30.0)

        self._discovered_devices = {}
        self._connected_address = ''

        # 增强功能1: BLE服务发现 - 已发现的服务列表
        self._discovered_services = {}

        # 增强功能2: 蓝牙信号监控 - RSSI跟踪
        self._rssi_check_interval = self.get_parameter('rssi_check_interval').value
        self._rssi_disconnect_threshold = self.get_parameter('rssi_disconnect_threshold').value
        self._rssi_warning_threshold = self.get_parameter('rssi_warning_threshold').value
        self._rssi_history_size = self.get_parameter('rssi_history_size').value
        self._rssi_history = deque(maxlen=self._rssi_history_size)
        self._rssi_timestamp_history = deque(maxlen=self._rssi_history_size)
        self._last_rssi_check = 0.0
        self._signal_degradation_detected = False

        # 增强功能3: 设备能力检测 - 已识别的设备类型
        self._device_capabilities = {}

        # 增强功能4: 连接统计
        self._stats_publish_rate = self.get_parameter('stats_publish_rate').value
        self._connection_stats = {
            'total_uptime_seconds': 0.0,
            'total_downtime_seconds': 0.0,
            'disconnect_count': 0,
            'reconnect_count': 0,
            'connection_start_time': None,
            'disconnection_start_time': None,
            'avg_rssi': 0.0,
            'min_rssi': 0,
            'max_rssi': 0,
            'last_connected_address': '',
            'last_connected_name': '',
        }
        self._last_stats_time = time.time()

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._bluetooth_devices_pub = self.create_publisher(
            BluetoothDevice, 'bluetooth_devices', qos_profile)

        self._bluetooth_status_pub = self.create_publisher(
            String, 'bluetooth_status', qos_profile)

        # 增强功能1: BLE服务发现发布
        self._ble_services_pub = self.create_publisher(
            String, 'bluetooth_services', qos_profile)

        # 增强功能2: 信号监控发布
        self._signal_monitor_pub = self.create_publisher(
            String, 'bluetooth_signal', qos_profile)

        # 增强功能3: 设备能力发布
        self._device_capability_pub = self.create_publisher(
            String, 'bluetooth_device_capabilities', qos_profile)

        # 增强功能4: 连接统计发布
        self._bt_stats_pub = self.create_publisher(
            String, 'bluetooth_stats', qos_profile)

        self._connect_bluetooth_srv = self.create_service(
            ConnectBluetooth, 'connect_bluetooth', self.connect_callback)

        self._disconnect_bluetooth_srv = self.create_service(
            Trigger, 'disconnect_bluetooth', self.disconnect_callback)

        self._scan_bluetooth_srv = self.create_service(
            Trigger, 'scan_bluetooth', self.scan_callback)

        # 增强功能1: 服务发现服务
        self._discover_services_srv = self.create_service(
            Trigger, 'discover_ble_services', self.discover_services_callback)

        # 增强功能4: 获取连接统计服务
        self._get_stats_srv = self.create_service(
            Trigger, 'get_bluetooth_stats', self.get_stats_callback)

        scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / scan_rate, self.timer_callback)

        # 增强功能2: RSSI监控定时器
        self._rssi_timer = self.create_timer(self._rssi_check_interval, self.monitor_signal)

        # 增强功能4: 统计发布定时器
        self._stats_timer = self.create_timer(
            1.0 / self._stats_publish_rate, self.publish_stats_callback)

        self._bluetoothctl_available = self._check_bluetoothctl()

        if not self._bluetoothctl_available:
            self.get_logger().warn('bluetoothctl is not available. Bluetooth management will be limited.')

    def _check_bluetoothctl(self):
        try:
            subprocess.run(
                ['bluetoothctl', '--version'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5)
            return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def _run_bluetoothctl(self, commands, timeout=15):
        if not self._bluetoothctl_available:
            return None, 'bluetoothctl is not available'
        try:
            cmd_str = '\n'.join(commands) + '\nquit\n'
            result = subprocess.run(
                ['bluetoothctl'],
                input=cmd_str,
                capture_output=True,
                text=True,
                timeout=timeout)
            return result.stdout.strip(), result.stderr.strip()
        except subprocess.TimeoutExpired:
            return None, 'bluetoothctl command timed out'
        except FileNotFoundError:
            self._bluetoothctl_available = False
            return None, 'bluetoothctl is not available'
        except Exception as e:
            return None, str(e)

    def timer_callback(self):
        if not self._bluetoothctl_available:
            status_msg = String()
            status_msg.data = 'unavailable'
            self._bluetooth_status_pub.publish(status_msg)
            return

        adapter = self.get_parameter('adapter').get_parameter_value().string_value

        stdout, _ = self._run_bluetoothctl(['show', adapter], timeout=10)

        status_msg = String()
        if stdout:
            if 'Powered: yes' in stdout:
                if 'Discovering: yes' in stdout:
                    status_msg.data = 'scanning'
                else:
                    status_msg.data = 'active'
            else:
                status_msg.data = 'powered_off'
        else:
            status_msg.data = 'unknown'

        self._bluetooth_status_pub.publish(status_msg)

        if self.get_parameter('auto_connect_paired').get_parameter_value().bool_value:
            self._auto_connect_paired()

        # 增强功能4: 更新连接统计
        now = time.time()
        dt = now - self._last_stats_time
        self._last_stats_time = now

        for address, device_info in self._discovered_devices.items():
            device_msg = BluetoothDevice()
            device_msg.name = device_info.get('name', '')
            device_msg.address = address
            device_msg.rssi = device_info.get('rssi', 0)
            device_msg.connected = (address == self._connected_address)
            # 增强功能3: 设置检测到的设备类型
            device_msg.device_type = self._device_capabilities.get(address, {}).get('type', device_info.get('type', ''))
            self._bluetooth_devices_pub.publish(device_msg)

        # 增强功能4: 更新在线/离线时间
        if self._connected_address:
            self._connection_stats['total_uptime_seconds'] += dt
            if self._connection_stats['connection_start_time'] is None:
                self._connection_stats['connection_start_time'] = now
                self._connection_stats['last_connected_address'] = self._connected_address
                if self._connected_address in self._discovered_devices:
                    self._connection_stats['last_connected_name'] = self._discovered_devices[self._connected_address].get('name', '')
            self._connection_stats['disconnection_start_time'] = None
        else:
            self._connection_stats['total_downtime_seconds'] += dt
            if self._connection_stats['disconnection_start_time'] is None:
                self._connection_stats['disconnection_start_time'] = now
                if self._connection_stats['connection_start_time'] is not None:
                    self._connection_stats['disconnect_count'] += 1
            self._connection_stats['connection_start_time'] = None

    # 增强功能2: 蓝牙信号监控 - 跟踪RSSI，提前检测断连
    def monitor_signal(self):
        if not self._connected_address:
            return

        stdout, _ = self._run_bluetoothctl(
            [f'info {self._connected_address}'], timeout=10)

        if stdout:
            rssi = 0
            for line in stdout.split('\n'):
                if 'RSSI' in line or 'rssi' in line.lower():
                    match = re.search(r'(-?\d+)', line)
                    if match:
                        rssi = int(match.group(1))
                        break

            if rssi != 0:
                now = time.time()
                self._rssi_history.append(rssi)
                self._rssi_timestamp_history.append(now)

                # 更新设备信息中的RSSI
                if self._connected_address in self._discovered_devices:
                    self._discovered_devices[self._connected_address]['rssi'] = rssi

                # 更新统计
                if len(self._rssi_history) > 0:
                    self._connection_stats['avg_rssi'] = round(
                        sum(self._rssi_history) / len(self._rssi_history), 1)
                    self._connection_stats['min_rssi'] = min(self._rssi_history)
                    self._connection_stats['max_rssi'] = max(self._rssi_history)

                # 检测信号退化
                self._check_signal_degradation(rssi)

                # 发布信号监控数据
                signal_data = {
                    'address': self._connected_address,
                    'rssi': rssi,
                    'avg_rssi': self._connection_stats['avg_rssi'],
                    'min_rssi': self._connection_stats['min_rssi'],
                    'max_rssi': self._connection_stats['max_rssi'],
                    'degradation_detected': self._signal_degradation_detected,
                    'timestamp': datetime.now().isoformat(),
                }
                signal_msg = String()
                signal_msg.data = json.dumps(signal_data, ensure_ascii=False)
                self._signal_monitor_pub.publish(signal_msg)

    # 增强功能2: 信号退化检测
    def _check_signal_degradation(self, current_rssi):
        if len(self._rssi_history) < 10:
            return

        recent_rssi = list(self._rssi_history)[-5:]
        older_rssi = list(self._rssi_history)[-10:-5]

        if len(older_rssi) < 5:
            return

        recent_avg = sum(recent_rssi) / len(recent_rssi)
        older_avg = sum(older_rssi) / len(older_rssi)
        degradation = older_avg - recent_avg

        if degradation > 10:
            if not self._signal_degradation_detected:
                self._signal_degradation_detected = True
                self.get_logger().warn(
                    f'检测到蓝牙信号退化: RSSI从 {older_avg:.0f} 降至 {recent_avg:.0f} (下降 {degradation:.0f}dBm)')
        else:
            self._signal_degradation_detected = False

        # 信号极弱时预警
        if current_rssi <= self._rssi_warning_threshold and current_rssi > self._rssi_disconnect_threshold:
            self.get_logger().warn(f'蓝牙信号弱: RSSI={current_rssi}dBm (阈值: {self._rssi_warning_threshold}dBm)')

        if current_rssi <= self._rssi_disconnect_threshold:
            self.get_logger().error(
                f'蓝牙信号极弱，即将断连: RSSI={current_rssi}dBm (断连阈值: {self._rssi_disconnect_threshold}dBm)')

    # 增强功能1: BLE服务发现
    def discover_services_callback(self, request, response):
        if not self._bluetoothctl_available:
            response.success = False
            response.message = 'bluetoothctl is not available'
            return response

        if not self._connected_address:
            response.success = False
            response.message = 'No device connected'
            return response

        self.get_logger().info(f'发现BLE服务: {self._connected_address}')

        stdout, _ = self._run_bluetoothctl(
            [f'info {self._connected_address}'], timeout=15)

        services = []
        if stdout:
            current_service = None
            for line in stdout.split('\n'):
                line_stripped = line.strip()
                if 'Service' in line_stripped and 'UUID' in line_stripped:
                    if current_service:
                        services.append(current_service)
                    uuid_match = re.search(r'UUID:\s*([0-9a-fA-F-]+)', line_stripped)
                    current_service = {
                        'uuid': uuid_match.group(1) if uuid_match else 'unknown',
                        'characteristics': [],
                    }
                elif 'Characteristic' in line_stripped and 'UUID' in line_stripped:
                    if current_service:
                        char_match = re.search(r'UUID:\s*([0-9a-fA-F-]+)', line_stripped)
                        current_service['characteristics'].append(
                            char_match.group(1) if char_match else 'unknown')

            if current_service:
                services.append(current_service)

        self._discovered_services[self._connected_address] = services

        # 发布服务发现结果
        services_data = {
            'address': self._connected_address,
            'services': services,
            'service_count': len(services),
            'timestamp': datetime.now().isoformat(),
        }
        services_msg = String()
        services_msg.data = json.dumps(services_data, ensure_ascii=False)
        self._ble_services_pub.publish(services_msg)

        # 增强功能3: 基于服务推断设备能力
        self._detect_device_capability(self._connected_address, services)

        response.success = True
        response.message = f'发现 {len(services)} 个BLE服务'
        return response

    # 增强功能3: 设备能力检测 - 识别设备类型
    def _detect_device_capability(self, address, services=None):
        device_name = ''
        if address in self._discovered_devices:
            device_name = self._discovered_devices[address].get('name', '').lower()

        detected_type = 'unknown'
        detected_capabilities = []

        # 基于设备名称识别
        for dev_type, patterns in DEVICE_TYPE_PATTERNS.items():
            for pattern in patterns:
                if pattern in device_name:
                    detected_type = dev_type
                    break
            if detected_type != 'unknown':
                break

        # 基于BLE服务UUID识别
        known_service_uuids = {
            '180d': 'heart_rate',
            '181a': 'environmental_sensing',
            '180f': 'battery',
            '180a': 'device_information',
            '181c': 'user_data',
            '1809': 'health_thermometer',
            '1816': 'cycling_speed_cadence',
            '1808': 'glucose',
            '1818': 'cycling_power',
            '1800': 'generic_access',
            '1801': 'generic_attribute',
            '1802': 'immediate_alert',
            '1803': 'link_loss',
            '1804': 'tx_power',
            '1805': 'current_time',
            '1807': 'blood_pressure',
            '180e': 'phone_alert',
            '1810': 'blood_pressure',
            '1811': 'alert_notification',
            '1812': 'human_interface_device',
            '1814': 'running_speed_cadence',
            '181b': 'body_composition',
            '181e': 'bond_management',
            '181f': 'continuous_glucose_monitoring',
            '1820': 'internet_protocol_support',
            '1821': 'indoor_positioning',
            '1822': 'pulse_oximeter',
        }

        if services:
            for service in services:
                uuid = service.get('uuid', '').lower()
                short_uuid = uuid[:4] if len(uuid.replace('-', '')) <= 8 else uuid[4:8]
                if short_uuid in known_service_uuids:
                    cap = known_service_uuids[short_uuid]
                    detected_capabilities.append(cap)

                    if cap in ['heart_rate', 'health_thermometer', 'blood_pressure', 'glucose',
                               'cycling_speed_cadence', 'cycling_power', 'body_composition',
                               'pulse_oximeter', 'continuous_glucose_monitoring']:
                        if detected_type == 'unknown':
                            detected_type = 'health'
                    elif cap == 'environmental_sensing':
                        if detected_type == 'unknown':
                            detected_type = 'sensor'
                    elif cap == 'human_interface_device':
                        if detected_type == 'unknown':
                            detected_type = 'input'

        # 更新设备能力信息
        capability_info = {
            'type': detected_type,
            'capabilities': detected_capabilities,
            'name': device_name,
            'detected_at': datetime.now().isoformat(),
        }
        self._device_capabilities[address] = capability_info

        # 发布设备能力信息
        cap_msg = String()
        cap_msg.data = json.dumps({
            'address': address,
            'capability': capability_info,
        }, ensure_ascii=False)
        self._device_capability_pub.publish(cap_msg)

        self.get_logger().info(
            f'设备能力检测: {address} -> 类型: {detected_type}, 能力: {detected_capabilities}')

    def _auto_connect_paired(self):
        if self._connected_address:
            return

        stdout, _ = self._run_bluetoothctl(['paired-devices'], timeout=10)
        if not stdout:
            return

        for line in stdout.split('\n'):
            match = re.match(r'Device\s+([0-9A-Fa-f:]+)\s+(.*)', line)
            if match:
                address = match.group(1)
                connect_stdout, _ = self._run_bluetoothctl(
                    ['connect', address], timeout=15)
                if connect_stdout and 'successful' in connect_stdout.lower():
                    self._connected_address = address
                    self.get_logger().info(f'Auto-connected to paired device: {address}')
                    # 增强功能3: 检测设备能力
                    self._detect_device_capability(address)
                    # 增强功能4: 记录重连
                    self._connection_stats['reconnect_count'] += 1
                    return

    def scan_callback(self, request, response):
        if not self._bluetoothctl_available:
            response.success = False
            response.message = 'bluetoothctl is not available'
            return response

        self.get_logger().info('Scanning for Bluetooth devices')

        stdout, stderr = self._run_bluetoothctl(
            ['power on', 'scan on'], timeout=20)

        if stdout is None:
            response.success = False
            response.message = f'Scan failed: {stderr}'
            return response

        self._discovered_devices.clear()

        devices_stdout, _ = self._run_bluetoothctl(['devices'], timeout=10)
        if devices_stdout:
            for line in devices_stdout.split('\n'):
                match = re.match(r'Device\s+([0-9A-Fa-f:]+)\s+(.*)', line)
                if match:
                    address = match.group(1)
                    name = match.group(2).strip()
                    self._discovered_devices[address] = {
                        'name': name,
                        'rssi': 0,
                        'type': '',
                    }
                    # 增强功能3: 扫描时即检测设备类型
                    self._detect_device_capability(address)

        info_stdout, _ = self._run_bluetoothctl(['scan off'], timeout=10)

        response.success = True
        response.message = f'Bluetooth scan completed, found {len(self._discovered_devices)} devices'

        return response

    def connect_callback(self, request, response):
        if not self._bluetoothctl_available:
            response.success = False
            response.message = 'bluetoothctl is not available'
            return response

        address = request.address
        self.get_logger().info(f'Connecting to Bluetooth device: {address}')

        commands = ['power on', f'connect {address}']
        stdout, stderr = self._run_bluetoothctl(commands, timeout=20)

        if stdout and 'successful' in stdout.lower():
            response.success = True
            response.message = f'Successfully connected to {address}'
            self._connected_address = address

            if address not in self._discovered_devices:
                self._discovered_devices[address] = {
                    'name': request.profile or '',
                    'rssi': 0,
                    'type': '',
                }

            # 增强功能3: 连接后检测设备能力
            self._detect_device_capability(address)

            # 增强功能4: 记录连接开始
            self._connection_stats['connection_start_time'] = time.time()
            self._connection_stats['last_connected_address'] = address
            if address in self._discovered_devices:
                self._connection_stats['last_connected_name'] = self._discovered_devices[address].get('name', '')
        else:
            response.success = False
            response.message = f'Failed to connect to {address}: {stderr or "unknown error"}'

        return response

    def disconnect_callback(self, request, response):
        if not self._bluetoothctl_available:
            response.success = False
            response.message = 'bluetoothctl is not available'
            return response

        if not self._connected_address:
            response.success = True
            response.message = 'No device currently connected'
            return response

        self.get_logger().info(f'Disconnecting from: {self._connected_address}')

        stdout, stderr = self._run_bluetoothctl(
            [f'disconnect {self._connected_address}'], timeout=15)

        if stdout and 'successful' in stdout.lower():
            response.success = True
            response.message = f'Successfully disconnected from {self._connected_address}'
            # 增强功能4: 记录断连
            self._connection_stats['disconnect_count'] += 1
            self._connection_stats['disconnection_start_time'] = time.time()
            self._connected_address = ''
        else:
            response.success = False
            response.message = f'Failed to disconnect: {stderr or "unknown error"}'

        return response

    # 增强功能4: 获取连接统计回调
    def get_stats_callback(self, request, response):
        stats = self._compute_stats()
        response.success = True
        response.message = json.dumps(stats, ensure_ascii=False)
        return response

    # 增强功能4: 计算连接统计
    def _compute_stats(self):
        uptime = self._connection_stats['total_uptime_seconds']
        downtime = self._connection_stats['total_downtime_seconds']
        total = uptime + downtime
        availability = (uptime / total * 100.0) if total > 0 else 0.0

        return {
            'uptime_seconds': round(uptime, 1),
            'downtime_seconds': round(downtime, 1),
            'availability_percent': round(availability, 2),
            'disconnect_count': self._connection_stats['disconnect_count'],
            'reconnect_count': self._connection_stats['reconnect_count'],
            'avg_rssi': self._connection_stats['avg_rssi'],
            'min_rssi': self._connection_stats['min_rssi'],
            'max_rssi': self._connection_stats['max_rssi'],
            'connected_address': self._connected_address,
            'last_connected_address': self._connection_stats['last_connected_address'],
            'last_connected_name': self._connection_stats['last_connected_name'],
            'signal_degradation_detected': self._signal_degradation_detected,
            'discovered_services_count': sum(
                len(s) for s in self._discovered_services.values()),
            'device_capabilities_count': len(self._device_capabilities),
        }

    # 增强功能4: 发布连接统计
    def publish_stats_callback(self):
        stats = self._compute_stats()
        stats_msg = String()
        stats_msg.data = json.dumps(stats, ensure_ascii=False)
        self._bt_stats_pub.publish(stats_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BluetoothManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
