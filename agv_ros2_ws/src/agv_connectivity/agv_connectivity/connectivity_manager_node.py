import json
import time
import subprocess
from collections import deque
from datetime import datetime

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy

from agv_interfaces.msg import WiFiStatus, BluetoothDevice
from std_msgs.msg import String


class ConnectivityManagerNode(Node):

    def __init__(self):
        super().__init__('connectivity_manager_node')

        # 声明参数
        self.declare_parameter('summary_rate', 2.0)
        self.declare_parameter('diagnostics_rate', 10.0)
        self.declare_parameter('ping_target', '8.8.8.8')
        self.declare_parameter('ping_timeout', 3)
        self.declare_parameter('bandwidth_test_duration', 2)
        self.declare_parameter('wifi_priority_threshold', 30)
        self.declare_parameter('bluetooth_priority_threshold', -70)
        self.declare_parameter('device_whitelist', [])
        self.declare_parameter('device_blacklist', [])
        self.declare_parameter('max_event_log_size', 200)
        self.declare_parameter('topology_history_size', 100)

        # 参数读取
        self._summary_rate = self.get_parameter('summary_rate').value
        self._diagnostics_rate = self.get_parameter('diagnostics_rate').value
        self._ping_target = self.get_parameter('ping_target').value
        self._ping_timeout = self.get_parameter('ping_timeout').value
        self._bandwidth_test_duration = self.get_parameter('bandwidth_test_duration').value
        self._wifi_priority_threshold = self.get_parameter('wifi_priority_threshold').value
        self._bluetooth_priority_threshold = self.get_parameter('bluetooth_priority_threshold').value
        self._max_event_log_size = self.get_parameter('max_event_log_size').value
        self._topology_history_size = self.get_parameter('topology_history_size').value

        # 增强功能1: 网络拓扑监控 - 跟踪所有连接设备及其信号质量
        self._topology = {
            'wifi_devices': {},
            'bluetooth_devices': {},
            'last_updated': 0.0,
        }
        self._topology_history = deque(maxlen=self._topology_history_size)

        # 增强功能2: 连接优先级管理 - WiFi优先于蓝牙，基于质量自动切换
        self._active_connection_type = 'none'
        self._active_connection_info = {}
        self._connection_priority = ['wifi', 'bluetooth']

        # 增强功能3: 网络诊断 - 定期ping测试、延迟测量、带宽估算
        self._diagnostics = {
            'ping_latency_ms': -1.0,
            'ping_success_rate': 0.0,
            'bandwidth_mbps': 0.0,
            'last_ping_time': 0.0,
            'last_bandwidth_time': 0.0,
            'ping_history': deque(maxlen=50),
        }

        # 增强功能4: 连接事件日志 - 记录所有连接/断开事件及时间戳
        self._event_log = deque(maxlen=self._max_event_log_size)

        # 增强功能5: 设备白名单/黑名单 - 蓝牙连接安全功能
        self._device_whitelist = self.get_parameter('device_whitelist').value
        self._device_blacklist = self.get_parameter('device_blacklist').value
        self._whitelist_enabled = len(self._device_whitelist) > 0
        self._blacklist_enabled = len(self._device_blacklist) > 0

        # 当前WiFi和蓝牙状态缓存
        self._wifi_status = {
            'connected': False,
            'ssid': '',
            'ip_address': '',
            'signal_strength': 0,
            'mac_address': '',
        }
        self._bluetooth_status = {
            'connected': False,
            'address': '',
            'name': '',
            'rssi': 0,
            'device_type': '',
        }

        # 诊断计数器
        self._ping_count = 0
        self._ping_success_count = 0

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 增强功能6: 发布连接状态摘要到 'connectivity_summary' 话题
        self._connectivity_summary_pub = self.create_publisher(
            String, 'connectivity_summary', qos_profile)

        # 订阅WiFi和蓝牙状态
        self._wifi_status_sub = self.create_subscription(
            WiFiStatus, 'wifi_status', self.wifi_status_callback, qos_profile)

        self._bluetooth_status_sub = self.create_subscription(
            BluetoothDevice, 'bluetooth_devices', self.bluetooth_status_callback, qos_profile)

        self._bluetooth_state_sub = self.create_subscription(
            String, 'bluetooth_status', self.bluetooth_state_callback, qos_profile)

        # 定时器
        self._summary_timer = self.create_timer(
            1.0 / self._summary_rate, self.publish_connectivity_summary)
        self._diagnostics_timer = self.create_timer(
            1.0 / self._diagnostics_rate, self.run_diagnostics)
        self._priority_timer = self.create_timer(
            5.0, self.evaluate_connection_priority)
        self._topology_timer = self.create_timer(
            10.0, self.snapshot_topology)

        self._log_event('system', 'ConnectivityManagerNode 初始化完成')

    # 增强功能1: 网络拓扑监控回调
    def wifi_status_callback(self, msg):
        old_connected = self._wifi_status['connected']
        old_ssid = self._wifi_status['ssid']

        self._wifi_status = {
            'connected': msg.connected,
            'ssid': msg.ssid,
            'ip_address': msg.ip_address,
            'signal_strength': msg.signal_strength,
            'mac_address': msg.mac_address,
        }

        # 更新拓扑信息
        if msg.connected:
            self._topology['wifi_devices'][msg.ssid] = {
                'signal_strength': msg.signal_strength,
                'ip_address': msg.ip_address,
                'mac_address': msg.mac_address,
                'last_seen': time.time(),
            }
        else:
            if msg.ssid in self._topology['wifi_devices']:
                del self._topology['wifi_devices'][msg.ssid]

        self._topology['last_updated'] = time.time()

        # 增强功能4: 记录连接事件
        if msg.connected and not old_connected:
            self._log_event('wifi_connect', f'WiFi已连接: {msg.ssid} (信号: {msg.signal_strength}%)')
        elif not msg.connected and old_connected:
            self._log_event('wifi_disconnect', f'WiFi已断开: {old_ssid}')

    def bluetooth_status_callback(self, msg):
        old_connected = self._bluetooth_status['connected']
        old_address = self._bluetooth_status['address']

        # 增强功能5: 黑名单检查
        if self._blacklist_enabled and msg.address in self._device_blacklist:
            self._log_event('bluetooth_blocked', f'蓝牙设备被黑名单阻止: {msg.address} ({msg.name})')
            return

        # 增强功能5: 白名单检查（如果启用）
        if self._whitelist_enabled and msg.address not in self._device_whitelist:
            self._log_event('bluetooth_blocked', f'蓝牙设备不在白名单中: {msg.address} ({msg.name})')
            return

        if msg.connected:
            self._bluetooth_status = {
                'connected': True,
                'address': msg.address,
                'name': msg.name,
                'rssi': msg.rssi,
                'device_type': msg.device_type,
            }

            self._topology['bluetooth_devices'][msg.address] = {
                'name': msg.name,
                'rssi': msg.rssi,
                'device_type': msg.device_type,
                'last_seen': time.time(),
            }

            if not old_connected:
                self._log_event('bluetooth_connect',
                    f'蓝牙已连接: {msg.address} ({msg.name}), RSSI: {msg.rssi}, 类型: {msg.device_type}')
        else:
            if msg.address in self._topology['bluetooth_devices']:
                del self._topology['bluetooth_devices'][msg.address]

            if old_connected and msg.address == old_address:
                self._bluetooth_status = {
                    'connected': False,
                    'address': '',
                    'name': '',
                    'rssi': 0,
                    'device_type': '',
                }
                self._log_event('bluetooth_disconnect', f'蓝牙已断开: {old_address}')

        self._topology['last_updated'] = time.time()

    def bluetooth_state_callback(self, msg):
        pass

    # 增强功能1: 拓扑快照
    def snapshot_topology(self):
        snapshot = {
            'timestamp': time.time(),
            'datetime': datetime.now().isoformat(),
            'wifi_devices': dict(self._topology['wifi_devices']),
            'bluetooth_devices': dict(self._topology['bluetooth_devices']),
            'active_connection': self._active_connection_type,
        }
        self._topology_history.append(snapshot)

    # 增强功能2: 连接优先级评估
    def evaluate_connection_priority(self):
        wifi_quality = self._wifi_status.get('signal_strength', 0)
        bt_rssi = self._bluetooth_status.get('rssi', 0)

        wifi_available = self._wifi_status.get('connected', False)
        bt_available = self._bluetooth_status.get('connected', False)

        old_active = self._active_connection_type

        # WiFi优先，但信号质量低于阈值时考虑切换到蓝牙
        if wifi_available and wifi_quality >= self._wifi_priority_threshold:
            self._active_connection_type = 'wifi'
            self._active_connection_info = self._wifi_status
        elif bt_available and bt_rssi >= self._bluetooth_priority_threshold:
            self._active_connection_type = 'bluetooth'
            self._active_connection_info = self._bluetooth_status
        elif wifi_available:
            self._active_connection_type = 'wifi'
            self._active_connection_info = self._wifi_status
        elif bt_available:
            self._active_connection_type = 'bluetooth'
            self._active_connection_info = self._bluetooth_status
        else:
            self._active_connection_type = 'none'
            self._active_connection_info = {}

        if old_active != self._active_connection_type:
            self._log_event('priority_switch',
                f'活动连接切换: {old_active} -> {self._active_connection_type}')

    # 增强功能3: 网络诊断
    def run_diagnostics(self):
        if self._wifi_status.get('connected', False):
            self._run_ping_test()
            current_time = time.time()
            if current_time - self._diagnostics['last_bandwidth_time'] > 60.0:
                self._estimate_bandwidth()
                self._diagnostics['last_bandwidth_time'] = current_time

    def _run_ping_test(self):
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', str(self._ping_timeout), self._ping_target],
                capture_output=True, text=True, timeout=self._ping_timeout + 2)
            self._ping_count += 1

            if result.returncode == 0:
                self._ping_success_count += 1
                for line in result.stdout.split('\n'):
                    if 'time=' in line:
                        try:
                            time_part = line.split('time=')[1].split(' ')[0]
                            latency = float(time_part)
                            self._diagnostics['ping_latency_ms'] = latency
                            self._diagnostics['ping_history'].append(latency)
                        except (ValueError, IndexError):
                            pass
            else:
                self._diagnostics['ping_latency_ms'] = -1.0
                self._diagnostics['ping_history'].append(-1.0)

            if self._ping_count > 0:
                self._diagnostics['ping_success_rate'] = (
                    self._ping_success_count / self._ping_count)
            self._diagnostics['last_ping_time'] = time.time()

        except (subprocess.TimeoutExpired, FileNotFoundError):
            self._diagnostics['ping_latency_ms'] = -1.0
            self._ping_count += 1

    def _estimate_bandwidth(self):
        # 通过ping延迟粗略估算带宽质量
        if len(self._diagnostics['ping_history']) > 0:
            recent_pings = [p for p in list(self._diagnostics['ping_history'])[-10:] if p > 0]
            if recent_pings:
                avg_latency = sum(recent_pings) / len(recent_pings)
                # 简化的带宽估算模型：延迟越低带宽越高
                if avg_latency < 5:
                    self._diagnostics['bandwidth_mbps'] = 100.0
                elif avg_latency < 20:
                    self._diagnostics['bandwidth_mbps'] = 50.0
                elif avg_latency < 50:
                    self._diagnostics['bandwidth_mbps'] = 20.0
                elif avg_latency < 100:
                    self._diagnostics['bandwidth_mbps'] = 10.0
                else:
                    self._diagnostics['bandwidth_mbps'] = 2.0

    # 增强功能4: 事件日志
    def _log_event(self, event_type, description):
        event = {
            'timestamp': time.time(),
            'datetime': datetime.now().isoformat(),
            'event_type': event_type,
            'description': description,
        }
        self._event_log.append(event)
        self.get_logger().info(f'[连接事件] {event_type}: {description}')

    # 增强功能5: 设备访问控制
    def is_device_allowed(self, address):
        if self._blacklist_enabled and address in self._device_blacklist:
            return False
        if self._whitelist_enabled and address not in self._device_whitelist:
            return False
        return True

    def get_event_log(self):
        return list(self._event_log)

    def get_topology(self):
        return {
            'wifi_devices': dict(self._topology['wifi_devices']),
            'bluetooth_devices': dict(self._topology['bluetooth_devices']),
            'last_updated': self._topology['last_updated'],
        }

    # 增强功能6: 发布连接状态摘要
    def publish_connectivity_summary(self):
        avg_latency = -1.0
        recent_pings = [p for p in list(self._diagnostics['ping_history'])[-10:] if p > 0]
        if recent_pings:
            avg_latency = sum(recent_pings) / len(recent_pings)

        summary = {
            'timestamp': time.time(),
            'active_connection': self._active_connection_type,
            'wifi': {
                'connected': self._wifi_status.get('connected', False),
                'ssid': self._wifi_status.get('ssid', ''),
                'signal_strength': self._wifi_status.get('signal_strength', 0),
                'ip_address': self._wifi_status.get('ip_address', ''),
            },
            'bluetooth': {
                'connected': self._bluetooth_status.get('connected', False),
                'address': self._bluetooth_status.get('address', ''),
                'name': self._bluetooth_status.get('name', ''),
                'rssi': self._bluetooth_status.get('rssi', 0),
                'device_type': self._bluetooth_status.get('device_type', ''),
            },
            'topology': {
                'wifi_device_count': len(self._topology['wifi_devices']),
                'bluetooth_device_count': len(self._topology['bluetooth_devices']),
            },
            'diagnostics': {
                'ping_latency_ms': avg_latency,
                'ping_success_rate': round(self._diagnostics['ping_success_rate'], 3),
                'estimated_bandwidth_mbps': self._diagnostics['bandwidth_mbps'],
            },
            'security': {
                'whitelist_enabled': self._whitelist_enabled,
                'blacklist_enabled': self._blacklist_enabled,
                'whitelist_count': len(self._device_whitelist),
                'blacklist_count': len(self._device_blacklist),
            },
            'recent_events': [
                {
                    'type': e['event_type'],
                    'description': e['description'],
                    'time': e['datetime'],
                }
                for e in list(self._event_log)[-5:]
            ],
        }

        msg = String()
        msg.data = json.dumps(summary, ensure_ascii=False)
        self._connectivity_summary_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConnectivityManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
