import subprocess
import time
import json
from collections import deque
from datetime import datetime

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy

from agv_interfaces.msg import WiFiStatus
from agv_interfaces.srv import ConnectWiFi
from std_msgs.msg import String
from std_srvs.srv import Trigger


class WiFiManagerNode(Node):

    def __init__(self):
        super().__init__('wifi_manager_node')

        self.declare_parameter('check_rate', 5.0)
        self.declare_parameter('interface', 'wlan0')
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('known_networks', [])
        self.declare_parameter('scan_cache_ttl', 30.0)
        self.declare_parameter('quality_check_interval', 10.0)

        # 增强功能1: WiFi信号质量历史 - 跟踪RSSI随时间变化，检测退化趋势
        self.declare_parameter('signal_history_size', 200)
        self.declare_parameter('degradation_threshold', 15)
        self.declare_parameter('degradation_window', 10)

        # 增强功能2: 指数退避自动重连
        self.declare_parameter('backoff_base_delay', 2.0)
        self.declare_parameter('backoff_max_delay', 120.0)
        self.declare_parameter('backoff_max_attempts', 10)

        # 增强功能4: 连接统计
        self.declare_parameter('stats_publish_rate', 30.0)

        self._current_ssid = ''
        self._current_password = ''
        self._known_networks = self.get_parameter('known_networks').value
        self._scan_cache_ttl = self.get_parameter('scan_cache_ttl').value
        self._quality_check_interval = self.get_parameter('quality_check_interval').value

        self._scan_cache = []
        self._scan_cache_time = 0.0
        self._connection_quality = 0
        self._last_quality_check = 0.0
        self._reconnect_attempts = 0
        self._max_reconnect_attempts = 5

        # 增强功能1: 信号质量历史记录
        self._signal_history_size = self.get_parameter('signal_history_size').value
        self._signal_history = deque(maxlen=self._signal_history_size)
        self._signal_timestamp_history = deque(maxlen=self._signal_history_size)
        self._degradation_threshold = self.get_parameter('degradation_threshold').value
        self._degradation_window = self.get_parameter('degradation_window').value
        self._degradation_detected = False

        # 增强功能2: 指数退避参数
        self._backoff_base_delay = self.get_parameter('backoff_base_delay').value
        self._backoff_max_delay = self.get_parameter('backoff_max_delay').value
        self._backoff_max_attempts = self.get_parameter('backoff_max_attempts').value
        self._backoff_current_delay = self._backoff_base_delay
        self._last_reconnect_attempt_time = 0.0

        # 增强功能3: 网络扫描与排名
        self._ranked_networks = []

        # 增强功能4: 连接统计
        self._stats_publish_rate = self.get_parameter('stats_publish_rate').value
        self._connection_stats = {
            'total_uptime_seconds': 0.0,
            'total_downtime_seconds': 0.0,
            'disconnect_count': 0,
            'reconnect_count': 0,
            'connection_start_time': None,
            'disconnection_start_time': None,
            'latency_history': deque(maxlen=100),
            'avg_latency_ms': 0.0,
            'last_connected_ssid': '',
            'total_bytes_sent': 0,
            'total_bytes_recv': 0,
        }
        self._last_stats_time = time.time()

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._wifi_status_pub = self.create_publisher(
            WiFiStatus, 'wifi_status', qos_profile)

        self._wifi_scan_results_pub = self.create_publisher(
            WiFiStatus, 'wifi_scan_results', qos_profile)

        # 增强功能1: 信号质量趋势发布
        self._signal_trend_pub = self.create_publisher(
            String, 'wifi_signal_trend', qos_profile)

        # 增强功能4: 连接统计发布
        self._wifi_stats_pub = self.create_publisher(
            String, 'wifi_stats', qos_profile)

        # 增强功能3: 排名网络发布
        self._ranked_networks_pub = self.create_publisher(
            String, 'wifi_ranked_networks', qos_profile)

        self._connect_wifi_srv = self.create_service(
            ConnectWiFi, 'connect_wifi', self.connect_wifi_callback)

        self._disconnect_wifi_srv = self.create_service(
            Trigger, 'disconnect_wifi', self.disconnect_wifi_callback)

        self._scan_wifi_srv = self.create_service(
            Trigger, 'scan_wifi', self.scan_wifi_callback)

        # 增强功能4: 获取连接统计服务
        self._get_stats_srv = self.create_service(
            Trigger, 'get_wifi_stats', self.get_stats_callback)

        check_rate = self.get_parameter('check_rate').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / check_rate, self.check_status_callback)

        # 增强功能4: 统计发布定时器
        self._stats_timer = self.create_timer(
            1.0 / self._stats_publish_rate, self.publish_stats_callback)

        # 增强功能1: 信号趋势检测定时器
        self._trend_timer = self.create_timer(15.0, self.detect_signal_trend)

        self._nmcli_available = self._check_nmcli()

        if not self._nmcli_available:
            self.get_logger().warn('nmcli is not available. WiFi management will be limited.')

    def _check_nmcli(self):
        try:
            subprocess.run(
                ['nmcli', '--version'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5)
            return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def _run_nmcli(self, args, timeout=30):
        if not self._nmcli_available:
            return None, 'nmcli is not available'
        try:
            result = subprocess.run(
                ['nmcli'] + args,
                capture_output=True,
                text=True,
                timeout=timeout)
            return result.stdout.strip(), result.stderr.strip()
        except subprocess.TimeoutExpired:
            return None, 'nmcli command timed out'
        except FileNotFoundError:
            self._nmcli_available = False
            return None, 'nmcli is not available'
        except Exception as e:
            return None, str(e)

    def _get_signal_quality(self):
        interface = self.get_parameter('interface').get_parameter_value().string_value
        signal_strength_stdout, _ = self._run_nmcli(
            ['-t', '-f', 'GENERAL.STRENGTH', 'dev', 'show', interface],
            timeout=10)
        if signal_strength_stdout:
            strength_line = signal_strength_stdout.split('\n')[0]
            if ':' in strength_line:
                try:
                    return int(strength_line.split(':')[1])
                except ValueError:
                    return 0
        return 0

    def _check_connection_quality(self):
        now = time.time()
        if now - self._last_quality_check < self._quality_check_interval:
            return
        self._last_quality_check = now

        if not self._current_ssid:
            return

        quality = self._get_signal_quality()
        self._connection_quality = quality

        # 增强功能1: 记录信号质量历史
        self._signal_history.append(quality)
        self._signal_timestamp_history.append(now)

        if quality < 20 and quality > 0:
            self.get_logger().warn(f'WiFi信号质量差: {quality}%')
            self._attempt_fallback()

    # 增强功能1: 信号退化趋势检测
    def detect_signal_trend(self):
        if len(self._signal_history) < self._degradation_window:
            return

        recent_signals = list(self._signal_history)[-self._degradation_window:]
        older_signals = list(self._signal_history)[-2 * self._degradation_window:-self._degradation_window]

        if len(older_signals) < self._degradation_window:
            return

        recent_avg = sum(recent_signals) / len(recent_signals)
        older_avg = sum(older_signals) / len(older_signals)
        degradation = older_avg - recent_avg

        trend_msg = String()
        if degradation > self._degradation_threshold:
            self._degradation_detected = True
            self.get_logger().warn(
                f'检测到WiFi信号退化趋势: 平均信号从 {older_avg:.1f}% 降至 {recent_avg:.1f}% (下降 {degradation:.1f}%)')
            trend_data = {
                'trend': 'degrading',
                'older_avg': round(older_avg, 1),
                'recent_avg': round(recent_avg, 1),
                'degradation': round(degradation, 1),
                'timestamp': datetime.now().isoformat(),
            }
        elif degradation < -self._degradation_threshold:
            self._degradation_detected = False
            trend_data = {
                'trend': 'improving',
                'older_avg': round(older_avg, 1),
                'recent_avg': round(recent_avg, 1),
                'improvement': round(-degradation, 1),
                'timestamp': datetime.now().isoformat(),
            }
        else:
            self._degradation_detected = False
            trend_data = {
                'trend': 'stable',
                'older_avg': round(older_avg, 1),
                'recent_avg': round(recent_avg, 1),
                'change': round(degradation, 1),
                'timestamp': datetime.now().isoformat(),
            }

        trend_msg.data = json.dumps(trend_data, ensure_ascii=False)
        self._signal_trend_pub.publish(trend_msg)

    def _attempt_fallback(self):
        if not self._known_networks:
            return
        known_ssids = [n if isinstance(n, str) else n.get('ssid', '') for n in self._known_networks]
        if self._current_ssid in known_ssids:
            current_idx = known_ssids.index(self._current_ssid)
            if current_idx < len(known_ssids) - 1:
                fallback_ssid = known_ssids[current_idx + 1]
                self.get_logger().info(f'尝试回退到网络: {fallback_ssid}')
                stdout, _ = self._run_nmcli(
                    ['device', 'wifi', 'connect', fallback_ssid],
                    timeout=30)
                if stdout and 'successfully' in stdout.lower():
                    self.get_logger().info(f'回退到 {fallback_ssid} 成功')
                    self._current_ssid = fallback_ssid
                    self._reconnect_attempts = 0
                    self._backoff_current_delay = self._backoff_base_delay

    def _get_cached_scan_results(self):
        now = time.time()
        if self._scan_cache and (now - self._scan_cache_time) < self._scan_cache_ttl:
            return self._scan_cache

        stdout, stderr = self._run_nmcli(
            ['-t', '-f', 'SSID,SIGNAL,SECURITY,FREQ', 'device', 'wifi', 'list'],
            timeout=30)

        if stdout is None:
            return self._scan_cache

        results = []
        for line in stdout.split('\n'):
            if not line:
                continue
            parts = line.split(':')
            if len(parts) >= 2 and parts[0]:
                scan_entry = {
                    'ssid': parts[0],
                    'signal': 0,
                    'security': parts[2] if len(parts) > 2 else '',
                }
                try:
                    scan_entry['signal'] = int(parts[1])
                except ValueError:
                    pass
                results.append(scan_entry)

        self._scan_cache = results
        self._scan_cache_time = now

        # 增强功能3: 更新排名网络列表
        self._update_ranked_networks(results)

        return results

    # 增强功能3: WiFi网络扫描与排名
    def _update_ranked_networks(self, scan_results):
        known_ssids = set()
        for n in self._known_networks:
            if isinstance(n, str):
                known_ssids.add(n)
            elif isinstance(n, dict):
                known_ssids.add(n.get('ssid', ''))

        scored_networks = []
        for entry in scan_results:
            if not entry.get('ssid'):
                continue
            score = entry.get('signal', 0)
            if entry.get('ssid') in known_ssids:
                score += 20
            if entry.get('security'):
                score += 10
            scored_networks.append({
                'ssid': entry['ssid'],
                'signal': entry.get('signal', 0),
                'security': entry.get('security', ''),
                'score': score,
            })

        scored_networks.sort(key=lambda x: x['score'], reverse=True)
        self._ranked_networks = scored_networks

        ranked_msg = String()
        ranked_data = {
            'networks': scored_networks[:10],
            'timestamp': datetime.now().isoformat(),
        }
        ranked_msg.data = json.dumps(ranked_data, ensure_ascii=False)
        self._ranked_networks_pub.publish(ranked_msg)

    def check_status_callback(self):
        status_msg = WiFiStatus()

        if not self._nmcli_available:
            status_msg.connected = False
            status_msg.ssid = ''
            status_msg.ip_address = ''
            status_msg.signal_strength = 0
            status_msg.mac_address = ''
            self._wifi_status_pub.publish(status_msg)
            return

        stdout, _ = self._run_nmcli(
            ['-t', '-f', 'NAME,TYPE,DEVICE', 'con', 'show', '--active'], timeout=10)

        active_ssid = ''
        if stdout:
            for line in stdout.split('\n'):
                if line and 'wifi' in line:
                    parts = line.split(':')
                    if len(parts) >= 3 and parts[2]:
                        active_ssid = parts[0]
                        break

        now = time.time()
        dt = now - self._last_stats_time
        self._last_stats_time = now

        if active_ssid:
            status_msg.connected = True
            status_msg.ssid = active_ssid

            ip_stdout, _ = self._run_nmcli(
                ['-t', '-f', 'IP4.ADDRESS', 'dev', 'show', self.get_parameter('interface').get_parameter_value().string_value],
                timeout=10)
            if ip_stdout:
                ip_line = ip_stdout.split('\n')[0]
                if ':' in ip_line:
                    status_msg.ip_address = ip_line.split(':')[1].split('/')[0]

            signal_strength_stdout, _ = self._run_nmcli(
                ['-t', '-f', 'GENERAL.STRENGTH', 'dev', 'show', self.get_parameter('interface').get_parameter_value().string_value],
                timeout=10)
            if signal_strength_stdout:
                strength_line = signal_strength_stdout.split('\n')[0]
                if ':' in strength_line:
                    try:
                        status_msg.signal_strength = int(strength_line.split(':')[1])
                    except ValueError:
                        status_msg.signal_strength = 0

            mac_stdout, _ = self._run_nmcli(
                ['-t', '-f', 'GENERAL.HWADDR', 'dev', 'show', self.get_parameter('interface').get_parameter_value().string_value],
                timeout=10)
            if mac_stdout:
                mac_line = mac_stdout.split('\n')[0]
                if ':' in mac_line:
                    status_msg.mac_address = mac_line.split(':')[1]

            if self.get_parameter('auto_reconnect').get_parameter_value().bool_value:
                self._current_ssid = active_ssid
                self._reconnect_attempts = 0
                self._backoff_current_delay = self._backoff_base_delay

            self._check_connection_quality()

            # 增强功能4: 更新连接统计 - 在线时间
            self._connection_stats['total_uptime_seconds'] += dt
            if self._connection_stats['connection_start_time'] is None:
                self._connection_stats['connection_start_time'] = now
                self._connection_stats['last_connected_ssid'] = active_ssid
            self._connection_stats['disconnection_start_time'] = None

            # 增强功能4: 测量延迟
            self._measure_latency()
        else:
            status_msg.connected = False
            status_msg.ssid = ''
            status_msg.ip_address = ''
            status_msg.signal_strength = 0
            status_msg.mac_address = ''

            # 增强功能4: 更新连接统计 - 离线时间
            self._connection_stats['total_downtime_seconds'] += dt
            if self._connection_stats['disconnection_start_time'] is None:
                self._connection_stats['disconnection_start_time'] = now
                if self._connection_stats['connection_start_time'] is not None:
                    self._connection_stats['disconnect_count'] += 1
            self._connection_stats['connection_start_time'] = None

            if (self.get_parameter('auto_reconnect').get_parameter_value().bool_value
                    and self._current_ssid):
                self._attempt_reconnect_with_backoff()

        self._wifi_status_pub.publish(status_msg)

    # 增强功能4: 延迟测量
    def _measure_latency(self):
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '2', '8.8.8.8'],
                capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'time=' in line:
                        try:
                            latency_str = line.split('time=')[1].split(' ')[0]
                            latency = float(latency_str)
                            self._connection_stats['latency_history'].append(latency)
                            if len(self._connection_stats['latency_history']) > 0:
                                self._connection_stats['avg_latency_ms'] = (
                                    sum(self._connection_stats['latency_history']) /
                                    len(self._connection_stats['latency_history']))
                        except (ValueError, IndexError):
                            pass
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

    # 增强功能2: 指数退避自动重连
    def _attempt_reconnect_with_backoff(self):
        if not self._current_ssid:
            return

        now = time.time()
        if now - self._last_reconnect_attempt_time < self._backoff_current_delay:
            return

        if self._reconnect_attempts >= self._backoff_max_attempts:
            self.get_logger().warn('达到最大退避重连次数，尝试回退网络')
            self._attempt_fallback()
            self._reconnect_attempts = 0
            self._backoff_current_delay = self._backoff_base_delay
            return

        self._reconnect_attempts += 1
        self._last_reconnect_attempt_time = now

        self.get_logger().info(
            f'尝试重连 {self._current_ssid} (第{self._reconnect_attempts}次, '
            f'退避延迟: {self._backoff_current_delay:.1f}秒)')

        stdout, _ = self._run_nmcli(
            ['device', 'wifi', 'connect', self._current_ssid],
            timeout=30)
        if stdout and 'successfully' in stdout.lower():
            self.get_logger().info(f'自动重连到 {self._current_ssid} 成功')
            self._reconnect_attempts = 0
            self._backoff_current_delay = self._backoff_base_delay
            self._connection_stats['reconnect_count'] += 1
        else:
            # 指数退避: 延迟翻倍，但不超过最大值
            self._backoff_current_delay = min(
                self._backoff_current_delay * 2, self._backoff_max_delay)
            self.get_logger().warn(
                f'自动重连到 {self._current_ssid} 失败 '
                f'(第{self._reconnect_attempts}次，下次延迟: {self._backoff_current_delay:.1f}秒)')

    def _attempt_reconnect(self):
        if not self._current_ssid:
            return
        if self._reconnect_attempts >= self._max_reconnect_attempts:
            self.get_logger().warn('Max reconnect attempts reached, trying fallback')
            self._attempt_fallback()
            self._reconnect_attempts = 0
            return

        self._reconnect_attempts += 1
        stdout, _ = self._run_nmcli(
            ['device', 'wifi', 'connect', self._current_ssid],
            timeout=30)
        if stdout and 'successfully' in stdout.lower():
            self.get_logger().info(f'Auto-reconnected to {self._current_ssid}')
            self._reconnect_attempts = 0
        else:
            self.get_logger().warn(f'Auto-reconnect to {self._current_ssid} failed (attempt {self._reconnect_attempts})')

    def connect_wifi_callback(self, request, response):
        if not self._nmcli_available:
            response.success = False
            response.message = 'nmcli is not available'
            response.ip_address = ''
            return response

        self.get_logger().info(f'Connecting to WiFi: {request.ssid}')

        args = ['device', 'wifi', 'connect', request.ssid]
        if request.password:
            args.extend(['password', request.password])

        stdout, stderr = self._run_nmcli(args, timeout=30)

        if stdout and 'successfully' in stdout.lower():
            response.success = True
            response.message = f'Successfully connected to {request.ssid}'
            self._current_ssid = request.ssid
            self._current_password = request.password
            self._reconnect_attempts = 0
            self._backoff_current_delay = self._backoff_base_delay

            ip_stdout, _ = self._run_nmcli(
                ['-t', '-f', 'IP4.ADDRESS', 'dev', 'show', self.get_parameter('interface').get_parameter_value().string_value],
                timeout=10)
            if ip_stdout:
                ip_line = ip_stdout.split('\n')[0]
                if ':' in ip_line:
                    response.ip_address = ip_line.split(':')[1].split('/')[0]
                else:
                    response.ip_address = ''
            else:
                response.ip_address = ''
        else:
            response.success = False
            response.message = f'Failed to connect to {request.ssid}: {stderr or "unknown error"}'
            response.ip_address = ''

        return response

    def disconnect_wifi_callback(self, request, response):
        if not self._nmcli_available:
            response.success = False
            response.message = 'nmcli is not available'
            return response

        self.get_logger().info('Disconnecting from WiFi')

        stdout, stderr = self._run_nmcli(
            ['device', 'disconnect', self.get_parameter('interface').get_parameter_value().string_value],
            timeout=15)

        if stdout or (stderr and 'disconnected' in stderr.lower()):
            response.success = True
            response.message = 'Successfully disconnected from WiFi'
            self._current_ssid = ''
            self._current_password = ''
        elif stderr and 'not connected' in stderr.lower():
            response.success = True
            response.message = 'Already disconnected'
        else:
            response.success = False
            response.message = f'Failed to disconnect: {stderr or "unknown error"}'

        return response

    def scan_wifi_callback(self, request, response):
        if not self._nmcli_available:
            response.success = False
            response.message = 'nmcli is not available'
            return response

        self.get_logger().info('Scanning for WiFi networks')

        self._scan_cache = []
        self._scan_cache_time = 0.0

        stdout, stderr = self._run_nmcli(
            ['-t', '-f', 'SSID,SIGNAL,SECURITY,FREQ', 'device', 'wifi', 'list'],
            timeout=30)

        if stdout is None:
            response.success = False
            response.message = f'Scan failed: {stderr}'
            return response

        results = []
        for line in stdout.split('\n'):
            if not line:
                continue
            parts = line.split(':')
            if len(parts) >= 2 and parts[0]:
                scan_msg = WiFiStatus()
                scan_msg.ssid = parts[0]
                try:
                    scan_msg.signal_strength = int(parts[1])
                except ValueError:
                    scan_msg.signal_strength = 0
                scan_msg.connected = False
                scan_msg.ip_address = ''
                scan_msg.mac_address = ''
                self._wifi_scan_results_pub.publish(scan_msg)
                results.append({
                    'ssid': parts[0],
                    'signal': scan_msg.signal_strength,
                })

        self._scan_cache = results
        self._scan_cache_time = time.time()

        # 增强功能3: 更新排名网络列表
        self._update_ranked_networks(results)

        response.success = True
        response.message = 'WiFi scan completed'

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
            'avg_latency_ms': round(self._connection_stats['avg_latency_ms'], 2),
            'current_ssid': self._current_ssid,
            'last_connected_ssid': self._connection_stats['last_connected_ssid'],
            'signal_quality': self._connection_quality,
            'degradation_detected': self._degradation_detected,
            'backoff_delay': round(self._backoff_current_delay, 1),
            'reconnect_attempts': self._reconnect_attempts,
        }

    # 增强功能4: 发布连接统计
    def publish_stats_callback(self):
        stats = self._compute_stats()
        stats_msg = String()
        stats_msg.data = json.dumps(stats, ensure_ascii=False)
        self._wifi_stats_pub.publish(stats_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WiFiManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
