import subprocess

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

        self._current_ssid = ''
        self._current_password = ''

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._wifi_status_pub = self.create_publisher(
            WiFiStatus, 'wifi_status', qos_profile)

        self._wifi_scan_results_pub = self.create_publisher(
            WiFiStatus, 'wifi_scan_results', qos_profile)

        self._connect_wifi_srv = self.create_service(
            ConnectWiFi, 'connect_wifi', self.connect_wifi_callback)

        self._disconnect_wifi_srv = self.create_service(
            Trigger, 'disconnect_wifi', self.disconnect_wifi_callback)

        self._scan_wifi_srv = self.create_service(
            Trigger, 'scan_wifi', self.scan_wifi_callback)

        check_rate = self.get_parameter('check_rate').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / check_rate, self.check_status_callback)

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

            signal_stdout, _ = self._run_nmcli(
                ['-t', '-f', 'GENERAL.WIFI', 'dev', 'show', self.get_parameter('interface').get_parameter_value().string_value],
                timeout=10)

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
        else:
            status_msg.connected = False
            status_msg.ssid = ''
            status_msg.ip_address = ''
            status_msg.signal_strength = 0
            status_msg.mac_address = ''

            if (self.get_parameter('auto_reconnect').get_parameter_value().bool_value
                    and self._current_ssid):
                self.get_logger().info(
                    f'WiFi disconnected. Attempting auto-reconnect to {self._current_ssid}')
                self._attempt_reconnect()

        self._wifi_status_pub.publish(status_msg)

    def _attempt_reconnect(self):
        if not self._current_ssid:
            return
        stdout, _ = self._run_nmcli(
            ['device', 'wifi', 'connect', self._current_ssid],
            timeout=30)
        if stdout and 'successfully' in stdout.lower():
            self.get_logger().info(f'Auto-reconnected to {self._current_ssid}')
        else:
            self.get_logger().warn(f'Auto-reconnect to {self._current_ssid} failed')

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

        stdout, stderr = self._run_nmcli(
            ['-t', '-f', 'SSID,SIGNAL,SECURITY,FREQ', 'device', 'wifi', 'list'],
            timeout=30)

        if stdout is None:
            response.success = False
            response.message = f'Scan failed: {stderr}'
            return response

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

        response.success = True
        response.message = 'WiFi scan completed'

        return response


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
