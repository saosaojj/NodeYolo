import subprocess
import re

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy

from agv_interfaces.msg import BluetoothDevice
from agv_interfaces.srv import ConnectBluetooth
from std_msgs.msg import String
from std_srvs.srv import Trigger


class BluetoothManagerNode(Node):

    def __init__(self):
        super().__init__('bluetooth_manager_node')

        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('adapter', 'hci0')
        self.declare_parameter('auto_connect_paired', False)

        self._discovered_devices = {}
        self._connected_address = ''

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._bluetooth_devices_pub = self.create_publisher(
            BluetoothDevice, 'bluetooth_devices', qos_profile)

        self._bluetooth_status_pub = self.create_publisher(
            String, 'bluetooth_status', qos_profile)

        self._connect_bluetooth_srv = self.create_service(
            ConnectBluetooth, 'connect_bluetooth', self.connect_callback)

        self._disconnect_bluetooth_srv = self.create_service(
            Trigger, 'disconnect_bluetooth', self.disconnect_callback)

        self._scan_bluetooth_srv = self.create_service(
            Trigger, 'scan_bluetooth', self.scan_callback)

        scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / scan_rate, self.timer_callback)

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

        for address, device_info in self._discovered_devices.items():
            device_msg = BluetoothDevice()
            device_msg.name = device_info.get('name', '')
            device_msg.address = address
            device_msg.rssi = device_info.get('rssi', 0)
            device_msg.connected = (address == self._connected_address)
            device_msg.device_type = device_info.get('type', '')
            self._bluetooth_devices_pub.publish(device_msg)

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
            self._connected_address = ''
        else:
            response.success = False
            response.message = f'Failed to disconnect: {stderr or "unknown error"}'

        return response


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
