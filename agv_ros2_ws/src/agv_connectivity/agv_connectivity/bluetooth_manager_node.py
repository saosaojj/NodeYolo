# 蓝牙管理节点模块，负责蓝牙设备的扫描、连接和断开操作
import subprocess
import re

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy

from agv_interfaces.msg import BluetoothDevice
from agv_interfaces.srv import ConnectBluetooth
from std_msgs.msg import String
from std_srvs.srv import Trigger


# 蓝牙管理节点类，通过bluetoothctl工具管理蓝牙设备的扫描、连接和状态监控
class BluetoothManagerNode(Node):

    def __init__(self):
        super().__init__('bluetooth_manager_node')

        # 声明蓝牙管理参数：扫描频率、适配器名称、是否自动连接已配对设备
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('adapter', 'hci0')
        self.declare_parameter('auto_connect_paired', False)

        # 已发现的设备列表和当前连接的设备地址
        self._discovered_devices = {}
        self._connected_address = ''

        # 创建可靠QoS配置
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 创建蓝牙设备信息发布者
        self._bluetooth_devices_pub = self.create_publisher(
            BluetoothDevice, 'bluetooth_devices', qos_profile)

        # 创建蓝牙状态发布者
        self._bluetooth_status_pub = self.create_publisher(
            String, 'bluetooth_status', qos_profile)

        # 创建蓝牙管理相关的三个服务：连接、断开、扫描
        self._connect_bluetooth_srv = self.create_service(
            ConnectBluetooth, 'connect_bluetooth', self.connect_callback)

        self._disconnect_bluetooth_srv = self.create_service(
            Trigger, 'disconnect_bluetooth', self.disconnect_callback)

        self._scan_bluetooth_srv = self.create_service(
            Trigger, 'scan_bluetooth', self.scan_callback)

        # 创建定时器，定期检查蓝牙状态
        scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / scan_rate, self.timer_callback)

        # 检查bluetoothctl工具是否可用
        self._bluetoothctl_available = self._check_bluetoothctl()

        if not self._bluetoothctl_available:
            self.get_logger().warn('bluetoothctl is not available. Bluetooth management will be limited.')

    # 检查系统中bluetoothctl命令是否可用
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

    # 执行bluetoothctl命令的通用方法，发送命令列表并获取输出
    def _run_bluetoothctl(self, commands, timeout=15):
        if not self._bluetoothctl_available:
            return None, 'bluetoothctl is not available'
        try:
            # 将命令拼接为输入字符串，末尾添加quit退出
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

    # 定时器回调，定期检查蓝牙适配器状态并发布设备和状态信息
    def timer_callback(self):
        # bluetoothctl不可用时发布unavailable状态
        if not self._bluetoothctl_available:
            status_msg = String()
            status_msg.data = 'unavailable'
            self._bluetooth_status_pub.publish(status_msg)
            return

        # 查询蓝牙适配器状态
        adapter = self.get_parameter('adapter').get_parameter_value().string_value

        stdout, _ = self._run_bluetoothctl(['show', adapter], timeout=10)

        # 根据适配器状态确定蓝牙状态字符串
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

        # 如果启用了自动连接已配对设备，尝试自动连接
        if self.get_parameter('auto_connect_paired').get_parameter_value().bool_value:
            self._auto_connect_paired()

        # 发布已发现的设备信息
        for address, device_info in self._discovered_devices.items():
            device_msg = BluetoothDevice()
            device_msg.name = device_info.get('name', '')
            device_msg.address = address
            device_msg.rssi = device_info.get('rssi', 0)
            device_msg.connected = (address == self._connected_address)
            device_msg.device_type = device_info.get('type', '')
            self._bluetooth_devices_pub.publish(device_msg)

    # 自动连接已配对的蓝牙设备
    def _auto_connect_paired(self):
        # 已有连接时不再自动连接
        if self._connected_address:
            return

        # 获取已配对设备列表
        stdout, _ = self._run_bluetoothctl(['paired-devices'], timeout=10)
        if not stdout:
            return

        # 尝试连接第一个已配对设备
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

    # 蓝牙扫描服务回调，扫描周围蓝牙设备并更新设备列表
    def scan_callback(self, request, response):
        if not self._bluetoothctl_available:
            response.success = False
            response.message = 'bluetoothctl is not available'
            return response

        self.get_logger().info('Scanning for Bluetooth devices')

        # 开启蓝牙并启动扫描
        stdout, stderr = self._run_bluetoothctl(
            ['power on', 'scan on'], timeout=20)

        if stdout is None:
            response.success = False
            response.message = f'Scan failed: {stderr}'
            return response

        # 清空已发现的设备列表
        self._discovered_devices.clear()

        # 获取扫描到的设备列表
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

        # 关闭扫描
        info_stdout, _ = self._run_bluetoothctl(['scan off'], timeout=10)

        response.success = True
        response.message = f'Bluetooth scan completed, found {len(self._discovered_devices)} devices'

        return response

    # 蓝牙连接服务回调，连接指定地址的蓝牙设备
    def connect_callback(self, request, response):
        if not self._bluetoothctl_available:
            response.success = False
            response.message = 'bluetoothctl is not available'
            return response

        address = request.address
        self.get_logger().info(f'Connecting to Bluetooth device: {address}')

        # 开启蓝牙并连接指定设备
        commands = ['power on', f'connect {address}']
        stdout, stderr = self._run_bluetoothctl(commands, timeout=20)

        if stdout and 'successful' in stdout.lower():
            response.success = True
            response.message = f'Successfully connected to {address}'
            self._connected_address = address

            # 如果设备不在已发现列表中，添加进去
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

    # 蓝牙断开服务回调，断开当前连接的蓝牙设备
    def disconnect_callback(self, request, response):
        if not self._bluetoothctl_available:
            response.success = False
            response.message = 'bluetoothctl is not available'
            return response

        # 没有已连接设备时直接返回成功
        if not self._connected_address:
            response.success = True
            response.message = 'No device currently connected'
            return response

        self.get_logger().info(f'Disconnecting from: {self._connected_address}')

        # 断开当前连接的设备
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


# 主函数，初始化ROS2并运行蓝牙管理节点
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
