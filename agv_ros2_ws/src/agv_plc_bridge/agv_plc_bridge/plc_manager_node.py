import yaml
from dataclasses import dataclass, field
from typing import Dict

import rclpy
from rclpy.node import Node
from agv_interfaces.msg import PlcData
from agv_interfaces.srv import ReadPlc, WritePlc
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


class PlcManagerNode(Node):

    def __init__(self):
        super().__init__('plc_manager_node')

        self.declare_parameter('plc_config_file', '')
        self.declare_parameter('poll_rate_ms', 100)

        self.devices: Dict[str, PlcDevice] = {}
        self.publishers: Dict[str, object] = {}

        config_file = self.get_parameter('plc_config_file').value
        if config_file:
            self._load_config(config_file)

        poll_rate = self.get_parameter('poll_rate_ms').value
        self.poll_timer = self.create_timer(poll_rate / 1000.0, self.poll_all)

        self.read_plc_srv = self.create_service(ReadPlc, 'read_plc', self.read_plc_callback)
        self.write_plc_srv = self.create_service(WritePlc, 'write_plc', self.write_plc_callback)

        self.get_logger().info(f'PlcManagerNode started with {len(self.devices)} device(s)')

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
                )
        except Exception as e:
            self.get_logger().error(f'Failed to load config file {config_file}: {e}')

    def add_plc(self, device_name, ip, port=502, slave_id=1,
                coil_read_start=0, coil_read_count=16,
                register_read_start=0, register_read_count=16):
        if device_name in self.devices:
            self.get_logger().warn(f'Device {device_name} already exists, replacing')

        device = PlcDevice(
            name=device_name,
            ip=ip,
            port=port,
            slave_id=slave_id,
            coil_read_start=coil_read_start,
            coil_read_count=coil_read_count,
            register_read_start=register_read_start,
            register_read_count=register_read_count,
        )

        device.client = ModbusTcpClient(host=ip, port=port)
        result = device.client.connect()
        device.connected = result

        if result:
            self.get_logger().info(f'Connected to {device_name} at {ip}:{port}')
        else:
            self.get_logger().warn(f'Failed to connect to {device_name} at {ip}:{port}')

        self.devices[device_name] = device
        self.publishers[device_name] = self.create_publisher(
            PlcData, f'plc_status/{device_name}', 10)

    def remove_plc(self, device_name):
        if device_name not in self.devices:
            self.get_logger().warn(f'Device {device_name} not found')
            return

        device = self.devices[device_name]
        if device.client:
            device.client.close()
        device.connected = False

        del self.devices[device_name]
        del self.publishers[device_name]

        self.get_logger().info(f'Removed device {device_name}')

    def poll_all(self):
        for name, device in self.devices.items():
            plc_data = PlcData()
            plc_data.device_name = name
            plc_data.ip_address = device.ip
            plc_data.connected = device.connected
            plc_data.timestamp = self.get_clock().now().to_msg()

            if not device.connected:
                result = device.client.connect()
                device.connected = result
                if not result:
                    plc_data.coil_values = []
                    plc_data.register_values = []
                    if name in self.publishers:
                        self.publishers[name].publish(plc_data)
                    continue

            coil_result = device.client.read_coils(
                address=device.coil_read_start,
                count=device.coil_read_count,
                slave=device.slave_id,
            )

            if not coil_result.isError():
                coil_bits = coil_result.bits[:device.coil_read_count]
                plc_data.coil_values = [int(b) for b in coil_bits]
            else:
                device.connected = False
                plc_data.coil_values = []

            register_result = device.client.read_holding_registers(
                address=device.register_read_start,
                count=device.register_read_count,
                slave=device.slave_id,
            )

            if not register_result.isError():
                register_values = register_result.registers[:device.register_read_count]
                plc_data.register_values = [int(v) for v in register_values]
            else:
                device.connected = False
                plc_data.register_values = []

            if name in self.publishers:
                self.publishers[name].publish(plc_data)

    def _find_device(self, device_name, ip_address):
        if device_name and device_name in self.devices:
            return self.devices[device_name]
        if ip_address:
            for dev in self.devices.values():
                if dev.ip == ip_address:
                    return dev
        return None

    def read_plc_callback(self, request, response):
        device = self._find_device(request.device_name, request.ip_address)

        if device is None:
            response.success = False
            response.message = f'Device not found: {request.device_name or request.ip_address}'
            response.values = []
            return response

        if not device.connected:
            result = device.client.connect()
            device.connected = result
            if not result:
                response.success = False
                response.message = 'Not connected to PLC'
                response.values = []
                return response

        read_result = device.client.read_holding_registers(
            address=request.start_address,
            count=request.quantity,
            slave=device.slave_id,
        )

        if not read_result.isError():
            response.success = True
            response.message = 'Read successful'
            response.values = [int(v) for v in read_result.registers[:request.quantity]]
        else:
            response.success = False
            response.message = f'Read failed: {read_result}'
            response.values = []

        return response

    def write_plc_callback(self, request, response):
        device = self._find_device(request.device_name, request.ip_address)

        if device is None:
            response.success = False
            response.message = f'Device not found: {request.device_name or request.ip_address}'
            return response

        if not device.connected:
            result = device.client.connect()
            device.connected = result
            if not result:
                response.success = False
                response.message = 'Not connected to PLC'
                return response

        write_result = device.client.write_registers(
            address=request.start_address,
            values=request.values,
            slave=device.slave_id,
        )

        if not write_result.isError():
            response.success = True
            response.message = 'Write successful'
        else:
            response.success = False
            response.message = f'Write failed: {write_result}'

        return response

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
