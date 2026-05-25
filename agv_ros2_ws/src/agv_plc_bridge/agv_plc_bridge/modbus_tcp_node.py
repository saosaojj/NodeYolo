import rclpy
from rclpy.node import Node
from std_msgs.msg import BoolMultiArray, Int32MultiArray
from agv_interfaces.msg import PlcData
from agv_interfaces.srv import ReadPlc, WritePlc
from pymodbus.client import ModbusTcpClient


class ModbusTcpNode(Node):

    def __init__(self):
        super().__init__('modbus_tcp_node')

        self.declare_parameter('plc_ip', '192.168.1.10')
        self.declare_parameter('plc_port', 502)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('poll_rate', 100)
        self.declare_parameter('coil_read_start', 0)
        self.declare_parameter('coil_read_count', 16)
        self.declare_parameter('register_read_start', 0)
        self.declare_parameter('register_read_count', 16)

        self.plc_ip = self.get_parameter('plc_ip').value
        self.plc_port = self.get_parameter('plc_port').value
        self.slave_id = self.get_parameter('slave_id').value
        poll_rate = self.get_parameter('poll_rate').value
        self.coil_read_start = self.get_parameter('coil_read_start').value
        self.coil_read_count = self.get_parameter('coil_read_count').value
        self.register_read_start = self.get_parameter('register_read_start').value
        self.register_read_count = self.get_parameter('register_read_count').value

        self.client = ModbusTcpClient(
            host=self.plc_ip,
            port=self.plc_port,
        )

        self.plc_data_pub = self.create_publisher(PlcData, 'plc_data', 10)
        self.coil_states_pub = self.create_publisher(BoolMultiArray, 'coil_states', 10)
        self.register_states_pub = self.create_publisher(Int32MultiArray, 'register_states', 10)

        self.write_coils_sub = self.create_subscription(
            BoolMultiArray, 'write_coils', self.write_coils_callback, 10)
        self.write_registers_sub = self.create_subscription(
            Int32MultiArray, 'write_registers', self.write_registers_callback, 10)

        self.read_plc_srv = self.create_service(ReadPlc, 'read_plc', self.read_plc_callback)
        self.write_plc_srv = self.create_service(WritePlc, 'write_plc', self.write_plc_callback)

        self.poll_timer = self.create_timer(poll_rate / 1000.0, self.poll_plc)

        self.connected = False
        self.connect()

    def connect(self):
        result = self.client.connect()
        self.connected = result
        if self.connected:
            self.get_logger().info(f'Connected to PLC at {self.plc_ip}:{self.plc_port}')
        else:
            self.get_logger().warn(f'Failed to connect to PLC at {self.plc_ip}:{self.plc_port}')

    def disconnect(self):
        self.client.close()
        self.connected = False
        self.get_logger().info('Disconnected from PLC')

    def poll_plc(self):
        if not self.connected:
            self.connect()
            if not self.connected:
                return

        plc_data = PlcData()
        plc_data.device_name = 'plc'
        plc_data.ip_address = self.plc_ip
        plc_data.connected = self.connected
        plc_data.timestamp = self.get_clock().now().to_msg()

        coil_result = self.client.read_coils(
            address=self.coil_read_start,
            count=self.coil_read_count,
            slave=self.slave_id,
        )

        if not coil_result.isError():
            coil_bits = coil_result.bits[:self.coil_read_count]
            plc_data.coil_values = [int(b) for b in coil_bits]

            coil_msg = BoolMultiArray()
            coil_msg.data = coil_bits
            self.coil_states_pub.publish(coil_msg)
        else:
            self.get_logger().warn(f'Failed to read coils: {coil_result}')
            plc_data.coil_values = []

        register_result = self.client.read_holding_registers(
            address=self.register_read_start,
            count=self.register_read_count,
            slave=self.slave_id,
        )

        if not register_result.isError():
            register_values = register_result.registers[:self.register_read_count]
            plc_data.register_values = [int(v) for v in register_values]

            reg_msg = Int32MultiArray()
            reg_msg.data = register_values
            self.register_states_pub.publish(reg_msg)
        else:
            self.get_logger().warn(f'Failed to read registers: {register_result}')
            plc_data.register_values = []

        self.plc_data_pub.publish(plc_data)

    def write_coils_callback(self, msg):
        if not self.connected:
            self.get_logger().warn('Not connected to PLC, cannot write coils')
            return

        result = self.client.write_coils(
            address=self.coil_read_start,
            values=msg.data,
            slave=self.slave_id,
        )

        if result.isError():
            self.get_logger().warn(f'Failed to write coils: {result}')
        else:
            self.get_logger().info(f'Successfully wrote {len(msg.data)} coils')

    def write_registers_callback(self, msg):
        if not self.connected:
            self.get_logger().warn('Not connected to PLC, cannot write registers')
            return

        result = self.client.write_registers(
            address=self.register_read_start,
            values=msg.data,
            slave=self.slave_id,
        )

        if result.isError():
            self.get_logger().warn(f'Failed to write registers: {result}')
        else:
            self.get_logger().info(f'Successfully wrote {len(msg.data)} registers')

    def read_plc_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = 'Not connected to PLC'
            response.values = []
            return response

        if request.quantity <= 0:
            response.success = False
            response.message = 'Quantity must be positive'
            response.values = []
            return response

        result = self.client.read_holding_registers(
            address=request.start_address,
            count=request.quantity,
            slave=self.slave_id,
        )

        if not result.isError():
            response.success = True
            response.message = 'Read successful'
            response.values = [int(v) for v in result.registers[:request.quantity]]
        else:
            response.success = False
            response.message = f'Read failed: {result}'
            response.values = []

        return response

    def write_plc_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = 'Not connected to PLC'
            return response

        result = self.client.write_registers(
            address=request.start_address,
            values=request.values,
            slave=self.slave_id,
        )

        if not result.isError():
            response.success = True
            response.message = 'Write successful'
        else:
            response.success = False
            response.message = f'Write failed: {result}'

        return response

    def destroy(self):
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModbusTcpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
