# Modbus TCP 通信节点，负责与PLC建立Modbus TCP连接，周期性轮询读写线圈和寄存器数据
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import BoolMultiArray, Int32MultiArray
from agv_interfaces.msg import PlcData
from agv_interfaces.srv import ReadPlc, WritePlc
from pymodbus.client import ModbusTcpClient


# ModbusTcpNode: 通过Modbus TCP协议与PLC通信的ROS2节点
# 支持线圈和保持寄存器的周期性读写，提供服务接口供其他节点调用
class ModbusTcpNode(Node):

    def __init__(self):
        super().__init__('modbus_tcp_node')

        # 声明PLC连接参数
        self.declare_parameter('plc_ip', '192.168.1.10')
        self.declare_parameter('plc_port', 502)
        self.declare_parameter('slave_id', 1)
        # 声明轮询参数
        self.declare_parameter('poll_rate', 100)
        # 声明线圈读取参数
        self.declare_parameter('coil_read_start', 0)
        self.declare_parameter('coil_read_count', 16)
        # 声明寄存器读取参数
        self.declare_parameter('register_read_start', 0)
        self.declare_parameter('register_read_count', 16)
        # 声明重连和看门狗参数
        self.declare_parameter('max_retry_interval', 30.0)
        self.declare_parameter('watchdog_timeout', 5.0)

        # 获取PLC连接参数
        self.plc_ip = self.get_parameter('plc_ip').value
        self.plc_port = self.get_parameter('plc_port').value
        self.slave_id = self.get_parameter('slave_id').value
        poll_rate = self.get_parameter('poll_rate').value
        # 获取线圈读取参数
        self.coil_read_start = self.get_parameter('coil_read_start').value
        self.coil_read_count = self.get_parameter('coil_read_count').value
        # 获取寄存器读取参数
        self.register_read_start = self.get_parameter('register_read_start').value
        self.register_read_count = self.get_parameter('register_read_count').value
        # 获取重连和看门狗参数
        self._max_retry_interval = self.get_parameter('max_retry_interval').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value

        # 创建Modbus TCP客户端
        self.client = ModbusTcpClient(
            host=self.plc_ip,
            port=self.plc_port,
        )

        # 传感器数据使用BEST_EFFORT策略，允许少量丢包以降低延迟
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # 命令数据使用RELIABLE策略，确保指令可靠传输
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 创建发布者：PLC数据、线圈状态、寄存器状态
        self.plc_data_pub = self.create_publisher(PlcData, 'plc_data', sensor_qos)
        self.coil_states_pub = self.create_publisher(BoolMultiArray, 'coil_states', sensor_qos)
        self.register_states_pub = self.create_publisher(Int32MultiArray, 'register_states', sensor_qos)

        # 创建订阅者：接收线圈和寄存器写入指令
        self.write_coils_sub = self.create_subscription(
            BoolMultiArray, 'write_coils', self.write_coils_callback, command_qos)
        self.write_registers_sub = self.create_subscription(
            Int32MultiArray, 'write_registers', self.write_registers_callback, command_qos)

        # 创建服务：提供PLC读写接口
        self.read_plc_srv = self.create_service(ReadPlc, 'read_plc', self.read_plc_callback)
        self.write_plc_srv = self.create_service(WritePlc, 'write_plc', self.write_plc_callback)

        # 启动轮询线程，使用条件变量控制轮询间隔
        self._poll_condition = threading.Condition()
        self._poll_interval = poll_rate / 1000.0
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread_active = True
        self._poll_thread.start()

        # 重连退避时间和上次成功轮询时间
        self._retry_backoff = 1.0
        self._last_successful_poll = self.get_clock().now()

        # 启动看门狗定时器，检测通信是否超时
        self._watchdog_timer = self.create_timer(1.0, self._watchdog_check)

        # 初始连接PLC
        self.connected = False
        self.connect()

    # 连接PLC，首次连接时调用
    def connect(self):
        result = self.client.connect()
        self.connected = result
        if self.connected:
            self._retry_backoff = 1.0
            self._last_successful_poll = self.get_clock().now()
            self.get_logger().info(f'Connected to PLC at {self.plc_ip}:{self.plc_port}')
        else:
            self.get_logger().warn(f'Failed to connect to PLC at {self.plc_ip}:{self.plc_port}')

    # 带退避重试的连接方法，连接失败时指数增加重试间隔
    def _connect_with_retry(self):
        result = self.client.connect()
        self.connected = result
        if self.connected:
            self._retry_backoff = 1.0
            self._last_successful_poll = self.get_clock().now()
            self.get_logger().info(f'Reconnected to PLC at {self.plc_ip}:{self.plc_port}')
        else:
            self._retry_backoff = min(self._retry_backoff * 2.0, self._max_retry_interval)
            self.get_logger().warn(
                f'Retry connection failed, next retry in {self._retry_backoff:.1f}s')

    # 断开与PLC的连接
    def disconnect(self):
        self.client.close()
        self.connected = False
        self.get_logger().info('Disconnected from PLC')

    # 轮询循环，在独立线程中运行，周期性读取PLC数据
    def _poll_loop(self):
        while self._poll_thread_active:
            with self._poll_condition:
                self._poll_condition.wait(timeout=self._poll_interval)

            if not self._poll_thread_active:
                break

            # 未连接时等待退避时间后尝试重连
            if not self.connected:
                with self._poll_condition:
                    self._poll_condition.wait(timeout=self._retry_backoff)
                self._connect_with_retry()
                if not self.connected:
                    continue

            self.poll_plc()

    # 看门狗检查，若长时间未成功轮询则重置连接
    def _watchdog_check(self):
        if not self.connected:
            return
        elapsed = (self.get_clock().now() - self._last_successful_poll).nanoseconds / 1e9
        if elapsed > self._watchdog_timeout:
            self.get_logger().warn(
                f'Watchdog: no successful poll for {elapsed:.1f}s, resetting connection')
            self.disconnect()
            self._connect_with_retry()

    # 轮询PLC数据，读取线圈和保持寄存器，发布到ROS2话题
    def poll_plc(self):
        plc_data = PlcData()
        plc_data.device_name = 'plc'
        plc_data.ip_address = self.plc_ip
        plc_data.connected = self.connected
        plc_data.timestamp = self.get_clock().now().to_msg()

        # 读取线圈状态
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
            self.connected = False
            return

        # 读取保持寄存器
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
            self.connected = False
            return

        # 更新成功轮询时间并发布PLC数据
        self._last_successful_poll = self.get_clock().now()
        self.plc_data_pub.publish(plc_data)

    # 线圈写入回调，接收BoolMultiArray消息写入PLC线圈
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
            self.connected = False
        else:
            self.get_logger().info(f'Successfully wrote {len(msg.data)} coils')

    # 寄存器写入回调，接收Int32MultiArray消息写入PLC寄存器
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
            self.connected = False
        else:
            self.get_logger().info(f'Successfully wrote {len(msg.data)} registers')

    # PLC读取服务回调，根据请求的地址和数量读取保持寄存器
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
            self.connected = False

        return response

    # PLC写入服务回调，将请求中的值写入指定地址的保持寄存器
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
            self.connected = False

        return response

    # 销毁节点，停止轮询线程并断开连接
    def destroy(self):
        self._poll_thread_active = False
        with self._poll_condition:
            self._poll_condition.notify_all()
        self._poll_thread.join(timeout=2.0)
        self.disconnect()
        super().destroy_node()


# 节点入口函数
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
