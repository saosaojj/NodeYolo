# AGV IO模拟器节点，用于在没有真实硬件时模拟IO点的状态
# 数字输入随机切换，模拟输入生成正弦波信号，用于开发和测试
import math
import random
import time

import rclpy
from rclpy.node import Node
from agv_interfaces.msg import IOState


# IO模拟器节点类，生成模拟的数字和模拟IO状态并发布
class IOSimulatorNode(Node):

    # 初始化IO模拟器节点，声明参数并创建定时器
    def __init__(self):
        super().__init__('io_simulator_node')

        # 声明各类IO通道数量和切换频率参数
        self.declare_parameter('num_digital_in', 8)
        self.declare_parameter('num_digital_out', 8)
        self.declare_parameter('num_analog_in', 4)
        self.declare_parameter('num_analog_out', 4)
        self.declare_parameter('toggle_rate', 2.0)

        # 读取参数值
        self.num_digital_in = self.get_parameter('num_digital_in').value
        self.num_digital_out = self.get_parameter('num_digital_out').value
        self.num_analog_in = self.get_parameter('num_analog_in').value
        self.num_analog_out = self.get_parameter('num_analog_out').value
        toggle_rate = self.get_parameter('toggle_rate').value

        # 初始化各类IO通道的状态
        self.digital_in_states = [False] * self.num_digital_in
        self.digital_out_states = [False] * self.num_digital_out
        self.analog_in_values = [0.0] * self.num_analog_in
        self.analog_out_values = [0.0] * self.num_analog_out

        # 记录启动时间，用于生成时变模拟信号
        self.start_time = time.time()

        # 创建IO状态发布者
        self.states_pub = self.create_publisher(IOState, 'io_simulator/states', 10)

        # 创建数字输入切换定时器，按指定频率随机切换数字输入状态
        self.toggle_timer = self.create_timer(1.0 / toggle_rate, self.toggle_callback)
        # 创建状态发布定时器，以10Hz频率发布所有IO状态
        self.publish_timer = self.create_timer(0.1, self.publish_callback)

        self.get_logger().info(
            f'IOSimulatorNode started with '
            f'{self.num_digital_in} DI, {self.num_digital_out} DO, '
            f'{self.num_analog_in} AI, {self.num_analog_out} AO')

    # 数字输入切换回调，以30%概率随机翻转每个数字输入的状态
    def toggle_callback(self):
        for i in range(self.num_digital_in):
            if random.random() < 0.3:
                self.digital_in_states[i] = not self.digital_in_states[i]

    # 状态发布回调，生成模拟输入信号并发布所有IO点状态
    def publish_callback(self):
        elapsed = time.time() - self.start_time

        # 为每个模拟输入生成不同频率和幅度的正弦波信号
        for i in range(self.num_analog_in):
            frequency = 0.1 + i * 0.05
            amplitude = 50.0 + i * 10.0
            offset = 50.0 + i * 5.0
            self.analog_in_values[i] = offset + amplitude * math.sin(
                2.0 * math.pi * frequency * elapsed)

        # 发布所有数字输入状态
        for i in range(self.num_digital_in):
            msg = IOState()
            msg.io_name = f'digital_in_{i}'
            msg.io_type = 'digital_in'
            msg.pin_number = i
            msg.value = float(self.digital_in_states[i])
            msg.state = self.digital_in_states[i]
            self.states_pub.publish(msg)

        # 发布所有数字输出状态
        for i in range(self.num_digital_out):
            msg = IOState()
            msg.io_name = f'digital_out_{i}'
            msg.io_type = 'digital_out'
            msg.pin_number = i
            msg.value = float(self.digital_out_states[i])
            msg.state = self.digital_out_states[i]
            self.states_pub.publish(msg)

        # 发布所有模拟输入状态
        for i in range(self.num_analog_in):
            msg = IOState()
            msg.io_name = f'analog_in_{i}'
            msg.io_type = 'analog_in'
            msg.pin_number = i
            msg.value = self.analog_in_values[i]
            msg.state = abs(self.analog_in_values[i]) > 1e-6
            self.states_pub.publish(msg)

        # 发布所有模拟输出状态
        for i in range(self.num_analog_out):
            msg = IOState()
            msg.io_name = f'analog_out_{i}'
            msg.io_type = 'analog_out'
            msg.pin_number = i
            msg.value = self.analog_out_values[i]
            msg.state = abs(self.analog_out_values[i]) > 1e-6
            self.states_pub.publish(msg)


# 节点主入口函数
def main(args=None):
    rclpy.init(args=args)
    node = IOSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
