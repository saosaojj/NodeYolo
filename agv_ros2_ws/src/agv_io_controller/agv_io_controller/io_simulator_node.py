import math
import random
import time

import rclpy
from rclpy.node import Node
from agv_interfaces.msg import IOState


class IOSimulatorNode(Node):

    def __init__(self):
        super().__init__('io_simulator_node')

        self.declare_parameter('num_digital_in', 8)
        self.declare_parameter('num_digital_out', 8)
        self.declare_parameter('num_analog_in', 4)
        self.declare_parameter('num_analog_out', 4)
        self.declare_parameter('toggle_rate', 2.0)

        self.num_digital_in = self.get_parameter('num_digital_in').value
        self.num_digital_out = self.get_parameter('num_digital_out').value
        self.num_analog_in = self.get_parameter('num_analog_in').value
        self.num_analog_out = self.get_parameter('num_analog_out').value
        toggle_rate = self.get_parameter('toggle_rate').value

        self.digital_in_states = [False] * self.num_digital_in
        self.digital_out_states = [False] * self.num_digital_out
        self.analog_in_values = [0.0] * self.num_analog_in
        self.analog_out_values = [0.0] * self.num_analog_out

        self.start_time = time.time()

        self.states_pub = self.create_publisher(IOState, 'io_simulator/states', 10)

        self.toggle_timer = self.create_timer(1.0 / toggle_rate, self.toggle_callback)
        self.publish_timer = self.create_timer(0.1, self.publish_callback)

        self.get_logger().info(
            f'IOSimulatorNode started with '
            f'{self.num_digital_in} DI, {self.num_digital_out} DO, '
            f'{self.num_analog_in} AI, {self.num_analog_out} AO')

    def toggle_callback(self):
        for i in range(self.num_digital_in):
            if random.random() < 0.3:
                self.digital_in_states[i] = not self.digital_in_states[i]

    def publish_callback(self):
        elapsed = time.time() - self.start_time

        for i in range(self.num_analog_in):
            frequency = 0.1 + i * 0.05
            amplitude = 50.0 + i * 10.0
            offset = 50.0 + i * 5.0
            self.analog_in_values[i] = offset + amplitude * math.sin(
                2.0 * math.pi * frequency * elapsed)

        for i in range(self.num_digital_in):
            msg = IOState()
            msg.io_name = f'digital_in_{i}'
            msg.io_type = 'digital_in'
            msg.pin_number = i
            msg.value = float(self.digital_in_states[i])
            msg.state = self.digital_in_states[i]
            self.states_pub.publish(msg)

        for i in range(self.num_digital_out):
            msg = IOState()
            msg.io_name = f'digital_out_{i}'
            msg.io_type = 'digital_out'
            msg.pin_number = i
            msg.value = float(self.digital_out_states[i])
            msg.state = self.digital_out_states[i]
            self.states_pub.publish(msg)

        for i in range(self.num_analog_in):
            msg = IOState()
            msg.io_name = f'analog_in_{i}'
            msg.io_type = 'analog_in'
            msg.pin_number = i
            msg.value = self.analog_in_values[i]
            msg.state = abs(self.analog_in_values[i]) > 1e-6
            self.states_pub.publish(msg)

        for i in range(self.num_analog_out):
            msg = IOState()
            msg.io_name = f'analog_out_{i}'
            msg.io_type = 'analog_out'
            msg.pin_number = i
            msg.value = self.analog_out_values[i]
            msg.state = abs(self.analog_out_values[i]) > 1e-6
            self.states_pub.publish(msg)


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
