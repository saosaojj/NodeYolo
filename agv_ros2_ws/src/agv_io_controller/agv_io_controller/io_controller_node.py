import time
import yaml
from collections import deque

import rclpy
from rclpy.node import Node
from agv_interfaces.msg import IOState
from agv_interfaces.srv import SetIO
from std_msgs.msg import Bool, Float64


class IOControllerNode(Node):

    def __init__(self):
        super().__init__('io_controller_node')

        self.declare_parameter('poll_rate', 100)
        self.declare_parameter('io_config_file', '')
        self.declare_parameter('simulate', True)
        self.declare_parameter('debounce_samples', 3)
        self.declare_parameter('analog_filter_alpha', 0.2)
        self.declare_parameter('watchdog_timeout', 5.0)

        self.io_points = {}
        self.gpio_chip = None
        self.gpio_lines = {}

        self.simulate = self.get_parameter('simulate').value
        self._debounce_samples = self.get_parameter('debounce_samples').value
        self._analog_filter_alpha = self.get_parameter('analog_filter_alpha').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value

        self._debounce_buffers = {}
        self._analog_filtered = {}
        self._change_callbacks = {}
        self._watchdog_timers = {}
        self._watchdog_last_update = {}

        config_file = self.get_parameter('io_config_file').value
        if config_file:
            self.load_config(config_file)

        if not self.simulate:
            try:
                import gpiod
                self.gpio_chip = gpiod.Chip('gpiochip0')
                self.get_logger().info('Real GPIO hardware initialized via gpiod')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize gpiod, falling back to simulation: {e}')
                self.simulate = True

        self.io_states_pub = self.create_publisher(IOState, 'io_states', 10)
        self.io_changed_pub = self.create_publisher(IOState, 'io_changed', 10)

        self.set_digital_output_sub = self.create_subscription(
            Bool, 'set_digital_output', self.set_digital_output_callback, 10)
        self.set_analog_output_sub = self.create_subscription(
            Float64, 'set_analog_output', self.set_analog_output_callback, 10)

        self.set_io_srv = self.create_service(SetIO, 'set_io', self.set_io_callback)

        poll_rate = self.get_parameter('poll_rate').value
        self.poll_timer = self.create_timer(poll_rate / 1000.0, self.poll_callback)

        self._watchdog_check_timer = self.create_timer(1.0, self._watchdog_check)

        self.get_logger().info(
            f'IOControllerNode started with {len(self.io_points)} IO points '
            f'(simulate={self.simulate})')

    def load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            params = config.get('io_controller', {}).get('ros__parameters', {})
            io_points_config = params.get('io_points', {})

            for category in ['digital_inputs', 'digital_outputs', 'analog_inputs', 'analog_outputs']:
                for point in io_points_config.get(category, []):
                    name = point['name']
                    self.io_points[name] = {
                        'pin': point['pin'],
                        'type': point['type'],
                        'value': 0.0,
                        'state': False,
                    }
                    if point['type'] in ('digital_in',):
                        self._debounce_buffers[name] = deque(maxlen=self._debounce_samples)
                    if point['type'] in ('analog_in',):
                        self._analog_filtered[name] = 0.0

            self.get_logger().info(f'Loaded {len(self.io_points)} IO points from {config_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config file {config_file}: {e}')

    def register_change_callback(self, io_name, callback):
        if io_name not in self._change_callbacks:
            self._change_callbacks[io_name] = []
        self._change_callbacks[io_name].append(callback)

    def register_watchdog(self, io_name, timeout=None):
        if io_name not in self.io_points:
            self.get_logger().warn(f'Cannot register watchdog for unknown IO point: {io_name}')
            return
        wd_timeout = timeout if timeout is not None else self._watchdog_timeout
        self._watchdog_timers[io_name] = wd_timeout
        self._watchdog_last_update[io_name] = time.time()

    def _watchdog_check(self):
        now = time.time()
        for io_name, timeout in self._watchdog_timers.items():
            if io_name in self._watchdog_last_update:
                elapsed = now - self._watchdog_last_update[io_name]
                if elapsed > timeout:
                    self.get_logger().warn(
                        f'Watchdog: IO point {io_name} has not updated for {elapsed:.1f}s')

    def _apply_debounce(self, name, raw_state):
        if name not in self._debounce_buffers:
            return raw_state
        buf = self._debounce_buffers[name]
        buf.append(raw_state)
        if len(buf) < self._debounce_samples:
            return raw_state
        return all(buf)

    def _apply_analog_filter(self, name, raw_value):
        if name not in self._analog_filtered:
            self._analog_filtered[name] = raw_value
            return raw_value
        alpha = self._analog_filter_alpha
        filtered = alpha * raw_value + (1.0 - alpha) * self._analog_filtered[name]
        self._analog_filtered[name] = filtered
        return filtered

    def read_all_io(self):
        if self.simulate:
            return

        try:
            import gpiod
            for name, point in self.io_points.items():
                io_type = point['type']
                pin = point['pin']

                if io_type == 'digital_in':
                    if pin in self.gpio_lines:
                        raw_val = self.gpio_lines[pin].get_value()
                        debounced = self._apply_debounce(name, bool(raw_val))
                        point['state'] = debounced
                        point['value'] = float(debounced)
                elif io_type == 'analog_in':
                    raw_val = point['value']
                    filtered = self._apply_analog_filter(name, raw_val)
                    point['value'] = filtered
                    point['state'] = abs(filtered) > 1e-6
        except Exception as e:
            self.get_logger().error(f'Failed to read IO: {e}')

    def write_io(self, io_name, value):
        if io_name not in self.io_points:
            self.get_logger().warn(f'IO point {io_name} not found')
            return False

        point = self.io_points[io_name]
        io_type = point['type']

        if io_type in ('digital_in', 'analog_in'):
            self.get_logger().warn(f'Cannot write to input point {io_name}')
            return False

        if io_type == 'digital_out':
            point['state'] = bool(value)
            point['value'] = float(bool(value))
        elif io_type == 'analog_out':
            point['value'] = float(value)
            point['state'] = abs(float(value)) > 1e-6

        if not self.simulate:
            try:
                import gpiod
                pin = point['pin']
                if io_type == 'digital_out':
                    if pin not in self.gpio_lines:
                        self.gpio_lines[pin] = self.gpio_chip.get_line(pin)
                        self.gpio_lines[pin].request(
                            consumer='agv_io_controller',
                            type=gpiod.LINE_REQ_DIR_OUT)
                    self.gpio_lines[pin].set_value(int(point['state']))
            except Exception as e:
                self.get_logger().error(f'Failed to write IO {io_name}: {e}')
                return False

        if io_name in self._watchdog_last_update:
            self._watchdog_last_update[io_name] = time.time()

        return True

    def poll_callback(self):
        previous_states = {
            name: {'value': p['value'], 'state': p['state']}
            for name, p in self.io_points.items()
        }

        self.read_all_io()

        for name, point in self.io_points.items():
            msg = IOState()
            msg.io_name = name
            msg.io_type = point['type']
            msg.pin_number = point['pin']
            msg.value = point['value']
            msg.state = point['state']
            self.io_states_pub.publish(msg)

            prev = previous_states.get(name)
            if prev and (prev['value'] != point['value'] or prev['state'] != point['state']):
                self.io_changed_pub.publish(msg)
                if name in self._change_callbacks:
                    for cb in self._change_callbacks[name]:
                        try:
                            cb(name, point)
                        except Exception as e:
                            self.get_logger().error(f'IO change callback error for {name}: {e}')

            if name in self._watchdog_last_update:
                if prev and (prev['value'] != point['value'] or prev['state'] != point['state']):
                    self._watchdog_last_update[name] = time.time()

    def set_digital_output_callback(self, msg):
        for name, point in self.io_points.items():
            if point['type'] == 'digital_out':
                self.write_io(name, msg.data)
                break

    def set_analog_output_callback(self, msg):
        for name, point in self.io_points.items():
            if point['type'] == 'analog_out':
                self.write_io(name, msg.data)
                break

    def set_io_callback(self, request, response):
        success = self.write_io(request.io_name, request.value)
        response.success = success
        if success:
            response.message = f'Successfully set {request.io_name}'
        else:
            response.message = f'Failed to set {request.io_name}'
        return response

    def destroy(self):
        if self.gpio_chip is not None:
            for line in self.gpio_lines.values():
                try:
                    line.release()
                except Exception:
                    pass
            self.gpio_lines.clear()
            self.gpio_chip.close()
            self.gpio_chip = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IOControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
