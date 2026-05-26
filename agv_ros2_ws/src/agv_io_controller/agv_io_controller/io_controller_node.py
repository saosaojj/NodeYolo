# AGV IO控制器节点，负责管理AGV的数字/模拟输入输出点
# 支持真实GPIO硬件（通过gpiod）和仿真模式，提供去抖动、模拟滤波、看门狗等功能
import time
import yaml
from collections import deque

import rclpy
from rclpy.node import Node
from agv_interfaces.msg import IOState
from agv_interfaces.srv import SetIO
from std_msgs.msg import Bool, Float64


# IO控制器节点类，管理所有IO点的读写、状态发布和变更通知
class IOControllerNode(Node):

    # 初始化IO控制器节点，声明参数、加载配置、初始化GPIO和通信接口
    def __init__(self):
        super().__init__('io_controller_node')

        # 声明ROS2参数
        self.declare_parameter('poll_rate', 200)
        self.declare_parameter('io_config_file', '')
        self.declare_parameter('simulate', True)
        self.declare_parameter('debounce_samples', 3)
        self.declare_parameter('analog_filter_alpha', 0.2)
        self.declare_parameter('watchdog_timeout', 5.0)

        # IO点字典，键为IO名称，值为包含pin/type/value/state的字典
        self.io_points = {}
        # GPIO芯片对象，用于真实硬件操作
        self.gpio_chip = None
        # GPIO线路字典，键为引脚号，值为gpiod线路对象
        self.gpio_lines = {}

        # 读取参数值
        self.simulate = self.get_parameter('simulate').value
        self._debounce_samples = self.get_parameter('debounce_samples').value
        self._analog_filter_alpha = self.get_parameter('analog_filter_alpha').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value

        # 去抖动缓冲区，用于数字输入的去抖动处理
        self._debounce_buffers = {}
        # 模拟滤波后的值，用于模拟输入的低通滤波
        self._analog_filtered = {}
        # IO变更回调函数字典，键为IO名称，值为回调函数列表
        self._change_callbacks = {}
        # 看门狗超时时间字典
        self._watchdog_timers = {}
        # 看门狗最后更新时间字典
        self._watchdog_last_update = {}

        # 加载IO配置文件
        config_file = self.get_parameter('io_config_file').value
        if config_file:
            self.load_config(config_file)

        # 如果非仿真模式，尝试初始化真实GPIO硬件
        if not self.simulate:
            try:
                import gpiod
                self.gpio_chip = gpiod.Chip('gpiochip0')
                self.get_logger().info('Real GPIO hardware initialized via gpiod')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize gpiod, falling back to simulation: {e}')
                self.simulate = True

        # 创建IO状态发布者，发布所有IO点的当前状态
        self.io_states_pub = self.create_publisher(IOState, 'io_states', 10)
        # 创建IO变更发布者，仅在IO状态发生变化时发布
        self.io_changed_pub = self.create_publisher(IOState, 'io_changed', 10)

        # 订阅数字输出设置话题
        self.set_digital_output_sub = self.create_subscription(
            Bool, 'set_digital_output', self.set_digital_output_callback, 10)
        # 订阅模拟输出设置话题
        self.set_analog_output_sub = self.create_subscription(
            Float64, 'set_analog_output', self.set_analog_output_callback, 10)

        # 创建IO设置服务，支持按名称设置IO点
        self.set_io_srv = self.create_service(SetIO, 'set_io', self.set_io_callback)

        # 创建轮询定时器，按指定频率读取所有IO点状态
        poll_rate = self.get_parameter('poll_rate').value
        self.poll_timer = self.create_timer(poll_rate / 1000.0, self.poll_callback)

        # 创建看门狗检查定时器，每秒检查一次IO点是否超时
        self._watchdog_check_timer = self.create_timer(1.0, self._watchdog_check)

        self.get_logger().info(
            f'IOControllerNode started with {len(self.io_points)} IO points '
            f'(simulate={self.simulate})')

    # 从YAML配置文件加载IO点定义
    def load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            params = config.get('io_controller', {}).get('ros__parameters', {})
            io_points_config = params.get('io_points', {})

            # 遍历四类IO点：数字输入、数字输出、模拟输入、模拟输出
            for category in ['digital_inputs', 'digital_outputs', 'analog_inputs', 'analog_outputs']:
                for point in io_points_config.get(category, []):
                    name = point['name']
                    self.io_points[name] = {
                        'pin': point['pin'],
                        'type': point['type'],
                        'value': 0.0,
                        'state': False,
                    }
                    # 为数字输入初始化去抖动缓冲区
                    if point['type'] in ('digital_in',):
                        self._debounce_buffers[name] = deque(maxlen=self._debounce_samples)
                    # 为模拟输入初始化滤波值
                    if point['type'] in ('analog_in',):
                        self._analog_filtered[name] = 0.0

            self.get_logger().info(f'Loaded {len(self.io_points)} IO points from {config_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config file {config_file}: {e}')

    # 注册IO状态变更回调函数，当指定IO点状态变化时触发
    def register_change_callback(self, io_name, callback):
        if io_name not in self._change_callbacks:
            self._change_callbacks[io_name] = []
        self._change_callbacks[io_name].append(callback)

    # 为指定IO点注册看门狗，超时未更新则发出警告
    def register_watchdog(self, io_name, timeout=None):
        if io_name not in self.io_points:
            self.get_logger().warn(f'Cannot register watchdog for unknown IO point: {io_name}')
            return
        wd_timeout = timeout if timeout is not None else self._watchdog_timeout
        self._watchdog_timers[io_name] = wd_timeout
        self._watchdog_last_update[io_name] = time.time()

    # 看门狗检查定时回调，检测IO点是否超时未更新
    def _watchdog_check(self):
        now = time.time()
        for io_name, timeout in self._watchdog_timers.items():
            if io_name in self._watchdog_last_update:
                elapsed = now - self._watchdog_last_update[io_name]
                if elapsed > timeout:
                    self.get_logger().warn(
                        f'Watchdog: IO point {io_name} has not updated for {elapsed:.1f}s')

    # 对数字输入应用去抖动滤波，只有连续多个采样值一致才认为状态改变
    def _apply_debounce(self, name, raw_state):
        if name not in self._debounce_buffers:
            return raw_state
        buf = self._debounce_buffers[name]
        buf.append(raw_state)
        if len(buf) < self._debounce_samples:
            return raw_state
        return all(buf)

    # 对模拟输入应用指数移动平均低通滤波，减少噪声影响
    def _apply_analog_filter(self, name, raw_value):
        if name not in self._analog_filtered:
            self._analog_filtered[name] = raw_value
            return raw_value
        alpha = self._analog_filter_alpha
        filtered = alpha * raw_value + (1.0 - alpha) * self._analog_filtered[name]
        self._analog_filtered[name] = filtered
        return filtered

    # 读取所有IO点的当前状态（仅真实硬件模式有效）
    def read_all_io(self):
        if self.simulate:
            return

        try:
            import gpiod
            for name, point in self.io_points.items():
                io_type = point['type']
                pin = point['pin']

                # 读取数字输入并应用去抖动
                if io_type == 'digital_in':
                    if pin in self.gpio_lines:
                        raw_val = self.gpio_lines[pin].get_value()
                        debounced = self._apply_debounce(name, bool(raw_val))
                        point['state'] = debounced
                        point['value'] = float(debounced)
                # 读取模拟输入并应用低通滤波
                elif io_type == 'analog_in':
                    raw_val = point['value']
                    filtered = self._apply_analog_filter(name, raw_val)
                    point['value'] = filtered
                    point['state'] = abs(filtered) > 1e-6
        except Exception as e:
            self.get_logger().error(f'Failed to read IO: {e}')

    # 向指定IO点写入值，支持数字输出和模拟输出
    def write_io(self, io_name, value):
        if io_name not in self.io_points:
            self.get_logger().warn(f'IO point {io_name} not found')
            return False

        point = self.io_points[io_name]
        io_type = point['type']

        # 输入点不允许写入
        if io_type in ('digital_in', 'analog_in'):
            self.get_logger().warn(f'Cannot write to input point {io_name}')
            return False

        # 更新数字输出状态
        if io_type == 'digital_out':
            point['state'] = bool(value)
            point['value'] = float(bool(value))
        # 更新模拟输出值
        elif io_type == 'analog_out':
            point['value'] = float(value)
            point['state'] = abs(float(value)) > 1e-6

        # 真实硬件模式下通过gpiod写入GPIO
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

        # 更新看门狗时间戳
        if io_name in self._watchdog_last_update:
            self._watchdog_last_update[io_name] = time.time()

        return True

    # 轮询定时回调，读取IO状态并发布，检测状态变更
    def poll_callback(self):
        # 保存上一轮状态用于变更检测
        previous_states = {
            name: {'value': p['value'], 'state': p['state']}
            for name, p in self.io_points.items()
        }

        # 读取所有IO点当前状态
        self.read_all_io()

        # 遍历所有IO点，发布状态和变更通知
        for name, point in self.io_points.items():
            msg = IOState()
            msg.io_name = name
            msg.io_type = point['type']
            msg.pin_number = point['pin']
            msg.value = point['value']
            msg.state = point['state']
            # 发布当前IO状态
            self.io_states_pub.publish(msg)

            # 检测状态是否发生变化
            prev = previous_states.get(name)
            if prev and (prev['value'] != point['value'] or prev['state'] != point['state']):
                # 发布变更状态
                self.io_changed_pub.publish(msg)
                # 触发注册的变更回调
                if name in self._change_callbacks:
                    for cb in self._change_callbacks[name]:
                        try:
                            cb(name, point)
                        except Exception as e:
                            self.get_logger().error(f'IO change callback error for {name}: {e}')

            # 更新看门狗时间戳（仅当状态变化时）
            if name in self._watchdog_last_update:
                if prev and (prev['value'] != point['value'] or prev['state'] != point['state']):
                    self._watchdog_last_update[name] = time.time()

    # 数字输出设置回调，将消息中的值写入第一个数字输出点
    def set_digital_output_callback(self, msg):
        for name, point in self.io_points.items():
            if point['type'] == 'digital_out':
                self.write_io(name, msg.data)
                break

    # 模拟输出设置回调，将消息中的值写入第一个模拟输出点
    def set_analog_output_callback(self, msg):
        for name, point in self.io_points.items():
            if point['type'] == 'analog_out':
                self.write_io(name, msg.data)
                break

    # IO设置服务回调，按名称设置指定IO点的值
    def set_io_callback(self, request, response):
        success = self.write_io(request.io_name, request.value)
        response.success = success
        if success:
            response.message = f'Successfully set {request.io_name}'
        else:
            response.message = f'Failed to set {request.io_name}'
        return response

    # 销毁节点，释放GPIO资源
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


# 节点主入口函数
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
