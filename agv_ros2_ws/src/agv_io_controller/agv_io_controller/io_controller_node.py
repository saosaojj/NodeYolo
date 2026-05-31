import time
import json
import yaml
import copy
from collections import deque

import rclpy
from rclpy.node import Node
from agv_interfaces.msg import IOState
from agv_interfaces.srv import SetIO, GetConfig, SetConfig
from std_msgs.msg import Bool, Float64, String, Int32
from std_srvs.srv import Trigger


# 安全状态定义：紧急停止时所有输出引脚的默认安全值
SAFE_OUTPUT_STATES = {
    'digital_out': False,
    'analog_out': 0.0,
}


class IOControllerNode(Node):

    def __init__(self):
        super().__init__('io_controller_node')

        self.declare_parameter('poll_rate', 100)
        self.declare_parameter('io_config_file', '')
        self.declare_parameter('simulate', True)
        self.declare_parameter('debounce_samples', 3)
        self.declare_parameter('analog_filter_alpha', 0.2)
        self.declare_parameter('watchdog_timeout', 5.0)
        # 新增参数：IO历史记录最大条目数
        self.declare_parameter('io_history_size', 100)
        # 新增参数：PWM默认频率(Hz)
        self.declare_parameter('pwm_default_frequency', 1000.0)
        # 新增参数：信号质量评估窗口大小
        self.declare_parameter('signal_quality_window', 1000)
        # 新增参数：信号噪声阈值（抖动次数占比超过此值视为不可靠）
        self.declare_parameter('signal_noise_threshold', 0.3)

        self.io_points = {}
        self.gpio_chip = None
        self.gpio_lines = {}

        self.simulate = self.get_parameter('simulate').value
        self._debounce_samples = self.get_parameter('debounce_samples').value
        self._analog_filter_alpha = self.get_parameter('analog_filter_alpha').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self._io_history_size = self.get_parameter('io_history_size').value
        self._pwm_default_freq = self.get_parameter('pwm_default_frequency').value
        self._signal_quality_window = self.get_parameter('signal_quality_window').value
        self._signal_noise_threshold = self.get_parameter('signal_noise_threshold').value

        self._debounce_buffers = {}
        self._analog_filtered = {}
        self._change_callbacks = {}
        self._watchdog_timers = {}
        self._watchdog_last_update = {}

        # 新增：IO信号历史记录 {io_name: deque of {timestamp, value, state}}
        self._io_history = {}
        # 新增：IO告警配置 {io_name: {'type': 'high'|'low'|'threshold', 'threshold': float, ...}}
        self._alert_configs = {}
        # 新增：PWM输出配置 {io_name: {'frequency': float, 'duty_cycle': float}}
        self._pwm_configs = {}
        # 新增：信号质量统计 {io_name: {'total_samples': int, 'state_changes': int, 'last_eval_time': float}}
        self._signal_quality_stats = {}
        # 新增：紧急状态标志
        self._emergency_active = False
        # 新增：紧急停止前的输出状态快照
        self._pre_emergency_states = {}
        # 新增：IO引脚配置（用于动态重配置）
        self._io_pin_config = {}
        # 新增：安全输出状态配置
        self._safe_output_states = copy.deepcopy(SAFE_OUTPUT_STATES)

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

        # 原有发布者
        self.io_states_pub = self.create_publisher(IOState, 'io_states', 10)
        self.io_changed_pub = self.create_publisher(IOState, 'io_changed', 10)

        # 新增发布者：IO告警
        self.io_alert_pub = self.create_publisher(String, 'io_alert', 10)
        # 新增发布者：PWM命令
        self.pwm_command_pub = self.create_publisher(String, 'pwm_command', 10)
        # 新增发布者：信号质量报告
        self.signal_quality_pub = self.create_publisher(String, 'signal_quality', 10)

        # 原有订阅
        self.set_digital_output_sub = self.create_subscription(
            Bool, 'set_digital_output', self.set_digital_output_callback, 10)
        self.set_analog_output_sub = self.create_subscription(
            Float64, 'set_analog_output', self.set_analog_output_callback, 10)

        # 原有服务
        self.set_io_srv = self.create_service(SetIO, 'set_io', self.set_io_callback)

        # 新增服务：获取IO历史记录
        self.get_io_history_srv = self.create_service(GetConfig, 'get_io_history', self.get_io_history_callback)
        # 新增服务：IO组操作（批量设置/获取）
        self.io_group_srv = self.create_service(SetConfig, 'io_group_operation', self.io_group_operation_callback)
        # 新增服务：紧急IO覆盖
        self.emergency_override_srv = self.create_service(Trigger, 'emergency_override', self.emergency_override_callback)
        # 新增服务：紧急恢复
        self.emergency_restore_srv = self.create_service(Trigger, 'emergency_restore', self.emergency_restore_callback)
        # 新增服务：动态重配置IO引脚
        self.reconfig_io_srv = self.create_service(SetConfig, 'reconfig_io', self.reconfig_io_callback)
        # 新增服务：配置IO告警
        self.config_alert_srv = self.create_service(SetConfig, 'config_io_alert', self.config_alert_callback)
        # 新增服务：配置PWM
        self.config_pwm_srv = self.create_service(SetConfig, 'config_pwm', self.config_pwm_callback)

        # 原有定时器
        poll_rate = self.get_parameter('poll_rate').value
        self.poll_timer = self.create_timer(poll_rate / 1000.0, self.poll_callback)

        self._watchdog_check_timer = self.create_timer(1.0, self._watchdog_check)

        # 新增定时器：信号质量评估
        self._signal_quality_timer = self.create_timer(5.0, self._signal_quality_check)

        self.get_logger().info(
            f'IOControllerNode已启动，共 {len(self.io_points)} 个IO点 '
            f'(simulate={self.simulate})，支持告警、历史记录、PWM、组操作、紧急覆盖')

    # ==================== 配置加载（增强） ====================

    def load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            params = config.get('io_controller', {}).get('ros__parameters', {})
            io_points_config = params.get('io_points', {})

            # 保存原始引脚配置
            self._io_pin_config = copy.deepcopy(io_points_config)

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

                    # 初始化IO历史记录
                    self._io_history[name] = deque(maxlen=self._io_history_size)
                    # 初始化信号质量统计
                    self._signal_quality_stats[name] = {
                        'total_samples': 0,
                        'state_changes': 0,
                        'last_eval_time': time.time(),
                        'noisy': False
                    }

                    # 加载告警配置（如果存在）
                    if 'alert' in point:
                        self._alert_configs[name] = point['alert']

                    # 加载PWM配置（如果存在）
                    if point['type'] in ('digital_out',) and 'pwm' in point:
                        self._pwm_configs[name] = point['pwm']

            self.get_logger().info(f'从 {config_file} 加载了 {len(self.io_points)} 个IO点')
        except Exception as e:
            self.get_logger().error(f'加载配置文件 {config_file} 失败: {e}')

    # ==================== 原有功能（保持不变） ====================

    def register_change_callback(self, io_name, callback):
        if io_name not in self._change_callbacks:
            self._change_callbacks[io_name] = []
        self._change_callbacks[io_name].append(callback)

    def register_watchdog(self, io_name, timeout=None):
        if io_name not in self.io_points:
            self.get_logger().warn(f'无法为未知IO点注册看门狗: {io_name}')
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
                        f'看门狗: IO点 {io_name} 已超过 {elapsed:.1f}s 未更新')

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
            self.get_logger().error(f'读取IO失败: {e}')

    def write_io(self, io_name, value):
        if io_name not in self.io_points:
            self.get_logger().warn(f'IO点 {io_name} 未找到')
            return False

        # 紧急状态下禁止写入
        if self._emergency_active:
            self.get_logger().warn(f'紧急状态激活中，禁止写入IO点 {io_name}')
            return False

        point = self.io_points[io_name]
        io_type = point['type']

        if io_type in ('digital_in', 'analog_in'):
            self.get_logger().warn(f'无法写入输入点 {io_name}')
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
                self.get_logger().error(f'写入IO {io_name} 失败: {e}')
                return False

        if io_name in self._watchdog_last_update:
            self._watchdog_last_update[io_name] = time.time()

        return True

    # ==================== 轮询回调（增强） ====================

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
            changed = prev and (prev['value'] != point['value'] or prev['state'] != point['state'])

            if changed:
                self.io_changed_pub.publish(msg)

                # 记录IO历史
                self._record_io_history(name, point)

                # 更新信号质量统计
                stats = self._signal_quality_stats.get(name)
                if stats:
                    stats['state_changes'] += 1

                # 检查告警条件
                self._check_alert(name, point, prev)

                if name in self._change_callbacks:
                    for cb in self._change_callbacks[name]:
                        try:
                            cb(name, point)
                        except Exception as e:
                            self.get_logger().error(f'IO变更回调错误 {name}: {e}')

            # 更新信号质量采样计数
            stats = self._signal_quality_stats.get(name)
            if stats:
                stats['total_samples'] += 1

            if name in self._watchdog_last_update:
                if changed:
                    self._watchdog_last_update[name] = time.time()

    # ==================== IO信号历史记录 ====================

    def _record_io_history(self, name, point):
        """记录IO状态变更到历史"""
        if name not in self._io_history:
            self._io_history[name] = deque(maxlen=self._io_history_size)
        self._io_history[name].append({
            'timestamp': time.time(),
            'value': point['value'],
            'state': point['state'],
        })

    def get_io_history_callback(self, request, response):
        """获取IO历史记录服务回调"""
        try:
            io_name = request.config_key
            if io_name == 'all':
                all_history = {}
                for name, history in self._io_history.items():
                    all_history[name] = list(history)
                response.config_value = json.dumps(all_history, ensure_ascii=False, default=str)
            elif io_name in self._io_history:
                response.config_value = json.dumps(
                    list(self._io_history[io_name]), ensure_ascii=False, default=str)
            else:
                response.config_value = ''
                response.success = False
                response.message = f'IO点 {io_name} 无历史记录'
                return response
            response.success = True
            response.message = 'Success'
        except Exception as e:
            response.success = False
            response.config_value = ''
            response.message = str(e)
        return response

    # ==================== IO告警监控 ====================

    def _check_alert(self, name, point, prev):
        """检查IO信号是否触发告警"""
        if name not in self._alert_configs:
            return

        alert_cfg = self._alert_configs[name]
        alert_type = alert_cfg.get('type', '')
        triggered = False
        alert_msg = ''

        if alert_type == 'high' and point['type'] in ('digital_in', 'digital_out'):
            if point['state'] is True and prev and prev['state'] is False:
                triggered = True
                alert_msg = f'IO点 {name} 触发高电平告警'
        elif alert_type == 'low' and point['type'] in ('digital_in', 'digital_out'):
            if point['state'] is False and prev and prev['state'] is True:
                triggered = True
                alert_msg = f'IO点 {name} 触发低电平告警'
        elif alert_type == 'threshold' and point['type'] in ('analog_in', 'analog_out'):
            threshold = alert_cfg.get('threshold', 0.0)
            direction = alert_cfg.get('direction', 'above')
            if direction == 'above' and point['value'] > threshold:
                triggered = True
                alert_msg = f'IO点 {name} 值 {point["value"]:.3f} 超过阈值 {threshold}'
            elif direction == 'below' and point['value'] < threshold:
                triggered = True
                alert_msg = f'IO点 {name} 值 {point["value"]:.3f} 低于阈值 {threshold}'
            elif direction == 'range':
                low = alert_cfg.get('low', 0.0)
                high = alert_cfg.get('high', 0.0)
                if point['value'] < low or point['value'] > high:
                    triggered = True
                    alert_msg = f'IO点 {name} 值 {point["value"]:.3f} 超出范围 [{low}, {high}]'

        if triggered:
            alert_data = {
                'io_name': name,
                'alert_type': alert_type,
                'message': alert_msg,
                'value': point['value'],
                'state': point['state'],
                'timestamp': time.time(),
                'config': alert_cfg
            }
            msg = String()
            msg.data = json.dumps(alert_data, ensure_ascii=False, default=str)
            self.io_alert_pub.publish(msg)
            self.get_logger().warn(alert_msg)

    def config_alert_callback(self, request, response):
        """配置IO告警服务回调"""
        try:
            data = json.loads(request.config_value)
            io_name = request.config_key

            if io_name not in self.io_points:
                response.success = False
                response.message = f'IO点 {io_name} 不存在'
                return response

            self._alert_configs[io_name] = data
            response.success = True
            response.message = f'IO点 {io_name} 告警配置已更新'
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'JSON解析失败: {e}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # ==================== PWM输出支持 ====================

    def config_pwm_callback(self, request, response):
        """配置PWM输出服务回调"""
        try:
            data = json.loads(request.config_value)
            io_name = request.config_key

            if io_name not in self.io_points:
                response.success = False
                response.message = f'IO点 {io_name} 不存在'
                return response

            point = self.io_points[io_name]
            if point['type'] != 'digital_out':
                response.success = False
                response.message = f'IO点 {io_name} 不是数字输出，不支持PWM'
                return response

            frequency = data.get('frequency', self._pwm_default_freq)
            duty_cycle = data.get('duty_cycle', 0.0)

            if frequency <= 0:
                response.success = False
                response.message = 'PWM频率必须大于0'
                return response
            if duty_cycle < 0.0 or duty_cycle > 100.0:
                response.success = False
                response.message = 'PWM占空比必须在0-100之间'
                return response

            self._pwm_configs[io_name] = {
                'frequency': frequency,
                'duty_cycle': duty_cycle,
                'enabled': data.get('enabled', True)
            }

            # 发布PWM命令
            self._publish_pwm_command(io_name)

            response.success = True
            response.message = f'IO点 {io_name} PWM配置已更新: freq={frequency}Hz, duty={duty_cycle}%'
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'JSON解析失败: {e}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _publish_pwm_command(self, io_name):
        """发布PWM命令"""
        if io_name not in self._pwm_configs:
            return
        pwm_cfg = self._pwm_configs[io_name]
        if not pwm_cfg.get('enabled', True):
            return
        point = self.io_points.get(io_name)
        if not point:
            return

        pwm_data = {
            'io_name': io_name,
            'pin': point['pin'],
            'frequency': pwm_cfg['frequency'],
            'duty_cycle': pwm_cfg['duty_cycle'],
            'enabled': pwm_cfg.get('enabled', True),
            'timestamp': time.time()
        }
        msg = String()
        msg.data = json.dumps(pwm_data, ensure_ascii=False)
        self.pwm_command_pub.publish(msg)

    # ==================== IO组操作 ====================

    def io_group_operation_callback(self, request, response):
        """IO组操作服务回调，支持批量设置/获取多个IO点"""
        try:
            data = json.loads(request.config_value)
            operation = data.get('operation', 'get')

            if operation == 'get':
                # 批量获取IO状态
                names = data.get('io_names', [])
                if not names:
                    names = list(self.io_points.keys())
                result = {}
                for name in names:
                    if name in self.io_points:
                        point = self.io_points[name]
                        result[name] = {
                            'pin': point['pin'],
                            'type': point['type'],
                            'value': point['value'],
                            'state': point['state']
                        }
                response.success = True
                response.message = json.dumps(result, ensure_ascii=False, default=str)

            elif operation == 'set':
                # 批量设置IO状态
                io_values = data.get('io_values', {})
                results = {}
                for name, value in io_values.items():
                    success = self.write_io(name, value)
                    results[name] = {'success': success, 'value': value}
                response.success = all(r['success'] for r in results.values())
                response.message = json.dumps(results, ensure_ascii=False, default=str)

            else:
                response.success = False
                response.message = f'不支持的操作类型: {operation}'

        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'JSON解析失败: {e}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # ==================== 信号质量监控 ====================

    def _signal_quality_check(self):
        """定期评估信号质量，检测噪声输入"""
        now = time.time()
        quality_report = {}

        for name, stats in self._signal_quality_stats.items():
            elapsed = now - stats['last_eval_time']
            if elapsed < 1.0:
                continue

            total = stats['total_samples']
            changes = stats['state_changes']

            # 计算状态变化率
            change_rate = changes / max(total, 1)

            # 判断是否为噪声信号
            is_noisy = change_rate > self._signal_noise_threshold and total > self._signal_quality_window * 0.1
            was_noisy = stats.get('noisy', False)

            stats['noisy'] = is_noisy
            stats['change_rate'] = change_rate

            if is_noisy != was_noisy:
                if is_noisy:
                    self.get_logger().warn(
                        f'信号质量: IO点 {name} 检测到噪声信号 '
                        f'(变化率: {change_rate:.3f}, 阈值: {self._signal_noise_threshold})')
                else:
                    self.get_logger().info(f'信号质量: IO点 {name} 噪声信号已恢复正常')

            quality_report[name] = {
                'total_samples': total,
                'state_changes': changes,
                'change_rate': change_rate,
                'noisy': is_noisy
            }

            # 重置统计周期
            stats['total_samples'] = 0
            stats['state_changes'] = 0
            stats['last_eval_time'] = now

        if quality_report:
            msg = String()
            msg.data = json.dumps(quality_report, ensure_ascii=False)
            self.signal_quality_pub.publish(msg)

    # ==================== 紧急IO覆盖 ====================

    def emergency_override_callback(self, request, response):
        """紧急IO覆盖：强制所有输出到安全状态"""
        try:
            if self._emergency_active:
                response.success = True
                response.message = '紧急状态已经处于激活状态'
                return response

            # 保存当前输出状态快照
            self._pre_emergency_states = {}
            for name, point in self.io_points.items():
                if point['type'] in ('digital_out', 'analog_out'):
                    self._pre_emergency_states[name] = {
                        'value': point['value'],
                        'state': point['state']
                    }

            # 强制所有输出到安全状态
            self._emergency_active = True
            for name, point in self.io_points.items():
                if point['type'] == 'digital_out':
                    safe_value = self._safe_output_states.get('digital_out', False)
                    point['state'] = safe_value
                    point['value'] = float(safe_value)
                    if not self.simulate:
                        try:
                            import gpiod
                            pin = point['pin']
                            if pin in self.gpio_lines:
                                self.gpio_lines[pin].set_value(int(safe_value))
                        except Exception as e:
                            self.get_logger().error(f'紧急设置IO {name} 失败: {e}')
                elif point['type'] == 'analog_out':
                    safe_value = self._safe_output_states.get('analog_out', 0.0)
                    point['value'] = safe_value
                    point['state'] = abs(safe_value) > 1e-6
                    # 模拟模式下analog_out无需硬件操作

            # 禁用所有PWM
            for name, pwm_cfg in self._pwm_configs.items():
                pwm_cfg['enabled'] = False
                self._publish_pwm_command(name)

            self.get_logger().error('紧急IO覆盖已激活！所有输出已切换到安全状态')
            response.success = True
            response.message = '紧急IO覆盖已激活，所有输出已切换到安全状态'
        except Exception as e:
            response.success = False
            response.message = f'紧急覆盖失败: {str(e)}'
        return response

    def emergency_restore_callback(self, request, response):
        """紧急恢复：从紧急状态恢复到正常操作"""
        try:
            if not self._emergency_active:
                response.success = True
                response.message = '未处于紧急状态'
                return response

            self._emergency_active = False

            # 恢复紧急停止前的输出状态
            for name, saved_state in self._pre_emergency_states.items():
                if name in self.io_points:
                    point = self.io_points[name]
                    point['value'] = saved_state['value']
                    point['state'] = saved_state['state']
                    if not self.simulate:
                        try:
                            import gpiod
                            if point['type'] == 'digital_out':
                                pin = point['pin']
                                if pin in self.gpio_lines:
                                    self.gpio_lines[pin].set_value(int(point['state']))
                        except Exception as e:
                            self.get_logger().error(f'恢复IO {name} 失败: {e}')

            self._pre_emergency_states.clear()
            self.get_logger().info('紧急状态已恢复，输出已还原')
            response.success = True
            response.message = '紧急状态已恢复，输出已还原到紧急前状态'
        except Exception as e:
            response.success = False
            response.message = f'紧急恢复失败: {str(e)}'
        return response

    # ==================== 动态引脚重配置 ====================

    def reconfig_io_callback(self, request, response):
        """动态重配置IO引脚服务回调"""
        try:
            data = json.loads(request.config_value)
            operation = data.get('operation', 'update')

            if operation == 'update':
                # 更新现有IO点配置
                io_name = request.config_key
                if io_name not in self.io_points:
                    response.success = False
                    response.message = f'IO点 {io_name} 不存在'
                    return response

                point = self.io_points[io_name]
                if 'pin' in data:
                    new_pin = data['pin']
                    # 释放旧GPIO线
                    if not self.simulate and point['pin'] in self.gpio_lines:
                        try:
                            self.gpio_lines[point['pin']].release()
                            del self.gpio_lines[point['pin']]
                        except Exception:
                            pass
                    point['pin'] = new_pin
                if 'type' in data:
                    point['type'] = data['type']

                # 更新去抖缓冲区
                if point['type'] in ('digital_in',) and io_name not in self._debounce_buffers:
                    self._debounce_buffers[io_name] = deque(maxlen=self._debounce_samples)
                if point['type'] in ('analog_in',) and io_name not in self._analog_filtered:
                    self._analog_filtered[io_name] = 0.0

                response.success = True
                response.message = f'IO点 {io_name} 配置已更新'

            elif operation == 'add':
                # 添加新的IO点
                io_name = request.config_key
                if io_name in self.io_points:
                    response.success = False
                    response.message = f'IO点 {io_name} 已存在'
                    return response

                self.io_points[io_name] = {
                    'pin': data.get('pin', 0),
                    'type': data.get('type', 'digital_in'),
                    'value': 0.0,
                    'state': False,
                }
                io_type = data.get('type', 'digital_in')
                if io_type in ('digital_in',):
                    self._debounce_buffers[io_name] = deque(maxlen=self._debounce_samples)
                if io_type in ('analog_in',):
                    self._analog_filtered[io_name] = 0.0
                self._io_history[io_name] = deque(maxlen=self._io_history_size)
                self._signal_quality_stats[io_name] = {
                    'total_samples': 0,
                    'state_changes': 0,
                    'last_eval_time': time.time(),
                    'noisy': False
                }

                response.success = True
                response.message = f'IO点 {io_name} 已添加'

            elif operation == 'remove':
                # 移除IO点
                io_name = request.config_key
                if io_name not in self.io_points:
                    response.success = False
                    response.message = f'IO点 {io_name} 不存在'
                    return response

                point = self.io_points[io_name]
                if not self.simulate and point['pin'] in self.gpio_lines:
                    try:
                        self.gpio_lines[point['pin']].release()
                        del self.gpio_lines[point['pin']]
                    except Exception:
                        pass

                del self.io_points[io_name]
                self._debounce_buffers.pop(io_name, None)
                self._analog_filtered.pop(io_name, None)
                self._io_history.pop(io_name, None)
                self._signal_quality_stats.pop(io_name, None)
                self._alert_configs.pop(io_name, None)
                self._pwm_configs.pop(io_name, None)

                response.success = True
                response.message = f'IO点 {io_name} 已移除'

            else:
                response.success = False
                response.message = f'不支持的操作: {operation}'

        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'JSON解析失败: {e}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # ==================== 原有订阅回调 ====================

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
            response.message = f'成功设置 {request.io_name}'
        else:
            response.message = f'设置 {request.io_name} 失败'
        return response

    # ==================== 清理 ====================

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
