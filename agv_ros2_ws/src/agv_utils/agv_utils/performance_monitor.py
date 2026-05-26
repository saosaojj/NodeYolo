# 性能监控模块，提供函数计时、调用计数和内存追踪功能，支持ROS2话题发布统计信息
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
import threading
import time
import functools
import psutil
import os
from typing import Dict, Any, Callable


# PerformanceMonitor: 性能监控ROS2节点
# 提供函数级别的计时、计数和内存追踪装饰器，定期发布性能统计数据
class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.declare_parameter('monitor_rate', 1.0)
        self.declare_parameter('alert_threshold_ms', 100.0)

        self._monitor_rate = self.get_parameter('monitor_rate').value
        self._alert_threshold_ms = self.get_parameter('alert_threshold_ms').value

        # 性能统计数据字典，键为函数名
        self._stats = {}
        self._stats_lock = threading.Lock()
        # 当前进程对象，用于获取CPU和内存信息
        self._process = psutil.Process(os.getpid())

        # 创建性能统计话题发布者
        self._stats_publisher = self.create_publisher(
            String,
            '/performance_stats',
            10
        )

        # 创建定时器，按指定频率发布统计信息
        self._timer = self.create_timer(
            1.0 / self._monitor_rate,
            self._publish_stats
        )

    # 定时发布性能统计数据，包含系统CPU和内存信息
    def _publish_stats(self):
        import json
        stats = self.get_stats()
        cpu_percent = self._process.cpu_percent()
        memory_info = self._process.memory_info()
        stats['system'] = {
            'cpu_percent': cpu_percent,
            'memory_mb': memory_info.rss / 1024 / 1024
        }
        msg = String()
        msg.data = json.dumps(stats)
        self._stats_publisher.publish(msg)

    # 获取当前性能统计数据的副本
    def get_stats(self) -> Dict[str, Any]:
        with self._stats_lock:
            return dict(self._stats)

    # 重置所有性能统计数据
    def reset_stats(self):
        with self._stats_lock:
            self._stats.clear()

    # 计时装饰器（方法版），记录函数执行耗时，保留最近100次记录
    def timed(self, func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = func(*args, **kwargs)
            elapsed = (time.perf_counter() - start) * 1000.0
            with self._stats_lock:
                if func.__name__ not in self._stats:
                    self._stats[func.__name__] = {'timings': [], 'count': 0}
                self._stats[func.__name__]['timings'].append(elapsed)
                self._stats[func.__name__]['timings'] = self._stats[func.__name__]['timings'][-100:]
                self._stats[func.__name__]['last_ms'] = elapsed
                self._stats[func.__name__]['count'] += 1
            return result
        return wrapper

    # 计数装饰器（方法版），统计函数调用次数
    def counted(self, func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            result = func(*args, **kwargs)
            with self._stats_lock:
                if func.__name__ not in self._stats:
                    self._stats[func.__name__] = {'count': 0}
                self._stats[func.__name__]['count'] += 1
            return result
        return wrapper

    # 内存追踪装饰器（方法版），记录函数执行前后的内存变化
    def memory_tracked(self, func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            mem_before = self._process.memory_info().rss / 1024 / 1024
            result = func(*args, **kwargs)
            mem_after = self._process.memory_info().rss / 1024 / 1024
            mem_delta = mem_after - mem_before
            with self._stats_lock:
                if func.__name__ not in self._stats:
                    self._stats[func.__name__] = {'memory_deltas': []}
                self._stats[func.__name__]['memory_deltas'].append(mem_delta)
                self._stats[func.__name__]['memory_deltas'] = self._stats[func.__name__]['memory_deltas'][-100:]
            return result
        return wrapper


# TimerContext: 计时上下文管理器，用于with语句中测量代码块耗时
class TimerContext:
    def __init__(self, monitor: PerformanceMonitor, name: str):
        self._monitor = monitor
        self._name = name
        self._start = None

    def __enter__(self):
        self._start = time.perf_counter()
        return self

    def __exit__(self, *args):
        elapsed = (time.perf_counter() - self._start) * 1000.0
        with self._monitor._stats_lock:
            if self._name not in self._monitor._stats:
                self._monitor._stats[self._name] = {'timings': [], 'count': 0}
            self._monitor._stats[self._name]['timings'].append(elapsed)
            self._monitor._stats[self._name]['timings'] = self._monitor._stats[self._name]['timings'][-100:]
            self._monitor._stats[self._name]['last_ms'] = elapsed
            self._monitor._stats[self._name]['count'] += 1


# 计时装饰器（函数版），接收monitor实例作为参数
def timed(monitor: PerformanceMonitor):
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = func(*args, **kwargs)
            elapsed = (time.perf_counter() - start) * 1000.0
            with monitor._stats_lock:
                if func.__name__ not in monitor._stats:
                    monitor._stats[func.__name__] = {'timings': [], 'count': 0}
                monitor._stats[func.__name__]['timings'].append(elapsed)
                monitor._stats[func.__name__]['timings'] = monitor._stats[func.__name__]['timings'][-100:]
                monitor._stats[func.__name__]['last_ms'] = elapsed
                monitor._stats[func.__name__]['count'] += 1
            return result
        return wrapper
    return decorator


# 计数装饰器（函数版），接收monitor实例作为参数
def counted(monitor: PerformanceMonitor):
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            result = func(*args, **kwargs)
            with monitor._stats_lock:
                if func.__name__ not in monitor._stats:
                    monitor._stats[func.__name__] = {'count': 0}
                monitor._stats[func.__name__]['count'] += 1
            return result
        return wrapper
    return decorator


# 内存追踪装饰器（函数版），接收monitor实例作为参数
def memory_tracked(monitor: PerformanceMonitor):
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            process = psutil.Process(os.getpid())
            mem_before = process.memory_info().rss / 1024 / 1024
            result = func(*args, **kwargs)
            mem_after = process.memory_info().rss / 1024 / 1024
            mem_delta = mem_after - mem_before
            with monitor._stats_lock:
                if func.__name__ not in monitor._stats:
                    monitor._stats[func.__name__] = {'memory_deltas': []}
                monitor._stats[func.__name__]['memory_deltas'].append(mem_delta)
                monitor._stats[func.__name__]['memory_deltas'] = monitor._stats[func.__name__]['memory_deltas'][-100:]
            return result
        return wrapper
    return decorator


# 节点入口函数
def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
