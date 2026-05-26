# ROS2辅助工具模块，提供消息缓冲、服务代理池、话题限流和节点运行等通用功能
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
from typing import Any, Optional, TypeVar, Generic
from rclpy.service import Service
from rclpy.client import Client

import rclpy.qos


T = TypeVar('T')


# MessageBuffer: 线程安全的消息缓冲器，支持等待消息到达
class MessageBuffer(Generic[T]):
    def __init__(self):
        self._buffer = None
        self._lock = threading.Lock()
        # 条件变量，用于等待消息到达的通知
        self._data_available = threading.Condition(self._lock)

    # 设置消息并通知等待线程
    def set(self, message: T):
        with self._data_available:
            self._buffer = message
            self._data_available.notify_all()

    # 获取缓冲的消息（带条件变量锁）
    def get(self) -> Optional[T]:
        with self._data_available:
            return self._buffer

    # 获取缓冲的消息（仅互斥锁，不使用条件变量）
    def get_latest(self) -> Optional[T]:
        with self._lock:
            return self._buffer

    # 等待指定话题的消息，超时返回None
    def wait_for_message(self, topic: str, msg_type: type, timeout: Optional[float] = None, node: Optional[Node] = None) -> Optional[T]:
        buffer = MessageBuffer()
        if node is None:
            node = rclpy.node.Node('_temp_message_waiter')

        subscription = node.create_subscription(
            msg_type,
            topic,
            lambda msg: buffer.set(msg),
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        try:
            with buffer._data_available:
                if buffer._data_available.wait(timeout=timeout):
                    return buffer.get()
                return None
        finally:
            if node:
                node.destroy_subscription(subscription)


# ServiceProxyPool: 服务客户端代理池，复用已创建的服务客户端
class ServiceProxyPool:
    def __init__(self, node: Node):
        self._node = node
        self._clients = {}
        self._lock = threading.Lock()

    # 获取或创建指定服务的客户端
    def get_client(self, service_name: str, service_type: type) -> Client:
        with self._lock:
            if service_name not in self._clients:
                self._clients[service_name] = self._node.create_client(
                    service_type,
                    service_name
                )
            return self._clients[service_name]

    # 异步调用服务，等待服务可用后发送请求
    def call_async(self, service_name: str, request: Any) -> Any:
        client = self.get_client(service_name, type(request))
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"Service {service_name} not available")
        future = client.call_async(request)
        return future


# TopicThrottler: 话题发布限流器，控制发布频率不超过指定速率
class TopicThrottler:
    def __init__(self, max_rate: float):
        self._min_interval = 1.0 / max_rate if max_rate > 0 else 0.0
        self._last_publish = 0.0
        self._lock = threading.Lock()

    # 判断当前是否允许发布，若允许则更新上次发布时间
    def should_publish(self) -> bool:
        with self._lock:
            now = time.monotonic()
            if now - self._last_publish >= self._min_interval:
                self._last_publish = now
                return True
            return False


# 使用多线程执行器运行节点，支持KeyboardInterrupt优雅退出
def spin_node(node: Node):
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()


# 安全获取节点参数值，参数不存在时返回默认值
def get_param_or(node: Node, name: str, default: Any) -> Any:
    if node.has_parameter(name):
        return node.get_parameter(name).value
    return default
