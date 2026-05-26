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


class MessageBuffer(Generic[T]):
    def __init__(self):
        self._buffer = None
        self._lock = threading.Lock()
        self._data_available = threading.Condition(self._lock)

    def set(self, message: T):
        with self._data_available:
            self._buffer = message
            self._data_available.notify_all()

    def get(self) -> Optional[T]:
        with self._data_available:
            return self._buffer

    def get_latest(self) -> Optional[T]:
        with self._lock:
            return self._buffer

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


class ServiceProxyPool:
    def __init__(self, node: Node):
        self._node = node
        self._clients = {}
        self._lock = threading.Lock()

    def get_client(self, service_name: str, service_type: type) -> Client:
        with self._lock:
            if service_name not in self._clients:
                self._clients[service_name] = self._node.create_client(
                    service_type,
                    service_name
                )
            return self._clients[service_name]

    def call_async(self, service_name: str, request: Any) -> Any:
        client = self.get_client(service_name, type(request))
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"Service {service_name} not available")
        future = client.call_async(request)
        return future


class TopicThrottler:
    def __init__(self, max_rate: float):
        self._min_interval = 1.0 / max_rate if max_rate > 0 else 0.0
        self._last_publish = 0.0
        self._lock = threading.Lock()

    def should_publish(self) -> bool:
        with self._lock:
            now = time.monotonic()
            if now - self._last_publish >= self._min_interval:
                self._last_publish = now
                return True
            return False


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


def get_param_or(node: Node, name: str, default: Any) -> Any:
    if node.has_parameter(name):
        return node.get_parameter(name).value
    return default
