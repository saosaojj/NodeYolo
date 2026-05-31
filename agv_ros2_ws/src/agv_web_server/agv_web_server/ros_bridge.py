import threading
import time
import json
from collections import deque


class RosBridge:

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()

        # 原有属性
        self._subscribers = {}
        self._latest_messages = {}
        self._accumulated_messages = {}
        self._publishers = {}
        self._service_clients = {}
        self._action_clients = {}
        self._serialization_cache = {}
        self._cache_ttl = 0.1
        self._cache_timestamps = {}
        self._service_timeouts = {}
        self._default_service_timeout = 10.0
        self._health_check_interval = 30.0
        self._last_health_check = time.time()
        self._subscription_health = {}

        # WebSocket重连处理
        self._reconnect_enabled = True
        self._reconnect_max_attempts = 5
        self._reconnect_interval = 3.0
        self._reconnect_backoff_factor = 1.5

        # 消息队列管理
        self._message_queues = {}
        self._message_queue_max_size = 100
        self._message_queue_lock = threading.Lock()

        # 话题订阅管理 - 引用计数
        self._subscription_ref_counts = {}
        self._subscription_lock = threading.Lock()

        # 服务调用超时和重试
        self._service_retry_count = 3
        self._service_retry_delay = 1.0

        # 桥接统计信息
        self._stats = {
            'messages_per_second': 0.0,
            'total_messages': 0,
            'total_service_calls': 0,
            'total_errors': 0,
            'messages_by_topic': {},
            'service_calls_by_name': {},
        }
        self._stats_lock = threading.Lock()
        self._stats_window = deque(maxlen=60)
        self._stats_last_time = time.time()

        # 消息节流
        self._throttle_config = {}
        self._throttle_last_publish = {}
        self._throttle_lock = threading.Lock()

        # 数据转换辅助
        self._type_converters = {}
        self._reverse_converters = {}

    def create_subscription(self, topic, msg_type, qos=10):
        with self._lock:
            if topic in self._subscribers:
                # 增加引用计数
                self._increment_subscription_ref(topic)
                return

            def callback(msg, t=topic):
                with self._lock:
                    self._latest_messages[t] = msg
                    self._serialization_cache.pop(t, None)
                    self._cache_timestamps.pop(t, None)
                    self._subscription_health[t] = time.time()

                # 消息队列管理 - 将消息放入队列
                self._enqueue_message(t, msg)

                # 更新统计信息
                self._record_message(t)

            sub = self._node.create_subscription(msg_type, topic, callback, qos)
            self._subscribers[topic] = sub
            self._latest_messages[topic] = None
            self._subscription_health[topic] = 0.0

            # 初始化引用计数
            self._increment_subscription_ref(topic)

            # 初始化消息队列
            with self._message_queue_lock:
                self._message_queues[topic] = deque(maxlen=self._message_queue_max_size)

            # 初始化节流配置
            with self._throttle_lock:
                if topic not in self._throttle_config:
                    self._throttle_config[topic] = 0.0
                if topic not in self._throttle_last_publish:
                    self._throttle_last_publish[topic] = 0.0

    def create_accumulating_subscription(self, topic, msg_type, key_field, qos=10):
        with self._lock:
            if topic in self._subscribers:
                self._increment_subscription_ref(topic)
                return

            def callback(msg, t=topic, kf=key_field):
                with self._lock:
                    self._latest_messages[t] = msg
                    if t not in self._accumulated_messages:
                        self._accumulated_messages[t] = {}
                    key_value = getattr(msg, kf, None)
                    if key_value is not None:
                        self._accumulated_messages[t][key_value] = msg
                    self._serialization_cache.pop(t, None)
                    self._cache_timestamps.pop(t, None)
                    self._subscription_health[t] = time.time()

                self._enqueue_message(t, msg)
                self._record_message(t)

            sub = self._node.create_subscription(msg_type, topic, callback, qos)
            self._subscribers[topic] = sub
            self._latest_messages[topic] = None
            self._accumulated_messages[topic] = {}
            self._subscription_health[topic] = 0.0

            self._increment_subscription_ref(topic)

            with self._message_queue_lock:
                self._message_queues[topic] = deque(maxlen=self._message_queue_max_size)

            with self._throttle_lock:
                if topic not in self._throttle_config:
                    self._throttle_config[topic] = 0.0
                if topic not in self._throttle_last_publish:
                    self._throttle_last_publish[topic] = 0.0

    def _increment_subscription_ref(self, topic):
        # 增加话题订阅引用计数
        with self._subscription_lock:
            self._subscription_ref_counts[topic] = self._subscription_ref_counts.get(topic, 0) + 1

    def _decrement_subscription_ref(self, topic):
        # 减少话题订阅引用计数
        with self._subscription_lock:
            if topic in self._subscription_ref_counts:
                self._subscription_ref_counts[topic] -= 1
                return self._subscription_ref_counts[topic]
            return 0

    def unsubscribe_topic(self, topic):
        # 当没有客户端需要时取消订阅
        ref_count = self._decrement_subscription_ref(topic)
        if ref_count <= 0:
            with self._lock:
                if topic in self._subscribers:
                    self._node.destroy_subscription(self._subscribers[topic])
                    del self._subscribers[topic]
                    self._node.get_logger().info(f'取消订阅话题: {topic} (无活跃客户端)')

            with self._subscription_lock:
                self._subscription_ref_counts.pop(topic, None)

    def _enqueue_message(self, topic, msg):
        # 消息队列管理 - 将消息放入队列，队列满时丢弃最旧消息
        with self._message_queue_lock:
            if topic not in self._message_queues:
                self._message_queues[topic] = deque(maxlen=self._message_queue_max_size)
            queue = self._message_queues[topic]
            if len(queue) >= queue.maxlen:
                queue.popleft()
            queue.append(msg)

    def get_queued_messages(self, topic, max_count=None):
        # 获取队列中的消息
        with self._message_queue_lock:
            queue = self._message_queues.get(topic, deque())
            messages = list(queue)
            queue.clear()
            if max_count and len(messages) > max_count:
                messages = messages[-max_count:]
            return messages

    def get_latest_message(self, topic):
        with self._lock:
            return self._latest_messages.get(topic, None)

    def get_cached_serialization(self, topic):
        with self._lock:
            if topic not in self._serialization_cache:
                return None
            cache_time = self._cache_timestamps.get(topic, 0.0)
            if time.time() - cache_time > self._cache_ttl:
                self._serialization_cache.pop(topic, None)
                self._cache_timestamps.pop(topic, None)
                return None
            return self._serialization_cache[topic]

    def set_cached_serialization(self, topic, data):
        with self._lock:
            self._serialization_cache[topic] = data
            self._cache_timestamps[topic] = time.time()

    def get_accumulated_messages(self, topic):
        with self._lock:
            return dict(self._accumulated_messages.get(topic, {}))

    def create_publisher(self, topic, msg_type, qos=10):
        with self._lock:
            if topic in self._publishers:
                return self._publishers[topic]
            pub = self._node.create_publisher(msg_type, topic, qos)
            self._publishers[topic] = pub
            return pub

    def publish(self, topic, msg_dict):
        # 消息节流 - 限制发布频率
        with self._throttle_lock:
            throttle_rate = self._throttle_config.get(topic, 0.0)
            if throttle_rate > 0.0:
                now = time.time()
                last_publish = self._throttle_last_publish.get(topic, 0.0)
                min_interval = 1.0 / throttle_rate
                if now - last_publish < min_interval:
                    return
                self._throttle_last_publish[topic] = now

        with self._lock:
            pub = self._publishers.get(topic, None)
            if pub is None:
                self._node.get_logger().warn(f'Publisher for topic {topic} not found')
                return
            msg = pub.msg_type()
            self._set_msg_fields(msg, msg_dict)
            pub.publish(msg)

        self._record_message(topic)

    def set_throttle_rate(self, topic, rate_hz):
        # 设置话题消息节流率
        with self._throttle_lock:
            self._throttle_config[topic] = rate_hz
            self._node.get_logger().info(f'话题 {topic} 节流率设置为 {rate_hz} Hz')

    def call_service(self, service_name, service_type, request_dict, timeout=None):
        with self._lock:
            if service_name not in self._service_clients:
                client = self._node.create_client(service_type, service_name)
                self._service_clients[service_name] = client

            client = self._service_clients[service_name]

        service_timeout = timeout if timeout is not None else self._default_service_timeout

        # 服务调用超时和重试
        for attempt in range(self._service_retry_count):
            if not client.service_is_ready():
                self._node.get_logger().info(f'Waiting for service {service_name}...')
                if not client.wait_for_service(timeout_sec=min(service_timeout, 5.0)):
                    if attempt < self._service_retry_count - 1:
                        self._node.get_logger().info(
                            f'服务 {service_name} 未就绪，重试 ({attempt + 1}/{self._service_retry_count})'
                        )
                        time.sleep(self._service_retry_delay)
                        continue
                    self._record_error()
                    return None

            request = service_type.Request()
            self._set_msg_fields(request, request_dict)

            future = client.call_async(request)

            # 更新服务调用统计
            with self._stats_lock:
                self._stats['total_service_calls'] += 1
                if service_name not in self._stats['service_calls_by_name']:
                    self._stats['service_calls_by_name'][service_name] = 0
                self._stats['service_calls_by_name'][service_name] += 1

            return future

        self._record_error()
        return None

    def call_action(self, action_name, action_type, goal_dict):
        from rclpy.action import ActionClient

        with self._lock:
            if action_name not in self._action_clients:
                client = ActionClient(self._node, action_type, action_name)
                self._action_clients[action_name] = client

            client = self._action_clients[action_name]

        if not client.server_is_ready():
            self._node.get_logger().info(f'Waiting for action server {action_name}...')
            if not client.wait_for_server(timeout_sec=5.0):
                return None

        goal_msg = action_type.Goal()
        self._set_msg_fields(goal_msg, goal_dict)

        send_goal_future = client.send_goal_async(goal_msg)
        return send_goal_future

    def check_health(self):
        now = time.time()
        if now - self._last_health_check < self._health_check_interval:
            return True
        self._last_health_check = now

        for topic, last_update in self._subscription_health.items():
            if last_update > 0 and (now - last_update) > 60.0:
                self._node.get_logger().warn(
                    f'Health: no messages on {topic} for {now - last_update:.0f}s')

        for service_name, client in self._service_clients.items():
            if not client.service_is_ready():
                self._node.get_logger().warn(
                    f'Health: service {service_name} is not ready')

        return True

    def _record_message(self, topic):
        # 记录消息统计信息
        with self._stats_lock:
            self._stats['total_messages'] += 1
            if topic not in self._stats['messages_by_topic']:
                self._stats['messages_by_topic'][topic] = 0
            self._stats['messages_by_topic'][topic] += 1

        self._stats_window.append(time.time())

    def _record_error(self):
        # 记录错误统计
        with self._stats_lock:
            self._stats['total_errors'] += 1

    def get_stats(self):
        # 获取桥接统计信息
        now = time.time()

        # 计算每秒消息数
        if len(self._stats_window) >= 2:
            window = list(self._stats_window)
            duration = window[-1] - window[0]
            if duration > 0:
                self._stats['messages_per_second'] = len(window) / duration

        with self._stats_lock:
            stats = dict(self._stats)
            stats['active_subscriptions'] = len(self._subscribers)
            stats['active_publishers'] = len(self._publishers)
            stats['active_service_clients'] = len(self._service_clients)
            stats['subscription_ref_counts'] = dict(self._subscription_ref_counts)

        return stats

    def msg_to_dict(self, msg):
        # 数据转换辅助 - 将ROS消息高效转换为字典
        if msg is None:
            return None
        if hasattr(msg, 'get_fields_and_field_types'):
            result = {}
            for field in msg.get_fields_and_field_types():
                value = getattr(msg, field)
                if hasattr(value, 'get_fields_and_field_types'):
                    result[field] = self.msg_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    result[field] = [
                        self.msg_to_dict(item) if hasattr(item, 'get_fields_and_field_types') else
                        self._convert_primitive(item)
                        for item in value
                    ]
                else:
                    result[field] = self._convert_primitive(value)
            return result
        return self._convert_primitive(msg)

    def dict_to_msg_fields(self, msg, fields_dict):
        # 数据转换辅助 - 将字典高效设置到ROS消息字段
        self._set_msg_fields(msg, fields_dict)

    def _convert_primitive(self, value):
        # 转换基本类型为JSON兼容类型
        if isinstance(value, (int, float, str, bool)):
            return value
        if isinstance(value, bytes):
            return list(value)
        if isinstance(value, (list, tuple)):
            return [self._convert_primitive(v) for v in value]
        return str(value)

    def _set_msg_fields(self, msg, fields_dict):
        for key, value in fields_dict.items():
            if hasattr(msg, key):
                attr = getattr(msg, key)
                if isinstance(value, dict):
                    self._set_msg_fields(attr, value)
                elif isinstance(value, (list, tuple)):
                    if len(value) > 0 and isinstance(value[0], dict):
                        for i, item in enumerate(value):
                            if i < len(attr):
                                self._set_msg_fields(attr[i], item)
                            else:
                                attr.append(item)
                    else:
                        try:
                            setattr(msg, key, type(attr)(value))
                        except (TypeError, ValueError):
                            setattr(msg, key, value)
                else:
                    try:
                        setattr(msg, key, type(attr)(value) if not isinstance(value, type(attr)) else value)
                    except (TypeError, ValueError):
                        setattr(msg, key, value)
