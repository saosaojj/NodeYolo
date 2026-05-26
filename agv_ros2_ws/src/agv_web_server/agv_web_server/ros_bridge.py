import threading
import time


class RosBridge:

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
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

    def create_subscription(self, topic, msg_type, qos=10):
        with self._lock:
            if topic in self._subscribers:
                return

            def callback(msg, t=topic):
                with self._lock:
                    self._latest_messages[t] = msg
                    self._serialization_cache.pop(t, None)
                    self._cache_timestamps.pop(t, None)
                    self._subscription_health[t] = time.time()

            sub = self._node.create_subscription(msg_type, topic, callback, qos)
            self._subscribers[topic] = sub
            self._latest_messages[topic] = None
            self._subscription_health[topic] = 0.0

    def create_accumulating_subscription(self, topic, msg_type, key_field, qos=10):
        with self._lock:
            if topic in self._subscribers:
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

            sub = self._node.create_subscription(msg_type, topic, callback, qos)
            self._subscribers[topic] = sub
            self._latest_messages[topic] = None
            self._accumulated_messages[topic] = {}
            self._subscription_health[topic] = 0.0

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
        with self._lock:
            pub = self._publishers.get(topic, None)
            if pub is None:
                self._node.get_logger().warn(f'Publisher for topic {topic} not found')
                return
            msg = pub.msg_type()
            self._set_msg_fields(msg, msg_dict)
            pub.publish(msg)

    def call_service(self, service_name, service_type, request_dict, timeout=None):
        with self._lock:
            if service_name not in self._service_clients:
                client = self._node.create_client(service_type, service_name)
                self._service_clients[service_name] = client

            client = self._service_clients[service_name]

        service_timeout = timeout if timeout is not None else self._default_service_timeout

        if not client.service_is_ready():
            self._node.get_logger().info(f'Waiting for service {service_name}...')
            if not client.wait_for_service(timeout_sec=min(service_timeout, 5.0)):
                return None

        request = service_type.Request()
        self._set_msg_fields(request, request_dict)

        future = client.call_async(request)
        return future

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
