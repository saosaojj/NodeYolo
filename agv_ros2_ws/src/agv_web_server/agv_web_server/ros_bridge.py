#!/usr/bin/env python3
"""
ROS2桥接模块

提供Web服务与ROS2系统之间的桥接功能，
支持话题订阅、发布、服务调用和Action调用，
并提供消息缓存、健康检查等功能。
"""

import threading
import time


class RosBridge:
    """
    ROS2桥接类
    
    负责管理Web服务与ROS2系统之间的通信，
    提供统一的接口来操作ROS2的话题、服务和Action。
    """

    def __init__(self, node):
        """
        初始化ROS桥接器
        
        Args:
            node: ROS2节点实例
        """
        self._node = node  # ROS2节点
        self._lock = threading.Lock()  # 线程安全锁
        
        # 存储订阅者
        self._subscribers = {}
        
        # 存储最新消息
        self._latest_messages = {}
        
        # 存储累积消息（按key分组）
        self._accumulated_messages = {}
        
        # 存储发布者
        self._publishers = {}
        
        # 存储服务客户端
        self._service_clients = {}
        
        # 存储Action客户端
        self._action_clients = {}
        
        # 消息序列化缓存
        self._serialization_cache = {}
        
        # 缓存存活时间（秒）
        self._cache_ttl = 0.1
        
        # 缓存时间戳
        self._cache_timestamps = {}
        
        # 服务超时记录
        self._service_timeouts = {}
        
        # 默认服务超时时间（秒）
        self._default_service_timeout = 10.0
        
        # 健康检查间隔（秒）
        self._health_check_interval = 30.0
        
        # 上次健康检查时间
        self._last_health_check = time.time()
        
        # 订阅健康状态
        self._subscription_health = {}

    def create_subscription(self, topic, msg_type, qos=10):
        """
        创建一个订阅者
        
        Args:
            topic: 话题名称
            msg_type: 消息类型
            qos: 服务质量参数
        """
        with self._lock:
            if topic in self._subscribers:
                return

            def callback(msg, t=topic):
                """
                话题回调函数
                
                Args:
                    msg: 接收到的消息
                    t: 话题名称
                """
                with self._lock:
                    self._latest_messages[t] = msg  # 保存最新消息
                    self._serialization_cache.pop(t, None)  # 清除缓存
                    self._cache_timestamps.pop(t, None)  # 清除时间戳
                    self._subscription_health[t] = time.time()  # 更新健康时间

            sub = self._node.create_subscription(msg_type, topic, callback, qos)
            self._subscribers[topic] = sub  # 保存订阅者
            self._latest_messages[topic] = None  # 初始化消息
            self._subscription_health[topic] = 0.0  # 初始化健康状态

    def create_accumulating_subscription(self, topic, msg_type, key_field, qos=10):
        """
        创建一个累积订阅者（按key分组存储消息）
        
        Args:
            topic: 话题名称
            msg_type: 消息类型
            key_field: 用于分组的字段名
            qos: 服务质量参数
        """
        with self._lock:
            if topic in self._subscribers:
                return

            def callback(msg, t=topic, kf=key_field):
                """
                累积回调函数
                
                Args:
                    msg: 接收到的消息
                    t: 话题名称
                    kf: 分组字段名
                """
                with self._lock:
                    self._latest_messages[t] = msg
                    if t not in self._accumulated_messages:
                        self._accumulated_messages[t] = {}
                    # 按key字段分组存储
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
        """
        获取某个话题的最新消息
        
        Args:
            topic: 话题名称
            
        Returns:
            最新消息或None
        """
        with self._lock:
            return self._latest_messages.get(topic, None)

    def get_cached_serialization(self, topic):
        """
        获取缓存的消息序列化结果
        
        Args:
            topic: 话题名称
            
        Returns:
            缓存的序列化数据或None
        """
        with self._lock:
            if topic not in self._serialization_cache:
                return None
            cache_time = self._cache_timestamps.get(topic, 0.0)
            # 检查缓存是否过期
            if time.time() - cache_time > self._cache_ttl:
                self._serialization_cache.pop(t, None)
                self._cache_timestamps.pop(t, None)
                return None
            return self._serialization_cache[topic]

    def set_cached_serialization(self, topic, data):
        """
        设置缓存的消息序列化结果
        
        Args:
            topic: 话题名称
            data: 序列化数据
        """
        with self._lock:
            self._serialization_cache[topic] = data
            self._cache_timestamps[topic] = time.time()

    def get_accumulated_messages(self, topic):
        """
        获取累积的消息（按key分组）
        
        Args:
            topic: 话题名称
            
        Returns:
            消息字典（key -> message）
        """
        with self._lock:
            return dict(self._accumulated_messages.get(topic, {}))

    def create_publisher(self, topic, msg_type, qos=10):
        """
        创建一个发布者
        
        Args:
            topic: 话题名称
            msg_type: 消息类型
            qos: 服务质量参数
            
        Returns:
            发布者实例
        """
        with self._lock:
            if topic in self._publishers:
                return self._publishers[topic]
            pub = self._node.create_publisher(msg_type, topic, qos)
            self._publishers[topic] = pub
            return pub

    def publish(self, topic, msg_dict):
        """
        发布消息到话题
        
        Args:
            topic: 话题名称
            msg_dict: 消息字典
        """
        with self._lock:
            pub = self._publishers.get(topic, None)
            if pub is None:
                self._node.get_logger().warn(f'Publisher for topic {topic} not found')
                return
            # 创建消息并设置字段
            msg = pub.msg_type()
            self._set_msg_fields(msg, msg_dict)
            pub.publish(msg)

    def call_service(self, service_name, service_type, request_dict, timeout=None):
        """
        调用ROS2服务
        
        Args:
            service_name: 服务名称
            service_type: 服务类型
            request_dict: 请求参数字典
            timeout: 超时时间（秒）
            
        Returns:
            Future对象或None（服务不可用）
        """
        with self._lock:
            if service_name not in self._service_clients:
                client = self._node.create_client(service_type, service_name)
                self._service_clients[service_name] = client

            client = self._service_clients[service_name]

        service_timeout = timeout if timeout is not None else self._default_service_timeout

        # 等待服务就绪
        if not client.service_is_ready():
            self._node.get_logger().info(f'Waiting for service {service_name}...')
            if not client.wait_for_service(timeout_sec=min(service_timeout, 5.0)):
                return None

        # 创建请求并设置字段
        request = service_type.Request()
        self._set_msg_fields(request, request_dict)

        # 异步调用服务
        future = client.call_async(request)
        return future

    def call_action(self, action_name, action_type, goal_dict):
        """
        调用ROS2 Action
        
        Args:
            action_name: Action名称
            action_type: Action类型
            goal_dict: 目标参数字典
            
        Returns:
            Future对象或None（服务不可用）
        """
        from rclpy.action import ActionClient

        with self._lock:
            if action_name not in self._action_clients:
                client = ActionClient(self._node, action_type, action_name)
                self._action_clients[action_name] = client

            client = self._action_clients[action_name]

        # 等待Action服务就绪
        if not client.server_is_ready():
            self._node.get_logger().info(f'Waiting for action server {action_name}...')
            if not client.wait_for_server(timeout_sec=5.0):
                return None

        # 创建目标并设置字段
        goal_msg = action_type.Goal()
        self._set_msg_fields(goal_msg, goal_dict)

        # 发送目标
        send_goal_future = client.send_goal_async(goal_msg)
        return send_goal_future

    def check_health(self):
        """
        检查ROS桥接的健康状态
        
        Returns:
            bool: 健康状态
        """
        now = time.time()
        # 检查是否需要执行健康检查
        if now - self._last_health_check < self._health_check_interval:
            return True
        self._last_health_check = now

        # 检查订阅健康状态
        for topic, last_update in self._subscription_health.items():
            if last_update > 0 and (now - last_update) > 60.0:
                self._node.get_logger().warn(
                    f'Health: no messages on {topic} for {now - last_update:.0f}s')

        # 检查服务健康状态
        for service_name, client in self._service_clients.items():
            if not client.service_is_ready():
                self._node.get_logger().warn(
                    f'Health: service {service_name} is not ready')

        return True

    def _set_msg_fields(self, msg, fields_dict):
        """
        递归设置ROS消息的字段值
        
        Args:
            msg: ROS消息对象
            fields_dict: 字段字典
        """
        for key, value in fields_dict.items():
            if hasattr(msg, key):
                attr = getattr(msg, key)
                # 如果值是字典，递归设置
                if isinstance(value, dict):
                    self._set_msg_fields(attr, value)
                # 如果值是列表或元组
                elif isinstance(value, (list, tuple)):
                    if len(value) > 0 and isinstance(value[0], dict):
                        # 列表元素是字典，递归设置
                        for i, item in enumerate(value):
                            if i < len(attr):
                                self._set_msg_fields(attr[i], item)
                            else:
                                attr.append(item)
                    else:
                        # 直接设置列表
                        try:
                            setattr(msg, key, type(attr)(value))
                        except (TypeError, ValueError):
                            setattr(msg, key, value)
                else:
                    # 单个值设置
                    try:
                        setattr(msg, key, type(attr)(value) if not isinstance(value, type(attr)) else value)
                    except (TypeError, ValueError):
                        setattr(msg, key, value)
