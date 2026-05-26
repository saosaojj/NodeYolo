#!/usr/bin/env python3
"""
WebSocket 处理模块

提供AGV系统的实时数据推送功能，
支持AGV状态、YOLO检测结果、IO状态、PLC数据等话题的WebSocket连接，
以及速度控制命令的接收。
"""

import asyncio
import json
import time
from collections import deque
from fastapi import WebSocket, WebSocketDisconnect

from agv_interfaces.msg import AGVStatus, YoloResult, IOState, PlcData
from geometry_msgs.msg import Twist


def _msg_to_dict(msg):
    """
    将ROS2消息转换为Python字典（递归处理嵌套消息）
    
    Args:
        msg: ROS2消息对象
        
    Returns:
        dict: 转换后的字典
    """
    if msg is None:
        return None
    result = {}
    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field)
        # 如果字段值本身也是ROS消息，递归处理
        if hasattr(value, 'get_fields_and_field_types'):
            result[field] = _msg_to_dict(value)
        # 如果是列表或元组
        elif isinstance(value, (list, tuple)):
            result[field] = [_msg_to_dict(item) if hasattr(item, 'get_fields_and_field_types') else item for item in value]
        else:
            result[field] = value
    return result


class ClientMessageQueue:
    """
    客户端消息队列
    
    用于在WebSocket连接中缓冲消息，防止消息丢失。
    """

    def __init__(self, max_size=100):
        """
        初始化消息队列
        
        Args:
            max_size: 队列最大长度
        """
        self._queue = deque(maxlen=max_size)  # 使用双端队列，自动移除旧消息
        self._lock = asyncio.Lock()  # 异步锁
        self._event = asyncio.Event()  # 事件，用于通知有新消息

    async def put(self, message):
        """
        放入消息（异步）
        
        Args:
            message: 要放入的消息
        """
        async with self._lock:
            # 如果队列已满，移除最旧的消息
            if len(self._queue) >= self._queue.maxlen:
                self._queue.popleft()
            self._queue.append(message)
            self._event.set()  # 通知有新消息

    async def get_all(self):
        """
        获取所有消息并清空队列（异步）
        
        Returns:
            list: 消息列表
        """
        async with self._lock:
            messages = list(self._queue)
            self._queue.clear()
            self._event.clear()
            return messages

    def put_sync(self, message):
        """
        放入消息（同步）
        
        Args:
            message: 要放入的消息
        """
        if len(self._queue) >= self._queue.maxlen:
            self._queue.popleft()
        self._queue.append(message)
        self._event.set()


def register_websocket(app, ros_bridge):
    """
    注册所有WebSocket路由
    
    Args:
        app: FastAPI应用实例
        ros_bridge: ROS2桥接器实例
    """
    # 创建订阅和发布者
    ros_bridge.create_subscription('/agv_status', AGVStatus)
    ros_bridge.create_subscription('/yolo_result', YoloResult)
    ros_bridge.create_accumulating_subscription('/io_states', IOState, 'io_name')
    ros_bridge.create_subscription('/plc_data', PlcData)
    ros_bridge.create_publisher('/cmd_vel', Twist)

    # 消息序列化缓存
    _serialization_cache = {}
    _cache_timestamps = {}
    _cache_ttl = 0.1  # 缓存过期时间（秒）

    def _get_cached_serialization(topic, msg):
        """
        获取缓存的消息序列化结果
        
        Args:
            topic: 话题名称
            msg: 消息对象
            
        Returns:
            序列化后的字典
        """
        now = time.time()
        if topic in _serialization_cache:
            if now - _cache_timestamps.get(topic, 0) < _cache_ttl:
                return _serialization_cache[topic]
        # 未命中缓存，重新序列化
        data = _msg_to_dict(msg)
        _serialization_cache[topic] = data
        _cache_timestamps[topic] = now
        return data

    @app.websocket('/ws/agv_status')
    async def ws_agv_status(websocket: WebSocket):
        """
        AGV状态WebSocket端点
        
        推送AGV的实时状态信息，包括位置、速度、电量等。
        """
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0  # Ping间隔（秒）
        pong_timeout = 60.0  # Pong超时（秒）
        last_pong = time.time()
        try:
            while True:
                now = time.time()
                
                # 发送Ping保持连接活跃
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break
                
                # 获取最新AGV状态
                msg = ros_bridge.get_latest_message('/agv_status')
                if msg is not None:
                    data = _get_cached_serialization('/agv_status', msg)
                    await msg_queue.put(data)
                
                # 发送队列中的消息
                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break
                
                # 接收客户端消息
                try:
                    raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.05)
                    received = json.loads(raw)
                    if received.get('type') == 'pong':
                        last_pong = time.time()
                except asyncio.TimeoutError:
                    pass
                except json.JSONDecodeError:
                    pass
                except Exception:
                    break
                
                # 检查Pong超时
                if now - last_pong > pong_timeout:
                    break

        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.websocket('/ws/yolo_result')
    async def ws_yolo_result(websocket: WebSocket):
        """
        YOLO检测结果WebSocket端点
        
        推送视觉检测结果，包括检测到的目标类别和位置。
        """
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                
                # 发送Ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break
                
                # 获取最新YOLO结果
                msg = ros_bridge.get_latest_message('/yolo_result')
                if msg is not None:
                    data = _get_cached_serialization('/yolo_result', msg)
                    await msg_queue.put(data)
                
                # 发送消息
                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break
                
                # 接收客户端消息
                try:
                    raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.05)
                    received = json.loads(raw)
                    if received.get('type') == 'pong':
                        last_pong = time.time()
                except asyncio.TimeoutError:
                    pass
                except json.JSONDecodeError:
                    pass
                except Exception:
                    break
                
                if now - last_pong > pong_timeout:
                    break

        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.websocket('/ws/io_states')
    async def ws_io_states(websocket: WebSocket):
        """
        IO状态WebSocket端点
        
        推送所有IO口的状态变化。
        """
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                
                # 发送Ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break
                
                # 获取IO状态（累积消息）
                accumulated = ros_bridge.get_accumulated_messages('/io_states')
                if accumulated:
                    data = [_msg_to_dict(m) for m in accumulated.values()]
                else:
                    msg = ros_bridge.get_latest_message('/io_states')
                    if msg is not None:
                        data = [_msg_to_dict(msg)]
                    else:
                        data = None
                
                if data is not None:
                    await msg_queue.put(data)
                
                # 发送消息
                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break
                
                # 接收客户端消息
                try:
                    raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.05)
                    received = json.loads(raw)
                    if received.get('type') == 'pong':
                        last_pong = time.time()
                except asyncio.TimeoutError:
                    pass
                except json.JSONDecodeError:
                    pass
                except Exception:
                    break
                
                if now - last_pong > pong_timeout:
                    break

        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.websocket('/ws/plc_data')
    async def ws_plc_data(websocket: WebSocket):
        """
        PLC数据WebSocket端点
        
        推送PLC的线圈和寄存器数据。
        """
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                
                # 发送Ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break
                
                # 获取最新PLC数据
                msg = ros_bridge.get_latest_message('/plc_data')
                if msg is not None:
                    data = _get_cached_serialization('/plc_data', msg)
                    await msg_queue.put(data)
                
                # 发送消息
                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break
                
                # 接收客户端消息
                try:
                    raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.05)
                    received = json.loads(raw)
                    if received.get('type') == 'pong':
                        last_pong = time.time()
                except asyncio.TimeoutError:
                    pass
                except json.JSONDecodeError:
                    pass
                except Exception:
                    break
                
                if now - last_pong > pong_timeout:
                    break

        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.websocket('/ws/cmd_vel')
    async def ws_cmd_vel(websocket: WebSocket):
        """
        速度控制WebSocket端点
        
        接收客户端发送的速度控制命令，转发到ROS2话题。
        """
        await websocket.accept()
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                
                # 发送Ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break
                
                # 接收客户端速度命令
                try:
                    data = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                    try:
                        cmd = json.loads(data)
                        if cmd.get('type') == 'pong':
                            last_pong = time.time()
                            continue
                        linear_x = float(cmd.get('linear_x', 0.0))
                        linear_y = float(cmd.get('linear_y', 0.0))
                        angular_z = float(cmd.get('angular_z', 0.0))
                        # 发布速度命令
                        ros_bridge.publish('/cmd_vel', {
                            'linear': {'x': linear_x, 'y': linear_y, 'z': 0.0},
                            'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z},
                        })
                    except (json.JSONDecodeError, ValueError, KeyError):
                        await websocket.send_json({'error': 'Invalid command format'})
                except asyncio.TimeoutError:
                    pass
                except Exception:
                    break
                
                if now - last_pong > pong_timeout:
                    break

        except WebSocketDisconnect:
            pass
        except Exception:
            pass
