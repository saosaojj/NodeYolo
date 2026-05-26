# WebSocket处理器模块，为前端提供实时数据推送和远程控制接口
import asyncio
import json
import time
from collections import deque
from fastapi import WebSocket, WebSocketDisconnect

from agv_interfaces.msg import AGVStatus, YoloResult, IOState, PlcData
from geometry_msgs.msg import Twist


# 将ROS2消息递归转换为Python字典，便于JSON序列化
def _msg_to_dict(msg):
    if msg is None:
        return None
    result = {}
    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field)
        # 递归处理嵌套消息
        if hasattr(value, 'get_fields_and_field_types'):
            result[field] = _msg_to_dict(value)
        # 处理列表类型，递归转换嵌套消息元素
        elif isinstance(value, (list, tuple)):
            result[field] = [_msg_to_dict(item) if hasattr(item, 'get_fields_and_field_types') else item for item in value]
        else:
            result[field] = value
    return result


# 客户端消息队列，支持异步和同步写入，用于缓存待发送的WebSocket消息
class ClientMessageQueue:
    def __init__(self, max_size=100):
        self._queue = deque(maxlen=max_size)
        self._lock = asyncio.Lock()
        self._event = asyncio.Event()

    # 异步写入消息，队列满时丢弃最旧消息
    async def put(self, message):
        async with self._lock:
            if len(self._queue) >= self._queue.maxlen:
                self._queue.popleft()
            self._queue.append(message)
            self._event.set()

    # 异步获取并清空所有消息
    async def get_all(self):
        async with self._lock:
            messages = list(self._queue)
            self._queue.clear()
            self._event.clear()
            return messages

    # 同步写入消息，供非异步上下文调用
    def put_sync(self, message):
        if len(self._queue) >= self._queue.maxlen:
            self._queue.popleft()
        self._queue.append(message)
        self._event.set()


# 注册所有WebSocket路由端点，初始化ROS2订阅和发布
def register_websocket(app, ros_bridge):
    # 创建AGV状态、YOLO结果、IO状态和PLC数据的订阅
    ros_bridge.create_subscription('/agv_status', AGVStatus)
    ros_bridge.create_subscription('/yolo_result', YoloResult)
    ros_bridge.create_accumulating_subscription('/io_states', IOState, 'io_name')
    ros_bridge.create_subscription('/plc_data', PlcData)
    # 创建速度控制话题的发布者
    ros_bridge.create_publisher('/cmd_vel', Twist)

    # 本地序列化缓存，避免重复转换相同消息
    _serialization_cache = {}
    _cache_timestamps = {}
    _cache_ttl = 0.1

    # 获取缓存的序列化数据，带TTL过期检查
    def _get_cached_serialization(topic, msg):
        now = time.time()
        if topic in _serialization_cache:
            if now - _cache_timestamps.get(topic, 0) < _cache_ttl:
                return _serialization_cache[topic]
        data = _msg_to_dict(msg)
        _serialization_cache[topic] = data
        _cache_timestamps[topic] = now
        return data

    # AGV状态WebSocket端点，实时推送AGV运行状态
    @app.websocket('/ws/agv_status')
    async def ws_agv_status(websocket: WebSocket):
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        pong_timeout = 60.0
        last_pong = time.time()
        try:
            while True:
                now = time.time()
                # 定时发送心跳ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

                # 获取最新AGV状态消息并推送给客户端
                msg = ros_bridge.get_latest_message('/agv_status')
                if msg is not None:
                    data = _get_cached_serialization('/agv_status', msg)
                    await msg_queue.put(data)

                # 批量发送队列中的消息
                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break

                # 非阻塞接收客户端消息，处理pong心跳响应
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

                # 心跳超时检测，断开无响应的连接
                if now - last_pong > pong_timeout:
                    break

        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    # YOLO检测结果WebSocket端点，实时推送目标检测信息
    @app.websocket('/ws/yolo_result')
    async def ws_yolo_result(websocket: WebSocket):
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                # 定时发送心跳ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

                # 获取最新YOLO检测结果并推送
                msg = ros_bridge.get_latest_message('/yolo_result')
                if msg is not None:
                    data = _get_cached_serialization('/yolo_result', msg)
                    await msg_queue.put(data)

                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break

                # 处理客户端心跳响应
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

    # IO状态WebSocket端点，推送累积的IO状态数据
    @app.websocket('/ws/io_states')
    async def ws_io_states(websocket: WebSocket):
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                # 定时发送心跳ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

                # 获取累积的IO状态消息（按键字段聚合）
                accumulated = ros_bridge.get_accumulated_messages('/io_states')
                if accumulated:
                    data = [_msg_to_dict(m) for m in accumulated.values()]
                else:
                    # 无累积消息时回退到最新单条消息
                    msg = ros_bridge.get_latest_message('/io_states')
                    if msg is not None:
                        data = [_msg_to_dict(msg)]
                    else:
                        data = None

                if data is not None:
                    await msg_queue.put(data)

                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break

                # 处理客户端心跳响应
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

    # PLC数据WebSocket端点，实时推送PLC通信数据
    @app.websocket('/ws/plc_data')
    async def ws_plc_data(websocket: WebSocket):
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                # 定时发送心跳ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

                # 获取最新PLC数据并推送
                msg = ros_bridge.get_latest_message('/plc_data')
                if msg is not None:
                    data = _get_cached_serialization('/plc_data', msg)
                    await msg_queue.put(data)

                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break

                # 处理客户端心跳响应
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

    # 速度控制WebSocket端点，接收前端的速度指令并发布到ROS2话题
    @app.websocket('/ws/cmd_vel')
    async def ws_cmd_vel(websocket: WebSocket):
        await websocket.accept()
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                # 定时发送心跳ping
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

                # 接收客户端发送的速度控制指令
                try:
                    data = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                    try:
                        cmd = json.loads(data)
                        # 处理心跳pong响应
                        if cmd.get('type') == 'pong':
                            last_pong = time.time()
                            continue
                        # 解析线速度和角速度，发布到/cmd_vel话题
                        linear_x = float(cmd.get('linear_x', 0.0))
                        linear_y = float(cmd.get('linear_y', 0.0))
                        angular_z = float(cmd.get('angular_z', 0.0))
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
