import asyncio
import json
import time
from collections import deque
from fastapi import WebSocket, WebSocketDisconnect

from agv_interfaces.msg import AGVStatus, YoloResult, IOState, PlcData
from geometry_msgs.msg import Twist


def _msg_to_dict(msg):
    if msg is None:
        return None
    result = {}
    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field)
        if hasattr(value, 'get_fields_and_field_types'):
            result[field] = _msg_to_dict(value)
        elif isinstance(value, (list, tuple)):
            result[field] = [_msg_to_dict(item) if hasattr(item, 'get_fields_and_field_types') else item for item in value]
        else:
            result[field] = value
    return result


class ClientMessageQueue:
    def __init__(self, max_size=100):
        self._queue = deque(maxlen=max_size)
        self._lock = asyncio.Lock()
        self._event = asyncio.Event()

    async def put(self, message):
        async with self._lock:
            if len(self._queue) >= self._queue.maxlen:
                self._queue.popleft()
            self._queue.append(message)
            self._event.set()

    async def get_all(self):
        async with self._lock:
            messages = list(self._queue)
            self._queue.clear()
            self._event.clear()
            return messages

    def put_sync(self, message):
        if len(self._queue) >= self._queue.maxlen:
            self._queue.popleft()
        self._queue.append(message)
        self._event.set()


def register_websocket(app, ros_bridge):
    ros_bridge.create_subscription('/agv_status', AGVStatus)
    ros_bridge.create_subscription('/yolo_result', YoloResult)
    ros_bridge.create_accumulating_subscription('/io_states', IOState, 'io_name')
    ros_bridge.create_subscription('/plc_data', PlcData)
    ros_bridge.create_publisher('/cmd_vel', Twist)

    _serialization_cache = {}
    _cache_timestamps = {}
    _cache_ttl = 0.1

    def _get_cached_serialization(topic, msg):
        now = time.time()
        if topic in _serialization_cache:
            if now - _cache_timestamps.get(topic, 0) < _cache_ttl:
                return _serialization_cache[topic]
        data = _msg_to_dict(msg)
        _serialization_cache[topic] = data
        _cache_timestamps[topic] = now
        return data

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
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

                msg = ros_bridge.get_latest_message('/agv_status')
                if msg is not None:
                    data = _get_cached_serialization('/agv_status', msg)
                    await msg_queue.put(data)

                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break

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
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

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
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

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

                messages = await msg_queue.get_all()
                for m in messages:
                    try:
                        await websocket.send_json(m)
                    except Exception:
                        break

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
        await websocket.accept()
        msg_queue = ClientMessageQueue(max_size=10)
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

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
        await websocket.accept()
        last_ping = time.time()
        ping_interval = 30.0
        last_pong = time.time()
        pong_timeout = 60.0
        try:
            while True:
                now = time.time()
                if now - last_ping > ping_interval:
                    try:
                        await websocket.send_json({'type': 'ping'})
                        last_ping = now
                    except Exception:
                        break

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
