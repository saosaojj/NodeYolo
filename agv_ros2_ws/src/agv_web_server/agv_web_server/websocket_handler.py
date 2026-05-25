import asyncio
import json
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


def register_websocket(app, ros_bridge):
    ros_bridge.create_subscription('/agv_status', AGVStatus)
    ros_bridge.create_subscription('/yolo_result', YoloResult)
    ros_bridge.create_accumulating_subscription('/io_states', IOState, 'io_name')
    ros_bridge.create_subscription('/plc_data', PlcData)
    ros_bridge.create_publisher('/cmd_vel', Twist)

    @app.websocket('/ws/agv_status')
    async def ws_agv_status(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                msg = ros_bridge.get_latest_message('/agv_status')
                if msg is not None:
                    await websocket.send_json(_msg_to_dict(msg))
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.websocket('/ws/yolo_result')
    async def ws_yolo_result(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                msg = ros_bridge.get_latest_message('/yolo_result')
                if msg is not None:
                    await websocket.send_json(_msg_to_dict(msg))
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.websocket('/ws/io_states')
    async def ws_io_states(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                accumulated = ros_bridge.get_accumulated_messages('/io_states')
                if accumulated:
                    data = [_msg_to_dict(m) for m in accumulated.values()]
                    await websocket.send_json(data)
                else:
                    msg = ros_bridge.get_latest_message('/io_states')
                    if msg is not None:
                        await websocket.send_json([_msg_to_dict(msg)])
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.websocket('/ws/plc_data')
    async def ws_plc_data(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                msg = ros_bridge.get_latest_message('/plc_data')
                if msg is not None:
                    await websocket.send_json(_msg_to_dict(msg))
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    @app.websocket('/ws/cmd_vel')
    async def ws_cmd_vel(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                data = await websocket.receive_text()
                try:
                    cmd = json.loads(data)
                    linear_x = float(cmd.get('linear_x', 0.0))
                    linear_y = float(cmd.get('linear_y', 0.0))
                    angular_z = float(cmd.get('angular_z', 0.0))
                    ros_bridge.publish('/cmd_vel', {
                        'linear': {'x': linear_x, 'y': linear_y, 'z': 0.0},
                        'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z},
                    })
                except (json.JSONDecodeError, ValueError, KeyError):
                    await websocket.send_json({'error': 'Invalid command format'})
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
