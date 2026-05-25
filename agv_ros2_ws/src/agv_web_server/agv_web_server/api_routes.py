import asyncio
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional

from agv_interfaces.msg import AGVStatus, PlcData, IOState, YoloResult, WiFiStatus, BluetoothDevice
from agv_interfaces.srv import ControlAGV, ReadPlc, WritePlc, SetIO, TrainModel, ConnectWiFi, ConnectBluetooth
from agv_interfaces.action import NavigateTo, Patrol
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class ControlCommand(BaseModel):
    command: str
    parameters: List[str] = []


class CmdVelRequest(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class NavigateRequest(BaseModel):
    target_x: float = 0.0
    target_y: float = 0.0
    target_theta: float = 0.0


class PatrolRequest(BaseModel):
    waypoints_x: List[float] = []
    waypoints_y: List[float] = []
    waypoints_theta: List[float] = []
    loops: int = 1


class PlcReadRequest(BaseModel):
    device_name: str = ""
    ip_address: str = ""
    start_address: int = 0
    quantity: int = 1


class PlcWriteRequest(BaseModel):
    device_name: str = ""
    ip_address: str = ""
    start_address: int = 0
    values: List[int] = []


class IOSetRequest(BaseModel):
    io_name: str = ""
    io_type: str = ""
    pin_number: int = 0
    value: float = 0.0


class TrainRequest(BaseModel):
    dataset_path: str = ""
    model_type: str = "yolov8n"
    epochs: int = 100
    learning_rate: float = 0.01
    output_path: str = ""


class WiFiConnectRequest(BaseModel):
    ssid: str = ""
    password: str = ""


class BluetoothConnectRequest(BaseModel):
    address: str = ""
    profile: str = "spp"


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


def create_api_router(ros_bridge):
    router = APIRouter(prefix="/api/v1")

    ros_bridge.create_subscription('/agv_status', AGVStatus)
    ros_bridge.create_subscription('/plc_data', PlcData)
    ros_bridge.create_accumulating_subscription('/io_states', IOState, 'io_name')
    ros_bridge.create_subscription('/yolo_result', YoloResult)
    ros_bridge.create_subscription('/wifi_status', WiFiStatus)
    ros_bridge.create_accumulating_subscription('/bluetooth_devices', BluetoothDevice, 'address')

    ros_bridge.create_publisher('/cmd_vel', Twist)

    @router.get('/agv/status')
    async def get_agv_status():
        msg = ros_bridge.get_latest_message('/agv_status')
        if msg is None:
            raise HTTPException(status_code=404, detail='AGV status not available')
        return _msg_to_dict(msg)

    @router.post('/agv/control')
    async def control_agv(body: ControlCommand):
        future = ros_bridge.call_service('/control_agv', ControlAGV, {
            'command': body.command,
            'parameters': body.parameters,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /control_agv not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /control_agv timed out')
        return {'success': response.success, 'message': response.message}

    @router.post('/agv/cmd_vel')
    async def publish_cmd_vel(body: CmdVelRequest):
        ros_bridge.publish('/cmd_vel', {
            'linear': {'x': body.linear_x, 'y': body.linear_y, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': body.angular_z},
        })
        return {'success': True, 'message': 'Command velocity published'}

    @router.post('/agv/navigate')
    async def navigate_to(body: NavigateRequest):
        send_goal_future = ros_bridge.call_action('/navigate_to', NavigateTo, {
            'target_x': body.target_x,
            'target_y': body.target_y,
            'target_theta': body.target_theta,
        })
        if send_goal_future is None:
            raise HTTPException(status_code=503, detail='Action server /navigate_to not available')
        try:
            goal_handle = await asyncio.wait_for(asyncio.wrap_future(send_goal_future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Action /navigate_to timed out')
        if not goal_handle.accepted:
            return {'success': False, 'message': goal_handle.status_message}
        return {'success': True, 'message': 'Navigation goal accepted'}

    @router.post('/agv/patrol')
    async def patrol(body: PatrolRequest):
        send_goal_future = ros_bridge.call_action('/patrol', Patrol, {
            'waypoints_x': body.waypoints_x,
            'waypoints_y': body.waypoints_y,
            'waypoints_theta': body.waypoints_theta,
            'loops': body.loops,
        })
        if send_goal_future is None:
            raise HTTPException(status_code=503, detail='Action server /patrol not available')
        try:
            goal_handle = await asyncio.wait_for(asyncio.wrap_future(send_goal_future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Action /patrol timed out')
        if not goal_handle.accepted:
            return {'success': False, 'message': goal_handle.status_message}
        return {'success': True, 'message': 'Patrol goal accepted'}

    @router.get('/plc/status')
    async def get_plc_status():
        msg = ros_bridge.get_latest_message('/plc_data')
        if msg is None:
            raise HTTPException(status_code=404, detail='PLC data not available')
        return _msg_to_dict(msg)

    @router.post('/plc/read')
    async def read_plc(body: PlcReadRequest):
        future = ros_bridge.call_service('/read_plc', ReadPlc, {
            'device_name': body.device_name,
            'ip_address': body.ip_address,
            'start_address': body.start_address,
            'quantity': body.quantity,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /read_plc not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /read_plc timed out')
        return {'success': response.success, 'message': response.message, 'values': list(response.values)}

    @router.post('/plc/write')
    async def write_plc(body: PlcWriteRequest):
        future = ros_bridge.call_service('/write_plc', WritePlc, {
            'device_name': body.device_name,
            'ip_address': body.ip_address,
            'start_address': body.start_address,
            'values': body.values,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /write_plc not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /write_plc timed out')
        return {'success': response.success, 'message': response.message}

    @router.get('/io/states')
    async def get_io_states():
        accumulated = ros_bridge.get_accumulated_messages('/io_states')
        if not accumulated:
            msg = ros_bridge.get_latest_message('/io_states')
            if msg is None:
                raise HTTPException(status_code=404, detail='IO states not available')
            return [_msg_to_dict(msg)]
        return [_msg_to_dict(m) for m in accumulated.values()]

    @router.post('/io/set')
    async def set_io(body: IOSetRequest):
        future = ros_bridge.call_service('/set_io', SetIO, {
            'io_name': body.io_name,
            'io_type': body.io_type,
            'pin_number': body.pin_number,
            'value': body.value,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /set_io not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /set_io timed out')
        return {'success': response.success, 'message': response.message}

    @router.get('/vision/detections')
    async def get_vision_detections():
        msg = ros_bridge.get_latest_message('/yolo_result')
        if msg is None:
            raise HTTPException(status_code=404, detail='Vision detections not available')
        return _msg_to_dict(msg)

    @router.post('/vision/train')
    async def train_model(body: TrainRequest):
        future = ros_bridge.call_service('/train_model', TrainModel, {
            'dataset_path': body.dataset_path,
            'model_type': body.model_type,
            'epochs': body.epochs,
            'learning_rate': body.learning_rate,
            'output_path': body.output_path,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /train_model not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=300.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /train_model timed out')
        return {
            'success': response.success,
            'message': response.message,
            'model_path': response.model_path,
            'training_time': response.training_time,
        }

    @router.get('/wifi/status')
    async def get_wifi_status():
        msg = ros_bridge.get_latest_message('/wifi_status')
        if msg is None:
            raise HTTPException(status_code=404, detail='WiFi status not available')
        return _msg_to_dict(msg)

    @router.post('/wifi/connect')
    async def connect_wifi(body: WiFiConnectRequest):
        future = ros_bridge.call_service('/connect_wifi', ConnectWiFi, {
            'ssid': body.ssid,
            'password': body.password,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /connect_wifi not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=30.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /connect_wifi timed out')
        return {'success': response.success, 'message': response.message, 'ip_address': response.ip_address}

    @router.post('/wifi/scan')
    async def scan_wifi():
        future = ros_bridge.call_service('/scan_wifi', Trigger, {})
        if future is None:
            raise HTTPException(status_code=503, detail='Service /scan_wifi not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=30.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /scan_wifi timed out')
        return {'success': response.success, 'message': response.message}

    @router.get('/bluetooth/devices')
    async def get_bluetooth_devices():
        accumulated = ros_bridge.get_accumulated_messages('/bluetooth_devices')
        if not accumulated:
            msg = ros_bridge.get_latest_message('/bluetooth_devices')
            if msg is None:
                raise HTTPException(status_code=404, detail='Bluetooth devices not available')
            return [_msg_to_dict(msg)]
        return [_msg_to_dict(m) for m in accumulated.values()]

    @router.post('/bluetooth/connect')
    async def connect_bluetooth(body: BluetoothConnectRequest):
        future = ros_bridge.call_service('/connect_bluetooth', ConnectBluetooth, {
            'address': body.address,
            'profile': body.profile,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /connect_bluetooth not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=20.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /connect_bluetooth timed out')
        return {'success': response.success, 'message': response.message}

    @router.post('/bluetooth/scan')
    async def scan_bluetooth():
        future = ros_bridge.call_service('/scan_bluetooth', Trigger, {})
        if future is None:
            raise HTTPException(status_code=503, detail='Service /scan_bluetooth not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=30.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /scan_bluetooth timed out')
        return {'success': response.success, 'message': response.message}

    @router.get('/system/info')
    async def get_system_info():
        import subprocess
        ros_version = ''
        try:
            result = subprocess.run(['ros2', '--version'], capture_output=True, text=True, timeout=5)
            ros_version = result.stdout.strip()
        except Exception:
            ros_version = 'unknown'

        node_list = []
        try:
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
            node_list = [n for n in result.stdout.strip().split('\n') if n]
        except Exception:
            pass

        topic_list = []
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
            topic_list = [t for t in result.stdout.strip().split('\n') if t]
        except Exception:
            pass

        return {
            'ros_version': ros_version,
            'node_list': node_list,
            'topic_list': topic_list,
        }

    @router.get('/system/health')
    async def get_system_health():
        agv_status = ros_bridge.get_latest_message('/agv_status')
        plc_data = ros_bridge.get_latest_message('/plc_data')
        wifi_status = ros_bridge.get_latest_message('/wifi_status')
        yolo_result = ros_bridge.get_latest_message('/yolo_result')

        health = {
            'agv': {
                'status': 'ok' if agv_status is not None else 'unavailable',
                'emergency_stop': agv_status.emergency_stop if agv_status else None,
                'battery_level': agv_status.battery_level if agv_status else None,
                'mode': agv_status.mode if agv_status else None,
            },
            'plc': {
                'status': 'ok' if plc_data is not None else 'unavailable',
                'connected': plc_data.connected if plc_data else None,
            },
            'wifi': {
                'status': 'ok' if wifi_status is not None else 'unavailable',
                'connected': wifi_status.connected if wifi_status else None,
            },
            'vision': {
                'status': 'ok' if yolo_result is not None else 'unavailable',
            },
        }

        all_ok = all(
            v.get('status') == 'ok' for v in health.values()
        )
        health['overall'] = 'healthy' if all_ok else 'degraded'

        return health

    return router
