import asyncio
import time
from collections import defaultdict
from functools import wraps

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, field_validator
from typing import List, Optional

from agv_interfaces.msg import AGVStatus, PlcData, IOState, YoloResult, WiFiStatus, BluetoothDevice, BatteryState, Scan3DData
from agv_interfaces.srv import ControlAGV, ReadPlc, WritePlc, SetIO, TrainModel, ConnectWiFi, ConnectBluetooth, SetCharging, SetModel, GenerateScanMap, StartScan
from agv_interfaces.action import NavigateTo, Patrol
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from agv_web_server.config_manager import ConfigManager
from agv_web_server.camera_manager import CameraManager
from agv_web_server.plc_manager import PlcManager

# 初始化管理器
config_mgr = ConfigManager()
camera_mgr = CameraManager()
plc_mgr = PlcManager()


class ControlCommand(BaseModel):
    command: str
    parameters: List[str] = []

    @field_validator('command')
    @classmethod
    def validate_command(cls, v):
        allowed = {'start', 'stop', 'pause', 'resume', 'charge', 'emergency_stop'}
        if v.lower() not in allowed:
            raise ValueError(f'Invalid command: {v}. Allowed: {allowed}')
        return v.lower()


class CmdVelRequest(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @field_validator('linear_x', 'linear_y', 'angular_z')
    @classmethod
    def validate_velocity(cls, v):
        if abs(v) > 10.0:
            raise ValueError('Velocity value out of range (-10.0, 10.0)')
        return v


class NavigateRequest(BaseModel):
    target_x: float = 0.0
    target_y: float = 0.0
    target_theta: float = 0.0


class PatrolRequest(BaseModel):
    waypoints_x: List[float] = []
    waypoints_y: List[float] = []
    waypoints_theta: List[float] = []
    loops: int = 1

    @field_validator('loops')
    @classmethod
    def validate_loops(cls, v):
        if v < 1:
            raise ValueError('Loops must be at least 1')
        return v


class PlcReadRequest(BaseModel):
    device_name: str = ""
    ip_address: str = ""
    start_address: int = 0
    quantity: int = 1

    @field_validator('quantity')
    @classmethod
    def validate_quantity(cls, v):
        if v < 1 or v > 125:
            raise ValueError('Quantity must be between 1 and 125')
        return v


class PlcWriteRequest(BaseModel):
    device_name: str = ""
    ip_address: str = ""
    start_address: int = 0
    values: List[int] = []

    @field_validator('values')
    @classmethod
    def validate_values(cls, v):
        if len(v) > 125:
            raise ValueError('Maximum 125 values allowed')
        return v


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

    @field_validator('epochs')
    @classmethod
    def validate_epochs(cls, v):
        if v < 1:
            raise ValueError('Epochs must be at least 1')
        return v

    @field_validator('learning_rate')
    @classmethod
    def validate_learning_rate(cls, v):
        if v <= 0.0 or v > 1.0:
            raise ValueError('Learning rate must be between 0 and 1')
        return v


class WiFiConnectRequest(BaseModel):
    ssid: str = ""
    password: str = ""

    @field_validator('ssid')
    @classmethod
    def validate_ssid(cls, v):
        if not v.strip():
            raise ValueError('SSID cannot be empty')
        return v


class BluetoothConnectRequest(BaseModel):
    address: str = ""
    profile: str = "spp"

    @field_validator('address')
    @classmethod
    def validate_address(cls, v):
        if not v.strip():
            raise ValueError('Bluetooth address cannot be empty')
        return v


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


class RateLimiter:
    def __init__(self, max_requests=60, window_seconds=60):
        self._max_requests = max_requests
        self._window = window_seconds
        self._requests = defaultdict(list)

    def is_allowed(self, key):
        now = time.time()
        self._requests[key] = [t for t in self._requests[key] if now - t < self._window]
        if len(self._requests[key]) >= self._max_requests:
            return False
        self._requests[key].append(now)
        return True


_rate_limiter = RateLimiter(max_requests=60, window_seconds=60)
_response_cache = {}
_cache_ttl = 0.5
_cache_timestamps = {}


def _get_cached_response(cache_key):
    now = time.time()
    if cache_key in _response_cache:
        if now - _cache_timestamps.get(cache_key, 0) < _cache_ttl:
            return _response_cache[cache_key]
    return None


def _set_cached_response(cache_key, data):
    now = time.time()
    _response_cache[cache_key] = data
    _cache_timestamps[cache_key] = now


def create_api_router(ros_bridge):
    router = APIRouter(prefix="/api/v1")

    ros_bridge.create_subscription('/agv_status', AGVStatus)
    ros_bridge.create_subscription('/plc_data', PlcData)
    ros_bridge.create_accumulating_subscription('/io_states', IOState, 'io_name')
    ros_bridge.create_subscription('/yolo_result', YoloResult)
    ros_bridge.create_subscription('/wifi_status', WiFiStatus)
    ros_bridge.create_accumulating_subscription('/bluetooth_devices', BluetoothDevice, 'address')
    ros_bridge.create_subscription('/battery_state', BatteryState)
    ros_bridge.create_subscription('/scan_3d_data', Scan3DData)

    ros_bridge.create_publisher('/cmd_vel', Twist)

    @router.middleware("http")
    async def rate_limit_middleware(request: Request, call_next):
        client_id = request.client.host if request.client else "unknown"
        if not _rate_limiter.is_allowed(client_id):
            raise HTTPException(status_code=429, detail='Rate limit exceeded')
        response = await call_next(request)
        return response

    @router.get('/agv/status')
    async def get_agv_status():
        cached = _get_cached_response('agv_status')
        if cached is not None:
            return cached
        msg = ros_bridge.get_latest_message('/agv_status')
        if msg is None:
            raise HTTPException(status_code=404, detail='AGV status not available')
        result = _msg_to_dict(msg)
        _set_cached_response('agv_status', result)
        return result

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
        cached = _get_cached_response('plc_status')
        if cached is not None:
            return cached
        msg = ros_bridge.get_latest_message('/plc_data')
        if msg is None:
            raise HTTPException(status_code=404, detail='PLC data not available')
        result = _msg_to_dict(msg)
        _set_cached_response('plc_status', result)
        return result

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
        cached = _get_cached_response('io_states')
        if cached is not None:
            return cached
        accumulated = ros_bridge.get_accumulated_messages('/io_states')
        if not accumulated:
            msg = ros_bridge.get_latest_message('/io_states')
            if msg is None:
                raise HTTPException(status_code=404, detail='IO states not available')
            result = [_msg_to_dict(msg)]
        else:
            result = [_msg_to_dict(m) for m in accumulated.values()]
        _set_cached_response('io_states', result)
        return result

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
        cached = _get_cached_response('vision_detections')
        if cached is not None:
            return cached
        msg = ros_bridge.get_latest_message('/yolo_result')
        if msg is None:
            raise HTTPException(status_code=404, detail='Vision detections not available')
        result = _msg_to_dict(msg)
        _set_cached_response('vision_detections', result)
        return result

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
        cached = _get_cached_response('wifi_status')
        if cached is not None:
            return cached
        msg = ros_bridge.get_latest_message('/wifi_status')
        if msg is None:
            raise HTTPException(status_code=404, detail='WiFi status not available')
        result = _msg_to_dict(msg)
        _set_cached_response('wifi_status', result)
        return result

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
        cached = _get_cached_response('bluetooth_devices')
        if cached is not None:
            return cached
        accumulated = ros_bridge.get_accumulated_messages('/bluetooth_devices')
        if not accumulated:
            msg = ros_bridge.get_latest_message('/bluetooth_devices')
            if msg is None:
                raise HTTPException(status_code=404, detail='Bluetooth devices not available')
            result = [_msg_to_dict(msg)]
        else:
            result = [_msg_to_dict(m) for m in accumulated.values()]
        _set_cached_response('bluetooth_devices', result)
        return result

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
        cached = _get_cached_response('system_info')
        if cached is not None:
            return cached
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

        result_data = {
            'ros_version': ros_version,
            'node_list': node_list,
            'topic_list': topic_list,
        }
        _set_cached_response('system_info', result_data)
        return result_data

    @router.get('/system/health')
    async def get_system_health():
        ros_bridge.check_health()
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

    @router.get('/power/status')
    async def get_power_status():
        cached = _get_cached_response('power_status')
        if cached is not None:
            return cached
        msg = ros_bridge.get_latest_message('/battery_state')
        if msg is None:
            return {
                'voltage': 0.0, 'current': 0.0, 'charge_level': 0.0,
                'temperature': 0.0, 'health_percent': 0.0, 'charging_state': 'unknown',
                'charge_rate': 0.0, 'discharge_rate': 0.0, 'estimated_time_remaining': 0.0,
                'charge_cycles': 0, 'battery_type': 'unknown'
            }
        result = _msg_to_dict(msg)
        _set_cached_response('power_status', result)
        return result

    @router.post('/power/charging')
    async def set_charging(body: dict):
        command = body.get('command', 'start_charging')
        future = ros_bridge.call_service('/set_charging', SetCharging, {
            'command': command,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /set_charging not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /set_charging timed out')
        return {'success': response.success, 'message': response.message, 'charge_level': response.charge_level}

    @router.post('/power/mode')
    async def set_power_mode(body: dict):
        mode = body.get('model_path', body.get('mode', 'balanced'))
        future = ros_bridge.call_service('/set_power_mode', SetModel, {
            'model_path': mode,
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /set_power_mode not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /set_power_mode timed out')
        return {'success': response.success, 'message': response.message}

    @router.get('/scan3d/points')
    async def get_scan_points():
        cached = _get_cached_response('scan_points')
        if cached is not None:
            return cached
        msg = ros_bridge.get_latest_message('/scan_3d_data')
        if msg is None:
            return {'points_x': [], 'points_y': [], 'points_z': [], 'num_points': 0}
        result = _msg_to_dict(msg)
        _set_cached_response('scan_points', result)
        return result

    @router.post('/scan3d/start')
    async def start_scan(body: dict):
        future = ros_bridge.call_service('/start_scan', StartScan, {
            'scan_pattern': body.get('scan_pattern', 'path'),
            'scan_resolution': body.get('scan_resolution', 0.05),
            'max_points': body.get('max_points', 100000),
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /start_scan not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /start_scan timed out')
        return {'success': response.success, 'message': response.message}

    @router.post('/scan3d/stop')
    async def stop_scan():
        future = ros_bridge.call_service('/stop_scan', Trigger, {})
        if future is None:
            raise HTTPException(status_code=503, detail='Service /stop_scan not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=10.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /stop_scan timed out')
        return {'success': response.success, 'message': response.message}

    @router.post('/scan3d/generate_map')
    async def generate_map(body: dict):
        future = ros_bridge.call_service('/generate_scan_map', GenerateScanMap, {
            'map_name': body.get('map_name', 'path_scan'),
            'export_path': body.get('export_path', '/tmp'),
            'format': body.get('format', 'xyz'),
            'include_path': body.get('include_path', True),
        })
        if future is None:
            raise HTTPException(status_code=503, detail='Service /generate_scan_map not available')
        try:
            response = await asyncio.wait_for(asyncio.wrap_future(future), timeout=30.0)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail='Service /generate_scan_map timed out')
        return {'success': response.success, 'message': response.message, 'output_file': response.output_file, 'total_points': response.total_points, 'process_time': response.process_time}

    # 摄像头配置API
    @router.get('/camera/config')
    async def get_camera_config():
        config = config_mgr.get('camera', {})
        return config

    @router.post('/camera/config')
    async def set_camera_config(body: dict):
        config_mgr.set('camera', body)
        camera_mgr.update_config(body)
        return {'success': True, 'message': 'Camera config updated'}

    @router.get('/camera/preview')
    async def get_camera_preview():
        preview = camera_mgr.get_preview_base64()
        return {'image': preview}

    # PLC配置API
    @router.get('/plc/config')
    async def get_plc_config():
        devices = config_mgr.get('plc.devices', [])
        return {'devices': devices}

    @router.post('/plc/config')
    async def set_plc_config(body: dict):
        devices = body.get('devices', [])
        plc_mgr.update_config(devices)
        return {'success': True, 'message': 'PLC config updated'}

    @router.get('/plc/status')
    async def get_plc_status_new():
        devices = plc_mgr.get_devices_status()
        return {'devices': devices}

    @router.post('/plc/send_slave')
    async def send_slave_command(body: dict):
        plc_mgr.send_slave_command(body)
        return {'success': True, 'message': 'Slave command sent'}

    return router
