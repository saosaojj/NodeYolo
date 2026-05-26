#!/usr/bin/env python3
import threading
import time
from .config_manager import ConfigManager

try:
    import pymodbus
    from pymodbus.client import ModbusTcpClient
    HAS_PYMODBUS = True
except ImportError:
    HAS_PYMODBUS = False
    print('[PlcManager] pymodbus 未安装，PLC功能不可用')


class PlcDevice:
    def __init__(self, config):
        self.name = config.get('name', 'unknown')
        self.ip = config.get('ip', '127.0.0.1')
        self.port = config.get('port', 502)
        self.slave_id = config.get('slave_id', 1)
        self.is_master = config.get('is_master', True)
        self.coil_read_start = config.get('coil_read_start', 0)
        self.coil_read_count = config.get('coil_read_count', 16)
        self.register_read_start = config.get('register_read_start', 0)
        self.register_read_count = config.get('register_read_count', 16)
        self.connected = False
        self.client = None
        self.coils = []
        self.registers = []
        self.lock = threading.Lock()

    def connect(self):
        if self.client:
            try:
                self.client.close()
            except:
                pass
        try:
            if HAS_PYMODBUS:
                self.client = ModbusTcpClient(self.ip, port=self.port, timeout=2)
                if self.client.connect():
                    self.connected = True
                    print(f'[PlcManager] PLC {self.name} 连接成功')
                else:
                    self.connected = False
                    print(f'[PlcManager] PLC {self.name} 连接失败')
            else:
                self.connected = False
        except Exception as e:
            print(f'[PlcManager] PLC {self.name} 连接异常: {e}')
            self.connected = False

    def disconnect(self):
        if self.client:
            try:
                self.client.close()
            except:
                pass
        self.connected = False

    def read_coils(self):
        if not self.connected or not self.client:
            return []
        try:
            result = self.client.read_coils(
                address=self.coil_read_start,
                count=self.coil_read_count,
                slave=self.slave_id
            )
            if not result.isError():
                self.coils = list(result.bits)[:self.coil_read_count]
                return self.coils
        except Exception as e:
            print(f'[PlcManager] 读取线圈失败: {e}')
            self.connected = False
        return []

    def read_registers(self):
        if not self.connected or not self.client:
            return []
        try:
            result = self.client.read_holding_registers(
                address=self.register_read_start,
                count=self.register_read_count,
                slave=self.slave_id
            )
            if not result.isError():
                self.registers = list(result.registers)
                return self.registers
        except Exception as e:
            print(f'[PlcManager] 读取寄存器失败: {e}')
            self.connected = False
        return []

    def write_coil(self, address, value):
        if not self.connected or not self.client:
            return False
        try:
            result = self.client.write_coil(
                address=address,
                value=value,
                slave=self.slave_id
            )
            return not result.isError()
        except Exception as e:
            print(f'[PlcManager] 写线圈失败: {e}')
            self.connected = False
        return False

    def write_register(self, address, value):
        if not self.connected or not self.client:
            return False
        try:
            result = self.client.write_register(
                address=address,
                value=value,
                slave=self.slave_id
            )
            return not result.isError()
        except Exception as e:
            print(f'[PlcManager] 写寄存器失败: {e}')
            self.connected = False
        return False

    def to_dict(self):
        return {
            'name': self.name,
            'ip': self.ip,
            'port': self.port,
            'slave_id': self.slave_id,
            'is_master': self.is_master,
            'connected': self.connected,
            'coils': self.coils,
            'registers': self.registers
        }


class PlcManager:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            self.config_mgr = ConfigManager()
            self.devices = []
            self._running = False
            self._thread = None
            self._lock = threading.Lock()
            self._load_devices()
            self._initialized = True

    def _load_devices(self):
        cfg = self.config_mgr.get('plc.devices', [])
        self.devices = [PlcDevice(c) for c in cfg]

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print('[PlcManager] 已启动')

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self):
        while self._running:
            with self._lock:
                for dev in self.devices:
                    if dev.is_master:
                        if not dev.connected:
                            dev.connect()
                        if dev.connected:
                            dev.read_coils()
                            dev.read_registers()
            time.sleep(0.5)

    def get_devices_status(self):
        with self._lock:
            return [d.to_dict() for d in self.devices]

    def update_config(self, devices_cfg):
        with self._lock:
            self.config_mgr.set('plc.devices', devices_cfg)
            self._load_devices()

    def send_slave_command(self, cmd):
        linear_x = cmd.get('linear_x', 0.0)
        linear_y = cmd.get('linear_y', 0.0)
        angular_z = cmd.get('angular_z', 0.0)
        # 将速度转换为寄存器值，这里假设将浮点数乘以1000存储为整数
        with self._lock:
            for dev in self.devices:
                if not dev.is_master and dev.connected:
                    try:
                        dev.write_register(0, int(linear_x * 1000))
                        dev.write_register(1, int(linear_y * 1000))
                        dev.write_register(2, int(angular_z * 1000))
                    except:
                        pass
        print(f'[PlcManager] 发送从站命令: x={linear_x}, y={linear_y}, ang={angular_z}')
