#!/usr/bin/env python3
"""
PLC管理模块

提供PLC设备的连接、读写和状态管理功能，
支持Modbus TCP协议，支持多设备配置。
"""

import threading
import time
from agv_web_server.config_manager import ConfigManager

# 尝试导入pymodbus库
try:
    import pymodbus
    from pymodbus.client import ModbusTcpClient
    HAS_PYMODBUS = True
except ImportError:
    HAS_PYMODBUS = False
    print('[PlcManager] pymodbus 未安装，PLC功能不可用')


class PlcDevice:
    """
    PLC设备类
    
    表示一个单独的PLC设备，负责管理该设备的连接和数据读写。
    """

    def __init__(self, config):
        """
        初始化PLC设备
        
        Args:
            config: 设备配置字典，包含name, ip, port, slave_id等
        """
        # 设备名称
        self.name = config.get('name', 'unknown')
        # PLC的IP地址
        self.ip = config.get('ip', '127.0.0.1')
        # 端口号（Modbus TCP默认502）
        self.port = config.get('port', 502)
        # Modbus从站地址
        self.slave_id = config.get('slave_id', 1)
        # 是否作为主站（主动读取数据）
        self.is_master = config.get('is_master', True)
        # 线圈读取起始地址
        self.coil_read_start = config.get('coil_read_start', 0)
        # 线圈读取数量
        self.coil_read_count = config.get('coil_read_count', 16)
        # 寄存器读取起始地址
        self.register_read_start = config.get('register_read_start', 0)
        # 寄存器读取数量
        self.register_read_count = config.get('register_read_count', 16)
        
        # 连接状态
        self.connected = False
        # Modbus客户端对象
        self.client = None
        # 读取到的线圈状态
        self.coils = []
        # 读取到的寄存器值
        self.registers = []
        # 线程安全锁
        self.lock = threading.Lock()

    def connect(self):
        """
        连接到PLC设备
        
        Returns:
            bool: 是否成功连接
        """
        # 先断开之前的连接
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass

        try:
            if HAS_PYMODBUS:
                # 创建Modbus TCP客户端
                self.client = ModbusTcpClient(self.ip, port=self.port, timeout=2)
                # 尝试连接
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
        """
        断开PLC设备连接
        """
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        self.connected = False

    def read_coils(self):
        """
        读取PLC线圈状态
        
        Returns:
            list: 线圈状态列表
        """
        if not self.connected or not self.client:
            return []
        try:
            # 使用Modbus功能码01读取线圈
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
        """
        读取PLC保持寄存器
        
        Returns:
            list: 寄存器值列表
        """
        if not self.connected or not self.client:
            return []
        try:
            # 使用Modbus功能码03读取保持寄存器
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
        """
        写入单个线圈
        
        Args:
            address: 线圈地址
            value: 线圈值（True/False）
        
        Returns:
            bool: 是否写入成功
        """
        if not self.connected or not self.client:
            return False
        try:
            # 使用Modbus功能码05写入单个线圈
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
        """
        写入单个保持寄存器
        
        Args:
            address: 寄存器地址
            value: 寄存器值（整数）
        
        Returns:
            bool: 是否写入成功
        """
        if not self.connected or not self.client:
            return False
        try:
            # 使用Modbus功能码06写入单个寄存器
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
        """
        将设备信息转换为字典格式
        
        Returns:
            dict: 设备信息字典
        """
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
    """
    PLC管理器单例类
    
    负责管理多个PLC设备，定期读取主站数据，并提供统一的控制接口。
    """
    
    # 单例实例
    _instance = None
    # 单例创建锁
    _lock = threading.Lock()

    def __new__(cls):
        """
        单例模式的实现：确保只创建一个PlcManager实例
        """
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """
        初始化PLC管理器
        """
        if not hasattr(self, '_initialized'):
            # 配置管理器实例
            self.config_mgr = ConfigManager()
            # PLC设备列表
            self.devices = []
            # 管理器运行标志
            self._running = False
            # 监控线程
            self._thread = None
            # 线程锁
            self._lock = threading.Lock()
            # 加载设备配置
            self._load_devices()
            # 初始化完成标志
            self._initialized = True

    def _load_devices(self):
        """
        从配置加载PLC设备
        """
        # 获取设备配置列表
        device_configs = self.config_mgr.get('plc.devices', [])
        # 创建PlcDevice对象
        self.devices = [PlcDevice(config) for config in device_configs]

    def start(self):
        """
        启动PLC管理器，开始定期读取数据
        """
        if self._running:
            return
        self._running = True
        # 启动监控线程
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print('[PlcManager] 已启动')

    def stop(self):
        """
        停止PLC管理器
        """
        self._running = False
        # 等待线程结束
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self):
        """
        PLC监控循环（运行在后台线程）
        
        定期连接和读取主站PLC的数据
        """
        while self._running:
            with self._lock:
                for dev in self.devices:
                    if dev.is_master:
                        # 如果是主站且未连接，尝试连接
                        if not dev.connected:
                            dev.connect()
                        # 如果已连接，定期读取数据
                        if dev.connected:
                            dev.read_coils()
                            dev.read_registers()
            # 间隔500毫秒
            time.sleep(0.5)

    def get_devices_status(self):
        """
        获取所有设备的状态信息
        
        Returns:
            list: 设备状态字典列表
        """
        with self._lock:
            return [dev.to_dict() for dev in self.devices]

    def update_config(self, device_configs):
        """
        更新PLC设备配置
        
        Args:
            device_configs: 新的设备配置列表
        """
        with self._lock:
            # 更新配置
            self.config_mgr.set('plc.devices', device_configs)
            # 重新加载设备
            self._load_devices()

    def send_slave_command(self, command):
        """
        发送从站（小车）控制命令
        
        将速度命令写入所有从站PLC的寄存器
        
        Args:
            command: 命令字典，包含 linear_x, linear_y, angular_z
        """
        linear_x = command.get('linear_x', 0.0)
        linear_y = command.get('linear_y', 0.0)
        angular_z = command.get('angular_z', 0.0)
        
        # 将速度转换为寄存器值（乘以1000转为整数）
        with self._lock:
            for dev in self.devices:
                if not dev.is_master and dev.connected:
                    try:
                        # 写入速度到寄存器（假设：
                        # 寄存器0: linear_x * 1000
                        # 寄存器1: linear_y * 1000
                        # 寄存器2: angular_z * 1000）
                        dev.write_register(0, int(linear_x * 1000))
                        dev.write_register(1, int(linear_y * 1000))
                        dev.write_register(2, int(angular_z * 1000))
                    except Exception:
                        pass
        print(f'[PlcManager] 发送从站命令: x={linear_x}, y={linear_y}, ang={angular_z}')
