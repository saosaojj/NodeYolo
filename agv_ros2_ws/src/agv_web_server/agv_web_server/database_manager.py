#!/usr/bin/env python3
"""
数据库管理模块

使用SQLite提供数据持久化存储功能，
支持记录和查询AGV运行历史数据、配置变更等。
"""

import sqlite3
import json
import threading
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any, Union


class DatabaseManager:
    """
    数据库管理器单例类
    
    提供SQLite数据库的统一管理接口，包括：
    - 数据表创建和管理
    - 数据插入、查询、更新、删除
    - 配置变更历史记录
    - AGV运行状态历史
    """
    
    # 单例实例
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        """单例模式的实现"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        """初始化数据库管理器"""
        if hasattr(self, '_initialized'):
            return
        
        # 数据库文件路径
        self.db_path = Path.home() / '.agv_web_config' / 'agv_data.db'
        self.db_path.parent.mkdir(exist_ok=True)
        
        # 线程安全的连接锁
        self._connection_lock = threading.Lock()
        
        # 初始化数据库表
        self._initialize_database()
        
        self._initialized = True
        print(f"[DatabaseManager] 数据库初始化完成: {self.db_path}")
    
    def _get_connection(self) -> sqlite3.Connection:
        """
        获取数据库连接
        
        Returns:
            sqlite3.Connection: 数据库连接对象
        """
        conn = sqlite3.connect(str(self.db_path), check_same_thread=False)
        conn.row_factory = sqlite3.Row
        return conn
    
    def _initialize_database(self):
        """初始化数据库表结构"""
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            # 1. 配置历史表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS config_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    config_type TEXT NOT NULL,
                    old_config TEXT,
                    new_config TEXT,
                    change_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    changed_by TEXT DEFAULT 'system'
                )
            ''')
            
            # 2. AGV运行状态历史表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS agv_status_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    mode TEXT,
                    linear_x REAL,
                    linear_y REAL,
                    angular_z REAL,
                    position_x REAL,
                    position_y REAL,
                    orientation REAL,
                    battery_level REAL,
                    emergency_stop BOOLEAN,
                    error_message TEXT
                )
            ''')
            
            # 3. PLC数据历史表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS plc_data_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    device_name TEXT,
                    ip_address TEXT,
                    connected BOOLEAN,
                    coils_data TEXT,
                    registers_data TEXT
                )
            ''')
            
            # 4. 视觉检测结果历史表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS vision_detection_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    detections TEXT,
                    confidence_threshold REAL,
                    image_snapshot TEXT
                )
            ''')
            
            # 5. IO状态历史表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS io_state_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    io_name TEXT,
                    io_type TEXT,
                    pin_number INTEGER,
                    value REAL
                )
            ''')
            
            # 6. 仿真状态表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS simulation_state (
                    id INTEGER PRIMARY KEY,
                    simulation_enabled BOOLEAN DEFAULT 0,
                    camera_simulation BOOLEAN DEFAULT 0,
                    plc_simulation BOOLEAN DEFAULT 0,
                    vision_simulation BOOLEAN DEFAULT 0,
                    last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            
            # 插入默认仿真状态（只在第一次执行）
            cursor.execute('''
                INSERT OR IGNORE INTO simulation_state 
                (id, simulation_enabled, camera_simulation, plc_simulation, vision_simulation)
                VALUES (1, 0, 0, 0, 0)
            ''')
            
            # 创建索引以提高查询性能
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_config_time ON config_history(change_time)')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_agv_status_time ON agv_status_history(timestamp)')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_plc_data_time ON plc_data_history(timestamp)')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_vision_time ON vision_detection_history(timestamp)')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_io_time ON io_state_history(timestamp)')
            
            conn.commit()
            conn.close()
    
    # ==================== 配置历史相关方法 ====================
    
    def record_config_change(self, config_type: str, old_config: Optional[Dict], new_config: Dict, 
                            changed_by: str = 'system') -> int:
        """
        记录配置变更
        
        Args:
            config_type: 配置类型 (camera, plc, vision等)
            old_config: 旧配置
            new_config: 新配置
            changed_by: 变更者
            
        Returns:
            int: 插入记录的ID
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO config_history 
                (config_type, old_config, new_config, changed_by)
                VALUES (?, ?, ?, ?)
            ''', (
                config_type,
                json.dumps(old_config, ensure_ascii=False) if old_config else None,
                json.dumps(new_config, ensure_ascii=False),
                changed_by
            ))
            
            insert_id = cursor.lastrowid
            conn.commit()
            conn.close()
            
            return insert_id
    
    def get_config_history(self, config_type: Optional[str] = None, limit: int = 100) -> List[Dict]:
        """
        获取配置变更历史
        
        Args:
            config_type: 配置类型筛选，None表示全部
            limit: 返回记录数量限制
            
        Returns:
            配置变更历史列表
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            if config_type:
                cursor.execute('''
                    SELECT * FROM config_history 
                    WHERE config_type = ?
                    ORDER BY change_time DESC
                    LIMIT ?
                ''', (config_type, limit))
            else:
                cursor.execute('''
                    SELECT * FROM config_history 
                    ORDER BY change_time DESC
                    LIMIT ?
                ''', (limit,))
            
            rows = cursor.fetchall()
            conn.close()
            
            result = []
            for row in rows:
                result.append({
                    'id': row['id'],
                    'config_type': row['config_type'],
                    'old_config': json.loads(row['old_config']) if row['old_config'] else None,
                    'new_config': json.loads(row['new_config']),
                    'change_time': row['change_time'],
                    'changed_by': row['changed_by']
                })
            
            return result
    
    # ==================== AGV状态相关方法 ====================
    
    def record_agv_status(self, status: Dict) -> int:
        """
        记录AGV运行状态
        
        Args:
            status: AGV状态字典
            
        Returns:
            int: 插入记录的ID
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO agv_status_history 
                (mode, linear_x, linear_y, angular_z, position_x, position_y, 
                 orientation, battery_level, emergency_stop, error_message)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                status.get('mode', 'manual'),
                status.get('linear_x', 0.0),
                status.get('linear_y', 0.0),
                status.get('angular_z', 0.0),
                status.get('position_x', 0.0),
                status.get('position_y', 0.0),
                status.get('orientation', 0.0),
                status.get('battery_level', 0.0),
                status.get('emergency_stop', False),
                status.get('error_message', '')
            ))
            
            insert_id = cursor.lastrowid
            conn.commit()
            conn.close()
            
            return insert_id
    
    def get_agv_status_history(self, start_time: Optional[str] = None, 
                               end_time: Optional[str] = None, limit: int = 1000) -> List[Dict]:
        """
        获取AGV状态历史
        
        Args:
            start_time: 开始时间，格式: "YYYY-MM-DD HH:MM:SS"
            end_time: 结束时间
            limit: 记录数量限制
            
        Returns:
            AGV状态历史列表
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            query = 'SELECT * FROM agv_status_history'
            params = []
            
            if start_time or end_time:
                query += ' WHERE'
                if start_time:
                    query += ' timestamp >= ?'
                    params.append(start_time)
                if end_time:
                    if start_time:
                        query += ' AND'
                    query += ' timestamp <= ?'
                    params.append(end_time)
            
            query += ' ORDER BY timestamp DESC LIMIT ?'
            params.append(limit)
            
            cursor.execute(query, params)
            rows = cursor.fetchall()
            conn.close()
            
            return [dict(row) for row in rows]
    
    # ==================== PLC数据相关方法 ====================
    
    def record_plc_data(self, device_name: str, ip_address: str, connected: bool,
                       coils: Optional[List] = None, registers: Optional[List] = None) -> int:
        """
        记录PLC数据
        
        Args:
            device_name: PLC设备名称
            ip_address: PLC设备IP
            connected: 连接状态
            coils: 线圈数据
            registers: 寄存器数据
            
        Returns:
            int: 插入记录的ID
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO plc_data_history 
                (device_name, ip_address, connected, coils_data, registers_data)
                VALUES (?, ?, ?, ?, ?)
            ''', (
                device_name,
                ip_address,
                connected,
                json.dumps(coils, ensure_ascii=False) if coils else None,
                json.dumps(registers, ensure_ascii=False) if registers else None
            ))
            
            insert_id = cursor.lastrowid
            conn.commit()
            conn.close()
            
            return insert_id
    
    # ==================== 视觉检测相关方法 ====================
    
    def record_vision_detection(self, detections: List[Dict], confidence_threshold: float = 0.5,
                               image_snapshot: Optional[str] = None) -> int:
        """
        记录视觉检测结果
        
        Args:
            detections: 检测结果列表
            confidence_threshold: 置信度阈值
            image_snapshot: 图像快照 (Base64编码)
            
        Returns:
            int: 插入记录的ID
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO vision_detection_history 
                (detections, confidence_threshold, image_snapshot)
                VALUES (?, ?, ?)
            ''', (
                json.dumps(detections, ensure_ascii=False),
                confidence_threshold,
                image_snapshot
            ))
            
            insert_id = cursor.lastrowid
            conn.commit()
            conn.close()
            
            return insert_id
    
    # ==================== IO状态相关方法 ====================
    
    def record_io_state(self, io_name: str, io_type: str, pin_number: int, value: float) -> int:
        """
        记录IO状态
        
        Args:
            io_name: IO名称
            io_type: IO类型
            pin_number: 引脚编号
            value: 状态值
            
        Returns:
            int: 插入记录的ID
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO io_state_history 
                (io_name, io_type, pin_number, value)
                VALUES (?, ?, ?, ?)
            ''', (io_name, io_type, pin_number, value))
            
            insert_id = cursor.lastrowid
            conn.commit()
            conn.close()
            
            return insert_id
    
    # ==================== 仿真状态相关方法 ====================
    
    def set_simulation_state(self, simulation_enabled: bool, 
                             camera_simulation: Optional[bool] = None,
                             plc_simulation: Optional[bool] = None,
                             vision_simulation: Optional[bool] = None) -> bool:
        """
        设置仿真状态
        
        Args:
            simulation_enabled: 总开关
            camera_simulation: 摄像头仿真
            plc_simulation: PLC仿真
            vision_simulation: 视觉仿真
            
        Returns:
            bool: 是否成功
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            # 获取当前状态
            cursor.execute('SELECT * FROM simulation_state WHERE id = 1')
            current = cursor.fetchone()
            
            if current:
                # 更新现有记录
                new_camera = camera_simulation if camera_simulation is not None else current['camera_simulation']
                new_plc = plc_simulation if plc_simulation is not None else current['plc_simulation']
                new_vision = vision_simulation if vision_simulation is not None else current['vision_simulation']
                
                cursor.execute('''
                    UPDATE simulation_state 
                    SET simulation_enabled = ?, camera_simulation = ?, 
                        plc_simulation = ?, vision_simulation = ?,
                        last_updated = CURRENT_TIMESTAMP
                    WHERE id = 1
                ''', (simulation_enabled, new_camera, new_plc, new_vision))
            else:
                # 插入新记录
                cursor.execute('''
                    INSERT INTO simulation_state 
                    (id, simulation_enabled, camera_simulation, plc_simulation, vision_simulation)
                    VALUES (1, ?, ?, ?, ?)
                ''', (simulation_enabled, 
                     camera_simulation if camera_simulation is not None else False,
                     plc_simulation if plc_simulation is not None else False,
                     vision_simulation if vision_simulation is not None else False))
            
            conn.commit()
            conn.close()
            return True
    
    def get_simulation_state(self) -> Dict:
        """
        获取仿真状态
        
        Returns:
            仿真状态字典
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute('SELECT * FROM simulation_state WHERE id = 1')
            row = cursor.fetchone()
            conn.close()
            
            if row:
                return {
                    'simulation_enabled': bool(row['simulation_enabled']),
                    'camera_simulation': bool(row['camera_simulation']),
                    'plc_simulation': bool(row['plc_simulation']),
                    'vision_simulation': bool(row['vision_simulation']),
                    'last_updated': row['last_updated']
                }
            else:
                return {
                    'simulation_enabled': False,
                    'camera_simulation': False,
                    'plc_simulation': False,
                    'vision_simulation': False,
                    'last_updated': None
                }
    
    # ==================== 清理方法 ====================
    
    def clear_old_data(self, days: int = 30) -> int:
        """
        清理过期数据
        
        Args:
            days: 保留天数
            
        Returns:
            int: 删除的记录总数
        """
        with self._connection_lock:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            delete_query = '''
                DELETE FROM {}
                WHERE timestamp < datetime('now', '-{} days')
            '''
            
            tables = ['config_history', 'agv_status_history', 'plc_data_history',
                     'vision_detection_history', 'io_state_history']
            
            total_deleted = 0
            
            for table in tables:
                cursor.execute(delete_query.format(table, days))
                total_deleted += cursor.rowcount
            
            conn.commit()
            conn.close()
            
            print(f"[DatabaseManager] 清理了 {total_deleted} 条过期记录")
            return total_deleted
