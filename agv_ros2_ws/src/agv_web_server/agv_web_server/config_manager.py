#!/usr/bin/env python3
"""
统一配置管理模块

提供线程安全的配置读写功能，支持配置持久化到本地文件，
并提供默认配置生成、读取、更新等功能。
"""

import os
import json
import threading
from pathlib import Path


class ConfigManager:
    """
    配置管理器单例类
    
    确保整个应用中只有一个配置管理器实例，统一管理所有配置项。
    采用线程安全的单例模式实现。
    """
    
    # 单例实例
    _instance = None
    # 单例创建锁
    _lock = threading.Lock()

    def __new__(cls):
        """
        单例模式的实现：确保只创建一个ConfigManager实例
        """
        # 双重检查锁定（Double-Checked Locking）
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """
        初始化配置管理器
        
        只会在第一次实例化时执行初始化逻辑
        """
        # 防止重复初始化
        if not hasattr(self, '_initialized'):
            # 配置文件存放目录（用户目录下的.agv_web_config文件夹）
            self.config_dir = Path.home() / '.agv_web_config'
            # 配置文件完整路径
            self.config_file = self.config_dir / 'config.json'
            # 内存中的配置字典
            self._config = {}
            # 配置读写锁
            self._lock = threading.Lock()
            # 加载现有配置或生成默认配置
            self._load()
            # 标记已初始化
            self._initialized = True

    def _load(self):
        """
        从文件加载配置，如果文件不存在则创建默认配置
        """
        try:
            # 确保配置目录存在
            self.config_dir.mkdir(exist_ok=True)
            
            # 如果配置文件存在，加载它
            if self.config_file.exists():
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    self._config = json.load(f)
            else:
                # 配置文件不存在，使用默认配置并保存
                self._config = self._get_default_config()
                self._save()
                
        except Exception as e:
            print(f'[ConfigManager] 加载配置失败: {e}')
            # 加载失败时使用默认配置
            self._config = self._get_default_config()

    def _save(self):
        """
        将当前配置保存到文件
        """
        try:
            # 确保配置目录存在
            self.config_dir.mkdir(exist_ok=True)
            # 以UTF-8编码保存，支持中文
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self._config, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f'[ConfigManager] 保存配置失败: {e}')

    def _get_default_config(self):
        """
        获取默认配置字典
        
        Returns:
            dict: 默认配置内容
        """
        return {
            # 摄像头相关配置
            'camera': {
                'device': '0',  # 摄像头设备索引或RTSP地址
                'use_rtsp': False,  # 是否使用RTSP流
                'fps': 30,  # 帧率
                'width': 640,  # 宽度
                'height': 480  # 高度
            },
            # PLC相关配置
            'plc': {
                'devices': [
                    {
                        'name': 'main_plc',
                        'ip': '127.0.0.1',
                        'port': 502,
                        'slave_id': 1,
                        'is_master': True,
                        'coil_read_start': 0,
                        'coil_read_count': 16,
                        'register_read_start': 0,
                        'register_read_count': 16
                    }
                ]
            },
            # YOLO视觉识别相关配置
            'yolo': {
                'model_path': '',
                'confidence': 0.5,
                'iou_threshold': 0.45
            }
        }

    def get(self, key, default=None):
        """
        获取配置项的值
        
        Args:
            key: 配置项的键，支持点号分隔，例如 'camera.fps'
            default: 默认值，当键不存在时返回
        
        Returns:
            配置项的值或default
        """
        with self._lock:
            keys = key.split('.')
            value = self._config
            # 逐级查找配置
            for k in keys:
                if isinstance(value, dict) and k in value:
                    value = value[k]
                else:
                    return default
            return value

    def set(self, key, value):
        """
        设置配置项的值并保存
        
        Args:
            key: 配置项的键，支持点号分隔
            value: 要设置的值
        """
        with self._lock:
            keys = key.split('.')
            config = self._config
            # 逐级创建配置结构
            for k in keys[:-1]:
                if k not in config:
                    config[k] = {}
                config = config[k]
            # 设置最终值
            config[keys[-1]] = value
            # 保存到文件
            self._save()

    def get_all(self):
        """
        获取完整的配置字典
        
        Returns:
            dict: 当前所有配置的副本
        """
        with self._lock:
            return dict(self._config)

    def set_all(self, config):
        """
        替换完整配置并保存
        
        Args:
            config: 要替换的完整配置字典
        """
        with self._lock:
            self._config = dict(config)
            self._save()
