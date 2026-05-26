#!/usr/bin/env python3
import os
import json
import threading
from pathlib import Path


class ConfigManager:
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
            self.config_dir = Path.home() / '.agv_web_config'
            self.config_file = self.config_dir / 'config.json'
            self._config = {}
            self._lock = threading.Lock()
            self._load()
            self._initialized = True

    def _load(self):
        try:
            self.config_dir.mkdir(exist_ok=True)
            if self.config_file.exists():
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    self._config = json.load(f)
            else:
                self._config = self._get_default_config()
                self._save()
        except Exception as e:
            print(f'[ConfigManager] 加载配置失败: {e}')
            self._config = self._get_default_config()

    def _save(self):
        try:
            self.config_dir.mkdir(exist_ok=True)
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self._config, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f'[ConfigManager] 保存配置失败: {e}')

    def _get_default_config(self):
        return {
            'camera': {
                'device': '0',
                'use_rtsp': False,
                'fps': 30,
                'width': 640,
                'height': 480
            },
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
            'yolo': {
                'model_path': '',
                'confidence': 0.5,
                'iou_threshold': 0.45
            }
        }

    def get(self, key, default=None):
        with self._lock:
            keys = key.split('.')
            val = self._config
            for k in keys:
                if isinstance(val, dict) and k in val:
                    val = val[k]
                else:
                    return default
            return val

    def set(self, key, value):
        with self._lock:
            keys = key.split('.')
            val = self._config
            for k in keys[:-1]:
                if k not in val:
                    val[k] = {}
                val = val[k]
            val[keys[-1]] = value
            self._save()

    def get_all(self):
        with self._lock:
            return dict(self._config)

    def set_all(self, config):
        with self._lock:
            self._config = dict(config)
            self._save()
