#!/usr/bin/env python3
"""
摄像头管理模块

提供统一的摄像头资源管理，支持本地设备或RTSP流，
供视频流和YOLO视觉识别共同使用，避免资源冲突。
"""

import threading
import time
import base64
import cv2
import numpy as np
from agv_web_server.config_manager import ConfigManager


class CameraManager:
    """
    摄像头管理器单例类
    
    负责统一管理摄像头资源，支持从本地设备或RTSP流读取视频，
    并提供线程安全的帧获取功能。
    """
    
    # 单例实例
    _instance = None
    # 单例创建锁
    _lock = threading.Lock()

    def __new__(cls):
        """
        单例模式的实现：确保只创建一个CameraManager实例
        """
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """
        初始化摄像头管理器
        """
        if not hasattr(self, '_initialized'):
            # 配置管理器实例
            self.config_mgr = ConfigManager()
            # OpenCV VideoCapture 对象
            self.cap = None
            # 管理器运行标志
            self._running = False
            # 摄像头捕获线程
            self._thread = None
            # 最新的视频帧
            self._frame = None
            # 帧访问锁
            self._frame_lock = threading.Lock()
            # 初始化完成标志
            self._initialized = True

    def _open_camera(self):
        """
        根据当前配置打开或重新打开摄像头
        
        Returns:
            bool: 是否成功打开摄像头
        """
        # 获取摄像头配置
        config = self.config_mgr.get('camera', {})
        device = config.get('device', '0')
        use_rtsp = False
        try:
            use_rtsp = config.get('use_rtsp', False)
        except Exception:
            use_rtsp = False

        # 确定视频源
        if use_rtsp:
            # 使用RTSP流地址
            source = device
        else:
            # 使用本地摄像头设备索引
            source = int(device)

        # 释放之前打开的摄像头
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass

        # 打开新的摄像头
        self.cap = cv2.VideoCapture(source)
        
        if self.cap.isOpened():
            # 设置摄像头参数
            width = int(config.get('width', 640))
            height = int(config.get('height', 480))
            fps = int(config.get('fps', 30))
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            
            print(f'[CameraManager] 摄像头已打开')
            return True
        else:
            print('[CameraManager] 无法打开摄像头')
            return False

    def start(self):
        """
        启动摄像头管理器，开始捕获视频
        """
        if self._running:
            return
        
        self._running = True
        # 尝试打开摄像头
        self._open_camera()
        # 启动视频捕获线程
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        
        print('[CameraManager] 已启动')

    def stop(self):
        """
        停止摄像头管理器，释放资源
        """
        self._running = False
        # 等待线程结束
        if self._thread:
            self._thread.join(timeout=2.0)
        # 释放摄像头资源
        if self.cap:
            self.cap.release()
            self.cap = None

    def _capture_loop(self):
        """
        视频捕获循环（运行在后台线程）
        
        持续从摄像头读取帧并保存到内存，供其他模块使用
        """
        while self._running:
            # 确保摄像头已打开且可用
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    # 成功读取到帧，保存到内存
                    with self._frame_lock:
                        self._frame = frame.copy()
                else:
                    # 读取失败，短暂等待
                    time.sleep(0.1)
            else:
                # 摄像头不可用，尝试重新打开
                time.sleep(0.5)
                self._open_camera()

    def get_frame(self):
        """
        获取最新的视频帧
        
        Returns:
            numpy.ndarray: 最新的视频帧，如果没有可用帧则返回None
        """
        with self._frame_lock:
            if self._frame is not None:
                return self._frame.copy()
            return None

    def get_preview_base64(self):
        """
        获取Base64编码的JPEG预览图
        
        Returns:
            str: Base64编码的JPEG图像，如果没有可用帧则返回None
        """
        frame = self.get_frame()
        if frame is None:
            return None
        # 将OpenCV的BGR图像编码为JPEG格式并转为Base64
        _, buffer = cv2.imencode('.jpg', frame)
        return base64.b64encode(buffer).decode('utf-8')

    def update_config(self, config):
        """
        更新摄像头配置并重新打开摄像头
        
        Args:
            config: 新的摄像头配置字典
        """
        self.config_mgr.set('camera', config)
        # 如果正在运行，重新打开摄像头
        if self._running:
            self._open_camera()
