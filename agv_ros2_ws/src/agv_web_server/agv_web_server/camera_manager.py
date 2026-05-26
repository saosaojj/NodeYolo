#!/usr/bin/env python3
"""
摄像头管理模块

提供统一的摄像头资源管理，支持本地设备或RTSP流，
供视频流和YOLO视觉识别共同使用，避免资源冲突，支持仿真模式。
"""

import threading
import time
import base64
import random
from agv_web_server.config_manager import ConfigManager
from agv_web_server.database_manager import DatabaseManager

# 尝试导入可选依赖
try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    cv2 = None
    np = None
    print("[CameraManager] Warning: OpenCV not available, camera functionality disabled")


class CameraManager:
    """
    摄像头管理器单例类
    
    负责统一管理摄像头资源，支持从本地设备或RTSP流读取视频，
    并提供线程安全的帧获取功能，支持仿真模式。
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
            # 数据库管理器实例
            self.db_mgr = DatabaseManager()
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
            # 仿真模式状态
            self._simulation_enabled = False
            # 仿真帧计数器
            self._simulation_frame_count = 0
            # 初始化完成标志
            self._initialized = True
            
            # 从数据库加载仿真状态
            self._load_simulation_state()
            print(f"[CameraManager] 初始化完成，仿真模式: {self._simulation_enabled}")

    def _load_simulation_state(self):
        """从数据库加载仿真状态"""
        state = self.db_mgr.get_simulation_state()
        self._simulation_enabled = state.get('simulation_enabled', False) or state.get('camera_simulation', False)

    def set_simulation(self, enabled: bool):
        """
        设置仿真模式
        
        Args:
            enabled: 是否启用仿真模式
        """
        self._simulation_enabled = enabled
        # 更新数据库
        state = self.db_mgr.get_simulation_state()
        self.db_mgr.set_simulation_state(
            simulation_enabled=state.get('simulation_enabled', False),
            camera_simulation=enabled,
            plc_simulation=state.get('plc_simulation', False),
            vision_simulation=state.get('vision_simulation', False)
        )
        print(f"[CameraManager] 仿真模式已{'开启' if enabled else '关闭'}")

    def get_simulation(self) -> bool:
        """获取仿真模式状态"""
        return self._simulation_enabled

    def _generate_simulation_frame(self):
        """
        生成仿真帧
        
        Returns:
            numpy.ndarray: 仿真图像
        """
        if not HAS_CV2:
            return None
        
        # 获取配置的分辨率
        config = self.config_mgr.get('camera', {})
        width = int(config.get('width', 640))
        height = int(config.get('height', 480))
        
        # 创建一个仿真图像
        self._simulation_frame_count += 1
        
        # 渐变背景
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 添加动画效果
        offset = (self._simulation_frame_count % 100) / 100.0
        for y in range(height):
            for x in range(width):
                # 彩色渐变背景
                r = int((x / width + offset) * 127 + 64)
                g = int((y / height + 0.5 - offset) * 127 + 64)
                b = int((offset + 0.3) * 127 + 64)
                img[y, x] = [b, g, r]
        
        # 添加一些模拟的检测框
        for i in range(3):
            bx = int((0.2 + i * 0.3 + random.uniform(-0.05, 0.05)) * width)
            by = int((0.3 + i * 0.2 + random.uniform(-0.03, 0.03)) * height)
            bw = int(0.15 * width)
            bh = int(0.2 * height)
            cv2.rectangle(img, (int(bx - bw // 2), int(by - bh // 2)),
                        (int(bx + bw // 2), int(by + bh // 2)), (0, 255, 0), 2)
        
        # 添加时间戳
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(img, f"CAMERA SIMULATION - {timestamp}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 添加帧计数
        cv2.putText(img, f"Frame: {self._simulation_frame_count}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        return img

    def _open_camera(self):
        """
        根据当前配置打开或重新打开摄像头
        
        Returns:
            bool: 是否成功打开摄像头
        """
        if self._simulation_enabled:
            print('[CameraManager] 仿真模式，跳过真实摄像头')
            return True
            
        if not HAS_CV2:
            print('[CameraManager] OpenCV not available, cannot open camera')
            return False

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
            try:
                source = int(device)
            except ValueError:
                source = device  # 如果不是数字，还是作为字符串处理

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
        # 获取配置的帧率
        config = self.config_mgr.get('camera', {})
        fps = int(config.get('fps', 30))
        frame_interval = 1.0 / max(fps, 1)
        
        while self._running:
            start_time = time.time()
            
            if self._simulation_enabled:
                # 仿真模式：生成仿真帧
                frame = self._generate_simulation_frame()
                if frame is not None:
                    with self._frame_lock:
                        self._frame = frame
            elif HAS_CV2:
                # 真实模式：从摄像头读取
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        with self._frame_lock:
                            self._frame = frame.copy()
                    else:
                        time.sleep(0.1)
                else:
                    time.sleep(0.5)
                    self._open_camera()
            else:
                time.sleep(0.5)
            
            # 控制帧率
            elapsed = time.time() - start_time
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)

    def get_frame(self):
        """
        获取最新的视频帧
        
        Returns:
            numpy.ndarray: 最新的视频帧，如果没有可用帧则返回None
        """
        if not HAS_CV2 and not self._simulation_enabled:
            return None
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
        if not HAS_CV2 and not self._simulation_enabled:
            return None
            
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
        old_config = self.config_mgr.get('camera', {})
        self.config_mgr.set('camera', config)
        # 记录配置变更到数据库
        self.db_mgr.record_config_change('camera', old_config, config, 'web')
        # 如果正在运行，重新打开摄像头
        if self._running:
            self._open_camera()
