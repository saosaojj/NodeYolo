#!/usr/bin/env python3
import threading
import time
import base64
import cv2
import numpy as np
from .config_manager import ConfigManager


class CameraManager:
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
            self.cap = None
            self._running = False
            self._thread = None
            self._frame = None
            self._frame_lock = threading.Lock()
            self._initialized = True

    def _open_camera(self):
        cfg = self.config_mgr.get('camera', {})
        device = cfg.get('device', '0')
        use_rtsp = False
        try:
            use_rtsp = cfg.get('use_rtsp', False)
        except:
            use_rtsp = False
        if use_rtsp:
            src = cfg['device']
        else:
            src = int(cfg['device'])

        if self.cap:
            try:
                self.cap.release()
            except:
                pass

        self.cap = cv2.VideoCapture(src)
        if self.cap.isOpened():
            w = int(cfg.get('width', 640))
            h = int(cfg.get('height', 480))
            fps = int(cfg.get('fps', 30))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            print(f'[CameraManager] 摄像头已打开')
            return True
        else:
            print('[CameraManager] 无法打开摄像头')
            return False

    def start(self):
        if self._running:
            return
        self._running = True
        self._open_camera()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print('[CameraManager] 已启动')

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
            self.cap = None

    def _capture_loop(self):
        while self._running:
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

    def get_frame(self):
        with self._frame_lock:
            if self._frame is not None:
                return self._frame.copy()
            return None

    def get_preview_base64(self):
        frame = self.get_frame()
        if frame is None:
            return None
        _, buffer = cv2.imencode('.jpg', frame)
        return base64.b64encode(buffer).decode('utf-8')

    def update_config(self, config):
        self.config_mgr.set('camera', config)
        if self._running:
            self._open_camera()
