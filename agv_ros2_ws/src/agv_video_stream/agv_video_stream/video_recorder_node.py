import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_image_camera, qos_profile_sensor_data

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs

from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import queue
import time
import os
from datetime import datetime
from collections import deque


class CircularBuffer:
    def __init__(self, max_size):
        self.max_size = max_size
        self.buffer = deque(maxlen=max_size)
        self.lock = threading.Lock()
    
    def append(self, item):
        with self.lock:
            self.buffer.append(item)
    
    def get_all(self):
        with self.lock:
            return list(self.buffer)
    
    def clear(self):
        with self.lock:
            self.buffer.clear()
    
    def __len__(self):
        with self.lock:
            return len(self.buffer)


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')
        
        self.declare_parameter('record_path', '/tmp/agv_recordings')
        self.declare_parameter('max_storage_gb', 10.0)
        self.declare_parameter('min_free_space_gb', 1.0)
        self.declare_parameter('detection_threshold', 0.7)
        self.declare_parameter('record_duration_sec', 5.0)
        self.declare_parameter('snapshot_interval_sec', 0.5)
        self.declare_parameter('trigger_classes', ['person', 'obstacle', 'forklift', 'pallet'])
        
        self.record_path = self.get_parameter('record_path').value
        self.max_storage_gb = self.get_parameter('max_storage_gb').value
        self.min_free_space_gb = self.get_parameter('min_free_space_gb').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.record_duration_sec = self.get_parameter('record_duration_sec').value
        self.snapshot_interval_sec = self.get_parameter('snapshot_interval_sec').value
        self.trigger_classes = self.get_parameter('trigger_classes').value
        
        os.makedirs(self.record_path, exist_ok=True)
        os.makedirs(os.path.join(self.record_path, 'snapshots'), exist_ok=True)
        os.makedirs(os.path.join(self.record_path, 'videos'), exist_ok=True)
        
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_yolo = None
        self.is_recording = False
        self.record_thread = None
        self.snapshot_thread = None
        self.frame_buffer = CircularBuffer(int(30 * self.record_duration_sec))
        
        self.recording_writer = None
        self.recording_start_time = None
        self.current_recording_path = None
        
        self.status_pub = self.create_publisher(std_msgs.String, 'recording_status', 10)
        self.snapshot_pub = self.create_publisher(sensor_msgs.Image, 'latest_snapshot', 10)
        
        self.image_sub = self.create_subscription(
            sensor_msgs.Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_image_camera
        )
        
        try:
            self.yolo_sub = self.create_subscription(
                'YoloResult',
                '/yolo_result',
                self.yolo_callback,
                qos_profile_sensor_data
            )
        except Exception:
            self.get_logger().warn('YoloResult topic not available')
            self.yolo_sub = None
        
        self.start_recording_srv = self.create_service(
            std_srvs.Trigger,
            '/video/start_recording',
            self.start_recording_callback
        )
        
        self.stop_recording_srv = self.create_service(
            std_srvs.Trigger,
            '/video/stop_recording',
            self.stop_recording_callback
        )
        
        self.snapshot_srv = self.create_service(
            std_srvs.Trigger,
            '/video/snapshot',
            self.snapshot_callback
        )
        
        self.set_detection_record_srv = self.create_service(
            'SetModel',
            '/video/set_detection_record',
            self.set_detection_record_callback
        )
        
        self.last_snapshot_time = time.time()
        self.snapshot_thread = threading.Thread(target=self.snapshot_worker)
        self.snapshot_thread.daemon = True
        self.snapshot_thread.start()
        
        self.get_logger().info('VideoRecorderNode initialized')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image.copy()
            self.frame_buffer.append({
                'frame': cv_image,
                'timestamp': time.time(),
                'header': msg.header
            })
            
            if self.is_recording:
                self.write_frame(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def yolo_callback(self, msg):
        self.latest_yolo = msg
        if self.trigger_classes and self.check_important_detection(msg):
            self.trigger_detection_recording()
    
    def check_important_detection(self, yolo_msg):
        try:
            class_names = yolo_msg.class_names
            scores = yolo_msg.scores
            
            for i, class_name in enumerate(class_names):
                if class_name in self.trigger_classes:
                    if i < len(scores) and scores[i] >= self.detection_threshold:
                        return True
            return False
        except Exception:
            return False
    
    def trigger_detection_recording(self):
        if not self.is_recording:
            self.get_logger().info('Important detection detected, starting recording')
            frames = self.frame_buffer.get_all()
            if frames:
                self.start_recording_from_buffer(frames)
    
    def start_recording_from_buffer(self, buffered_frames):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.current_recording_path = os.path.join(
            self.record_path, 'videos', f'event_{timestamp}.mp4'
        )
        
        if not buffered_frames:
            return
        
        frame_shape = buffered_frames[0]['frame'].shape
        height, width = frame_shape[:2]
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.recording_writer = cv2.VideoWriter(
            self.current_recording_path,
            fourcc,
            30.0,
            (width, height)
        )
        
        for frame_data in buffered_frames:
            self.write_frame_with_overlay(frame_data['frame'])
        
        self.recording_start_time = time.time()
        self.is_recording = True
        self.publish_status('recording')
    
    def start_recording_callback(self, request, response):
        if not self.is_recording:
            self.start_continuous_recording()
            response.success = True
            response.message = 'Recording started'
        else:
            response.success = True
            response.message = 'Already recording'
        return response
    
    def start_continuous_recording(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.current_recording_path = os.path.join(
            self.record_path, 'videos', f'continuous_{timestamp}.mp4'
        )
        
        if self.latest_frame is not None:
            height, width = self.latest_frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.recording_writer = cv2.VideoWriter(
                self.current_recording_path,
                fourcc,
                30.0,
                (width, height)
            )
        
        self.recording_start_time = time.time()
        self.is_recording = True
        self.record_thread = threading.Thread(target=self.recording_worker)
        self.record_thread.daemon = True
        self.record_thread.start()
        self.publish_status('recording')
        self.get_logger().info('Continuous recording started')
    
    def recording_worker(self):
        while self.is_recording:
            if self.latest_frame is not None and self.recording_writer:
                self.write_frame_with_overlay(self.latest_frame)
            time.sleep(1.0 / 30.0)
    
    def stop_recording_callback(self, request, response):
        if self.is_recording:
            self.stop_recording()
            response.success = True
            response.message = 'Recording stopped'
        else:
            response.success = True
            response.message = 'Not recording'
        return response
    
    def stop_recording(self):
        self.is_recording = False
        if self.recording_writer:
            self.recording_writer.release()
            self.recording_writer = None
        self.publish_status('stopped')
        self.get_logger().info(f'Recording saved to {self.current_recording_path}')
    
    def write_frame(self, frame):
        if self.recording_writer:
            self.recording_writer.write(frame)
    
    def write_frame_with_overlay(self, frame):
        overlay = frame.copy()
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(overlay, timestamp, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if self.latest_yolo and self.check_important_detection(self.latest_yolo):
            cv2.putText(overlay, 'IMPORTANT DETECTION', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        self.write_frame(overlay)
    
    def snapshot_callback(self, request, response):
        if self.latest_frame is not None:
            self.take_snapshot(self.latest_frame)
            response.success = True
            response.message = 'Snapshot taken'
        else:
            response.success = False
            response.message = 'No frame available'
        return response
    
    def snapshot_worker(self):
        while True:
            if (time.time() - self.last_snapshot_time) >= self.snapshot_interval_sec:
                if self.latest_frame is not None:
                    self.take_snapshot(self.latest_frame)
                    self.last_snapshot_time = time.time()
            time.sleep(0.1)
    
    def take_snapshot(self, frame):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.record_path, 'snapshots', f'{timestamp}.jpg')
        
        overlay = frame.copy()
        timestamp_text = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(overlay, timestamp_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imwrite(filename, overlay)
        
        try:
            snapshot_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            self.snapshot_pub.publish(snapshot_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing snapshot: {e}')
        
        self.get_logger().debug(f'Snapshot saved: {filename}')
    
    def set_detection_record_callback(self, request, response):
        self.trigger_classes = request.name.split(',')
        response.success = True
        response.message = f'Detection classes set to: {self.trigger_classes}'
        self.get_logger().info(f'Detection classes updated: {self.trigger_classes}')
        return response
    
    def check_storage_and_cleanup(self):
        try:
            stat = os.statvfs(self.record_path)
            free_gb = (stat.f_bavail * stat.f_frsize) / (1024 ** 3)
            
            if free_gb < self.min_free_space_gb:
                self.cleanup_old_recordings()
        except Exception as e:
            self.get_logger().error(f'Error checking storage: {e}')
    
    def cleanup_old_recordings(self):
        videos_dir = os.path.join(self.record_path, 'videos')
        snapshots_dir = os.path.join(self.record_path, 'snapshots')
        
        for directory in [videos_dir, snapshots_dir]:
            if not os.path.exists(directory):
                continue
            
            files = []
            for f in os.listdir(directory):
                filepath = os.path.join(directory, f)
                if os.path.isfile(filepath):
                    files.append((filepath, os.path.getsize(filepath)))
            
            files.sort(key=lambda x: os.path.getctime(x[0]))
            
            total_size = sum(f[1] for f in files)
            max_bytes = self.max_storage_gb * (1024 ** 3)
            
            while total_size > max_bytes and files:
                oldest_file = files.pop(0)
                try:
                    os.remove(oldest_file[0])
                    total_size -= oldest_file[1]
                    self.get_logger().info(f'Cleaned up old file: {oldest_file[0]}')
                except Exception as e:
                    self.get_logger().error(f'Error removing file: {e}')
    
    def publish_status(self, status):
        msg = std_msgs.String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def destroy_node(self):
        self.is_recording = False
        if self.record_thread:
            self.record_thread.join(timeout=2)
        if self.recording_writer:
            self.recording_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
