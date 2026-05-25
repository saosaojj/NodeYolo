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
import subprocess


class VideoServerNode(Node):
    def __init__(self):
        super().__init__('video_server_node')
        
        self.declare_parameter('port', 8554)
        self.declare_parameter('stream_quality', 85)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        self.declare_parameter('rtsp_path', 'stream')
        
        self.port = self.get_parameter('port').value
        self.stream_quality = self.get_parameter('stream_quality').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.resolution_width = self.get_parameter('resolution_width').value
        self.resolution_height = self.get_parameter('resolution_height').value
        self.rtsp_path = self.get_parameter('rtsp_path').value
        
        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=2)
        self.latest_frame = None
        self.latest_yolo = None
        self.is_streaming = False
        self.stream_thread = None
        self.rtsp_server = None
        
        self.status_pub = self.create_publisher(std_msgs.String, 'rtsp_stream_status', 10)
        
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
            self.get_logger().warn('YoloResult topic not available, skipping YOLO subscription')
            self.yolo_sub = None
        
        self.start_stream_srv = self.create_service(
            std_srvs.Trigger,
            '/video/start_stream',
            self.start_stream_callback
        )
        
        self.stop_stream_srv = self.create_service(
            std_srvs.Trigger,
            '/video/stop_stream',
            self.stop_stream_callback
        )
        
        self.set_quality_srv = self.create_service(
            'SetConfidence',
            '/video/set_quality',
            self.set_quality_callback
        )
        
        self.get_logger().info(f'VideoServerNode initialized on port {self.port}')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def yolo_callback(self, msg):
        self.latest_yolo = msg
    
    def overlay_yolo_detections(self, frame):
        if self.latest_yolo is None:
            return frame
        
        try:
            class_names = self.latest_yolo.class_names
            boxes = self.latest_yolo.boxes
            scores = self.latest_yolo.scores
            
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = [int(b) for b in box]
                class_id = i if i < len(class_names) else 0
                score = scores[i] if i < len(scores) else 0.0
                label = f'{class_names[class_id]}: {score:.2f}'
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        except Exception as e:
            self.get_logger().warn(f'Error overlaying YOLO detections: {e}')
        
        return frame
    
    def stream_worker(self):
        rtsp_url = f'rtsp://localhost:{self.port}/{self.rtsp_path}'
        ffmpeg_cmd = [
            'ffmpeg',
            '-re',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.resolution_width}x{self.resolution_height}',
            '-r', str(self.frame_rate),
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-b:v', '2M',
            '-pix_fmt', 'yuv420p',
            '-y',
            rtsp_url
        ]
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdout=subprocess.PIPE
            )
            
            self.get_logger().info(f'RTSP stream started at {rtsp_url}')
            self.publish_status('streaming')
            
            while self.is_streaming:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                    frame = self.overlay_yolo_detections(frame)
                    frame = cv2.resize(frame, (self.resolution_width, self.resolution_height))
                    
                    try:
                        self.ffmpeg_process.stdin.write(frame.tobytes())
                    except Exception:
                        break
                time.sleep(1.0 / self.frame_rate)
            
            if self.ffmpeg_process:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait(timeout=5)
                
        except Exception as e:
            self.get_logger().error(f'Stream worker error: {e}')
            self.publish_status('error')
        finally:
            self.publish_status('stopped')
    
    def start_stream_callback(self, request, response):
        if not self.is_streaming:
            self.is_streaming = True
            self.stream_thread = threading.Thread(target=self.stream_worker)
            self.stream_thread.daemon = True
            self.stream_thread.start()
            response.success = True
            response.message = 'Stream started'
            self.get_logger().info('Stream started')
        else:
            response.success = True
            response.message = 'Stream already running'
        
        return response
    
    def stop_stream_callback(self, request, response):
        if self.is_streaming:
            self.is_streaming = False
            if self.stream_thread:
                self.stream_thread.join(timeout=2)
            if hasattr(self, 'ffmpeg_process') and self.ffmpeg_process:
                self.ffmpeg_process.terminate()
            response.success = True
            response.message = 'Stream stopped'
            self.get_logger().info('Stream stopped')
        else:
            response.success = True
            response.message = 'Stream not running'
        
        return response
    
    def set_quality_callback(self, request, response):
        self.stream_quality = int(request.confidence)
        response.success = True
        response.message = f'Quality set to {self.stream_quality}'
        self.get_logger().info(f'Quality set to {self.stream_quality}')
        return response
    
    def publish_status(self, status):
        msg = std_msgs.String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def destroy_node(self):
        self.is_streaming = False
        if self.stream_thread:
            self.stream_thread.join(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
