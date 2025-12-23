#!/usr/bin/env python3
"""
Integrated Object Detection Node
支持两种模式：独立模式和协调器模式
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime, timedelta
import threading

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class ObjectDetectorIntegrated(Node):
    def __init__(self):
        super().__init__('object_detector_integrated')
        
        # 声明参数
        self.declare_parameter('use_coordinator', False)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('tts_interval', 3.0)
        
        self.use_coordinator = self.get_parameter('use_coordinator').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.tts_interval = self.get_parameter('tts_interval').value
        
        # 加载YOLO模型
        self.model = None
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'YOLO model loaded from {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
        else:
            self.get_logger().warn('YOLO not available, install ultralytics')
        
        # 初始化ROS
        self.bridge = CvBridge()
        
        # 条件订阅
        if self.use_coordinator:
            self.subscription = self.create_subscription(
                Image, 'navigation/rgb_processed', self.image_callback, 10)
            self.get_logger().info('Operating in COORDINATOR mode')
        else:
            self.subscription = self.create_subscription(
                Image, '/camera/color/image_raw', self.image_callback, 10)
            self.get_logger().info('Operating in INDEPENDENT mode')
        
        # TTS客户端
        self.SetString = None
        self.tts_client = None
        try:
            from service_define.srv import SetString
            self.SetString = SetString
            self.tts_client = self.create_client(SetString, 'tts_service')
        except ImportError:
            self.get_logger().warn('SetString service not available')
        
        self.last_tts_time = datetime.now() - timedelta(seconds=self.tts_interval)
        self.last_detected_objects = set()
        self.tts_lock = threading.Lock()
    
    def image_callback(self, msg: Image):
        """处理图像回调"""
        if self.model is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 运行YOLO检测
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
            
            if len(results) > 0:
                detected_classes = set()
                class_names = []
                
                for result in results:
                    if result.boxes is not None:
                        for box in result.boxes:
                            class_id = int(box.cls[0])
                            class_name = self.model.names[class_id]
                            detected_classes.add(class_name)
                            
                            if class_name not in class_names:
                                class_names.append(class_name)
                
                # 输出检测结果
                if class_names:
                    detection_text = '检测到: ' + ', '.join(class_names)
                    self.get_logger().info(detection_text)
                    
                    # 检查是否需要输出语音
                    if detected_classes != self.last_detected_objects:
                        self._send_tts(detection_text)
                        self.last_detected_objects = detected_classes
        
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
    
    def _send_tts(self, text: str):
        """发送TTS请求"""
        if self.SetString is None or self.tts_client is None:
            return
        
        with self.tts_lock:
            current_time = datetime.now()
            if (current_time - self.last_tts_time).total_seconds() < self.tts_interval:
                return
            
            try:
                request = self.SetString.Request()
                request.data = text
                self.tts_client.call_async(request)
                self.last_tts_time = current_time
                self.get_logger().info(f'TTS sent: {text}')
            except Exception as e:
                self.get_logger().error(f'TTS error: {e}')


def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetectorIntegrated()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
