#!/usr/bin/env python3
"""
Navigation Coordinator Node
协调目标检测和路径规划模块的运行，处理运动检测和数据转发
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import threading
from datetime import datetime, timedelta


class NavigationCoordinator(Node):
    def __init__(self):
        super().__init__('navigation_coordinator')
        
        # 声明参数
        self.declare_parameter('motion_detection_threshold', 5000)
        self.declare_parameter('motion_detection_interval', 0.5)
        self.declare_parameter('buffer_size', 10)
        
        self.motion_threshold = self.get_parameter('motion_detection_threshold').value
        self.detection_interval = self.get_parameter('motion_detection_interval').value
        self.buffer_size = self.get_parameter('buffer_size').value
        
        # 初始化ROS订阅和发布
        self.rgb_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        
        # 发布处理后的数据供检测节点使用
        self.rgb_publisher = self.create_publisher(Image, 'navigation/rgb_processed', 10)
        self.depth_publisher = self.create_publisher(Image, 'navigation/depth_processed', 10)
        
        self.bridge = CvBridge()
        
        # 运动检测相关
        self.last_gray_frame = None
        self.last_detection_time = datetime.now()
        self.motion_detected = False
        self.frame_lock = threading.Lock()
        
        # 帧缓冲
        self.rgb_buffer = deque(maxlen=self.buffer_size)
        self.depth_buffer = deque(maxlen=self.buffer_size)
        
        self.get_logger().info('Navigation Coordinator started')
    
    def rgb_callback(self, msg: Image):
        """RGB图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            with self.frame_lock:
                self.rgb_buffer.append(cv_image)
                
                current_time = datetime.now()
                if (current_time - self.last_detection_time).total_seconds() >= self.detection_interval:
                    motion = self._detect_motion(cv_image)
                    
                    if motion and not self.motion_detected:
                        self.motion_detected = True
                        self.get_logger().info('Motion detected!')
                        self.rgb_publisher.publish(msg)
                    elif not motion and self.motion_detected:
                        self.motion_detected = False
                        self.get_logger().info('Motion stopped')
                    
                    self.last_detection_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')
    
    def depth_callback(self, msg: Image):
        """深度图像回调"""
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            with self.frame_lock:
                self.depth_buffer.append(cv_depth)
                
                if self.motion_detected:
                    self.depth_publisher.publish(msg)
                    
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')
    
    def _detect_motion(self, frame) -> bool:
        """使用帧差分检测运动"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        if self.last_gray_frame is None:
            self.last_gray_frame = gray
            return False
        
        diff = cv2.absdiff(self.last_gray_frame, gray)
        thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        motion_area = sum(cv2.contourArea(c) for c in contours)
        self.last_gray_frame = gray
        
        return motion_area > self.motion_threshold


def main(args=None):
    rclpy.init(args=args)
    coordinator = NavigationCoordinator()
    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
