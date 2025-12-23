#!/usr/bin/env python3
"""
Integrated Path Planning Node
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


class PathPlannerIntegrated(Node):
    def __init__(self):
        super().__init__('path_planner_integrated')
        
        # 声明参数
        self.declare_parameter('use_coordinator', False)
        self.declare_parameter('safety_distance_threshold', 0.5)
        self.declare_parameter('tts_interval_change', 2.0)
        self.declare_parameter('tts_interval_repeat', 4.0)
        
        self.use_coordinator = self.get_parameter('use_coordinator').value
        self.safety_threshold = self.get_parameter('safety_distance_threshold').value
        self.tts_interval_change = self.get_parameter('tts_interval_change').value
        self.tts_interval_repeat = self.get_parameter('tts_interval_repeat').value
        
        # 初始化ROS
        self.bridge = CvBridge()
        
        # 条件订阅
        if self.use_coordinator:
            self.rgb_subscription = self.create_subscription(
                Image, 'navigation/rgb_processed', self.rgb_callback, 10)
            self.depth_subscription = self.create_subscription(
                Image, 'navigation/depth_processed', self.depth_callback, 10)
            self.get_logger().info('Operating in COORDINATOR mode')
        else:
            self.rgb_subscription = self.create_subscription(
                Image, '/camera/color/image_raw', self.rgb_callback, 10)
            self.depth_subscription = self.create_subscription(
                Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
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
        
        # 深度图缓存
        self.current_depth_image = None
        self.current_rgb_image = None
        self.depth_lock = threading.Lock()
        
        # TTS状态管理
        self.last_tts_time = datetime.now() - timedelta(seconds=self.tts_interval_change)
        self.last_direction = None
        self.tts_lock = threading.Lock()
    
    def rgb_callback(self, msg: Image):
        """RGB图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.depth_lock:
                self.current_rgb_image = cv_image
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')
    
    def depth_callback(self, msg: Image):
        """深度图像回调"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            with self.depth_lock:
                self.current_depth_image = depth_image
            
            # 分析深度图
            direction = self._analyze_depth(depth_image)
            
            # 输出语音
            if direction:
                self._output_direction(direction)
        
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')
    
    def _analyze_depth(self, depth_image) -> str:
        """
        分析深度图，确定最佳方向
        返回: 'left'、'center'、'right'、'blocked'
        优先级: center > left > right
        """
        if depth_image is None or depth_image.size == 0:
            return None
        
        height, width = depth_image.shape
        center_width = width // 3
        
        left_roi = depth_image[:, :center_width]
        center_roi = depth_image[:, center_width:2*center_width]
        right_roi = depth_image[:, 2*center_width:]
        
        def get_safe_distance(roi):
            valid_mask = (roi > 0) & (~np.isnan(roi))
            if not np.any(valid_mask):
                return float('inf')
            return np.nanmean(roi[valid_mask]) / 1000.0
        
        left_distance = get_safe_distance(left_roi)
        center_distance = get_safe_distance(center_roi)
        right_distance = get_safe_distance(right_roi)
        
        distances = {
            'left': left_distance,
            'center': center_distance,
            'right': right_distance
        }
        
        # 检查前方是否受阻
        if center_distance < self.safety_threshold:
            return 'blocked'
        
        # 找最安全的方向
        if all(d < self.safety_threshold for d in distances.values()):
            return 'blocked'
        
        valid_directions = {k: v for k, v in distances.items() 
                          if v >= self.safety_threshold}
        
        if not valid_directions:
            return 'blocked'
        
        # 按优先级返回：如果center可用，优先选择center
        if valid_directions.get('center', 0) >= self.safety_threshold:
            return 'center'
        
        # 否则在left和right中选择距离最远的
        remaining = {k: v for k, v in valid_directions.items() if k != 'center'}
        if remaining:
            return max(remaining, key=remaining.get)
        
        return 'blocked' 
    
    def _output_direction(self, direction: str):
        """输出方向信息"""
        direction_map = {
            'left': '左边是空地，可以左转',
            'center': '前方是空地，可以直走',
            'right': '右边是空地，可以右转',
            'blocked': '前方受阻，请小心'
        }
        
        text = direction_map.get(direction, '')
        
        with self.tts_lock:
            current_time = datetime.now()
            time_diff = (current_time - self.last_tts_time).total_seconds()
            
            if direction != self.last_direction:
                required_interval = self.tts_interval_change
            else:
                required_interval = self.tts_interval_repeat
            
            if time_diff >= required_interval:
                self._send_tts(text)
                self.last_tts_time = current_time
                self.last_direction = direction
    
    def _send_tts(self, text: str):
        """发送TTS请求"""
        if self.SetString is None or self.tts_client is None:
            return
        
        try:
            request = self.SetString.Request()
            request.data = text
            self.tts_client.call_async(request)
            self.get_logger().info(f'TTS sent: {text}')
        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlannerIntegrated()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
