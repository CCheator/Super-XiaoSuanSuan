import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from service_define.srv import SetString
import time

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        
        # 参数声明
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        
        self.get_logger().info(f'Subscribing to Depth: {depth_topic}')
        self.get_logger().info(f'Subscribing to RGB: {rgb_topic}')

        # 订阅者
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10)
        self.rgb_sub = self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            10)
            
        # TTS 客户端
        self.tts_client = self.create_client(SetString, 'tts_service')
        # 不阻塞等待，在回调中检查
        
        self.bridge = CvBridge()
        
        # 状态
        self.last_command = ""
        self.last_speak_time = 0
        self.speak_interval = 4.0 # 语音播报最小间隔（秒）
        
    def rgb_callback(self, msg):
        # RGB数据暂时只用于订阅，如果需要可视化可以在这里处理
        pass
        
    def depth_callback(self, msg):
        try:
            # 将ROS图像转换为OpenCV格式
            # passthrough 会保持原始编码，深度图通常是 16UC1 (mm) 或 32FC1 (m)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # 处理深度图
        height, width = cv_image.shape
        
        # 将图像分为左、中、右三部分
        col_w = width // 3
        left_roi = cv_image[:, :col_w]
        center_roi = cv_image[:, col_w:2*col_w]
        right_roi = cv_image[:, 2*col_w:]
        
        # 计算每个区域的平均深度
        # 注意：深度值为0通常表示无效或太近/太远，计算平均值时应排除0
        
        def get_score(roi):
            # 转换为float以避免溢出，并处理无效值
            roi_float = roi.astype(np.float32)
            valid_mask = roi_float > 0
            
            if np.count_nonzero(valid_mask) == 0:
                return 0.0
                
            # 计算有效像素的平均深度
            mean_depth = np.mean(roi_float[valid_mask])
            return mean_depth
            
        left_score = get_score(left_roi)
        center_score = get_score(center_roi)
        right_score = get_score(right_roi)
        
        scores = {'左边': left_score, '前方': center_score, '右边': right_score}
        
        # 找出深度最大的方向（即最空旷的方向）
        best_dir = max(scores, key=scores.get)
        max_score = scores[best_dir]
        
        # 简单的单位判断和阈值检查
        # 假设如果数值很大（>100），单位是mm。如果很小（<100），单位是m。
        # 设定一个最小安全距离，例如 0.5米
        is_mm = max_score > 100
        safe_threshold = 500.0 if is_mm else 0.5
        
        if max_score < safe_threshold:
            command = "前方受阻，请小心"
        else:
            command = f"{best_dir}是空地"
        
        # 语音播报逻辑
        current_time = time.time()
        
        # 如果命令改变了，或者距离上次播报超过了较长时间
        should_speak = False
        if command != self.last_command:
            if current_time - self.last_speak_time > 2.0: # 改变指令时，间隔可以短一点
                should_speak = True
        else:
            if current_time - self.last_speak_time > self.speak_interval: # 重复指令，间隔长一点
                should_speak = True
                
        if should_speak:
            self.speak(command)
            self.last_command = command
            self.last_speak_time = current_time
            self.get_logger().info(f"Decision: {command} (L:{left_score:.1f}, C:{center_score:.1f}, R:{right_score:.1f})")

    def speak(self, text):
        if not self.tts_client.service_is_ready():
            self.get_logger().warn('TTS service not ready')
            return

        req = SetString.Request()
        req.data = text
        # 异步调用，不等待结果
        self.tts_client.call_async(req)
        
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
