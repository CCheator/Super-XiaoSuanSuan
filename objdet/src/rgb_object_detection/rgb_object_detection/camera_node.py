import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')

        self.image_path = 'test_img.jpg' #目前是一个示例图片

        if not os.path.exists(self.image_path):
            self.get_logger().error(f'Image not found: {self.image_path}')
            return

        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()

        self.image = cv2.imread(self.image_path)
        if self.image is None:
            self.get_logger().error('Failed to load image')
            return

        self.get_logger().info(f'Loaded image: {self.image_path}')

        # 每30秒发布一次
        self.timer = self.create_timer(30, self.timer_callback)

    def timer_callback(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published image')


def main():
    rclpy.init()
    node = ImagePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
