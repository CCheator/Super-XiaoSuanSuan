import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLOv8 model loaded')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, verbose=False)

        labels = set()
        for r in results:
            for cls in r.boxes.cls:
                labels.add(self.model.names[int(cls)])

        if labels:
            self.get_logger().info(
                'Detected objects: ' + ', '.join(labels)
            ) # 此处可作为语音输出


def main():
    rclpy.init()
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
