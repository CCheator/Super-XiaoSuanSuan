import rclpy
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import String
import json
from body_controller import global_body_controller
import asyncio
from dotenv import load_dotenv

class TurnHeadNode(LifecycleNode):
    def __init__(self):
        super().__init__('turn_head')
        self.latest_msg = None  # 保存最新的消息

    def on_configure(self, state):
        self.controller = global_body_controller
        self.dead_zone = 0.02
        self.controller.yhead('up', 0.34, blocking=False, mode='relative') 
        return super().on_configure(state)
        
    def on_activate(self, state):
        self.subscription = self.create_subscription(
            String,
            'face_recognition_result',
            self._subscription_callback,
            10)
        return super().on_activate(state)
    
    def on_deactivate(self, state):
        self.controller.reset_body()
        self.destroy_subscription(self.subscription)
        return super().on_deactivate(state)
    
    def _subscription_callback(self, msg):
        # 保存最新的消息
        self.latest_msg = msg

    async def listener_callback(self, msg):
        faces = json.loads(msg.data)
        if len(faces) != 0:
            face = faces[0]
            y = (face['location'][0] + face['location'][2]) / 2
            x = (face['location'][1] + face['location'][3]) / 2
            x_error = (x - 320) / 640
            y_error = (y - 240) / 480

            if abs(x_error) < self.dead_zone:
                x_error = 0
            if abs(y_error) < self.dead_zone:
                y_error = 0
                    
            if x_error != 0:
                direction = 'left' if x_error < 0 else 'right'
                angle = abs(x_error) * 20
                await self.controller.async_xhead(direction, angle, blocking=False, mode='relative')
                    
            if y_error != 0:
                direction = 'up' if y_error < 0 else 'down'
                self.get_logger().info(direction)
                angle = abs(y_error) * 10
                await self.controller.async_yhead(direction, angle, blocking=False, mode='relative')
            
            print('x:', x, "  y:", y)

def main(args=None):
    rclpy.init(args=args)
    node = TurnHeadNode()

    loop = asyncio.get_event_loop()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # 非阻塞spin

            if node.latest_msg is not None:
                msg = node.latest_msg
                node.latest_msg = None  # 处理完清空
                loop.run_until_complete(node.listener_callback(msg))

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    load_dotenv()
    main()
