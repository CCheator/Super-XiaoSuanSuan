import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from std_msgs.msg import String
import json
import re
from body_controller_can import global_body_controller
from typing import Dict, Callable, List
import asyncio
from dotenv import load_dotenv
from std_msgs.msg import Float32

class TurnHeadNode(LifecycleNode):
    def __init__(self):
        super().__init__('turn_head')
        self.latest_msg = None  # 保存最新的消息

    def on_configure(self, state):
        self.controller = global_body_controller
        self.dead_zone = 0.02
        self.controller.yhead('up', 0.34, blocking=False, mode='relative')
        self.gestures: Dict[str, Callable] = {
            "wave_left_hand": self.wave_left_hand,
            "wave_right_hand": self.wave_right_hand,
            "head_nod": self.head_nod,
            "head_shake": self.head_shake,
            "greet": self.greet,
            "surprise": self.surprise,
            "reset": self.reset,
            # ---- 参数化动作 ----
            "head_left": self.head_left,
            "head_right": self.head_right,
            "head_up": self.head_up,
            "head_down": self.head_down,
            "arm_left_up": self.arm_left_up,
            "arm_left_down": self.arm_left_down,
            "arm_right_up": self.arm_right_up,
            "arm_right_down": self.arm_right_down,
        }
        return super().on_configure(state)
        
    def on_activate(self, state):
        self.subscription = self.create_subscription(
            String,
            'face_recognition_result',
            self._subscription_callback,
            10)
        self.gesture_subscription = self.create_subscription(
            String,
            'gesture',
            self.gesture_callback,
            10)
        self.instruction = self.create_subscription(
            Float32,
            'turn_head_instruction',
            self.instruction_callback,
            10
        )
        return super().on_activate(state)
    
    def on_deactivate(self, state):
        self.controller.reset_body()
        self.destroy_subscription(self.subscription)
        self.destroy_subscription(self.gesture_subscription)
        self.destroy_subscription(self.instruction)
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
    async def do_gesture(self,gesture_name,angle=None,**kwargs):
        if angle:
            await self.gestures[gesture_name](angle=angle, **kwargs)
        else:
            await self.gestures[gesture_name](**kwargs)

    def instruction_callback(self,msg):
        doa = msg.data
        if doa > 0 :
            direction = 'left'
        else:
            direction = 'right'
            doa = -doa
        if doa >30:
            doa = 30
        self.controller.xhead(direction,doa,blocking=True,mode='relative')

    def gesture_callback(self, msg):
        # 当收到消息时的回调函数
        self.get_logger().info(f'Received gesture: "{msg.data}"')
        gesture_str = msg.data
        if gesture_str == "":
            return
        param_match = re.match(r"(\w+)\((.*)\)", gesture_str)
        if param_match:
            gesture_name = param_match.group(1)
            param_str = param_match.group(2)
            if gesture_name not in self.gestures:
                raise ValueError(f"未知姿态: {gesture_name}")
            if param_str.strip() == "":
                asyncio.run(self.do_gesture(gesture_name,speed=1.5))
            else:
                try:
                    angle = float(param_str)
                    asyncio.run(self.do_gesture(gesture_name,angle=angle,speed=1.5))
                except ValueError:
                    raise ValueError(f"参数格式错误，应为数字如'head_left(45)'")
        else:
            if gesture_str not in self.gestures:
                raise ValueError(f"未知姿态: {gesture_str}")
            asyncio.run(self.do_gesture(gesture_name,speed=1.5))           

    async def _move_head(
        self, axis: str, direction: str, angle: float, mode: str = "absolute"
    ):
        #新版系统中暂时先不做头部运动
        pass
        # """头部运动基础方法"""
        # if axis == "x":
        #     await self.controller.control_xhead(direction, angle, mode)  # 修改模式
        # elif axis == "y":
        #     await self.controller.control_yhead(direction, angle, mode)  # 修改模式

    async def _move_arm(
        self, side: str, direction: str, angle: float, mode: str = "absolute"
    ):
        """手臂运动基础方法"""
        if side == "left":
            self.get_logger().info("arm_left")
            await self.controller.async_left_arm(direction,angle,blocking=False,mode=mode)
        elif side == "right":
            self.get_logger().info("arm_right")
            await self.controller.async_right_arm(direction,angle,blocking=False,mode=mode)

    async def wave_left_hand(self, speed=1.0):
        """左手招手动作：从 0°(下垂) → 90° → 70°-110°摆动 → 复位到 0°"""
        await self._move_arm("left", "up", 90)
        for _ in range(3):
            await self._move_arm("left", "up", 70)
            await asyncio.sleep(0.3 / speed)
            await self._move_arm("left", "up", 110)
            await asyncio.sleep(0.3 / speed)
        await self._move_arm("left", "up", 0)

    async def wave_right_hand(self, speed=1.0):
        """右手招手动作：从 0°(下垂) → 90° → 70°-110°摆动 → 复位到 0°"""
        await self._move_arm("right", "up", 90)
        for _ in range(3):
            await self._move_arm("right", "up", 70)
            await asyncio.sleep(0.3 / speed)
            await self._move_arm("right", "up", 110)
            await asyncio.sleep(0.3 / speed)
        await self._move_arm("right", "up", 0)

    async def head_nod(self, speed=1.0):
        """点头动作：平视(0°) → 低头(-15°) → 平视"""
        for _ in range(2):
            await self._move_head("y", "down", 15, 'relative')  # 低头到-15°
            await asyncio.sleep(0.3 / speed)
            await self._move_head("y", "up", 15, 'relative')  # 复位到0°
            await asyncio.sleep(0.3 / speed)

    async def head_shake(self, speed=1.0):
        """摇头动作：中心(0°) → 右转(+30°) → 左转(-30°) → 中心"""
        for _ in range(2):
            await self._move_head("x", "right", 30, "absolute")
            await asyncio.sleep(0.3 / speed)
            await self._move_head("x", "left", 30, "absolute")
            await asyncio.sleep(0.3 / speed)
        await self._move_head("x", "right", 0, "absolute")

    async def greet(self, speed=1.0):
        """欢迎动作：点头 + 右手招手"""
        await self.head_nod(speed)
        await self.wave_right_hand(speed)

    async def surprise(self, speed=1.0):
        """惊讶动作"""
        await self._move_head("y", "up", 8)
        await asyncio.gather(
            self._move_arm("left", "up", 40), self._move_arm("right", "up", 40)
        )
        await asyncio.sleep(1.0 / speed)
        await self.controller.reset_body()

    async def reset(self, speed=1.0):
        """复位所有动作"""
        await self.controller.reset_body()

    async def head_left(self, angle: float, speed=1.0):
        """头部向左转动指定角度"""
        await self._move_head("x", "left", angle, "relative")
        await asyncio.sleep(0.3 / speed)

    async def head_right(self, angle: float, speed=1.0):
        """头部向右转动指定角度"""
        await self._move_head("x", "right", angle, "relative")
        await asyncio.sleep(0.3 / speed)

    async def head_up(self, angle: float, speed=1.0):
        """头部向上抬起指定角度"""
        await self._move_head("y", "up", angle, "relative")
        await asyncio.sleep(0.3 / speed)

    async def head_down(self, angle: float, speed=1.0):
        """头部向下低头指定角度"""
        await self._move_head("y", "down", angle, "relative")
        await asyncio.sleep(0.3 / speed)

    # 新增参数化手臂控制方法
    async def arm_left_up(self, angle: float, speed=1.0):
        """左臂向上抬起指定角度"""
        await self._move_arm("left", "up", angle, "relative")
        await asyncio.sleep(0.3 / speed)

    async def arm_left_down(self, angle: float, speed=1.0):
        """左臂向下放下指定角度"""
        await self._move_arm("left", "down", angle, "relative")
        await asyncio.sleep(0.3 / speed)

    async def arm_right_up(self, angle: float, speed=1.0):
        """右臂向上抬起指定角度"""
        await self._move_arm("right", "up", angle, "relative")
        await asyncio.sleep(0.3 / speed)

    async def arm_right_down(self, angle: float, speed=1.0):
        """右臂向下放下指定角度"""
        await self._move_arm("right", "down", angle, "relative")
        await asyncio.sleep(0.3 / speed)

    async def wait_done(self):
        await self.controller.wait_completion()

    async def stop(self):
        self.controller.stop()

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
