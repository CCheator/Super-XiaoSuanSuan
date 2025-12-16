#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
from typing import Dict, Callable, List
import asyncio
from body_controller import global_body_controller

class GestureSubscriber(Node):
    def __init__(self):
        super().__init__('gesture')  # 创建名为'gesture'的节点
        # 创建订阅者，订阅名为'gesture'的topic，消息类型为String
        self.client = global_body_controller
        self.subscription = self.create_subscription(
            String,
            'gesture',
            self.listener_callback,
            10)  # 队列大小为10
        self.subscription  # 防止未使用变量警告
        self.get_logger().info('Gesture subscriber node has been started and is listening on the "gesture" topic.')
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
    async def do_gesture(self,gesture_name,angle=None,**kwargs):
        if angle:
            await self.gestures[gesture_name](angle=angle, **kwargs)
        else:
            await self.gestures[gesture_name](**kwargs)

    def listener_callback(self, msg):
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
        #     await self.client.control_xhead(direction, angle, mode)  # 修改模式
        # elif axis == "y":
        #     await self.client.control_yhead(direction, angle, mode)  # 修改模式

    async def _move_arm(
        self, side: str, direction: str, angle: float, mode: str = "absolute"
    ):
        """手臂运动基础方法"""
        if side == "left":
            self.get_logger().info("arm_left")
            await self.client.async_left_arm(direction,angle,blocking=False,mode=mode)
        elif side == "right":
            self.get_logger().info("arm_right")
            await self.client.async_right_arm(direction,angle,blocking=False,mode=mode)

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
        await self.client.reset_body()

    async def reset(self, speed=1.0):
        """复位所有动作"""
        await self.client.reset_body()

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
        await self.client.wait_completion()

    async def stop(self):
        self.client.stop()


def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2 Python客户端库
    gesture_subscriber = GestureSubscriber()  # 创建节点实例
    
    try:
        rclpy.spin(gesture_subscriber)  # 保持节点运行
    except KeyboardInterrupt:
        pass  # 允许通过Ctrl+C优雅地退出
    
    # 销毁节点并关闭rclpy
    gesture_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()