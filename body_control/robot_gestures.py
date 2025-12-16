import asyncio
from typing import Dict, Callable, List

from body_control_client import BodyControlClient


class RobotGestures:
    """
    机器人姿态控制类，使用字典映射姿态名称到运动控制函数。
    """

    def __init__(self, client: BodyControlClient):
        """
        初始化机器人姿态控制类。

        Args:
            client: BodyControlClient 实例，用于与底层运动控制服务器通信。
        """
        self.client = client
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
        self.gesture_descriptions = {
            "wave_left_hand()": "左手招手动作：从 0°(下垂) → 90° → 70°-110°摆动 → 复位到 0°",
            "wave_right_hand()": "右手招手动作：从 0°(下垂) → 90° → 70°-110°摆动 → 复位到 0°",
            "head_nod()": "点头动作：平视(0°) → 低头(-15°) → 平视, 表示赞同、认可、感谢",
            "head_shake()": "摇头动作：中心(0°) → 右转(+30°) → 左转(-30°) → 中心，表示反对、否认、拒绝",
            "greet()": "欢迎动作：点头 + 右手招手",
            "surprise()": "惊讶动作：头部微抬 + 双臂抬起40°",
            "reset()": "复位动作/立正：所有关节复位到初始位置",
            # ---- 参数化动作 ----
            "head_left(angle)": "头部向左转动指定角度(参数: angle)，比如 head_left(45) 代表头部向左旋转45度",
            "head_right(angle)": "头部向右转动指定角度(参数: angle)，比如 head_right(30) 代表头部向右旋转30度",
            "head_up(angle)": "头部向上抬起指定角度(参数: angle)，比如 head_up(15) 代表头部向上抬起15度",
            "head_down(angle)": "头部向下低头指定角度(参数: angle)，比如 head_down(20) 代表头部向下低头20度",
            "arm_left_up(angle)": "左臂向上抬起指定角度(参数: angle)，比如 arm_left_up(60) 代表左臂向上抬起60度",
            "arm_left_down(angle)": "左臂向下放下指定角度(参数: angle)，比如 arm_left_down(30) 代表左臂向下放下30度",
            "arm_right_up(angle)": "右臂向上抬起指定角度(参数: angle)，比如 arm_right_up(60) 代表右臂向上抬起60度",
            "arm_right_down(angle)": "右臂向下放下指定角度(参数: angle)，比如 arm_right_down(30) 代表右臂向下放下30度",
        }

    def list_gestures(self) -> List[str]:
        """
        获取所有注册的姿态名称。

        Returns:
            包含所有姿态名称的列表。
        """
        return list(self.gestures.keys())

    def list_gestures_with_descriptions(self) -> Dict[str, str]:
        """
        获取所有注册的姿态名称及其描述。
        Returns:
            包含姿态名称和描述的字典。
        """
        return self.gesture_descriptions

    async def _parse_and_execute_single_gesture(self, gesture_str: str, **kwargs):
        """
        解析并执行单个动作字符串

        Args:
            gesture_str: 动作字符串，可以是"wave_left_hand"或"head_left(45)"格式
            **kwargs: 额外参数如speed

        Returns:
            None

        Raises:
            ValueError: 如果动作未注册或参数格式错误
        """
        import re

        param_match = re.match(r"(\w+)\((.*)\)", gesture_str)
        if param_match:
            gesture_name = param_match.group(1)
            param_str = param_match.group(2)

            if gesture_name not in self.gestures:
                raise ValueError(f"未知姿态: {gesture_name}")

            if param_str.strip() == "":
                await self.gestures[gesture_name](**kwargs)
            else:
                try:
                    angle = float(param_str)
                    await self.gestures[gesture_name](angle=angle, **kwargs)
                except ValueError:
                    raise ValueError(f"参数格式错误，应为数字如'head_left(45)'")

        else:
            if gesture_str not in self.gestures:
                raise ValueError(f"未知姿态: {gesture_str}")
            await self.gestures[gesture_str](**kwargs)

    async def execute(self, name: str, **kwargs):
        """
        执行一个或多个姿态动作，多个动作用换行符(\n)分隔

        Args:
            name: 姿态名称或多个动作字符串，如"wave_left_hand"或"head_left(45)\nhead_right(30)"
            **kwargs: 传递给姿态动作函数的额外参数(如speed)

        Raises:
            ValueError: 如果指定的姿态名称未注册或参数格式错误
        """
        gestures = name.split("\n")
        for gesture in gestures:
            gesture = gesture.strip()
            if gesture:  # 跳过空行
                await self._parse_and_execute_single_gesture(gesture, **kwargs)

        await self.client.wait_completion()

    async def _move_head(
        self, axis: str, direction: str, angle: float, mode: str = "absolute"
    ):
        """头部运动基础方法"""
        if axis == "x":
            await self.client.control_xhead(direction, angle, mode)  # 修改模式
        elif axis == "y":
            await self.client.control_yhead(direction, angle, mode)  # 修改模式

    async def _move_arm(
        self, side: str, direction: str, angle: float, mode: str = "absolute"
    ):
        """手臂运动基础方法"""
        if side == "left":
            await self.client.control_left_arm(direction, angle, mode)  # 修改模式
        elif side == "right":
            await self.client.control_right_arm(direction, angle, mode)  # 修改模式

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


async def test_all_gestures():
    """测试所有注册的手势动作"""
    client = BodyControlClient()
    controller = RobotGestures(client)

    print("可用手势列表:")
    for name in controller.list_gestures():
        print(f"- {name}")

    try:
        # for gesture_name in controller.list_gestures():
        #     print(f"\n=== 正在执行 {gesture_name} ===")
        #     await controller.execute(gesture_name, speed=1.5)

        await controller.execute("surprise()", speed=1.5)
        await controller.wait_done()

    finally:
        await client.reset_body()
        await client.wait_completion()
        await client.stop_motors()
        await client.close()


if __name__ == "__main__":
    asyncio.run(test_all_gestures())
