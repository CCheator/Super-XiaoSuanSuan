import math
import serial
from loguru import logger
import time
import asyncio
import os
from dotenv import load_dotenv
# 导入舵机控制代码
from uservo import UartServoManager

load_dotenv()
#############################################
# 电机包装类：包装底层舵机接口，添加软限位与简单移动接口
#############################################
class MotorWrapper:
    def __init__(
        self, name, servo_manager, servo_id, soft_limit=None, limit_speed=1.0, reverse=False
    ):
        """
        :param name: 舵机名称，用于日志标识
        :param servo_manager: UartServoManager 实例（舵机管理器）
        :param servo_id: 舵机ID
        :param soft_limit: 软限位，格式为 (min_angle, max_angle)，单位：度；为 None 则不限制
        :param limit_speed: 限速参数（舵机转速，单位 rad/s）
        :param reverse: 是否反向，反向时正的角度值对应逆时针
        """
        self.name = name
        self.servo_manager = servo_manager
        self.servo_id = servo_id
        self.reverse = -1 if reverse else 1
        self.soft_limit = soft_limit
        self.limit_speed = limit_speed * 180 / math.pi  # 转换为度/秒
        self.target_angle = None
        self.diff = None

        # 舵机初始化
        self.servo_manager.set_servo_angle(self.servo_id, 0.0, velocity=20)
        self.servo_manager.wait()

        # 记录启动时的位置，即初始位置
        self.init_angle = self.get_position()
        logger.debug(
            f"[{self.name}] 初始化：当前位置 {self.init_angle:.2f}°，软限位 {self.soft_limit}，限速 {self.limit_speed:.1f}°/s"
        )

    def _apply_soft_limit(self, target_angle):
        """
        对目标角度进行软限位裁剪
        :param target_angle: 单位：度
        :return: 裁剪后的角度
        """
        if self.soft_limit:
            min_lim, max_lim = self.soft_limit
            return max(min_lim, min(max_lim, target_angle))
        return target_angle

    def stop(self):
        """停止舵机运动 - 设置为阻尼模式"""
        # self.servo_manager.set_damping(self.servo_id, power=50)
        self.servo_manager.disable_torque(self.servo_id)
        time.sleep(0.1)
        logger.info(f"[{self.name}] 停止")

    async def async_move(self, delta_deg):
        """异步相对运动"""
        target_angle = self.get_position() + delta_deg
        target_angle = self._apply_soft_limit(target_angle)
        logger.info(
            f"[{self.name}] async_move: 从 {self.get_position():.2f}° 移动 {delta_deg:+.2f}° 到 {target_angle:.2f}°"
        )
        await self.async_move_to(target_angle)

    async def async_move_to(self, target_angle_deg):
        """异步绝对定位"""
        target_angle = self._apply_soft_limit(target_angle_deg)
        self.target_angle = target_angle
        target_angle_servo = target_angle * self.reverse
        logger.info(
            f"[{self.name}] async_move_to: 移动到绝对位置 {target_angle:.2f}°"
        )
        self.servo_manager.set_servo_angle(
            self.servo_id, 
            target_angle_servo, 
            velocity=self.limit_speed,
            power=0
        )

    async def async_wait_for_completion(self):
        """异步等待舵机到达目标位置"""
        if self.is_done():
            return
        start_time = time.time()
        while True:
            if self.is_done():
                return
            await asyncio.sleep(0.05)
            if time.time() - start_time > 3:
                logger.warning(f"[{self.name}] 异步操作超过3秒超时")
                return

    def is_done(self, tolerance=2.0):
        """检查舵机是否已经到达目标位置"""
        if self.target_angle is None:
            return True
        current_angle = self.get_position()
        diff = abs(current_angle - self.target_angle)
        return diff <= tolerance

    def move_to(self, target_angle_deg):
        """
        绝对定位到 target_angle_deg
        :param target_angle_deg: 目标位置角度（度）
        """
        target_angle = self._apply_soft_limit(target_angle_deg)
        self.target_angle = target_angle
        target_angle_servo = target_angle * self.reverse
        logger.info(
            f"[{self.name}] move_to: 移动到绝对位置 {target_angle:.2f}°"
        )
        self.servo_manager.set_servo_angle(
            self.servo_id, 
            target_angle_servo, 
            velocity=self.limit_speed,
            power=0
        )

    def move(self, delta_deg):
        """
        相对运动 delta_deg
        :param delta_deg: 相对运动角度（度）
        """
        target_angle = self.get_position() + delta_deg
        target_angle = self._apply_soft_limit(target_angle)
        logger.info(
            f"[{self.name}] move: 从 {self.get_position():.2f}° 移动 {delta_deg:+.2f}° 到 {target_angle:.2f}°"
        )
        self.move_to(target_angle)

    def wait_for_completion(self):
        """
        等待舵机到达目标位置
        此方法将阻塞当前线程，直到舵机到达目标位置
        """
        if self.is_done():
            return
        start_time = time.time()
        while True:
            if self.is_done():
                return
            time.sleep(0.05)
            if time.time() - start_time > 3:
                logger.warning(f"[{self.name}] 同步操作超过3秒超时")
                return

    def get_position(self):
        """返回当前舵机的绝对角度（度）"""
        try:
            # 更新舵机状态并获取角度
            self.servo_manager.update()
            angle = self.servo_manager.query_servo_angle(self.servo_id)
            print(angle)
            return angle * self.reverse if angle is not None else 0.0
        except Exception as e:
            logger.warning(f"[{self.name}] 获取位置失败: {e}")
            return 0.0

    def reset(self):
        """
        将舵机移动到启动时的位置（init_angle）
        """
        logger.info(
            f"[{self.name}] 开始复位，目标位置 {self.init_angle:.2f}°，现在位置 {self.get_position():.2f}°"
        )
        self.move_to(self.init_angle)
        self.wait_for_completion()


#############################################
# 整体机器人控制类：封装头部和手臂控制接口
#############################################
class BodyController:
    def __init__(self):
        self.do_gesture = os.getenv('do_gesture')
        if self.do_gesture == "True":
            # 初始化串口（根据实际情况修改设备端口和波特率）
            uart_port = os.getenv("SERVO_PORT", "/dev/ttyUSB0")
            uart_baudrate = int(os.getenv("SERVO_BAUDRATE", "115200"))
            
            self.uart = serial.Serial(
                port=uart_port,
                baudrate=uart_baudrate,
                parity=serial.PARITY_NONE,
                stopbits=1,
                bytesize=8,
                timeout=0
            )
            
            # 创建舵机管理器
            self.servo_manager = UartServoManager(self.uart, is_debug=False)

            # 创建舵机实例：
            # 手臂：右手 servo_id=101, 左手 servo_id=102
            self.right_arm_motor = MotorWrapper(
                name="right_arm",
                servo_manager=self.servo_manager,
                servo_id=int(os.getenv("RIGHT_ARM_SERVO_ID", "101")),
                soft_limit=None,  # 软限位：无
                limit_speed=float(os.getenv("RIGHT_ARM_LIMIT_SPEED", "3.0")),
                reverse=True,  # 因为安装在右侧所以正方向是逆时针
            )
            self.left_arm_motor = MotorWrapper(
                name="left_arm",
                servo_manager=self.servo_manager,
                servo_id=int(os.getenv("LEFT_ARM_SERVO_ID", "103")),
                soft_limit=None,  # 软限位：无
                limit_speed=float(os.getenv("LEFT_ARM_LIMIT_SPEED", "3.0")),
            )

            # 头部：旋转 servo_id=103, 俯仰 servo_id=104
            self.head_rotate_motor = MotorWrapper(
                name="head_rotate",
                servo_manager=self.servo_manager,
                servo_id=int(os.getenv("HEAD_ROTATE_SERVO_ID", "104")),
                soft_limit=(
                    float(os.getenv("HEAD_ROTATE_SOFT_LIMIT_MIN", "-40")),
                    float(os.getenv("HEAD_ROTATE_SOFT_LIMIT_MAX", "40"))
                ),
                limit_speed=float(os.getenv("HEAD_ROTATE_LIMIT_SPEED", "1.2")),
            )
            self.head_tilt_motor = MotorWrapper(
                name="head_tilt",
                servo_manager=self.servo_manager,
                servo_id=int(os.getenv("HEAD_TILT_SERVO_ID", "102")),
                soft_limit=(
                    float(os.getenv("HEAD_TILT_SOFT_LIMIT_MIN", "-10")),
                    float(os.getenv("HEAD_TILT_SOFT_LIMIT_MAX", "10"))
                ),
                limit_speed=float(os.getenv("HEAD_TILT_LIMIT_SPEED", "1.5")),
                reverse=False,
            )

    async def _async_execute_motion(self, motor, angle, mode, blocking):
        """异步执行运动的通用方法"""
        if mode == "relative":
            await motor.async_move(angle)
        elif mode == "absolute":
            await motor.async_move_to(angle)
        else:
            logger.error(f"无效的运动模式: {mode}")
        if blocking:
            await motor.async_wait_for_completion()

    async def async_xhead(self, direction, angle, blocking, mode="absolute"):
        """
        异步控制头部左右运动
        :param direction: 头部左右运动的方向，可选值为 'left', 'right'
        :param angle: 运动的角度，单位为度（°），表示相对运动的增量或绝对位置
        :param mode: 运动模式，'relative' 表示相对运动，'absolute' 表示绝对位置，默认为 'relative'
        """
        # 检查输入的方向是否有效，如果无效则记录错误日志并返回
        if direction not in ["left", "right"]:
            logger.error(f"无效的头部左右运动方向: {direction}")
            return

        # 左右方向使用头部旋转电机
        motor = self.head_rotate_motor
        # 根据方向确定角度的正负，向右运动时使用正值，向左运动时使用负值
        angle = abs(angle) if direction == "right" else -abs(angle)
        # 根据运动模式选择合适的异步运动方法
        await self._async_execute_motion(motor, angle, mode, blocking)

    async def async_yhead(self, direction, angle, blocking, mode="absolute"):
        """
        异步控制头部上下运动
        :param direction: 头部上下运动的方向，可选值为 'up', 'down'
        :param angle: 运动的角度，单位为度（°），表示相对运动的增量或绝对位置
        :param mode: 运动模式，'relative' 表示相对运动，'absolute' 表示绝对位置，默认为 'relative'
        """
        # 检查输入的方向是否有效，如果无效则记录错误日志并返回
        if direction not in ["up", "down"]:
            logger.error(f"无效的头部上下运动方向: {direction}")
            return

        # 上下方向使用头部俯仰电机
        motor = self.head_tilt_motor
        # 根据方向确定角度的正负，向下运动时使用正值，向上运动时使用负值
        angle = -abs(angle) if direction == "up" else abs(angle)
        # 根据运动模式选择合适的异步运动方法
        await self._async_execute_motion(motor, angle, mode, blocking)

    async def async_left_arm(self, direction, angle, blocking, mode="absolute"):
        """异步控制左臂运动"""
        if self.do_gesture == "True":
            if direction not in ["up", "down"]:
                logger.error(f"无效的左臂运动方向: {direction}")
                return
            motor = self.left_arm_motor
            angle = abs(angle) if direction == "up" else -abs(angle)
            await self._async_execute_motion(motor, angle, mode, blocking)
        else:
            logger.info(f"左臂运动方向：{direction}")

    async def async_right_arm(self, direction, angle, blocking, mode="absolute"):
        if self.do_gesture == "True":
            """异步控制右臂运动"""
            if direction not in ["up", "down"]:
                logger.error(f"无效的右臂运动方向: {direction}")
                return
            motor = self.right_arm_motor
            angle = abs(angle) if direction == "up" else -abs(angle)
            await self._async_execute_motion(motor, angle, mode, blocking)
        else:
            logger.info(f"右臂运动方向：{direction}")

    async def async_wait_all(self):
        """异步等待所有电机到达目标位置"""
        if self.do_gesture == "True":
            await asyncio.gather(
                self.right_arm_motor.async_wait_for_completion(),
                self.left_arm_motor.async_wait_for_completion(),
                self.head_rotate_motor.async_wait_for_completion(),
                self.head_tilt_motor.async_wait_for_completion(),
            )
        else:
            logger.info("等待所有舵机完成（模拟模式）")

    async def async_reset_body(self):
        """异步复位所有电机"""
        if self.do_gesture == "True":
            await asyncio.gather(
                self.right_arm_motor.async_move_to(self.right_arm_motor.init_angle),
                self.left_arm_motor.async_move_to(self.left_arm_motor.init_angle),
                self.head_rotate_motor.async_move_to(self.head_rotate_motor.init_angle),
                self.head_tilt_motor.async_move_to(self.head_tilt_motor.init_angle),
            )
            await asyncio.gather(
                self.right_arm_motor.async_wait_for_completion(),
                self.left_arm_motor.async_wait_for_completion(),
                self.head_rotate_motor.async_wait_for_completion(),
                self.head_tilt_motor.async_wait_for_completion(),
            )
            logger.info("[BodyController] 所有舵机已异步复位到启动位置")
        else:
            logger.info("舵机已复位")

    def _execute_motion(self, motor, angle, mode, blocking):
        """同步执行运动的通用方法"""
        if mode == "relative":
            motor.move(angle)
        elif mode == "absolute":
            motor.move_to(angle)
        else:
            logger.error(f"无效的运动模式: {mode}")
        if blocking:
            motor.wait_for_completion()

    def xhead(self, direction, angle, blocking, mode="absolute"):
        """同步控制头部左右运动"""
        if self.do_gesture == "True":
            if direction not in ["left", "right"]:
                logger.error(f"无效的头部左右运动方向: {direction}")
                return
            motor = self.head_rotate_motor
            angle = abs(angle) if direction == "right" else -abs(angle)
            self._execute_motion(motor, angle, mode, blocking)
        else:
            logger.info(f"头部左右运动方向：{direction}")

    def yhead(self, direction, angle, blocking, mode="absolute"):
        """同步控制头部上下运动"""
        if self.do_gesture == "True":
            if direction not in ["up", "down"]:
                logger.error(f"无效的头部上下运动方向: {direction}")
                return
            motor = self.head_tilt_motor
            angle = abs(angle) if direction == "up" else -abs(angle)
            self._execute_motion(motor, angle, mode, blocking)
        else:
            logger.info(f"头部上下运动方向：{direction}")

    def left_arm(self, direction, angle, blocking, mode="absolute"):
        """同步控制左臂运动"""
        if self.do_gesture == "True":
            if direction not in ["up", "down"]:
                logger.error(f"无效的左臂运动方向: {direction}")
                return
            motor = self.left_arm_motor
            angle = abs(angle) if direction == "up" else -abs(angle)
            self._execute_motion(motor, angle, mode, blocking)
        else:
            logger.info(f"左臂运动方向：{direction}")

    def right_arm(self, direction, angle, blocking, mode="absolute"):
        """同步控制右臂运动"""
        if self.do_gesture == "True":
            if direction not in ["up", "down"]:
                logger.error(f"无效的右臂运动方向: {direction}")
                return
            motor = self.right_arm_motor
            angle = abs(angle) if direction == "up" else -abs(angle)
            self._execute_motion(motor, angle, mode, blocking)
        else:
            logger.info(f"右臂运动方向：{direction}")

    def wait_all(self):
        """等待所有电机到达目标位置"""
        if self.do_gesture == "True":
            self.right_arm_motor.wait_for_completion()
            self.left_arm_motor.wait_for_completion()
            self.head_rotate_motor.wait_for_completion()
            self.head_tilt_motor.wait_for_completion()
        else:
            logger.info("等待所有舵机完成（模拟模式）")

    def status(self):
        """
        返回当前各关节角度的字符串，用于日志打印
        """
        if self.do_gesture == "True":
            status_msg = (
                f"左臂: {self.left_arm_motor.get_position():.2f}°\n"
                f"右臂: {self.right_arm_motor.get_position():.2f}°\n"
                f"头部旋转: {self.head_rotate_motor.get_position():.2f}°\n"
                f"头部俯仰: {self.head_tilt_motor.get_position():.2f}°"
            )
            logger.info("\n" + status_msg)
            return status_msg
        else:
            status_msg = "模拟模式 - 所有舵机状态：正常"
            logger.info(status_msg)
            return status_msg

    def stop(self):
        if self.do_gesture == "True":
            self.left_arm_motor.stop()
            self.right_arm_motor.stop()
            self.head_rotate_motor.stop()
            self.head_tilt_motor.stop()
        else:
            logger.info("停止所有舵机（模拟模式）")

    async def reset_body(self):
        """
        将整个机器人回到启动时的位置，依次调用各个电机的 reset() 方法
        """
        if self.do_gesture =="True":
            self.right_arm_motor.reset()
            self.left_arm_motor.reset()
            #self.head_rotate_motor.reset()
            #self.head_tilt_motor.reset()
            logger.info("[BodyController] 所有电机均已复位到启动位置")
        else:
            logger.info("电机已复位")

    def shutdown(self):
        self.stop()
        if self.do_gesture == "True":
            self.uart.close()


global_body_controller = BodyController()


#############################################
# 主程序：示例调用
#############################################
async def async_main():
    try:
        controller = global_body_controller
        logger.info("测试异步非阻塞控制")
        await controller.async_left_arm("up", 180, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_left_arm("up", 0, blocking=False)
        await controller.async_wait_all()
        input()
        # controller.stop()
        # await controller.async_reset_body()
        
        time.sleep(0.2)
        await controller.async_right_arm("up", 180, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_right_arm("up", 0, blocking=False)
        await controller.async_wait_all()
        input()
        # controller.stop()
        # await controller.async_reset_body()

        await controller.async_xhead("left", 20, blocking=False)
        await controller.async_wait_all()
        await controller.async_xhead("right", 20, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_xhead("left", 0, blocking=False)
        await controller.async_wait_all()
        input()
        # controller.stop()
        # await controller.async_reset_body()

        await controller.async_yhead("up", 10, blocking=False)
        await controller.async_wait_all()
        await controller.async_yhead("down", 0, blocking=False)
        await controller.async_yhead("down", 5, blocking=False)
        await controller.async_yhead("down", 10, blocking=False)
        await controller.async_wait_all()
        input()
        controller.stop()
        await controller.async_reset_body()

    finally:
        await controller.async_reset_body()
        controller.stop()
        controller.shutdown()
        logger.info("舵机控制器已关闭")


async def async_main_2():
    try:
        controller = global_body_controller
        logger.info("测试异步非阻塞控制")

        await controller.async_reset_body()

        input("press enter to up")
        await controller.async_yhead("up", 0, blocking=False)
        await controller.async_yhead("up", 5, blocking=False)
        await controller.async_yhead("up", 10, blocking=False)
        await controller.async_wait_all()
        
        input("press enter to down")
        await controller.async_yhead("down", 0, blocking=False)
        await controller.async_yhead("down", 5, blocking=False)
        await controller.async_yhead("down", 10, blocking=False)
        await controller.async_wait_all()
        
        input("press enter to reset")
        await controller.async_reset_body()

        input("press enter to end")
        controller.stop()


    finally:
        await controller.async_reset_body()
        controller.stop()
        controller.shutdown()
        logger.info("舵机控制器已关闭")


async def sync_main():
    try:
        controller = global_body_controller
        logger.info("测试同步阻塞控制")
        if controller.do_gesture == "True":
            controller.left_arm("up", 90, blocking=True)
            controller.left_arm("up", 0, blocking=True)
            controller.wait_all()

            input()
            # controller.stop()
            # await controller.async_reset_body()
            controller.right_arm("up", 90, blocking=True)
            controller.right_arm("up", 0, blocking=True)
            controller.wait_all()
            input()
            # controller.stop()
            # await controller.async_reset_body()
            
        else:
            logger.info("同步控制测试完成（模拟模式）")

    finally:
        controller.stop()
        await controller.async_reset_body()
        controller.shutdown()
        logger.info("舵机控制器已关闭")



async def async_main_3():
    try:
        controller = global_body_controller
        logger.info("测试异步非阻塞控制")

        await controller.async_reset_body()

        input("press enter to test head tilt")
        await controller.async_yhead("up", 10, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_yhead("down", 10, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_yhead("up", 10, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_yhead("down", 10, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_wait_all()

        input("press enter to test head rotate")
        await controller.async_xhead("left", 20, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_xhead("right", 20, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_xhead("left", 20, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_xhead("right", 10, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_wait_all()

        input("press enter to test left arm")
        await controller.async_left_arm("up", 90, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_left_arm ("down", 0, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_left_arm("down", 90, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_left_arm("up", 90, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_left_arm("down", 10, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_wait_all()

        input("press enter to test right arm")
        await controller.async_right_arm("up", 90, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_right_arm ("down", 0, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_right_arm("down", 90, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_right_arm("up", 90, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_right_arm("down", 10, blocking=False)
        await asyncio.sleep(0.5)
        await controller.async_wait_all()



        input("press enter to reset")
        await controller.async_reset_body()

        input("press enter to stop")
        controller.stop()


    finally:
        await controller.async_reset_body()
        controller.stop()
        controller.shutdown()
        logger.info("舵机控制器已关闭")


async def async_main_4():
    try:
        controller = global_body_controller
        logger.info("测试异步非阻塞控制")

        await controller.async_reset_body()

        input("press enter to test head tilt")
        await controller.async_yhead("up", 10, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_yhead("down", 10, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_yhead("up", 10, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_yhead("down", 10, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_wait_all()

        # input("press enter to test head rotate")
        await controller.async_xhead("left", 20, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_xhead("right", 20, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_xhead("left", 20, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_xhead("right", 10, blocking=False)
        # await asyncio.sleep(0.5)
        await controller.async_wait_all()

        input("press enter to test left arm")
        await controller.async_left_arm("up", 90, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_left_arm ("down", 0, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_left_arm("down", 90, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_left_arm("up", 90, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_left_arm("down", 10, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_wait_all()

        # input("press enter to test right arm")
        await controller.async_right_arm("up", 90, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_right_arm ("down", 0, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_right_arm("down", 90, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_right_arm("up", 90, blocking=False)
        # await asyncio.sleep(0.5)
        # await controller.async_right_arm("down", 10, blocking=False)
        # await asyncio.sleep(0.5)
        await controller.async_wait_all()



        input("press enter to reset")
        await controller.async_reset_body()

        input("press enter to stop")
        controller.stop()


    finally:
        await controller.async_reset_body()
        controller.stop()
        controller.shutdown()
        logger.info("舵机控制器已关闭")



if __name__ == "__main__":
    
    # asyncio.run(async_main())
    # asyncio.run(sync_main())
    # asyncio.run(async_main_2())
    # asyncio.run(async_main_3())
    asyncio.run(async_main_4())

    ## 不要频繁调用反复调用 async_reset_body 和 stop 方法
