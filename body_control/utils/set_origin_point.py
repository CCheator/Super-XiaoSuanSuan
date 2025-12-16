'''
总线伺服舵机
> Python SDK角度设置 Example <
注意：以下包括单圈角度和多圈角度指令
--------------------------------------------------
 * 作者: 深圳市华馨京科技有限公司
 * 网站：https://fashionrobo.com/
 * 更新时间: 2024/12/23
--------------------------------------------------
'''
# 添加uservo.py的系统路径
import sys
sys.path.append("../../src")
# 导入依赖
import time
import struct
import serial
from uservo import UartServoManager

# 参数配置
# 角度定义
SERVO_PORT_NAME =  '/dev/ttyUSB0'		# 舵机串口号
SERVO_BAUDRATE = 115200			# 舵机的波特率
SERVO_HAS_MTURN_FUNC = False	# 舵机是否拥有多圈模式

# 初始化串口
uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,\
					 parity=serial.PARITY_NONE, stopbits=1,\
					 bytesize=8,timeout=0)
# 初始化舵机管理器
uservo = UartServoManager(uart, is_debug=True)


SERVO_ID = 104					# 舵机的ID号


for SERVO_ID in range(101, 105):
    uservo.disable_torque(SERVO_ID)
    uservo.set_origin_point(SERVO_ID)
