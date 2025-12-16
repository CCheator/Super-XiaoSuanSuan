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

############################################################################

for SERVO_ID in range(101, 105):
    uservo.set_servo_angle(SERVO_ID, 0.0, velocity=20)
    uservo.wait() # 等待舵机静止

print("--------------head_tilt------------------")
############################################################################

SERVO_ID = 104					# 舵机的ID号

input('press enter to continue')
uservo.set_servo_angle(SERVO_ID, 20.0, velocity=100)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

input('press enter to continue')
# uservo.set_servo_angle(SERVO_ID, 00.0, velocity=100)
# uservo.wait() # 等待舵机静止
# print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

uservo.set_servo_angle(SERVO_ID, -20.0, velocity=100)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

input('press enter to continue')
uservo.set_servo_angle(SERVO_ID, 00.0, velocity=100)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

print("--------------head_rotate------------------")
#############################################################################
SERVO_ID = 102					# 舵机的ID号

input('press enter to continue')
uservo.set_servo_angle(SERVO_ID, 20.0, velocity=100)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

input('press enter to continue')

uservo.set_servo_angle(SERVO_ID, -20.0, velocity=100)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

input('press enter to continue')

uservo.set_servo_angle(SERVO_ID, 0.0, velocity=100)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

print("--------------right_arm------------------")
#############################################################################
SERVO_ID = 101					# 舵机的ID号

input('press enter to continue')
uservo.set_servo_angle(SERVO_ID, 90.0, velocity=500)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

input('press enter to continue')

uservo.set_servo_angle(SERVO_ID, -90.0, velocity=500)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

input('press enter to continue')

uservo.set_servo_angle(SERVO_ID, 0.0, velocity=500)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))


print("--------------left_arm------------------")
#############################################################################
SERVO_ID = 103					# 舵机的ID号

input('press enter to continue')
uservo.set_servo_angle(SERVO_ID, 90.0, velocity=500)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

input('press enter to continue')

uservo.set_servo_angle(SERVO_ID, -90.0, velocity=500)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))

input('press enter to continue')

uservo.set_servo_angle(SERVO_ID, 0.0, velocity=500)
uservo.wait() # 等待舵机静止
print("-> {}".format(uservo.query_servo_angle(SERVO_ID)))


############################################################################

input('press enter to continue')

for SERVO_ID in range(101, 105):
    uservo.disable_torque(SERVO_ID)

