#程序版本 2024年11月25日
#调用库
#jiayige arm_rejection
print("程序启动")
print("请务必保证舱内挡板处于收起的状态！")
import RPi.GPIO as GPIO
import gpiozero
import pygame
import time
import cv2
import numpy as np
import serial
from picamera2 import Picamera2, Preview
import glob
import os
import threading
import json
import os
import smbus2
import time
import math
import struct
from gpiozero import Device #最后两个是专门用来配配置引脚库的
from gpiozero.pins.rpigpio import RPiGPIOFactory
from concurrent.futures import ThreadPoolExecutor
print("库导入完成")


#调试参数
auto_run = True #是否需要开机就执行程序，必须要开启才能正常执行程序
body_check = False #
auto_calibration = False  # 是否强制重新校正
side = False #红蓝方切换，False  红, True 蓝，后续可以使用按键切换
test_mode = True #调试模式标记，调试模式开启状态下不需要按下按键就可以启动程序
test_part = True #直接执行测试部分代码，在合并到主代码之前很有用
if test_mode == True and test_part == False:
    auto_completed = False#指定测试自动模式或手动模式，只有在调试模式激活的情况下才有效
fanin_bypass = True #吸入风扇旁通，代替挡板控制，此时SERVO3作为吸入风机
fanout_bypass = False #吹出风扇旁通，代替挡板控制，此时GPIO25作为吹出风机
roller_bypass = True #大滚轮旁通，使用满速的蜗杆电机代替360连续旋转舵机
board_place = True #（此选项一般不需要更改）舱内挡板是否归位，True为收起状态，False为伸出状态
ball_out_assist = True #使用滚轮辅助出球
servo5_switch = False #摄像头水平舵机 是否启用
servo6_switch = True #摄像头竖直舵机 是否启用
 
#GPIO参数
AIN1 = 17 #左电机
AIN2 = 27
PWM1 = 14
BIN1 = 10 #右电机
BIN2 = 22
PWM2 = 15
SERVO0 = 18 #滚轮上下舵机（左侧）
SERVO1 = 23 #大滚轮舵机
SERVO2 = 19 # 滚轮上下舵机（右侧）
SERVO3 = 5 # 舱内挡板舵机
#SERVO4 = 24 #后侧舵机
#SERVO5 = 12 #摄像头水平舵机
SERVO6 = 12 #摄像头竖直舵机
BUTTON1 = 11 #启动按键
BUTTON2 = 9 #红蓝方切换按键
LED1 = 20 #程序运行指示灯
LED2 = 16 #红方指示灯
LED3 = 6 #蓝方指示灯
BUZZER1 = 26 #蜂鸣器
FANIN = 25
ROLLER_REVERSE = 21
GYROSCOPE_SDA = 2 #陀螺仪I2C的SDA接口
GYROSCOPE_SCL = 3 #陀螺仪I2C的SCL接口
#E1A = 4 #以下四个参数为电机编码器反馈使用来进行PID调速
#E1B = 24
#E2A = 8
#E2B = 7



#控制参数
PERIOD1 = 1000 #左电机PWM信号频率
DUTY1 = 100 #左电机占空比
PERIOD2 = 1000 #右电机PWM信号频率
DUTY2 = 100 #右电机PWM占空比
ANGLE_UP_SERVO0 = -90 #滚轮上下舵机(左侧)的上限角度  下面十个变量需要修改！！！！！
ANGLE_BALL_SERVO0 = -40 #滚轮上下舵机（左侧）的进球角度
ANGLE_DOWN_SERVO0 = -25  #滚轮上下舵机（左侧）的下限角度
ANGLE_UP_SERVO2 = 80  #滚轮上下舵机(右侧)的上限角度
ANGLE_BALL_SERVO2 = 27 #滚轮上下舵机（右侧）的进球角度
ANGLE_DOWN_SERVO2 = 10  #滚轮上下舵机（右侧）的下限角度
#ANGEL_UP_SERVO4 = 90 #后侧舵机的抬起角度
#ANGEL_DOWN_SERVO4 = 90 #后侧舵机的放下角度
ANGLE_UP_SERVO5 = -70 # 摄像头水平舵机 左极限
ANGLE_FIELD_SERVO5 = -30 # 摄像头水平舵机 场地跟踪角度 ##（后期实现自动化后此函数废弃）
ANGLE_BALL_SERVO5 = 30  # 摄像头水平舵机 单球跟踪角度 ##（后期实现自动化后此函数废弃）
ANGLE_DOWN_SERVO5 = 90 # 摄像头水平舵机 右极限
ANGLE_UP_SERVO6 = 0 # 摄像头竖直舵机 上极限
ANGLE_FIELD_SERVO6 = 10 # 摄像头竖直舵机 场地跟踪角度 ##（后期实现自动化后此函数作为初始化阶段使用的角度）
ANGLE_BALL_SERVO6 = 40 # 摄像头竖直舵机 单球跟踪角度 ##（后期实现自动化之后此函数废弃）
ANGLE_DOWN_SERVO6 = 50 # 摄像头竖直舵机 下极限
LONG_PRESS_TIME  = 1.5 #定义长按的时间
SHORT_PRESS_TIME = 0.2 #定义短按的时间
AUTO_BEEP_DELAY = 0.25 #定义自动模式响声的时延
AUTO_BEEP_TIMES = 2 #定义自动模式响声的次数
MANUAL_BEEP_DELAY = 0.75 #定义手动模式响声的时延
MANUAL_BEEP_TIMES = 1 #定义手动模式响声的次数
ZERO_SERVO5 = 0 #摄像头水平舵机 归零位置
ZERO_SERVO6 = -80 #摄像头竖直舵机 归零位置
SERVO5_SWITCH = True # 摄像头水平舵机 是否启用
SERVO6_SWITCH = True #  摄像头竖直舵机 是否启用
TIME_SLEEP_SERVO = 0.5
SPEED_FIRST_SERVO3 = 0.17 #挡板初始化运行速度
TIME_FIRST_SERVO3 = 3 #挡板初始化复位时间
TIME_FIRST_SERVO = 1


if roller_bypass == False:
    SPEED_SERVO1 = 1 #滚轮舵机的速度
    #PWM_SERVO1 = 0 #此函数只有排查解释器报错时刻才有用
if roller_bypass == True:
    SPEED_SERVO1 = 1 #旁通模式下是蜗杆电机的速度
    PWM_SERVO1 = 100

if fanin_bypass == False:
    #SPEED_FANIN = 0.5 #舱内挡板的速度
    #PWM_FANIN = 0 #此函数只有排查解释器报错时刻才有用
    pass

if fanin_bypass == True:
    SPEED_FANIN = 1 #旁通模式下是吸入风机的速度
    PWM_FANIN = 1000
    TIME_FANIN = 3
    #TIME_FANOUT = 7#此函数只有排查解释器报错时刻才有用

if fanout_bypass == True: #旁通模式下是吹出风机的速度
    SPEED_FANOUT = 1
    PWM_FANOUT = 1000
    TIME_FANOUT = 7
    #TIME_FANIN = 3#此函数只有排查解释器报错时刻才有用

if fanout_bypass == False: #此函数只有排查解释器报错时刻才有用
    SPEED_SERVO3 = 1
    PWM_SERVO3 = 1000


if fanin_bypass == False or fanout_bypass == False:
    TIME_SERVO3 = 4 #舱内挡板的运行时间，单位为秒

#if fanin_bypass == True and fanout_bypass == True:#此函数只有排查解释器报错时刻才有用
    #TIME_SERVO3 = 3 #舱内挡板的运行时间，单位为秒



#手柄键位（默认配置），此处使用北通阿修罗2pro+星闪手柄
A = 0
B = 1
X = 2
Y = 3
RT = 5
LT = 2
LB = 4
RB = 5
BACK = 6
START = 7
CENTRAL = 8
LEFT_Y = 1
LEFT_X = 0
LEFT_PRESS = 9
RIGHT_Y = 3
RIGHT_X = 4
RIGHT_PRESS = 10

#检测是否要执行程序
if auto_run == False:
    exit()

# 设置全局 pin_factory
Device.pin_factory = RPiGPIOFactory()
gpiozero.Device.pin_factory = gpiozero.pins.rpigpio.RPiGPIOFactory()
print(gpiozero.Device.pin_factory)
print("请确认程序是否在")



#引脚初始化阶段
print("引脚初始化")
try:
    GPIO.cleanup()
except:
    pass
try:
    pygame.quit() 
except:
    pass



#配置初始化
print("配置初始化")
GPIO.setmode(GPIO.BCM)#重置GPIO引脚

#定义收球角度变量检查
class 收球角度离谱(Exception):
    """预防收球的角度比上限大，比下限小"""
    pass

##收球角度安全检查
if min(ANGLE_UP_SERVO0, ANGLE_DOWN_SERVO0) > ANGLE_BALL_SERVO0:
    raise 收球角度离谱("滚轮臂 左侧舵机 收球角度 都比下限小了啊")
if max(ANGLE_UP_SERVO0, ANGLE_DOWN_SERVO0) < ANGLE_BALL_SERVO0:
    raise 收球角度离谱("滚轮臂 左侧舵机 收球角度 都比上限大了啊")
if min(ANGLE_UP_SERVO2, ANGLE_DOWN_SERVO2) > ANGLE_BALL_SERVO2:
    raise 收球角度离谱("滚轮臂 右侧舵机 收球角度 都比下限小了啊")
if max(ANGLE_UP_SERVO2, ANGLE_DOWN_SERVO2) < ANGLE_BALL_SERVO2:
    raise 收球角度离谱("滚轮臂 右侧舵机 收球角度 都比上限小了啊")
try:
    if min(ANGLE_UP_SERVO5, ANGLE_DOWN_SERVO5) > ANGLE_FIELD_SERVO5:
        raise 收球角度离谱("摄像头 水平舵机 场地跟踪角度 都比左极限还靠左了啊")
    if max(ANGLE_UP_SERVO5, ANGLE_DOWN_SERVO5) < ANGLE_FIELD_SERVO5:
        raise 收球角度离谱("摄像头 水平舵机 场地跟踪角度 都比右极限还靠右了啊")
except NameError:
    pass
try:
    if min(ANGLE_UP_SERVO5, ANGLE_DOWN_SERVO5) > ANGLE_BALL_SERVO5:
        raise 收球角度离谱("摄像头 水平舵机 单球跟踪角度 都比左极限靠左了啊")
    if max(ANGLE_UP_SERVO5, ANGLE_DOWN_SERVO5) < ANGLE_BALL_SERVO5:
        raise 收球角度离谱("摄像头 水平舵机 单球跟踪角度 都比右极限还靠右了啊")
except NameError:
    pass

try:
    if min(ANGLE_UP_SERVO6, ANGLE_DOWN_SERVO6) > ANGLE_BALL_SERVO6:
        raise 收球角度离谱("摄像头 竖直舵机 场地跟踪角度 都比下限还小了啊")
    if max(ANGLE_UP_SERVO6, ANGLE_DOWN_SERVO6) < ANGLE_BALL_SERVO6:
        raise 收球角度离谱("摄像头 竖直舵机 场地跟踪角度 都比上限还大了啊")
except NameError:
    pass
try:
    if min(ANGLE_UP_SERVO6, ANGLE_DOWN_SERVO6) > ANGLE_FIELD_SERVO6:
        raise 收球角度离谱("摄像头 竖直舵机 单球跟踪角度 都比下限还小了啊")
    if max(ANGLE_UP_SERVO6, ANGLE_DOWN_SERVO6) < ANGLE_FIELD_SERVO6:
        raise 收球角度离谱("摄像头 竖直舵机 单球跟踪角度 都比上限还大了啊")
except NameError:
    pass
if test_mode == False or test_part == True:
    auto_completed = False#指定测试自动模式或手动模式，只有在调试模式激活的情况下才有效

executor = ThreadPoolExecutor(max_workers=1000)  # 一个够用的线程值

#指示器件初始化
buzzer1 = gpiozero.Buzzer(BUZZER1)#蜂鸣器初始化
buzzer1.beep(on_time=0.5, off_time=0.1, n=1, background=True)
print("蜂鸣器 通过")

#LED初始化，完成之前关闭指示灯
led1 = gpiozero.LED(LED1)
led2 = gpiozero.LED(LED2)
led3 = gpiozero.LED(LED3)
def led_test():
    try:
        """
        利用多线程
        """
        led1.on()
        led2.on()
        led3.on()
        print("请注意观察LED指示灯是否正常")
        time.sleep(1)
        led1.off()
        led2.off()
        led3.off()
        print("指示灯 通过")
    except Exception as e:
        print(f"LED测试失败: {e}")
led_thread = threading.Thread(target=led_test)
led_thread.start()

#电机初始化
robot = gpiozero.Robot(left=(AIN1, AIN2), right=(BIN1, BIN2))
GPIO.setup(PWM1, GPIO.OUT)
GPIO.setup(PWM2, GPIO.OUT)
pwm_A = GPIO.PWM(PWM1, PERIOD1)
pwm_B = GPIO.PWM(PWM2, PERIOD2)
pwm_A.start(DUTY1)
pwm_B.start(DUTY2)
robot.stop()
print("左侧与右侧电机 已就绪")

#舵机初始化
servo0 = gpiozero.AngularServo(pin = SERVO0, min_pulse_width = 1 / 2000, max_pulse_width = 5 / 2000)

if roller_bypass == False:
    servo1 = gpiozero.Servo(pin = SERVO1, min_pulse_width = 1 / 2000, max_pulse_width = 5 / 2000)
if roller_bypass == True:
    GPIO.setup(SERVO1, GPIO.OUT)
    GPIO.setup(ROLLER_REVERSE, GPIO.OUT)
    pwm_roller_forward = GPIO.PWM(SERVO1, PWM_SERVO1)
    pwm_roller_reverse = GPIO.PWM(ROLLER_REVERSE, PWM_SERVO1)
    pwm_roller_forward.start(0)
    pwm_roller_reverse.start(0)

servo2 = gpiozero.AngularServo(pin = SERVO2, min_pulse_width = 1 / 2000, max_pulse_width = 5 / 2000)

if fanout_bypass == False:
    servo3 = gpiozero.Servo(pin = SERVO3, min_pulse_width = 1 / 2000, max_pulse_width = 5 / 2000)

if fanin_bypass == True:
    GPIO.setup(FANIN, GPIO.OUT)
    pwm_fanin = GPIO.PWM(FANIN, PWM_FANIN)
    pwm_fanin.start(0)

if fanout_bypass == True:
    GPIO.setup(SERVO3, GPIO.OUT)
    pwm_fanout = GPIO.PWM(SERVO3, PWM_FANOUT)
    pwm_fanout.start(0)

#servo6 = gpiozero.AngularServo(pin = SERVO6, min_pulse_width = 1 / 2000, max_pulse_width = 5 / 2000)
GPIO.setup(SERVO6, GPIO.OUT)

def set_servo_angle(angle):  #单独定义servo6

    duty_cycle = 2.5 + ((angle + 90) / 180.0) * (12.5 - 2.5)
    pwm_servo6.ChangeDutyCycle(duty_cycle)  # 设置占空比
    time.sleep(0.1)  # 给舵机足够时间移动到目标位置
    pwm_servo6.ChangeDutyCycle(0)

pwm_servo6 = GPIO.PWM(SERVO6, 50)
pwm_servo6.start(0)

time.sleep(0.1)
servo0.angle = ANGLE_UP_SERVO0
servo2.angle = ANGLE_UP_SERVO2
#servo5.angle = ZERO_SERVO5
servo3.value = 0
set_servo_angle(ZERO_SERVO6)
time.sleep(TIME_FIRST_SERVO)
servo0.detach()
servo2.detach()
#servo5.detach()
servo3.detach()

#servo4 = gpiozero.AngularServo(SERVO4)

if fanin_bypass == False and fanout_bypass == False and roller_bypass == False:
    print("舵机 已配置，稍后将完成初始化")
elif fanin_bypass == False and fanout_bypass == False and roller_bypass == True:
    print("舵机与蜗杆电机 已配置，稍后将完成初始化")
else:
    if roller_bypass == False:
        print("舵机与风机 已配置，稍后将完成初始化")
    else:
        print("舵机、风机与蜗杆电机 已配置，稍后将完成初始化")


#按键初始化
button1 = gpiozero.Button(BUTTON1) #按键初始化
button2 = gpiozero.Button(BUTTON2)
print("按键 已就绪")

#手柄初始化
pygame.init() #初始化pygame
pygame.joystick.init() #初始化手柄模块
joystick_count = pygame.joystick.get_count() #获取手柄的链接数量
if joystick_count == 0:
    print("手柄 未连接")
else:
    print(f"检测到 {joystick_count} 个手柄已连接。")

    # 循环获取每个手柄的信息
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        # 打印手柄的名称和轴数等信息
        print(f"手柄 {i + 1}:")
        print(f"  名称: {joystick.get_name()}")
        print(f"  轴数: {joystick.get_numaxes()}")
        print(f"  按钮数: {joystick.get_numbuttons()}")



#准备函数定义

#定义红蓝方切换
def update_side_leds():
    try:
        """
        更新指示灯状态，根据当前的红蓝方状态点亮相应的灯。
        """
        if side == True:
            led2.off()
            led3.on()
            print("当前模式：蓝方")
        elif side == False:
            led2.on()
            led3.off()
            print("当前模式：红方")
            #红蓝双方LED灯切换
    except Exception as e:
        print(f"指示灯更新失败: {e}")

# 指示灯闪烁函数
def led1_blink():
    try:
        while True:
            led1.on()  # 指示灯亮
            time.sleep(0.5)
            led1.off()  # 指示灯灭
            time.sleep(0.5)
    except Exception as e:
        print(f"主程序指示灯闪烁失败: {e}")

##定义更改红蓝方操作
def change_side():
    global side
    try:
        side = not side  # 切换红蓝方状态
        update_side_leds()  # 更新指示灯状态
    except Exception as e:
        print(f"红蓝方切换失败: {e}")

##以上为基础功能性定义函数，以下是实际功能函数

##滚轮舵机 正转
def roller_forward():
    try:
        if roller_bypass == False:
            servo1.value = SPEED_SERVO1 # 传入舵机1和速度
        if roller_bypass == True:
            pwm_roller_forward.ChangeDutyCycle(100*SPEED_SERVO1)
            pwm_roller_reverse.ChangeDutyCycle(0) 
    except Exception as e:
        print(f"滚轮舵机正转失败: {e}")

##滚轮舵机 反转
def roller_backward():
    try:
        if ball_out_assist == True:
            if roller_bypass == False:
                servo1.value = -SPEED_SERVO1  # 传入舵机1和速度
            if roller_bypass == True:
                pwm_roller_forward.ChangeDutyCycle(0)  
                pwm_roller_reverse.ChangeDutyCycle(100*SPEED_SERVO1)
        if ball_out_assist == False:
            pass
    except Exception as e:
        print(f"滚轮舵机反转失败: {e}")

##滚轮舵机 关闭
def roller_off():
    try:
        if roller_bypass == False:
            servo1.value = 0  # 传入舵机1和停止命令
        if roller_bypass == True:
            pwm_roller_forward.ChangeDutyCycle(0)
            pwm_roller_reverse.ChangeDutyCycle(0)
    except Exception as e:
        print(f"滚轮舵机关闭失败: {e}")

##滚轮臂舵机 升起
arm_lock = threading.Lock() #定义滚轮锁

def arm_up():
    if arm_lock.acquire(timeout=2):
        try:
            servo2.angle = ANGLE_UP_SERVO2
            servo0.angle = ANGLE_UP_SERVO0
            time.sleep(TIME_SLEEP_SERVO)
            servo2.detach()
            servo0.detach()
        except Exception as e:
            print(f"滚轮抬起失败: {e}")
        finally:
            arm_lock.release()
    else:
        buzzer1.beep(on_time=0.5, off_time=0.1, n=1, background=True)
        print("貌似滚轮操作有的大量未操作的卡顿，已经丢弃")


##滚轮臂舵机 降下
def arm_down():
    if arm_lock.acquire(timeout=2):
        try:
            servo2.angle = ANGLE_DOWN_SERVO2
            servo0.angle = ANGLE_DOWN_SERVO0 
            time.sleep(TIME_SLEEP_SERVO)
            servo2.detach()
            servo0.detach()
        except Exception as e:
            print(f"滚轮降下失败: {e}")
        finally:
            arm_lock.release()
    else:
        buzzer1.beep(on_time=0.5, off_time=0.1, n=1, background=True)
        print("貌似滚轮操作有的大量未操作的卡顿，已经丢弃")

##滚轮臂舵机 调整到收球状态
def arm_ball():
    if arm_lock.acquire(timeout=2):
        try:
            servo2.angle = ANGLE_BALL_SERVO2
            servo0.angle = ANGLE_BALL_SERVO0 
            time.sleep(TIME_SLEEP_SERVO)
            servo2.detach()
            servo0.detach()
        except Exception as e:
            print(f"滚轮置于收球位失败: {e}")
        finally:
            arm_lock.release()
    else:
        buzzer1.beep(on_time=0.5, off_time=0.1, n=1, background=True)
        print("貌似滚轮操作有的大量未操作的卡顿，已经丢弃")


##定义舱内挡板的往复运动
board_lock = threading.Lock()
def board_movement(): #实现舱内挡板的移动
    if board_lock.acquire(blocking=False):
        try:
            servo3.value = SPEED_SERVO3 
            time.sleep(TIME_SERVO3 + 0.3)
            servo3.value = 0
            board_place = False
            time.sleep(0.2) ##略做缓冲
            servo3.value = -SPEED_SERVO3 # 传入舵机3和速度
            time.sleep(TIME_SERVO3) #旋转一段时间
            servo3.value = -0.6
            time.sleep(0.5)
            servo3.value = 0 #终止旋转
            board_place = True
        except Exception as e:
            print(f"挡板移动失败: {e}")
        finally:
            board_lock.release()
    else:
        buzzer1.beep(on_time=0.3, off_time=0.2, n=2, background=True)
        print("注意挡板冲突")

##舱内舵机 推出
ball_lock = threading.Lock() #定义一把球相关的程序锁

def ball_out():
    if ball_lock.acquire(blocking=False):
        try:
            executor.submit(arm_ball)
            if fanout_bypass == False: #使用挡板出球
                executor.submit(board_movement)
                if ball_out_assist == True:
                    roller_backward()

            if fanout_bypass == True:
                pwm_fanout.ChangeDutyCycle(100*SPEED_FANOUT)
                if fanin_bypass == True:
                    pwm_fanin.ChangeDutyCycle(0)
                if ball_out_assist == True:
                    roller_backward()#使用time.sleep(TIME_FANOUT)来隔一段时间
                if ball_out_assist == False:
                    roller_off()
        except Exception as e:
            print(f"出球失败: {e}")
        finally:
            ball_lock.release() #确保锁能够被正确释放
    else:
        buzzer1.beep(on_time=0.7, off_time=0.1, n=1, background=True)
        print("任务冲突,出球失败")

##舱内舵机 收回
def ball_in():
    if ball_lock.acquire(blocking=False):
        try:
            if fanin_bypass == True:
                pwm_fanin.ChangeDutyCycle(100*SPEED_FANIN)
                if fanout_bypass == True:
                    pwm_fanout.ChangeDutyCycle(0)
            executor.submit(arm_ball)
            roller_forward()
        except Exception as e:
            print(f"进球失败: {e}")
        finally:
            ball_lock.release() #确保锁能够被正确释放
    else:
        buzzer1.beep(on_time=0.7, off_time=0.1, n=1, background=True)
        print("任务冲突，收球失败")

    

##b 收球动作 停止
def ball_off():
    if ball_lock.acquire(blocking=5):
        try:
            if fanin_bypass == True:
                pwm_fanin.ChangeDutyCycle(0)
            if fanout_bypass == True:
                pwm_fanout.ChangeDutyCycle(0)
            roller_off()
            executor.submit(arm_down)
        except Exception as e:
            print(f"停止收球失败: {e}")
        finally:
            ball_lock.release() #确保锁能够被正确释放
    else:
        buzzer1.beep(on_time=1, off_time=0.1, n=1, background=True)
        print("任务冲突，无法停止，请尝试重启程序")

def ball_rejection():
    if ball_lock.acquire(blocking=False):
        try:
            if fanout_bypass == True:
                pwm_fanout.ChangeDutyCycle(0)
            executor.submit(arm_down)
        except Exception as e:
            print(f"拒绝收球失败: {e}")
        finally:
            ball_lock.release() #确保锁能够被正确释放
    else:
        buzzer1.beep(on_time=0.7, off_time=0.1, n=1, background=True)
        print("任务冲突，收球失败")

# I2C 总线  
I2C_BUS = 1  
class SensorDataThread(threading.Thread):  
    """传感器数据读取线程"""  

    def __init__(self, device_address, *args, **kwargs):  
        super().__init__(*args, **kwargs)  
        self.device_address = device_address  
        self.bus = smbus2.SMBus(I2C_BUS)  
        self._stop_event = threading.Event()
        self.gyro_z = 0.0  
        self.yaw = 0.0  
        self.lock = threading.Lock()

    def run(self):  
        """线程的执行体"""  
        print("传感器数据读取线程已启动")  

        try:  
            while not self._stop_event.is_set():  
                with self.lock:  
                    self.gyro_z = self.read_calibrated_gyro_z()  
                    self.yaw = self.read_yaw()  
                # 打印数据  
#                 print("-" * 30)  
#                 print(f"Z 轴校准角速度: {self.gyro_z:.3f} °/s")  
#                 print(f"偏航角: {self.yaw:.3f} °")  
                time.sleep(0.2) 
        except Exception as e:  
            print(f"传感器数据读取线程发生错误: {e}")  
        finally:  
            print("传感器数据读取线程已停止")  

    def read_word(self, reg_addr):  
        """  
        从一个寄存器读取完整的 16 位有符号数据  
        """  
        raw_data = self.bus.read_word_data(self.device_address, reg_addr)  

        # 将读取的数据转换为有符号 short 类型  
        data = struct.unpack('<h', struct.pack('<H', raw_data))[0]  
        return data  

    def read_calibrated_gyro_z(self):  
        """  
        读取校准后的 Z 轴角速度  
        """  
        REG_GYRO_Z_CALIBRATED = 0x39  # 校准 Z 轴角速度寄存器  

        # 从寄存器读取 16 位有符号数据  
        raw_gyro_z = self.read_word(REG_GYRO_Z_CALIBRATED)  

        # 转换为角速度 (单位：°/s)  
        gyro_z_dps = raw_gyro_z / 32768.0 * 2000.0  
        return gyro_z_dps  

    def read_yaw(self):  
        """  
        读取偏航角  
        """  
        REG_YAW = 0x3F  # 偏航角寄存器  

        # 从寄存器读取 16 位有符号数据  
        raw_yaw = self.read_word(REG_YAW)  

        # 转换为角度 (单位：°)  
        yaw_angle = raw_yaw / 32768.0 * 180.0  
        return yaw_angle  

    def stop(self):  
        """停止线程"""  
        self._stop_event.set()  

def detect_i2c_device():  
    """自动检测 I²C 设备地址"""  
    try:  
        output = subprocess.check_output(["i2cdetect", "-y", str(I2C_BUS)], text=True)  
        for line in output.split("\n"):  
            if line.strip() and line[0].isdigit():  
                for cell in line.split()[1:]:  
                    if cell != "--":  
                        return int(cell, 16)  
    except Exception as e:  
        print(f"检测 I²C 设备地址失败: {e}")  
    return None
def auto_in():
    if end_time-start_time<=4.0:
        servo3.value=-1
    elif 4.0<end_time-start_time<=4.5:
        servo3.value=-0.6
    else:
        servo3.value=0
# # 配置选项
# CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "calibration.json")  # 存储在程序目录下

# # MPU6050 寄存器地址
# MPU6050_ADDR = 0x68
# ACCEL_XOUT_H = 0x3B
# GYRO_XOUT_H = 0x43
# PWR_MGMT_1 = 0x6B

# # 初始化 I2C 总线
# bus = smbus2.SMBus(1)

# def init_mpu6050():
#     bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

# # 读取 MPU6050 原始数据
# def read_raw_data(addr):
#     high = bus.read_byte_data(MPU6050_ADDR, addr)
#     low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
#     value = (high << 8) | low
#     if value > 32768:
#         value -= 65536
#     return value

# # 去掉最大值和最小值后计算平均值
# def calculate_average(data):
#     if len(data) > 2:
#         data.sort()
#         data = data[1:-1]  # 去掉最大值和最小值
#     return sum(data) / len(data) if data else 0

# # 收集多组数据进行校正
# def collect_data(samples=100):
#     accel_data = {"x": [], "y": [], "z": []}
#     gyro_data = {"x": [], "y": [], "z": []}
#     for _ in range(samples):
#         accel_data["x"].append(read_raw_data(ACCEL_XOUT_H))
#         accel_data["y"].append(read_raw_data(ACCEL_XOUT_H + 2))
#         accel_data["z"].append(read_raw_data(ACCEL_XOUT_H + 4))
#         gyro_data["x"].append(read_raw_data(GYRO_XOUT_H))
#         gyro_data["y"].append(read_raw_data(GYRO_XOUT_H + 2))
#         gyro_data["z"].append(read_raw_data(GYRO_XOUT_H + 4))
#         time.sleep(0.01)  # 采样间隔

#     accel_offsets = {axis: calculate_average(values) for axis, values in accel_data.items()}
#     gyro_offsets = {axis: calculate_average(values) for axis, values in gyro_data.items()}
#     return accel_offsets, gyro_offsets

# # 校正加速度计数据
# def calibrate_accelerometer(accel_offset):
#     accel_offset["z"] -= 16384  # 调整 Z 轴以反映重力 1g
#     return accel_offset

# # 校正数据
# def calibrate():
#     print("正在校准中...")
#     accel_offset, gyro_offset = collect_data()
#     accel_offset = calibrate_accelerometer(accel_offset)
#     print("校准已完成.")
#     return {"accel_offset": accel_offset, "gyro_offset": gyro_offset}

# # 保存校正值到文件
# def save_calibration(calibration):
#     with open(CALIBRATION_FILE, "w") as f:
#         json.dump(calibration, f)
#     print(f"校准文件已经保存到 {CALIBRATION_FILE}")

# # 加载校正值
# def load_calibration():
#     if os.path.exists(CALIBRATION_FILE):
#         with open(CALIBRATION_FILE, "r") as f:
#             calibration = json.load(f)
#         print(f"成功加载来自 {CALIBRATION_FILE} 的校准描述文件")
#         return calibration
#     else:
#         print(f"校准描述文件 {CALIBRATION_FILE} 不存在.")
#         return None
# lock = threading.Lock()
# def update_position():  
#     global x_pos, y_pos, theta,vx,vy

#     # 读取IMU数据  
#     acc_x = read_raw_data(ACCEL_XOUT_H) - calibration["accel_offset"]["x"]
#     acc_y = read_raw_data(ACCEL_XOUT_H + 2) - calibration["accel_offset"]["y"]
#     acc_z = read_raw_data(ACCEL_XOUT_H + 4) - calibration["accel_offset"]["z"]

#     gyro_x = read_raw_data(GYRO_XOUT_H) - calibration["gyro_offset"]["x"]
#     gyro_y = read_raw_data(GYRO_XOUT_H + 2) - calibration["gyro_offset"]["y"]
#     gyro_z = read_raw_data(GYRO_XOUT_H + 4) - calibration["gyro_offset"]["z"]


#     gyro_z=gyro_z
#     if -20<gyro_z<20:
#         gyro_z=0
#     if -50<acc_x<100:
#         acc_x=0

#         # 打印数据
#     if -50<acc_y<100:
#         acc_y=0
#     #print(f"Accelerometer: X={acc_x}, Y={acc_y}, Z={acc_z},T={gyro_z}")

#     acc_x=acc_x*9.8/16384
#     acc_y=acc_y*9.8/16384
#     # 计算速度和位置  
#     vx = vx + acc_x * dt  
#     vy = vy + acc_y * dt  
#     x_pos = x_pos + vx * dt  
#     y_pos = y_pos + vy * dt  
#     if vx<0.01:
#         vx=0
#     if vy<0.01:
#         vy=0
#     #print(f"celerometer: X={vx}, Y={vy}")
#     # 计算角度  
#     theta = theta + gyro_z * dt  

#     # 限制位置在场地范围内  
#     x_pos = max(0, min(x_pos, FIELD_WIDTH))  
#     y_pos = max(0, min(y_pos, FIELD_LENGTH))  

#     # 限制角度在0-360度之间  
#     theta = theta % 360
#     with lock:

#         return x_pos, y_pos, theta
# def imu_thread():
#     """专门处理IMU数据的线程"""
#     while True:
#         # 更新位置和角度
#         update_position()
#         time.sleep(0.01)  # 控制IMU更新频率
    


##尾部舵机 升起
#def servo4_up():
    #set_servo_angle(servo4, ANGEL_UP_SERVO4)

##尾部舵机 降下
#def servo4_down():
    #set_servo_angle(servo4, ANGEL_DOWN_SERVO4)
# def camera_ready():
#     # 准备棋盘格的尺寸
#     checkerboard_size = (9, 6)  # 内角点的数量，假定有9x6个交点
# 
#     # 准备对象点（假设棋盘格的每个格子为1单位）
#     object_points = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
#     object_points[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
# 
#     # 存储所有图像的对象点和图像点
#     obj_points = []
#     img_points = []
# 
# #     # 读取所有棋盘格图像
#     images = glob.glob('/home/smartcar/图片/test_picture01/*.jpg')
#     for img in images:
#         image = cv2.imread(img)
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# 
#          # 找到棋盘格角点
#         ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
# 
#         if ret:
#             obj_points.append(object_points)  # 添加对象点
#             img_points.append(corners)         # 添加图像点
#             cv2.drawChessboardCorners(image, checkerboard_size, corners, ret)
# 
#      # 标定相机
#     ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
# 
# #     # 打印内参矩阵  
#     print("Camera Matrix:\n", camera_matrix)  
#     print("Distortion Coefficients:\n", dist_coeffs)  
# 
#      # 保存内参以供后续使用  
#     np.savez('camera_calibration.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
# 
# camera_ready()
def find_largest_blue_region(frame):  
    # 将图像转换为HSV色彩空间  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  

    # 定义的HSV范围  
    lower_blue = np.array([100, 150, 100])  
    upper_blue = np.array([140, 255, 255])  
 

    # 使用掩膜提取区域  
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  
    kernel = np.ones((5, 5), np.uint8)  
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  
    # 寻找轮廓

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  

    # 找到面积最大的且面积大于200的蓝色区域  
    largest_area = 16000
    largest_contour = None  
    for cnt in contours:  
        area = cv2.contourArea(cnt)  
        if area >= 16000 and area > largest_area:  
            largest_area = area  
            largest_contour = cnt  
            
    # 如果找到了符合条件的蓝色区域,计算其质心坐标并返回  
    if largest_contour is not None:  
        M = cv2.moments(largest_contour)  
        if M["m00"] != 0:  
            cx = int(M["m10"] / M["m00"])  
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            print(largest_area)
            return cx, cy  
            
    return None, None
def find_largest_red_region(frame):
    # 将图像转换为HSV色彩空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 定义红色的HSV范围
    lower_RED = np.array([0, 100, 100])
    upper_RED = np.array([10, 255, 255])

    mask = cv2.inRange(hsv, lower_RED, upper_RED)  
    kernel = np.ones((5, 5), np.uint8)  
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  
    # 寻找轮廓

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  

    # 找到面积最大的且面积大于16000的蓝色区域  
    largest_area = 16000
    largest_contour = None  
    for cnt in contours:  
        area = cv2.contourArea(cnt)  
        if area >= 16000 and area > largest_area:  
            largest_area = area  
            largest_contour = cnt  
            
    # 如果找到了符合条件的蓝色区域,计算其质心坐标并返回  
    if largest_contour is not None:  
        M = cv2.moments(largest_contour)  
        if M["m00"] != 0:  
            cx = int(M["m10"] / M["m00"])  
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            print(largest_area)
            return cx, cy  
    return None, None
def leave():
    robot.stop()
    executor.submit(arm_up)
    #ball_out()
    time.sleep(2.0)
    ball_off()
    executor.submit(arm_down)


def move_camera(target_angle,distance,y,height):
    diff =y- height//2
    acc_angle=6-(distance*100)/30.0
    if diff < 0 :
        target_angle=target_angle-acc_angle
    if  diff>0:
        target_angle=target_angle+acc_angle
    return target_angle
# def move_camera(target_angle, distance, y, height):  
#     # 根据实际测试数据拟合角度变化与距离的关系  
#     a, b = 0.5, 3.0  # 根据测试结果确定  
#     acc_angle = a * distance*100 + b  
# 
#     # 限制角度变化范围  
#     acc_angle = max(-5, min(5, acc_angle))  
# 
#     # 使用线性插值平滑角度变化  
#     diff = y - height // 2  
#     if diff < 0:  
#         target_angle = max(target_angle - acc_angle, target_angle - 2)  
#     elif diff > 0:  
#         target_angle = min(target_angle + acc_angle, target_angle + 2)  
# 
#     return target_angle
def move_camera1(target_angle,y,height):
    diff =y- height//2
    if diff<0 :
        target_angle=target_angle-2
    if  diff>0:
        target_angle=target_angle+2
    return target_angle

def move_robot(x,width):

    if x>width//2:
        robot.value=(0.14,-0.14)

        print("left")

    elif x<width//2:
        robot.value=(-0.14,0.14)

        print("right")

        
    


def nothing(a):
    pass


def Trackbar_Init():
    # 1 create windows
    cv2.namedWindow('h_binary')
    cv2.namedWindow('s_binary')
    cv2.namedWindow('v_binary')
    # 2 Create Trackbar
    cv2.createTrackbar('hmin', 'h_binary', 4, 179, nothing)
    cv2.createTrackbar('hmax', 'h_binary', 32, 179, nothing)
    cv2.createTrackbar('smin', 's_binary', 180, 255, nothing)
    cv2.createTrackbar('smax', 's_binary', 255, 255, nothing)
    cv2.createTrackbar('vmin', 'v_binary', 156, 255, nothing)
    cv2.createTrackbar('vmax', 'v_binary', 255, 255, nothing)
    #   创建滑动条     滑动条值名称 窗口名称   滑动条值 滑动条阈值 回调函数


# 在HSV色彩空间下得到二值图
def Get_HSV(image):
    hmin = cv2.getTrackbarPos('hmin', 'h_binary')
    hmax = cv2.getTrackbarPos('hmax', 'h_binary')
    smin = cv2.getTrackbarPos('smin', 's_binary')
    smax = cv2.getTrackbarPos('smax', 's_binary')
    vmin = cv2.getTrackbarPos('vmin', 'v_binary')
    vmax = cv2.getTrackbarPos('vmax', 'v_binary')
    
    # 2 to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv', hsv)
    h, s, v = cv2.split(hsv)
    
    # 3 set threshold (binary image)
    # if value in (min, max):white; otherwise:black
    h_binary = cv2.inRange(np.array(h), np.array(hmin), np.array(hmax))
    s_binary = cv2.inRange(np.array(s), np.array(smin), np.array(smax))
    v_binary = cv2.inRange(np.array(v), np.array(vmin), np.array(vmax))
    
    # 4 get binary（对H、S、V三个通道分别与操作）
    binary = cv2.bitwise_and(h_binary, cv2.bitwise_and(s_binary, v_binary))
    
    # 5 Show
    # cv2.imshow('h_binary', h_binary)
    # cv2.imshow('s_binary', s_binary)
    # cv2.imshow('v_binary', v_binary)
    # cv2.imshow('binary', binary)
    
    return binary
def GET_HSV_BLUE(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv', hsv)

    # 定义红色的阈值范围
    lower_blue = np.array([100, 150, 100])
    upper_blue = np.array([140, 255, 255])

    # 创建掩膜
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    return mask
def GET_HSV_OTHER_B(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义黑色/蓝色/黄色的阈值范围
    lower_black = np.array([0, 0, 20])
    upper_black = np.array([150, 255,50])
    lower_red_1 = np.array([0, 150, 150])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([160, 150, 150])
    upper_red_2 = np.array([180, 255, 255])
    lower_yellow = np.array([20, 150, 150])
    upper_yellow = np.array([35, 255, 255])

    # 创建掩膜
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask_red = cv2.bitwise_or(mask1, mask2)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.bitwise_or(mask_black, mask_red)
    mask = cv2.bitwise_or(mask, mask_yellow)

    return mask
def GET_HSV_RED(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv', hsv)
    
    # 定义红色的阈值范围
    lower_red_1 = np.array([0, 150, 150])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([160, 150, 150])
    upper_red_2 = np.array([180, 255, 255])

    # 创建掩膜
    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.bitwise_or(mask1, mask2)
    return mask
def GET_HSV_OTHER(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义黑色/蓝色/黄色的阈值范围
    lower_black = np.array([0, 0, 20])
    upper_black = np.array([150, 255,50])
    lower_blue = np.array([100, 150, 100])
    upper_blue = np.array([140, 255, 255])
    lower_yellow = np.array([20, 150, 150])
    upper_yellow = np.array([35, 255, 255])

    # 创建掩膜
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.bitwise_or(mask_black, mask_blue)
    mask = cv2.bitwise_or(mask, mask_yellow)

    return mask


# 图像处理
def Image_Processing1(frame):

    ball_RED = []
    ball_OTHER = []
    
    global h, s, v
    image = frame
    cv2.imshow('frame', frame)
    
    # 2 get HSV
    binary_RED = GET_HSV_RED(frame)
    binary_OTHER = GET_HSV_OTHER(frame)


    
    # 3 Gausi blur
    blur = cv2.GaussianBlur(binary_RED,(5,5),1)
    blur_OTHER = cv2.GaussianBlur(binary_OTHER, (5, 5), 1)

    # 4 Open
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
    Open_RED = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel)
    Open_OTHER = cv2.morphologyEx(blur_OTHER, cv2.MORPH_OPEN, kernel)

    # cv2.imshow('Open',Open)
    # 5 Close
    Close_RED = cv2.morphologyEx(Open_RED, cv2.MORPH_CLOSE, kernel)
    Close_OTHER = cv2.morphologyEx(Open_OTHER, cv2.MORPH_CLOSE, kernel)

    # cv2.imshow('Close',Close)
    Close_RED= cv2.Canny(Close_RED,100,150)
    Close_OTHER = cv2.Canny(Close_OTHER, 100, 150)
    # 6 Hough Circle detect
                        #   param2:决定圆能否被检测到（越少越容易检测到圆，但相应的也更容易出错）
    circles_RED = cv2.HoughCircles(Close_RED,cv2.HOUGH_GRADIENT,2,100,param1=150,param2=50,minRadius=5,maxRadius=180)
    # judge if circles is exist
    circles_OTHER = cv2.HoughCircles(Close_OTHER, cv2.HOUGH_GRADIENT, 2, 100, param1=150, param2=80, minRadius=5, maxRadius=180)

    if circles_RED is not None:
        for i in circles_RED[0, :]:
        # 1 获取圆的圆心和半径
            x, y, r = int(i[0]),int(i[1]),int(i[2])
            # print(x, y, r)
            pixel_width = 2 * r # 小球的像素直径
            if pixel_width > 0:
                focal_length = camera_matrix[0, 0]
                distance = (known_diameter * focal_length) / pixel_width*2  
            # 2 画圆
                ball_RED.append((x,y,distance))


                cv2.circle(image, (x, y), r, (255,0,255),5)
                cv2.putText(frame, f'Distance: {distance:.2f} m', (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            else:
                (x,y),r = (0,0), 0
                

        ball_RED.sort(key=lambda x:x[2])
    if circles_OTHER is not None:
        for i in circles_OTHER[0, :]:
            x, y, r = int(i[0]), int(i[1]), int(i[2])
            pixel_width = 2 * r
            if pixel_width > 0:
                focal_length = camera_matrix[0, 0]
                distance_OTHER = (known_diameter * focal_length) / pixel_width * 2
            ball_OTHER.append((x, y, distance_OTHER))
            cv2.circle(image, (x, y), r, (0, 255, 0), 5)
            cv2.putText(frame, f'Distance: {distance_OTHER:.2f} m', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        ball_OTHER.sort(key=lambda x:x[2])
    cv2.imshow('image1', image)
    if ball_RED and ball_OTHER:
        return (ball_RED[0],ball_OTHER[0])
    elif ball_RED:
        return (ball_RED[0],0)
    elif ball_OTHER:
        return (0,ball_OTHER[0])
    else:
        return (0,0)

def Image_Processing2(frame):
    ball_BLUE = []
    ball_OTHER = []
    
    global h, s, v
    image = frame
    cv2.imshow('frame', frame)
    
    # 2 get HSV
    binary_BLUE = GET_HSV_BLUE(frame)
    binary_OTHER = GET_HSV_OTHER_B(frame)

    
    
    # 3 Gausi blur
    blur = cv2.GaussianBlur(binary_BLUE,(5,5),1)
    blur_OTHER = cv2.GaussianBlur(binary_OTHER, (5, 5), 1)

    # 4 Open
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
    Open_BLUE = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel)
    Open_OTHER = cv2.morphologyEx(blur_OTHER, cv2.MORPH_OPEN, kernel)

    # cv2.imshow('Open',Open)
    # 5 Close
    Close_BLUE = cv2.morphologyEx(Open_BLUE, cv2.MORPH_CLOSE, kernel)
    Close_OTHER = cv2.morphologyEx(Open_OTHER, cv2.MORPH_CLOSE, kernel)

    # cv2.imshow('Close',Close)
    Close_BLUE= cv2.Canny(Close_BLUE,100,150)  
    Close_OTHER = cv2.Canny(Close_OTHER, 100, 150)
    # 6 Hough Circle detect
                        #   param2:决定圆能否被检测到（越少越容易检测到圆，但相应的也更容易出错）
    circles_BLUE = cv2.HoughCircles(Close_BLUE,cv2.HOUGH_GRADIENT,2,100,param1=150,param2=50,minRadius=5,maxRadius=180)
    # judge if circles is exist
    circles_OTHER = cv2.HoughCircles(Close_OTHER, cv2.HOUGH_GRADIENT, 2, 100, param1=150, param2=80, minRadius=5, maxRadius=180)

    if circles_BLUE is not None:
        for i in circles_BLUE[0, :]:
        # 1 获取圆的圆心和半径
            x, y, r = int(i[0]),int(i[1]),int(i[2])
            #print(x, y, r)
            pixel_width = 2 * r # 小球的像素直径
            if pixel_width > 0:
                focal_length = camera_matrix[0, 0]
                distance = (known_diameter * focal_length) / pixel_width*2
            # 2 画圆
                ball_BLUE.append((x,y,distance))


                cv2.circle(image, (x, y), r, (255,0,255),5)
                cv2.putText(frame, f'Distance: {distance:.2f} m', (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            else:
                (x,y),r = (0,0), 0
                
        
        ball_BLUE.sort(key=lambda x:x[2])
    if circles_OTHER is not None:
        for i in circles_OTHER[0, :]:
            x, y, r = int(i[0]), int(i[1]), int(i[2])
            pixel_width = 2 * r
            if pixel_width > 0:
                focal_length = camera_matrix[0, 0]
                distance_OTHER = (known_diameter * focal_length) / pixel_width * 2
            ball_OTHER.append((x, y, distance_OTHER))
            cv2.circle(image, (x, y), r, (0, 255, 0), 5)
            cv2.putText(frame, f'Distance: {distance_OTHER:.2f} m', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        ball_OTHER.sort(key=lambda x:x[2])
    cv2.imshow('image1', image)
    if ball_BLUE and ball_OTHER:
        return (ball_BLUE[0],ball_OTHER[0])
    elif ball_BLUE:
        return (ball_BLUE[0],0)
    elif ball_OTHER:
        return (0,ball_OTHER[0])
    else:
        return (0,0)
print("定义函数 已就绪")


#舵机初始化
print("请检查大滚轮是否故障")
def rollercheck():
    if ball_lock.acquire(blocking=False):
        try:
            roller_off()
            roller_forward()
            time.sleep(0.5)
            roller_off()
            time.sleep(0.1)
            roller_backward()
            time.sleep(0.25)
            roller_off()
            print("大滚轮检查结束")
        finally:
            ball_lock.release() #确保锁能够被正确释放
    else:
        buzzer1.beep(on_time=0.7, off_time=0.1, n=1, background=True)
        print("大滚轮检查失败")



arm_down()
time.sleep(0.5)
arm_up()
#servo4_down()
print("舵机 已就绪")

#挡板复位程序
def board_return():
    if fanout_bypass == False:
        if board_lock.acquire(blocking=False):
            try:
                servo3.value=(-SPEED_FIRST_SERVO3)
                time.sleep(TIME_FIRST_SERVO3)
                servo3.value=(0)
            except Exception as e:
                print(f"挡板复位失败: {e}")
            finally:
                board_lock.release()
        else:
            buzzer1.beep(on_time=0.3, off_time=0.2, n=2, background=True)
            print("注意挡板冲突")
    else:
        pass
#if body_check == False:
    #board_return_thread.start()
#if body_check == True:
    #if board_lock.acquire(blocking=False):
        #try:
            #servo3.value=(SPEED_FIRST_SERVO3)
            #time.sleep(TIME_FIRST_SERVO3)
            ##button2.wait_for_press()
            #servo3.value=(0)   
        #finally: 
            #board_lock.release()
            #exit()
    #else:
        #buzzer1.beep(on_time=0.3, off_time=0.2, n=2, background=True)
        #print("Fail to make the board out!")
        #exit()
    


    
#设定比赛方，为正式比赛做好准备
executor.submit(update_side_leds)
led1.on()
print("初始化完成，准备就绪，按下按键开始比赛")
rb_pressed_time = None #标记按压时间
start_pressed_time = None
while True:   

    # 检查调试模式是否为 True，如果是则跳过按键检测直接开始主程序
    if test_mode:
        print("调试模式生效，直接开始")
        break

    # 检测红蓝方切换按键是否按下
    if button1.is_pressed:
        if side == False:
            side = True
            executor.submit(update_side_leds)
            print("当前选择 蓝方")
        time.sleep(0.25)

    if button1.is_pressed == False:
        if side == True:
            side = False
            executor.submit(update_side_leds)
            print("当前选择 红方")

        time.sleep(0.25)

    if button2.is_pressed:
        print("发车！")
        break #发车

    # 使用手柄直接绕过自动模式
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN and event.button == START:
            start_pressed_time = time.time()  # 记录按下时间
        elif event.type == pygame.JOYBUTTONUP and event.button == START:
            start_pressed_time = None  # 松开时重置时间

    # 检查 start 按钮长按超过定义的长按时间秒
    if start_pressed_time and (time.time() - start_pressed_time >= 1):
        auto_completed = True
        print("手柄已激活，自动模式已绕过")
        start_pressed_time = None  # 重置按下时间，防止重复设置
        break

#调试状态下请复制以上所有代码

#——————————————————————————————————————————————————————————#

#欢迎来到主程序

#开启子线程，指示灯闪烁，蜂鸣器响声，开始比赛。
##通过观察指示灯是否闪烁可以快速得知是否已经死机
FIELD_WIDTH = 2.4  # 单位:米  
FIELD_LENGTH = 2.4  # 单位:米  

# 小车初始位置  
INITIAL_X = FIELD_WIDTH / 2  # 初始X坐标为场地中点  
INITIAL_Y = 0  # 初始Y坐标为场地边缘  
# 定义状态变量
# dt=0.01
# vx=0
# vy=0
# x_pos = INITIAL_X  
# y_pos = INITIAL_Y  
# theta = 0  # 角度,初始为0度

if test_mode == False:
    test_part = False

if test_part == False:
    if auto_completed == False :##检查是否完成了自动部分
        buzzer1.beep(on_time = AUTO_BEEP_DELAY, off_time = AUTO_BEEP_DELAY, n = AUTO_BEEP_TIMES)
    # 创建陀螺仪数据读取线程
        device_address = detect_i2c_device() or 0x50  # 替换为您的设备地址  

    # 创建并启动传感器数据读取线程  
        sensor_data_thread = SensorDataThread(device_address, name="Sensor Data Thread")  
        sensor_data_thread.start()

        
        
        known_diameter=0.04
        calibration_data = np.load('/home/smartcar/下载/camera_calibration.npz')
        camera_matrix = calibration_data['camera_matrix']
        dist_coeffs = calibration_data['dist_coeffs']
        time.sleep(0.1)
        set_servo_angle(10)
        time.sleep(0.1)
        arm_ball()
        target_angle=10
        side=True
        picam2 = Picamera2()
# 配置摄像头参数
        cap=picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})

        picam2.configure(cap)
# 启动摄像头


        picam2.start()
        
#         Trackbar_Init()
        picam2.set_controls({
            "AeEnable": True,   # 自动曝光
            "AwbEnable": True,   # 自动白平衡
            "AeMeteringMode": 2,  # 对应中心加权测光模式
            "NoiseReductionMode": 1, # 噪声消除
            "AeExposureMode": 1 ,#短曝光模式，减少运动模糊
#             "ColourGains": (1.2, 1.5)  # 手动设置白平衡增益 (红/蓝通道)
            })


        time.sleep(0.5)
        start_time=time.time()
        while True:

            with sensor_data_thread.lock:  
                gyro_z = sensor_data_thread.gyro_z  
                yaw = sensor_data_thread.yaw  
            print(f"偏航角: {yaw:.3f} °")
            time.sleep(0.01)


            frame = picam2.capture_array()
            if frame is None:
                print("无法读取视频流")
                break
            cv2.imshow("frame",frame)
            # 图像处理
            # Image Process
            height,width=frame.shape[:2]
            if side==False:
                ball_RED,ball_OTHER= Image_Processing1(frame)

            elif side==True:
                ball_RED,ball_OTHER= Image_Processing2(frame)
                
#            if ball_RED==0 :
            
                
            if ball_RED!=0:
                x_RED,y_RED,distance_RED=ball_RED
                
                if (height//2>y_RED or height//2<y_RED):
                    target_angle=move_camera(target_angle,distance_RED,y_RED,height)
                    target_angle=target_angle
                if 70<=target_angle:

                    target_angle=70
                if target_angle<=10:

                    target_angle=10
                set_servo_angle(target_angle)
                print(target_angle)
                if (width//2-150>x_RED or x_RED>width//2+150):
                    move_robot(x_RED,width)
                        #time.sleep(0.1)
                    continue

            if ball_OTHER!=0:
                
                x_OTHER,y_OTHER,distance_OTHER=ball_OTHER
                

            if ball_RED!=0:
                v_tem=0.126+distance_RED/30.0
                print(v_tem,"!")
                if (target_angle>=62 and distance_RED<=0.1):
                    print(distance_RED)
                    print(v_tem)
                    robot.stop()
                    print("stop1")
                    time.sleep(0.1)
                    ball_in()
                    time.sleep(2.5)
                    ball_off()
                    break
                else:
                    print(distance_RED)
                    robot.value=(v_tem,v_tem)
                    print("stright")
            end_time=time.time()
            #executor.submit(auto_in)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                
                break
        
        cv2.destroyAllWindows()



        while yaw>-90:
            with sensor_data_thread.lock:  
                gyro_z = sensor_data_thread.gyro_z  
                yaw = sensor_data_thread.yaw  
            print(f"偏航角: {yaw:.3f} °")
            robot.value=(0.18,-0.18)
            time.sleep(0.1)
        robot.stop()
        target_angle=15
        set_servo_angle(15)
        time.sleep(0.5)
        while True:
            with sensor_data_thread.lock:  
                gyro_z = sensor_data_thread.gyro_z  
                yaw = sensor_data_thread.yaw  
            print(f"偏航角: {yaw:.3f} °")
            frame = picam2.capture_array()
            
            if frame is None:  
                print("无法读取视频流")  
                break
    
    # 找到最大的红色区域  
            if side==False:
                cx, cy = find_largest_red_region(frame)
            if side==True:
                cx, cy = find_largest_blue_region(frame)
            if -85<yaw and cx is None:
                robot.value=(0.18,-0.18)
                continue
                print("continue1")
            if  yaw<-95 and cx is None:
                robot.value=(-0.18,0.18)
                continue
                print("continue2")                
            if (cx is not None and cy is not None)and -95<yaw<-85:
                print("stright")
                robot.value=(0.15,0.15)

                
                if (height//2-20>cy or height//2+20<cy):
                    target_angle=move_camera1(target_angle,cy,height)  

                print("@",target_angle)
                if 60<=target_angle:
                    target_angle=60
                if target_angle<=10:
                    target_angle=10
                set_servo_angle(target_angle)
            if target_angle>20:
                break
                    
                    
        
        # 在这里添加控制小车的代码  

    
    # 显示图像  
            cv2.imshow("Frame", frame)  
    
    # 按 'q' 退出  
            if cv2.waitKey(1) & 0xFF == ord('q'):

                break
        picam2.close() 
        cv2.destroyAllWindows()
        sensor_data_thread.stop()  
        sensor_data_thread.join()
        robot.stop()
        auto_completed = True #顺利完成自动部分，标记一下，然后进入手动模式
    

    if auto_completed == True :#只有完成了自动部分才能进入手动部分#
        buzzer1.beep(on_time = MANUAL_BEEP_TIMES, off_time = MANUAL_BEEP_DELAY, n = MANUAL_BEEP_TIMES)

        #定义变量
        click_count = 0
        reverse_mode = False
        last_click_time = None
        last_click_time1 = None
        jump = False




       #看门狗变量
        last_signal_time = time.time()
        watchdog_lock = threading.Lock()

        def watchdog():
            global last_signal_time
            while True:
                time.sleep(1)  # 每秒检查一次
                with watchdog_lock:
                    if time.time() - last_signal_time > 15:
                        buzzer1.beep(on_time=10, off_time=0.1, n=1, background=True)
                        os.system("sudo reboot")

        # 看门狗手柄事件处理
        def handle_joystick_event(event):
            global last_signal_time
            if event.type in [pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP, pygame.JOYAXISMOTION]:
                with watchdog_lock:
                    last_signal_time = time.time()

        # 启动看门狗线程
        watchdog_thread = threading.Thread(target=watchdog, daemon=True)
        watchdog_thread.start()

        #定义速度更改函数
        def change_speed():
            # 获取摇杆和触发器的值
            left_y_axis = joystick.get_axis(LEFT_Y)  # 左摇杆 Y 轴
            right_x_axis = joystick.get_axis(LEFT_X)  # 左摇杆 X 轴
            left_trigger = joystick.get_axis(LT)  # LT 触发器
            right_trigger = joystick.get_axis(RT)  # RT 触发器
            left_trigger = (left_trigger + 1) / 2
            right_trigger = (right_trigger + 1) / 2


            # 基于 LT 触发器控制左轮
            if left_trigger > 1:
                left_trigger = 1


            # 基于 RT 触发器控制右轮
            if right_trigger > 1:
                right_trigger = 1 # 转换为速度


            # 左摇杆控制小车整体前后运动和旋转
            speed = left_y_axis  # 速度
            if speed > 1:
                speed = 1
            turn = right_x_axis  # 旋转
            if turn > 1:
                turn = 1

        
            left_wheel_speed = -(speed + turn)
            right_wheel_speed = -(speed - turn)
            a = left_wheel_speed + left_trigger / 2 - right_trigger / 2
            b = right_wheel_speed + right_trigger / 2 - left_trigger / 2
            if a > 1:
                a = 1
            elif a < -1:
                a = -1
            if b > 1:
                b = 1
            elif b < -1:
                b = -1


            # 控制左轮和右轮速度
            robot.value=(0.6 * a, 0.6 * b)

            # 这里可以使用 speed 和 turn 调整小车的行驶逻辑

        while True:
            for event in pygame.event.get():
                # 按键按下事件
                executor.submit(handle_joystick_event,event) 
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == A:  # A 键按下
                        executor.submit(ball_in)
                    if event.button == Y:  # Y 键按下
                        executor.submit(ball_out)
                    if event.button == B:
                        executor.submit(ball_rejection)
                    if event.button == START:
                        # 记录 Start 键按下的时间
                        start_pressed_time = time.time()
                        # 检查是否为连续点击
                        if last_click_time is None or (time.time() - last_click_time) > 1:
                            click_count = 1  # 重置计数器
                        else:
                            click_count += 1
                        last_click_time = time.time()
                    if event.button == CENTRAL:
                        # 记录 Start 键按下的时间
                        start_pressed_time1 = time.time()
                        # 检查是否为连续点击
                        if last_click_time1 is None or (time.time() - last_click_time1) > 1:
                            click_count1 = 1  # 重置计数器
                        else:
                            click_count1 += 1
                        last_click_time1 = time.time()
                        

                # 按键松开事件
                if event.type == pygame.JOYBUTTONUP:
                    if event.button == A:  # A 键松开
                        executor.submit(ball_off)
                    if event.button == Y:  # Y 键松开
                        executor.submit(ball_off)
                    if event.button == X:  # X 键按下，切换前进/后退模式
                        reverse_mode = False
                    if event.button == START:
                        # 检查是否长按超过 3 秒
                        if start_pressed_time and (time.time() - start_pressed_time) >= 3:
                            # 重置按压时间
                            start_pressed_time = None
                            os.execv(sys.executable, ['python3'] + sys.argv)
                    if event.button == CENTRAL:
                        # 检查是否长按超过 2 秒
                        if start_pressed_time1 and (time.time() - start_pressed_time1) >= 2:
                            # 重置按压时间
                            start_pressed_time1 = None
                            jump = True
                            break
                    if jump == True:
                        break
                if jump == True:
                    break
                if click_count >= 5:
                    click_count = 0  # 重置计数 
                    os.system('sudo reboot')
                executor.submit(change_speed)

            time.sleep(0.1)  # 避免过高的 CPU 占用
            if jump == True:
                break
    
            #捕捉手柄信号，同时蜂鸣器响，只有连接到手柄之后才能停止响声，继续比赛
            #塞入手柄控制相关部分代码
            #如果有需要，可以直接在
    print("比赛顺利完成，程序正常退出") #比赛顺利完成.
    try:
        robot.stop()
    except:
        pass
    try:
        pygame.quit()
    except:
        pass
    try:
        GPIO.cleanup()
    except:
        pass
    exit()
elif test_part == True:
    print("调试模式开启")
#     servo3.value=-1
#     time.sleep(4.0)
#     servo3.value=-1
#     time.sleep(0.5)
    

#     servo3.value=0.2

# recover
#     servo3.value=1
#     time.sleep(2.5)
#     servo3.value=1
#     time.sleep(0.5)
#     servo3.value=0

    robot.value=(0.2,0.2)
    time.sleep(1.5)
    robot.stop()
#     arm_down()
#     time.sleep(1.5)
#     arm_ball()
#     roller_forward()
#     time.sleep(1.5)
#     roller_off()
#     arm_up()
#     time.sleep(1.5)


