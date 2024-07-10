import math
import os
import sys
import threading
import time

import cv2
import keyboard
import numpy as np
from cv2 import aruco
import myDynamixel
from myNowTime import get_now

from sub_code.myUniversalRobot_v2 import myUniversalRobot

# URのインスタンスを作成
ur = myUniversalRobot()

# dynamixel初期設定
dxl = myDynamixel.Dxlfunc()  # インスタンス化
MotorNum = dxl.init('COM3', baudrate=4000000)  # COM通信容量を指定
print(MotorNum)

handspeed = 50
handcurrent = 40

if MotorNum > 0:
    print("dynamixel初期化成功")
else:
    print("初期化失敗")
Motor_ID = 1  # モーターIDを設定

dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)
dxl.write(Motor_ID, dxl.Address.TorqueEnable, True)  # モーターのトルクをオンにする

dxl.CurrentCnt_Vbase(Motor_ID,handcurrent,handspeed)