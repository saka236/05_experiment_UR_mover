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
#事前設定項目


hand_tcp_distance = 266 #ハンドの先端とTCPのY座標の差(ハンドの長さ)
handspeed = 50
handcurrent = 100
G_distance = 200
floor_distance = 0


# dynamixel初期設定
dxl = myDynamixel.Dxlfunc()  # インスタンス化
MotorNum = dxl.init('COM3', baudrate=4000000)  # COM通信容量を指定
print(MotorNum)


if MotorNum > 0:
    print("dynamixel初期化成功")
else:
    print("初期化失敗")
Motor_ID = 1  # モーターIDを設定

ur.start_posture = np.array([-180, 0, 0])  # 把持前の基本位置
ur.standard_position = np.array([-145.0, -450.0, hand_tcp_distance + G_distance])
P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect = np.hstack([P_detect_position, ur.start_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=5)

dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)
dxl.write(Motor_ID, dxl.Address.TorqueEnable, True)  # モーターのトルクをオンにする

dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.velocity_control)  # モーターを速度コントロール

dxl.write(Motor_ID, dxl.Address.GoalVelocity, 100)  # 外側のハンドをハンドを開く初期化)


while True:
    current = dxl.read(Motor_ID, dxl.Address.PresentCurrent)  # トルク読み取り
    if current > 350:
        print("外爪がひらきました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break

    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break

inner_finger_dis = 1800
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl - inner_finger_dis,100)
t_p_start = time.time()
while True:
    now_velocity = dxl.read(Motor_ID, dxl.Address.PresentVelocity)
    program_time = time.time() - t_p_start

    if keyboard.is_pressed("q"):  # qが押されたら終了
        break

    if program_time >= 0.5 and now_velocity == 0:
        now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
        dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.position_control)
        dxl.write(Motor_ID, dxl.Address.GoalPosition, now_pos_dxl)
        break

ur.start_posture = np.array([-180, 0, 0])  # 把持前の基本位置
ur.standard_position = np.array([-145.0, -450.0, hand_tcp_distance + floor_distance])
P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect = np.hstack([P_detect_position, ur.start_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=3)

inhand_open_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)

dxl.CurrentCnt_Vbase(Motor_ID,handcurrent,handspeed)#ハンドを閉じる
time.sleep(3)

ur.start_posture = np.array([-180, 0, 0])  # 把持前の基本位置
ur.standard_position = np.array([-145.0, -450.0, hand_tcp_distance + G_distance])
P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect = np.hstack([P_detect_position, ur.start_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=5)

time.sleep(2)

ur.start_posture = np.array([-180, 0, 0])  # 把持前の基本位置
ur.standard_position = np.array([-145.0, -450.0, hand_tcp_distance + floor_distance])
P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect = np.hstack([P_detect_position, ur.start_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=5)

dxl.PosCnt_Vbase(Motor_ID,inhand_open_pos_dxl,handspeed)
time.sleep(2)

ur.start_posture = np.array([-180, 0, 0])  # 把持前の基本位置
ur.standard_position = np.array([-145.0, -450.0, hand_tcp_distance + G_distance])
P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect = np.hstack([P_detect_position, ur.start_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=5)

dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)




ur.exit()
sys.exit()