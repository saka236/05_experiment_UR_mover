import math
import os
import sys
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


hand_tcp_distance = 175 #ハンドの先端とTCPのY座標の差(ハンドの長さ)
handspeed = 200
handcurrent = 200
G_distance = 250
floor_distance = 5
bag_hight = 50

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
#アームが初期位置に移動
dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)
dxl.write(Motor_ID, dxl.Address.TorqueEnable, True)  # モーターのトルクをオンにする

dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.velocity_control)  # モーターを速度コントロール

dxl.write(Motor_ID, dxl.Address.GoalVelocity, -handspeed)  # ハンド閉じる

while True:
    current = dxl.read(Motor_ID, dxl.Address.PresentCurrent)  # トルク読み取り
    if current < -80:
        print("ハンドが閉じました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break

    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break

#袋の口にアプローチ
ur.start_posture = np.array([-180, 0, 0])
ur.standard_position = np.array([-145.0, -450.0, hand_tcp_distance + floor_distance + bag_hight])
P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect = np.hstack([P_detect_position, ur.start_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=3)

hand_close_pos = dxl.read(Motor_ID, dxl.Address.PresentPosition)

#ハンドを開く

dxl.CurrentCnt_Vbase(Motor_ID,handcurrent,handspeed)

time.sleep(2)

dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.velocity_control)  # モーターを速度コントロール

dxl.write(Motor_ID, dxl.Address.GoalVelocity, handspeed)  # ハンド閉じる

while True:
    now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)  # トルク読み取り
    if now_pos_dxl- hand_close_pos > 42800:
        print("ハンドが開きました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break

    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break


#物体にアプローチ
ur.start_posture = np.array([-180, 0, 0])  # 把持前の基本位置
ur.standard_position = np.array([-145.0, -450.0, hand_tcp_distance + floor_distance])
P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect = np.hstack([P_detect_position, ur.start_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=5)

#把持
dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.velocity_control)  # モーターを速度コントロール

dxl.write(Motor_ID, dxl.Address.GoalVelocity, -handspeed)  # ハンド閉じる

while True:
    current = dxl.read(Motor_ID, dxl.Address.PresentCurrent)  # トルク読み取り
    if current < -50:
        print("ハンドが閉じました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break

    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break


#持ち上げる
ur.start_posture = np.array([-180, 0, 0])  # 把持前の基本位置
ur.standard_position = np.array([-145.0, -450.0, hand_tcp_distance + floor_distance + G_distance])
P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect = np.hstack([P_detect_position, ur.start_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=5)


dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)




ur.exit()
sys.exit()