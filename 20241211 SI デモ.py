import math
import os
import sys
import threading
import time

import keyboard
import numpy as np
import myDynamixel
from myNowTime import get_now


#事前設定項目


handspeed = 80
handcurrent = 200
inner_finger_dis = 1600
outer_finger_dis = 6700


# dynamixel初期設定
dxl = myDynamixel.Dxlfunc()  # インスタンス化
MotorNum = dxl.init('COM4', baudrate=4000000)  # COM通信容量を指定
print(MotorNum)


if MotorNum > 0:
    print("dynamixel初期化成功")
else:
    print("初期化失敗")
Motor_ID = 1  # モーターIDを設定

#ダイナミクセル初期セットアップ
dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)
dxl.write(Motor_ID, dxl.Address.TorqueEnable, True)  # モーターのトルクをオンにする

#ハンドを内爪開 外爪閉に移行
dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.velocity_control)  # モーターを速度コントロール
dxl.write(Motor_ID, dxl.Address.GoalVelocity, -handspeed)  # 内爪を開く
while True:
    current = dxl.read(Motor_ID, dxl.Address.PresentCurrent)  # トルク読み取り
    if current < -350:
        print("外爪が閉じ内爪が開きました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break

    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break


#内爪を閉じる(全閉状態)
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl + inner_finger_dis,handspeed)
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
print("ハンドを狭隘空間に差し込み")
time.sleep(2)
print("外爪開")

#外爪開(内爪閉状態から)
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl + outer_finger_dis,handspeed)
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

#内爪を開く
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl - inner_finger_dis,handspeed)
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

time.sleep(2)
print("物体把持")
#内爪を閉じる
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl + inner_finger_dis,handspeed)
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




dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)



sys.exit()