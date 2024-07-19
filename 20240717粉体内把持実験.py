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
frame_width = 1920
frame_height = 1080
outer_bag_marker_id = 1
inner_bag_marker_id = 2
marker_length = 20  # mm
center = (frame_width // 2, frame_height // 2)

marker_detect_height = 700 #マーカーを読み取る高さ
bag_mouth_height = 150 #バッグの高さ
inner_bag_object_height = 200 #バッグ内のオブジェクトの高さ(使わなくてもいい？)
hand_tcp_distance = 260 #ハンドの先端とTCPのY座標の差(ハンドの長さ)
marker_slide_dis_x = 0 #マーカーの位置からバッグの口をどんだけずらすか
marker_slide_dis_y = 50 #マーカーの位置からバッグの口をどんだけずらすか
experiment_motor_speed = 80 #実験のハンドスピード

inner_finger_dis = 1800
outer_finger_dis = 8500
now_sequence = "waiting"
# カメラ,マーカー初期設定ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
cap1 = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # カメラを開く
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width + 1)  # フレーム幅を設定
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height + 1)  # フレーム高さを設定
cap1.set(cv2.CAP_PROP_FPS, 60)  # FPSを設定
cap1.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # オート フォーカスをオフ
cap1.set(cv2.CAP_PROP_FOCUS, 0)  # フォーカスを設定
cap1.set(cv2.CAP_PROP_ZOOM, 0)  # ズームを設定
cap1.set(cv2.CAP_PROP_AUTO_EXPOSURE, 100)  # 明るさを設定0
# ArUcoのパラメータを設定
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)  # ArUco辞書を取得
aruco_params = aruco.DetectorParameters()  # ArUcoの検出パラメータを設定
img_save = []

# -----関数定義-------
# スレッド処理の定義

def get_camera_capture():
    global frame
    while True:
        # カメラ画像の取得
        _, frame = cap1.read()
        img_save.append(frame.copy())

        cv2.waitKey(1)
        cv2.imshow("frame", frame)


# 画像取り込みスレッドの作成
thread = threading.Thread(target=get_camera_capture, daemon=True)
thread.start()



def check_brightness(frame):
    # 画像の平均輝度値を計算
    mean_brightness = np.mean(frame)

    # 閾値を設定(暗いと判断する輝度値)　ピクセルの輝度　０から255
    brightness_threshold = 30
    print(mean_brightness)

    # 平均輝度値が閾値より小さければ真っ暗と判断
    if mean_brightness < brightness_threshold:
        return False  # 暗い
    else:
        return True  # 明るい




frame = None
while frame is None:
    #print("Noneループが回ってます")
    pass

# ハンド初期キャリブレーション------------------------------------------------------------------------------------------------------
handspeed = 100

# dynamixel初期設定
dxl = myDynamixel.Dxlfunc()  # インスタンス化
MotorNum = dxl.init('COM3', baudrate=4000000)  # COM通信容量を指定
print(MotorNum)

if MotorNum > 0:
    print("dynamixel初期化成功")
else:
    print("初期化失敗")
Motor_ID = 1  # モーターIDを設定

dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)
dxl.write(Motor_ID, dxl.Address.TorqueEnable, True)  # モーターのトルクをオンにする

print("初期セットアップ完了")  # --------------------------------------------------------------------------------------------------------------------



# URの姿勢設定
ur.standard_position = np.array([-145.0, -450.0, marker_detect_height])  # 把持前の基本位置
ur.start_posture = np.array([-180, 0, 0])  # 把持前の基本位置

P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                              ur.standard_position[ur.Pos.y],
                              ur.standard_position[ur.Pos.z]])
P_detect_posture = ur.start_posture
P_detect = np.hstack([P_detect_position, P_detect_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=2)
time.sleep(1)

# 赤坂啓輔
# 把持前----------------------------------------------------------------------------------------------------------------



P_wait_position = np.array([ur.standard_position[ur.Pos.x],
                            ur.standard_position[ur.Pos.y],
                            hand_tcp_distance + 200])

P_wait = np.hstack([P_wait_position, ur.start_posture])
ur.moveL(P_wait, unit_is_DEG=True, _time=5)

#ここでハンド閉
now_pos = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos + inner_finger_dis,experiment_motor_speed)
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

#ハンドを挿入
P_approachZ_pos = P_wait_position + np.array([0,0,- 150])
P_approachZ = np.hstack([P_approachZ_pos, ur.start_posture])
ur.moveL(P_approachZ, unit_is_DEG=True, _time=2)

#ここで外爪拡張
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl + outer_finger_dis,experiment_motor_speed)
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

#ここでハンド開
now_pos_dxl = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.PosCnt_Vbase(Motor_ID,now_pos_dxl - inner_finger_dis,experiment_motor_speed)
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

#物体にアプローチ

Grasp_Pos = np.array([P_approachZ_pos[ur.Pos.x],
                      P_approachZ_pos[ur.Pos.y],
                      hand_tcp_distance + 30])
Grasp_P = np.hstack([Grasp_Pos, ur.start_posture])
ur.moveL(Grasp_P, unit_is_DEG=True, _time=2)

dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.current_control)
dxl.write(Motor_ID, dxl.Address.GoalCurrent, 60)

#物体把持完了
time.sleep(3)

Pick_up_pos = Grasp_Pos + np.array([0,0,400])
Pick_up_p = np.hstack([Pick_up_pos, ur.start_posture])
ur.moveL(Pick_up_p, unit_is_DEG=True, _time=2)
save_path = "img/"+get_now()
os.mkdir(save_path)
save_num = len(img_save)

for i in range (save_num):
    img_path = save_path + "/img_" + str(i).zfill(6) + ".jpg"
    cv2.imwrite(img_path,img_save[i])

ur.exit()
sys.exit()
# 赤坂啓輔