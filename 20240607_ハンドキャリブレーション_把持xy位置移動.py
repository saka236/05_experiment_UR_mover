import math
import sys
import threading
import time

import cv2
import keyboard
import myDynamixel
import numpy as np
from cv2 import aruco

from sub_code.myUniversalRobot_v2 import myUniversalRobot

# URのインスタンスを作成
ur = myUniversalRobot()

#事前設定項目
frame_width = 1920
frame_height = 1080
outer_bag_marker_id = 1
inner_bag_marker_id = 2
marker_length = 20  # mm

# カメラ,マーカー初期設定ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
cap1 = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # カメラを開く
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width + 1)  # フレーム幅を設定
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height + 1)  # フレーム高さを設定
cap1.set(cv2.CAP_PROP_FPS, 60)  # FPSを設定
cap1.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # オート フォーカスをオフ
cap1.set(cv2.CAP_PROP_FOCUS, 0)  # フォーカスを設定
cap1.set(cv2.CAP_PROP_ZOOM, 0)  # ズームを設定
cap1.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 明るさを設定0
# ArUcoのパラメータを設定
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)  # ArUco辞書を取得
aruco_params = aruco.DetectorParameters()  # ArUcoの検出パラメータを設定


# -----関数定義-------
# スレッド処理の定義

def get_camera_capture():
    global frame
    while True:
        # カメラ画像の取得
        _, frame = cap1.read()


# 画像取り込みスレッドの作成
thread = threading.Thread(target=get_camera_capture, daemon=True)
thread.start()


# 座標間の座標をミリメートル単位で計算する関数
# def calculate_distance(pt1, marker_length_pixel):
#    pixel_x = pt1[0]
#    pixel_y = pt1[1]
#    x_mm = (marker_length / marker_length_pixel) * pixel_x  # ピクセルからセンチメートルに変換
#    y_mm = (marker_length / marker_length_pixel) * pixel_y
#    return x_mm, y_mm

def calculate_distance(pt1, marker_length_pixel):
    cam_center_pix_x = frame_width/2
    cam_center_pix_y = frame_height/2
    pix_x_dis = pt1[0] - cam_center_pix_x
    pix_y_dis = pt1[1] - cam_center_pix_y
    x_dis_mm = (marker_length / marker_length_pixel) * pix_x_dis  # ピクセルからセンチメートルに変換
    y_dis_mm = (marker_length / marker_length_pixel) * pix_y_dis
    return x_dis_mm, y_dis_mm


# ArUcoマーカーを検出し、マーカー中心点の座標と長さを取得する関数
def detect_aruco_markers(image):
    # ArUcoマーカーを検出
    corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)

    marker_centers = {}
    marker_lengths = {}
    if ids is not None:
        for i, marker_id in enumerate(np.ravel(ids)):
            index = np.where(ids == marker_id)[0][0]
            corner_ul = corners[index][0][0]
            corner_br = corners[index][0][2]
            corner_ur = corners[index][0][1]
            center = [(corner_ul[0] + corner_br[0]) / 2, (corner_ul[1] + corner_br[1]) / 2]
            marker_length_pixel = math.sqrt((corner_ul[0] - corner_ur[0]) ** 2 + (corner_ul[1] - corner_ur[1]) ** 2)
            marker_centers[marker_id] = (int(center[0]), int(center[1]))  # 座標を整数に変換
            marker_lengths[marker_id] = marker_length_pixel

    # 検出されたマーカーを別ウィンドウで表示
    aruco.drawDetectedMarkers(image, corners, ids)

    return marker_centers, marker_lengths, image  # 画像を返すように変更


def check_brightness(frame):
    # 画像の平均輝度値を計算
    mean_brightness = np.mean(frame)

    # 閾値を設定(暗いと判断する輝度値)　ピクセルの輝度　０から255
    brightness_threshold = 30

    # 平均輝度値が閾値より小さければ真っ暗と判断
    if mean_brightness < brightness_threshold:
        return False  # 暗い
    else:
        return True  # 明るい


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

frame = None
while frame is None:
    print("Noneループが回ってます")
    pass

dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.velocity_control)  # モーターを速度コントロール

print("キャリブレーションを実施")
dxl.write(Motor_ID, dxl.Address.GoalVelocity, -handspeed)  # 外側のハンドをハンドを閉じる(初期化)

while True:
    current = dxl.read(Motor_ID, dxl.Address.PresentCurrent)  # トルク読み取り
    # print(current)
    if current < -600:
        print("外爪が閉じました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break

    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break

dxl.write(Motor_ID, dxl.Address.GoalVelocity, handspeed)  # 内側のハンドをハンドを閉じる(初期化)

while True:
    ret1, img1 = cap1.read()
    bright_check = check_brightness(img1)

    if not bright_check:
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        print("内爪が閉じました")
        all_hand_close_posision = dxl.read(Motor_ID, dxl.Address.PresentPosition)
        break
    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break
all_hand_close_pos = dxl.read(Motor_ID, dxl.Address.PresentPosition)

time.sleep(0.5)

print("外爪を開いて距離を取得します")
dxl.write(Motor_ID, dxl.Address.GoalVelocity, handspeed)  # 外側のハンドをハンドを開く(初期化)

while True:
    current = dxl.read(Motor_ID, dxl.Address.PresentCurrent)  # トルク読み取り
    if current > 600:
        print("外爪が開きました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break
    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break

outer_hand_open_pos = dxl.read(Motor_ID, dxl.Address.PresentPosition)

outer_finger_dis = outer_hand_open_pos - all_hand_close_pos
print(f"外爪の開閉移動距離は{outer_finger_dis}です")
print("初期位置に移行")
# 再度初期位置に移行

dxl.write(Motor_ID, dxl.Address.GoalVelocity, -handspeed)  # 外側のハンドをハンドを閉じる(初期化)

while True:
    current = dxl.read(Motor_ID, dxl.Address.PresentCurrent)  # トルク読み取り
    if current < -600:
        print("外爪が閉じました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break

    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break
dxl.write(Motor_ID, dxl.Address.GoalVelocity, handspeed)  # 内側のハンドをハンドを閉じる(初期化)
inner_finger_open_position = dxl.read(Motor_ID, dxl.Address.PresentPosition)
while True:
    bright_check = check_brightness(frame)

    if not bright_check:
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        print("内爪が閉じました")
        break
    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break
inner_finger_close_position = dxl.read(Motor_ID, dxl.Address.PresentPosition)
inner_finger_dis = inner_finger_close_position - inner_finger_open_position
print(f"内爪の開閉移動距離は{inner_finger_dis}です")
# カメラを使用するため内爪を開く
dxl.PosCnt_Vbase(Motor_ID, inner_finger_open_position, handspeed)
print(
    "初期セットアップ完了")  # --------------------------------------------------------------------------------------------------------------------

# URの姿勢設定
ur.standard_position = np.array([-145.0, -450.0, 650.0])  # 把持前の基本位置
ur.start_posture = np.array([-180,0,0])  # 把持前の基本位置

P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                            ur.standard_position[ur.Pos.y],
                            ur.standard_position[ur.Pos.z]])
P_detect_posture = ur.start_posture
P_detect = np.hstack([P_detect_position, P_detect_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=5)

# ロボット動作----------------------------------------------------------------------------------------------------------------

marker_centers, marker_lengths, img = detect_aruco_markers(frame)
outer_bag_pt = marker_centers.get(outer_bag_marker_id)
marker_length_pixel = marker_lengths.get(outer_bag_marker_id)
x_dis_mm, y_dis_mm = calculate_distance(outer_bag_pt, marker_length_pixel)

P_wait_position = np.array([ur.standard_position[ur.Pos.x] - x_dis_mm,
                            ur.standard_position[ur.Pos.y] + y_dis_mm,
                            ur.standard_position[ur.Pos.z]])
P_wait_posture = ur.start_posture
P_wait = np.hstack([P_wait_position, P_wait_posture])
ur.moveL(P_wait, unit_is_DEG=False, _time=2)
dxl.PosCnt_Vbase(Motor_ID, inner_finger_close_position, handspeed)

### ハンドを袋に挿入
# P_start_position = np.array([ur.standard_position[ur.Pos.x] - x_dis_mm,
#                             ur.standard_position[ur.Pos.y] + y_dis_mm,
#                             ur.standard_position[ur.Pos.z] - 100])
# P_start_posture = ur.get_rotvec(ur.start_posture, 0)
# P_start = np.hstack([P_start_position, P_start_posture])
# ur.moveL(P_start, unit_is_DEG=False, _time=2)
#
## 外爪で拡張
# p1 = dxl.read(Motor_ID, dxl.Address.PresentPosition)
# dxl.PosCnt_Vbase(Motor_ID, p1 + outer_finger_dis, handspeed)
#
# t1 = time.time()
# while True:
#    t2 = time.time() - t1
#    now_velocity = dxl.read(Motor_ID, dxl.Address.PresentVelocity)
#
#    if keyboard.is_pressed("q"):  # qが押されたら終了
#        break
#
#    if t2 >= 0.5 and now_velocity == 0:
#        # print("内爪が閉じました")
#        break
#
# p2 = dxl.read(Motor_ID, dxl.Address.PresentPosition)
# dxl.PosCnt_Vbase(Motor_ID, p2 - inner_finger_dis, handspeed)
#
# t3 = time.time()
# while True:
#    t4 = time.time() - t3
#    now_velocity = dxl.read(Motor_ID, dxl.Address.PresentVelocity)
#
#    if keyboard.is_pressed("q"):  # qが押されたら終了
#        break
#
#    if t4 >= 0.5 and now_velocity == 0:
#        # print("内爪が閉じました")
#        break
## ハンド内の物体を認識
# marker_centers, marker_lengths, img = detect_aruco_markers(frame)
# outer_bag_pt = marker_centers.get(inner_bag_marker_id)
# marker_length_pixel = marker_length.get(outer_bag_marker_id)
# x_dis_mm, y_dis_mm = calculate_distance(outer_bag_pt, marker_length_pixel)
#
# G_wait_position = np.array([P_start_position[ur.Pos.x] - x_dis_mm,
#                            P_start_position[ur.Pos.y] + y_dis_mm,
#                            P_start_position[ur.Pos.z]])
# G_wait_posture = ur.get_rotvec(ur.start_posture, 0)
# G_wait = np.hstack([G_wait_position, G_wait_posture])
# ur.moveL(P_start, unit_is_DEG=False, _time=2)
#
# G_start_position = np.array([P_start_position[ur.Pos.x] - x_dis_mm,
#                             P_start_position[ur.Pos.y] + y_dis_mm,
#                             P_start_position[ur.Pos.z]])
# G_start_posture = ur.get_rotvec(ur.start_posture, 0)
# G_start = np.hstack([G_start_position, G_start_posture])
# ur.moveL(P_start, unit_is_DEG=False, _time=2)


ur.exit()
dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
sys.exit()
# 赤坂啓輔
