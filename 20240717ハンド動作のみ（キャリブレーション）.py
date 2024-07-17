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
    global frame, marker_centers, marker_lengths,img_save
    while True:
        # カメラ画像の取得
        _, frame = cap1.read()
        img_save.append(frame.copy())
        marker_centers, marker_lengths, img = detect_aruco_markers(frame.copy())
        #if marker_centers:
        #    for marker_id, marker_center in marker_centers.items():
        #        # マーカーの中央とカメラの中央を結ぶ線を描画
        #        if marker_id == outer_bag_marker_id:
        #            color = (0, 0, 255)  # 赤色
        #        elif marker_id == inner_bag_marker_id:
        #            color = (255, 0, 0)  # 青色
        #        else:
        #            color = (0, 255, 0)  # その他の色（緑色）
#
        #        cv2.line(img, marker_center, center, color, 2)
        #        cv2.circle(img, marker_center, 5, (0, 0, 255), -1)
        #cv2.putText(img,now_sequence, (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 5, cv2.LINE_AA)
        cv2.waitKey(1)
        cv2.imshow("frame", img)


# 画像取り込みスレッドの作成
thread = threading.Thread(target=get_camera_capture, daemon=True)
thread.start()



def calculate_distance(pt1, marker_length_pixel):
    cam_center_pix_x = frame_width / 2
    cam_center_pix_y = frame_height / 2
    pix_x_dis = pt1[0] - cam_center_pix_x
    pix_y_dis = pt1[1] - cam_center_pix_y
    x_dis_mm = (marker_length / marker_length_pixel) * pix_x_dis  # ピクセルからミリメートルに変換
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
    print(mean_brightness)

    # 平均輝度値が閾値より小さければ真っ暗と判断
    if mean_brightness < brightness_threshold:
        return False  # 暗い
    else:
        return True  # 明るい




marker_centers = {}
marker_lengths = {}
frame = None
while frame is None:
    #print("Noneループが回ってます")
    pass

# ハンド初期キャリブレーション------------------------------------------------------------------------------------------------------
handspeed = 100




# dynamixel初期設定
dxl = myDynamixel.Dxlfunc()  # インスタンス化
MotorNum = dxl.init('COM7', baudrate=4000000)  # COM通信容量を指定
print(MotorNum)



if MotorNum > 0:
    print("dynamixel初期化成功")
else:
    print("初期化失敗")
Motor_ID = 1  # モーターIDを設定

dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)  # モーターのトルクをオフにする(初期化)
dxl.write(Motor_ID, dxl.Address.TorqueEnable, True)  # モーターのトルクをオンにする



dxl.Change_OperatingMode(Motor_ID, dxl.operating_mode.velocity_control)  # モーターを速度コントロール

print("キャリブレーションを実施")
dxl.write(Motor_ID, dxl.Address.GoalVelocity, -handspeed)  # 外側のハンドをハンドを閉じる(初期化)


while True:
    current = dxl.read(Motor_ID, dxl.Address.PresentCurrent)  # トルク読み取り
    # print(current)
    if current < -350:
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
    if current > 350:
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
    if current < -350:
        print("外爪が閉じました")
        dxl.write(Motor_ID, dxl.Address.GoalVelocity, 0)
        break

    elif keyboard.is_pressed("q"):  # 3を押すとハンドを開いてプログラムを終了
        dxl.write(Motor_ID, dxl.Address.TorqueEnable, False)
        break
inner_finger_open_position = dxl.read(Motor_ID, dxl.Address.PresentPosition)
dxl.write(Motor_ID, dxl.Address.GoalVelocity, handspeed)  # 内側のハンドをハンドを閉じる(初期化)


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
inner_finger_dis = 1800
# カメラを使用するため内爪を開く
dxl.PosCnt_Vbase(Motor_ID, inner_finger_open_position, handspeed)

print("初期セットアップ完了")  # --------------------------------------------------------------------------------------------------------------------

