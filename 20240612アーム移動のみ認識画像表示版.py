import math
import sys
import threading
import time

import cv2
import keyboard
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
center = (frame_width//2 , frame_height//2)
center = (frame_width//2 , frame_height//2)
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
    global frame,marker_centers,marker_lengths
    while True:
        # カメラ画像の取得
        _, frame = cap1.read()
        marker_centers, marker_lengths, img = detect_aruco_markers(frame.copy())
        if marker_centers:
            for marker_id, marker_center in marker_centers.items():
                # マーカーの中央とカメラの中央を結ぶ線を描画
                if marker_id == outer_bag_marker_id:
                    color = (0, 0, 255)  # 赤色
                elif marker_id == inner_bag_marker_id:
                    color = (255, 0, 0)  # 青色
                else:
                    color = (0, 255, 0)  # その他の色（緑色）

                cv2.line(img, marker_center, center, color, 2)
                cv2.circle(img, marker_center, 5, (0, 0, 255), -1)
        cv2.waitKey(1)
        cv2.imshow("frame", img)






# 画像取り込みスレッドの作成
thread = threading.Thread(target=get_camera_capture, daemon=True)
thread.start()



def calculate_distance(pt1, marker_length_pixel):
    cam_center_pix_x = frame_width/2
    cam_center_pix_y = frame_height/2
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

    # 平均輝度値が閾値より小さければ真っ暗と判断
    if mean_brightness < brightness_threshold:
        return False  # 暗い
    else:
        return True  # 明るい

marker_centers = {}
marker_lengths = {}
frame  = None
while frame is None:
    print("Noneループが回ってます")
    pass

print("初期セットアップ完了")  # --------------------------------------------------------------------------------------------------------------------

# URの姿勢設定
ur.standard_position = np.array([-145.0, -450.0, 650.0])  # 把持前の基本位置
ur.start_posture = np.array([-180,0,0])  # 把持前の基本位置

P_detect_position = np.array([ur.standard_position[ur.Pos.x],
                            ur.standard_position[ur.Pos.y],
                            ur.standard_position[ur.Pos.z]])
P_detect_posture = ur.start_posture
P_detect = np.hstack([P_detect_position, P_detect_posture])
ur.moveL(P_detect, unit_is_DEG=True, _time=5)
time.sleep(1)

# ロボット動作----------------------------------------------------------------------------------------------------------------


#marker_centers, marker_lengths, img = detect_aruco_markers(frame)
outer_bag_pt = marker_centers.get(outer_bag_marker_id)
marker_length_pixel = marker_lengths.get(outer_bag_marker_id)
print(marker_length_pixel)
x_dis_mm, y_dis_mm = calculate_distance(outer_bag_pt, marker_length_pixel)

P_wait_position = np.array([ur.standard_position[ur.Pos.x] - x_dis_mm,
                            ur.standard_position[ur.Pos.y] + y_dis_mm,
                            ur.standard_position[ur.Pos.z] - 100])
P_wait_posture = ur.start_posture
P_wait = np.hstack([P_wait_position, P_wait_posture])
ur.moveL(P_wait, unit_is_DEG=True, _time=2)

time.sleep(1)

inner_bag_pt = marker_centers.get(inner_bag_marker_id)
marker_length_pixel = marker_lengths.get(inner_bag_marker_id)
print(marker_length_pixel)
x_dis_mm, y_dis_mm = calculate_distance(inner_bag_pt, marker_length_pixel)

P_2Pos = P_wait_position + np.array([- x_dis_mm,y_dis_mm,-50])
P_wait_posture = ur.start_posture
P_2 = np.hstack([P_2Pos, P_wait_posture])
ur.moveL(P_2, unit_is_DEG=True, _time=2)


time.sleep(5)

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
sys.exit()
# 赤坂啓輔
