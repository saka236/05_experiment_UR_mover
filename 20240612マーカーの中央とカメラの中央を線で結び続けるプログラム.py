import math
import sys
import threading
import time

import cv2
import keyboard
import numpy as np
from cv2 import aruco


#事前設定項目
frame_width = 1920
frame_height = 1080
outer_bag_marker_id = 1
inner_bag_marker_id = 2
marker_length = 20  # mm
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

time.sleep(3)

while True:
    if keyboard.is_pressed("q"):
        break

    time.sleep(0.1)

cap1.release()
cv2.destroyAllWindows()
sys.exit()
# 赤坂啓輔
