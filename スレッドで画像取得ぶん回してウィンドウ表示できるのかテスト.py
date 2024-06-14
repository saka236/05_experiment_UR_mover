# OpenCV、NumPy、その他必要なライブラリをインポート
import math
import sys
import threading

import cv2
import keyboard
import numpy as np
from cv2 import aruco

# カメラ,マーカー初期設定ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # カメラを開く
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1921)  # フレーム幅を設定
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1081)  # フレーム高さを設定
cap.set(cv2.CAP_PROP_FPS, 60)  # FPSを設定
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # オート フォーカスをオフ
cap.set(cv2.CAP_PROP_FOCUS, 0)  # フォーカスを設定
cap.set(cv2.CAP_PROP_ZOOM, 0)  # ズームを設定
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 明るさを設定0
# ArUcoのパラメータを設定
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)  # ArUco辞書を取得
aruco_params = aruco.DetectorParameters()  # ArUcoの検出パラメータを設定

fixed_marker_id = 1  # 固定されているマーカーのID
movable_marker_ids = [2, 3]  # 動くマーカーのIDリスト


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


def get_camera_capture():
    global frame
    while True:
        # カメラ画像の取得
        _, frame = cap.read()
        _, _, img = detect_aruco_markers(frame.copy())
        cv2.waitKey(1)
        cv2.imshow("frame", img)


# 画像取り込みスレッドの作成
thread = threading.Thread(target=get_camera_capture, daemon=True)
thread.start()

while True:
    print("main_program_looping")
    if keyboard.is_pressed("q"):
        print("ループ終わり")
        break

sys.exit()
