# OpenCV、NumPy、その他必要なライブラリをインポート
import math
import sys
import time

import cv2
import keyboard
import numpy as np
from cv2 import aruco

# カメラのセットアップ(anker)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # カメラを開く
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1921)  # フレーム幅を設定
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1081)  # フレーム高さを設定
cap.set(cv2.CAP_PROP_FPS, 60)  # FPSを設定
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # オートフォーカスをオフ
cap.set(cv2.CAP_PROP_FOCUS, 0)  # フォーカスを設定
cap.set(cv2.CAP_PROP_ZOOM, 0)  # ズームを設定
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)  # 明るさを設定
cap.set(cv2.CAP_PROP_TILT, 0)  # 傾きを設定
cap.set(cv2.CAP_PROP_PAN, 0)  # パンを設定
# ArUcoのパラメータを設定
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)  # ArUco辞書を取得
aruco_params = aruco.DetectorParameters()  # ArUcoの検出パラメータを設定


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


i = 0
x2 = time.time()
while True:
    cv2.waitKey(1)
    x1 = time.time()  # 現在の時間を記録

    if i > 5:
        t1 = x1 - x2  # 前回のループからの経過時間を計算
        fps = 1 / t1
        print(fps)

    x2 = time.time()  # 現在の時間を次回のために保存

    i += 1  # iを加算

    ret, frame = cap.read()

    marker_centers, marker_lengths, image = detect_aruco_markers(frame.copy())
    cv2.imshow("image", image)
    print(marker_centers)

    # cv2.imshow("frame", frame)
    if keyboard.is_pressed("q"):  # qが押されたら終了
        cv2.destroyAllWindows()
        break

sys.exit()
