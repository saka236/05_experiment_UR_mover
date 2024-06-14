import queue
import threading

import cv2

# カメラを開く
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # フレーム幅を設定
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)  # フレーム高さを設定
cap.set(cv2.CAP_PROP_FPS, 30)  # FPSを設定
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # オート フォーカスをオフ
cap.set(cv2.CAP_PROP_FOCUS, 60)  # フォーカスを設定
cap.set(cv2.CAP_PROP_ZOOM, 176)  # ズームを設定
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 30)  # 明るさを設定
cap.set(cv2.CAP_PROP_TILT, -10)  # 傾きを設定
cap.set(cv2.CAP_PROP_PAN, 0)  # パンを設定
cap.set(cv2.CAP_PROP_CONTRAST, 70)

# フレーム共用のキュー
q_frames = queue.Queue()


def frame_update():
    '''画像取込'''
    while True:
        # カメラ画像の取得
        _, frame = cap.read()
        # 取得した画像をキューに追加
        q_frames.put(frame)


# 画像取り込みスレッドの作成
thread = threading.Thread(target=frame_update, daemon=True)
# スレッドの開始
thread.start()

# 画像表示用スレッド
while True:
    frame = q_frames.get()  # queueが空の場合、キューに何か入るまで、スレッドをブロックする。
    cv2.imshow("Image", frame)

    # キーを押すとループを終了する
    if cv2.waitKey(1) > 0:
        break

# カメラを閉じる
cap.release()
# すべてのウィンドウを閉じる
cv2.destroyAllWindows()
