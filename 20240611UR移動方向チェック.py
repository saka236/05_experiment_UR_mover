import math
import sys
import threading
import time

import cv2
import keyboard
import myDynamixel
import numpy as np
from cv2 import aruco
from tkinter import messagebox as msg
from sub_code.myUniversalRobot_v2 import myUniversalRobot
import os

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"  # 環境変数を設定して、HWトランスフォームを無効化
# URのインスタンスを作成
ur = myUniversalRobot()

# 把持前の事前設定
ur.standard_position = np.array([-145.0, -450.0, 600.0])  # 把持前の基本位置
ur.start_posture = np.array([-180,0,0])  # 把持前の基本位置
P_wait_position = np.array([ur.standard_position[ur.Pos.x],
                            ur.standard_position[ur.Pos.y],
                            ur.standard_position[ur.Pos.z]])
P_wait_posture = ur.start_posture
P_wait = np.hstack([P_wait_position, P_wait_posture])
ur.moveL(P_wait, unit_is_DEG=True, _time=5)

P_wait_position = np.array([ur.standard_position[ur.Pos.x]+100,
                            ur.standard_position[ur.Pos.y],
                            ur.standard_position[ur.Pos.z]])
P_wait_posture = ur.start_posture
P_wait = np.hstack([P_wait_position, P_wait_posture])
ur.moveL(P_wait, unit_is_DEG=True, _time=5)

P_wait_position = np.array([ur.standard_position[ur.Pos.x]+100,
                            ur.standard_position[ur.Pos.y]+100,
                            ur.standard_position[ur.Pos.z]])
P_wait_posture = ur.start_posture
P_wait = np.hstack([P_wait_position, P_wait_posture])
ur.moveL(P_wait, unit_is_DEG=True, _time=5)





ur.exit()
sys.exit()
# 赤坂啓輔
