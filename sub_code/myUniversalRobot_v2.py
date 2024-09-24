import struct  # 構造化されたバイナリデータの操作のためのライブラリをインポート受信データ関連
import time  # 時間関連の操作のためのライブラリをインポート
from socket import socket, AF_INET, SOCK_STREAM  # ソケット通信のためのライブラリをインポート
import numpy as np  # 数値計算のためのライブラリをインポート
from scipy.spatial.transform import Rotation  # 3D回転の操作のためのライブラリをインポート
from threading import Thread  # スレッド処理のためのライブラリをインポート バックグラウンドで情報を受け取り続けられるような処理

class myUniversalRobot:
    class Pos:
        # ロボットの姿勢を表す各要素のインデックスを定義するクラス
        x = 0
        y = 1
        z = 2
        rx = 3
        ry = 4
        rz = 5

    class package_type:
        # ロボットから受信するパケットの種類を表す定数を定義するクラス
        ROBOT_MODE_DATA = 0
        JOINT_DATA = 1
        TOOL_DATA = 2
        MASTERBOARD_DATA = 3
        CARTESIAN_INFO = 4
        KINEMATICS_INFO = 5
        CONFIGURATION_DATA = 6
        FORCE_MODE_DATA = 7
        ADDITIONAL_INFO = 8
        NEEDED_FOR_CALIB_DATA = 9
        TOOL_COMM_INFO = 11
        TOOL_MODE_INFO = 12

    class message_type:
        # ロボットから受信するメッセージの種類を表す定数を定義するクラス
        ROBOT_STATE = 16
        ROBOT_MESSAGE = 20

    def __init__(self):
        # クラスの初期化メソッド
        ip = '192.168.11.3'  # ロボットのIPアドレス
        port = 30001  # ロボットとの通信に使用するポート番号
        self.s = socket(AF_INET, SOCK_STREAM)  # ソケットを作成
        self.s.settimeout(2)  # ソケットのタイムアウトを設定
        self.connected = False  # ロボットへの接続状態を示すフラグを初期化
        try:
            self.s.connect((ip, port))  # ロボットに接続
            self.s.send(self._toByte('ConnectCheck\n'))  # ロボットに接続確認メッセージを送信
            self.connected = True  # 接続成功時にフラグをTrueに設定
        except OSError:
            print('接続失敗')  # 接続に失敗した場合にエラーメッセージを表示
        th = Thread(target=self.back_loop, daemon=True)  # バックグラウンドでデータを受信するスレッドを作成
        th.start()  # スレッドを開始
        self.joint_angle = np.array([0.0]*6)  # ロボットの関節角度を格納する配列を初期化
        self.tcp_pos = np.array([0.0]*6)  # ロボットのTCP位置姿勢を格納する配列を初期化

    def exit(self):
        # ロボットとの通信を終了するメソッド
        self.stop()  # ロボットの動作を停止
        self.s.close()  # ソケットを閉じる

    def back_loop(self):
        # バックグラウンドでロボットからデータを受信するメソッド
        while True:
            data = self.s.recv(4096)  # ロボットからデータを受信
            i = 0
            all_data_length = (struct.unpack('!i', data[0:4]))[0]  # パケット全体の長さを取得
            now_msg = data[4]  # 現在のメッセージの種類を取得
            if now_msg == self.message_type.ROBOT_STATE:
                # パケットの種類がロボットの状態情報の場合
                while i + 5 < all_data_length:
                    pkg_len = (struct.unpack('!i', data[5 + i:9 + i]))[0]  # パケットの長さを取得
                    now_pkg_data = data[5 + i:5 + i + pkg_len]  # パケットのデータ部分を取得
                    pkg_type = now_pkg_data[4]  # パケットの種類を取得

                    if pkg_type == self.package_type.JOINT_DATA:
                        # パケットの種類が関節データの場合
                        for j in range(6):
                            # 各関節の角度データを抽出し、配列に格納
                            self.joint_angle[j] = np.rad2deg((struct.unpack('!d', now_pkg_data[5 + (j * 41):13 + (j * 41)]))[0])
                    elif pkg_type == self.package_type.CARTESIAN_INFO:
                        # パケットの種類がカートジアン情報の場合
                        self.tcp_pos[self.Pos.x] = (struct.unpack('!d', data[10 + i:18 + i]))[0]*1000  # X座標を取得
                        self.tcp_pos[self.Pos.y] = (struct.unpack('!d', data[18 + i:26 + i]))[0]*1000  # Y座標を取得
                        self.tcp_pos[self.Pos.z] = (struct.unpack('!d', data[26 + i:34 + i]))[0]*1000  # Z座標を取得
                        self.tcp_pos[self.Pos.rx] = (struct.unpack('!d', data[34 + i:42 + i]))[0]  # X軸回転を取得
                        self.tcp_pos[self.Pos.ry] = (struct.unpack('!d', data[42 + i:50 + i]))[0]  # Y軸回転を取得
                        self.tcp_pos[self.Pos.rz] = (struct.unpack('!d', data[50 + i:58 + i]))[0]  # Z軸回転を取得
                    i += pkg_len

    def moveJ(self, _angles, _acc=1.0, _vel=1.0, _time=None, unit_is_DEG=True):
        # 関節空間でロボットを動かすメソッド
        if not self.connected:
            return
        if unit_is_DEG:
            _angles = np.deg2rad(_angles)  # 角度をラジアンに変換
        if _time == None:
            command = 'movej({angles}, a={acc}, v={vel})'.format(angles=_angles.tolist(), acc=_acc, vel=_vel)  # 移動コマンドを生成
            self.s.send(self._toByte(command))  # ロボットにコマンドを送信
        elif _time > 0:
            command = 'movej({angles}, t={time})'.format(angles=_angles.tolist(), time=_time)  # 指定時間で移動するコマンドを生成
            self.s.send(self._toByte(command))  # ロボットにコマンドを送信
            time.sleep(_time)  # 指定時間待つ

    def moveL(self, _position, _acc=1000, _vel=150, _time=None, unit_is_DEG=False):
        # 直線移動でロボットを動かすメソッド
        if not self.connected:
            return
        _goal_position = np.zeros(6)  # 目標位置姿勢を格納する配列を初期化
        _goal_position[self.Pos.x:self.Pos.z+1] = _position[self.Pos.x:self.Pos.z+1]/1000  # X, Y, Z座標をメートルに変換
        if unit_is_DEG:
            _goal_position[self.Pos.rx:self.Pos.rz+1] = np.deg2rad(_position[self.Pos.rx:self.Pos.rz+1])  # 回転角度をラジアンに変換
        else:
            _goal_position[self.Pos.rx:self.Pos.rz+1] = _position[self.Pos.rx:self.Pos.rz+1]  # 回転角度をそのまま使用
        if _time == None:
            command = 'movel(p{position}, a={acc}, v={vel})'.format(position=_goal_position.tolist(), acc=_acc/1000, vel=_vel/1000)  # 移動コマンドを生成
            self.s.send(self._toByte(command))  # ロボットにコマンドを送信
        elif _time > 0:
            command = 'movel(p{position}, t={time})'.format(position=_goal_position.tolist(), time=_time)  # 指定時間で移動するコマンドを生成
            self.s.send(self._toByte(command))  # ロボットにコマンドを送信
            time.sleep(_time)  # 指定時間待つ

    def speedL(self, _speed, _acc=1000):
        # リニアスピードを設定するメソッド
        if not self.connected:
            return
        _speed[self.Pos.x:self.Pos.z+1] = _speed[self.Pos.x:self.Pos.z+1]/1000  # X, Y, Z方向の速度をメートル/秒に変換
        command = 'speedl({speed}, a={acc}, t=10.0)'.format(speed=_speed.tolist(), acc=_acc/1000)  # リニアスピードコマンドを生成
        self.s.send(self._toByte(command))  # ロボットにコマンドを送信

    def stop(self):
        # ロボットの動作を停止するメソッド
        command = 'stopj(2)'  # 動作停止コマンドを生成
        self.s.send(self._toByte(command))  # ロボットにコマンドを送信

    def get_rotvec(self, relative_posture):
        """ 対的な回転ベクトル(R^3)を絶対回転ベクトルに変換するメソッド
        Args:
            relative_rotation (): 相対的な回転ベクトル
        Returns:
            絶対的な回転ベクトル
        """
        now_posture = self.tcp_pos[self.Pos.rx:]  # 現在のTCP姿勢を取得
        rot_now = Rotation.from_rotvec(now_posture)  # 現在の回転ベクトルをRotationクラスに変換
        rot_relative = Rotation.from_rotvec(relative_posture)  # 相対的な回転ベクトルをRotationクラスに変換
        ret = rot_relative * rot_now  # 絶対的な回転ベクトルを計算
        return ret.as_rotvec()  # 絶対的な回転ベクトルを回転ベクトルとして返す

    def _toByte(self, str):
        # 文字列をバイト列に変換するメソッド
        message = str + '\n'  # 改行を追加
        return bytes(message.encode())  # 文字列をバイト列に変換して返す