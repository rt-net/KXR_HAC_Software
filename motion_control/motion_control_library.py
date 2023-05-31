import math
import time
import sys

import numpy as np
import adafruit_bno055
import board
from Rcb4BaseLib import Rcb4BaseLib
from matplotlib import pyplot as plt
# from matplotlib import animation

import motion_control.parameters

class MotionLibrary:   
    def __init__(self): 
        print("\n[RCB4初期化中]")
        sys.path.append('../Rcb4Lib') #Rcb4Libの検索パスを追加
        self.rcb4 = Rcb4BaseLib()      #rcb4をインスタンス(定義)
        self.rcb4.open('/dev/ttyUSB0',115200,1.3) #RCB4とのシリアル通信ポートをオープン
        
        if self.rcb4.checkAcknowledge() == True:  #通信が返ってきたとき
            print("RCB-4 Open")
            print("[RCB4初期化完了]")
        else:
            print("RCB-4 Error")
            print("[RCB4初期化失敗]")
        
        i2c = board.I2C() #I2Cのインスタンス
        self.sensor = adafruit_bno055.BNO055_I2C(i2c) #IMUのインスタンス
        
        self.field_absolute_coordinate_x = 0
        self.field_absolute_coordinate_y = 0
        self.field_absolute_angle = 0
        
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        
        self.yaw_origin = 0
        self.pitch_origin = 0
        self.roll_origin = 0
        
        self.x = []
        self.y = []
        
        self.is_plot_required = False
        self.fig, self.ax = plt.subplots()
        
        self.is_button_pressed = False
        
    def IMU_calibration(self): #任意でIMUのキャリブレーションを行う関数
        try:
            self.yaw_origin = self.sensor.euler[0]
            if self.yaw_origin > 180: #もしyaw軸角度が180度を超えていた時
                self.yaw_origin = (self.yaw_origin-180)-180 #±180度の値に修正
            self.pitch_origin = self.sensor.euler[1]
            self.roll_origin = self.sensor.euler[2]
        except:
            print("sensor callibration failed") 
             
    def get_body_angle(self): #IMUによって機体の絶対角度を得る関数　戻り値は3要素のタプル
        try:
            self.yaw = self.sensor.euler[0] - self.yaw_origin #field_absolute_coordinate_yaw軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
            if self.yaw > 180: #値が180度を超えている場合
                self.yaw = (self.yaw-180)-180 #±180度の値に補正する
                
            self.pitch = self.sensor.euler[1] - self.pitch_origin #Pitch軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
            self.roll = self.sensor.euler[2] - self.roll_origin #Roll軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
        except: #センサー値の取得が上手くいかなかったとき
            print("sensor error") 
        body_angle = (self.yaw, self.pitch, self.roll)
            
        return body_angle #戻り値は3軸の角度のタプル
            
    def button_state(self):
        return self.is_button_pressed    
        
    def walk_forward_continue(self): #指定距離前進
        #KRCの上ボタンを擬似的に押す
        self.rcb4.setKrrButtonData(Rcb4BaseLib.KRR_BUTTON.UP.value)
        self.is_button_pressed = True
        
    def motion_stop(self): #指定距離前進
        #KRCの上ボタンを擬似的に押す
        self.rcb4.setKrrButtonData(Rcb4BaseLib.KRR_BUTTON.NONE.value)
        self.is_button_pressed = False
        time.sleep(0.5)        
        
    def touch_ball(self): #ボールを触るモーション
        rcb4.motionPlay(motion_control.parameters.RCB4_TOUCH_BALL) #モーション番号3(左横移動)を再生
        while True: #モーションの再生が終わるまで繰り返し
            motion_number = self.rcb4.getMotionPlayNum() #現在再生されているモーション番号を取得
            if motion_number < 0: #モーション番号が0より小さい場合はエラー
                break
            if motion_number == 0: #モーション番号が0のときは再生されていない状態
                break
        
    def walk_forward(self, walk_distance): #指定距離前進
        i = 0 #繰り返しに使うiの初期値設定
        
        step_count = abs(round(walk_distance/motion_control.parameters.FORWARD_SINGLE_STEP_TRAVEL)) #歩行動作を行う回数を確定
        
        while i < step_count:
            self.rcb4.motionPlay(motion_control.parameters.RCB4_WALK_FORWARD)
            self.calculate_field_coordinate("FORWARD")

            while True: #モーションの再生が終わるまで繰り返し
                motion_number = self.rcb4.getMotionPlayNum() #現在再生されているモーション番号を取得
                if motion_number < 0: #モーション番号が0より小さい場合はエラー
                    break
                if motion_number == 0: #モーション番号が0のときは再生されていない状態
                    break   
            i = i+1
            
    def walk_sideway(self, walk_distance): #指定距離横移動
        i = 0 #繰り返しに使うiの初期値設定
        
        step_count = (round(walk_distance/motion_control.parameters.SIDE_SINGLE_STEP_TRAVEL)) #歩行動作を行う回数を確定
        #print(step_count) #歩行動作繰り返し回数を表示
        
        if walk_distance < 0: #移動量が負の時(左へ移動のとき)
            step_count = abs(step_count)
            while i < step_count: #定められた歩行回数まで繰り返し
                self.rcb4.motionPlay(motion_control.parameters.RCB4_WALK_LEFT) #モーション番号3(左横移動)を再生
                self.calculate_field_coordinate("LEFT")
                
                while True: #モーションの再生が終わるまで繰り返し
                    motion_number = self.rcb4.getMotionPlayNum() #現在再生されているモーション番号を取得
                    if motion_number < 0: #モーション番号が0より小さい場合はエラー
                        break
                    if motion_number == 0: #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
        else: #移動量が正の時(右へ移動の時)
            while i < step_count: #定められた歩行回数まで繰り返し                
                self.rcb4.motionPlay(motion_control.parameters.RCB4_WALK_RIGHT) #モーション番号4(右横移動)を再生
                self.calculate_field_coordinate("RIGHT")
                    
                while True: #モーションの再生が終わるまで繰り返し
                    motion_number = self.rcb4.getMotionPlayNum() #現在再生されているモーション番号を取得
                    if motion_number < 0: #モーション番号が0より小さい場合はエラー
                        break
                    if motion_number == 0: #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
                
    def turn(self, turn_angle): #指定角度旋回
        i = 0 #繰り返しに使うiの初期値設定
        turn_count = abs(round(turn_angle/motion_control.parameters.TURN_SINGLE_STEP_ANGLE)) #旋回動作を行う回数を確定
        if turn_angle < 0: #旋回角度が負の時(左旋回のとき)
            while i < turn_count: #定められた旋回回数まで繰り返し
                self.rcb4.motionPlay(motion_control.parameters.RCB4_TURN_LEFT) #モーション番号13(左旋回)を再生
                while True: #モーションの再生が終わるまで繰り返し
                    motion_number = self.rcb4.getMotionPlayNum() #現在再生されているモーション番号を取得
                    if motion_number < 0: #モーション番号が0より小さい場合はエラー
                        break
                    if motion_number == 0: #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
        else: #旋回角度が正の時(右旋回の時)
            while i < turn_count: #定められた旋回回数まで繰り返し
                self.rcb4.motionPlay(motion_control.parameters.RCB4_TURN_RIGHT) #モーション番号14(右旋回)を再生
                while True:     #モーションの再生が終わるまで繰り返し
                    motion_number = self.rcb4.getMotionPlayNum() #現在再生されているモーション番号を取得
                    if motion_number < 0: #モーション番号が0より小さい場合はエラー
                        break
                    if motion_number == 0: #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
                
    def calculate_field_coordinate(self, motion_type_or_time):
        self.field_absolute_angle = self.get_body_angle()[0] #IMUの価を読み込む
        if motion_type_or_time == "LEFT":
            self.field_absolute_coordinate_x = self.field_absolute_coordinate_x + math.cos(math.radians(self.field_absolute_angle)) * (-motion_control.parameters.SIDE_SINGLE_STEP_TRAVEL) #自己位置に[cos(θ)*一歩の移動量 = 負]を加算
            self.field_absolute_coordinate_y = self.field_absolute_coordinate_y + math.sin(math.radians(self.field_absolute_angle)) * (motion_control.parameters.SIDE_SINGLE_STEP_TRAVEL) #自己位置に[sin(θ)*一歩の移動量]を加算
        elif motion_type_or_time == "RIGHT":
            self.field_absolute_coordinate_x = self.field_absolute_coordinate_x + math.cos(math.radians(self.field_absolute_angle)) * (motion_control.parameters.SIDE_SINGLE_STEP_TRAVEL) #自己位置に[cos(θ)*一歩の移動量]を加算
            self.field_absolute_coordinate_y = self.field_absolute_coordinate_y + math.sin(math.radians(self.field_absolute_angle)) * (-motion_control.parameters.SIDE_SINGLE_STEP_TRAVEL) #自己位置に[sin(θ)*一歩の移動量 = 負]を加算
        elif motion_type_or_time == "FORWARD":
            self.field_absolute_coordinate_x = self.field_absolute_coordinate_x + math.sin(math.radians(self.field_absolute_angle)) * (motion_control.parameters.FORWARD_SINGLE_STEP_TRAVEL) #フィールド座標系に置ける自身の位置を更新
            self.field_absolute_coordinate_y = self.field_absolute_coordinate_y + math.cos(math.radians(self.field_absolute_angle)) * (motion_control.parameters.FORWARD_SINGLE_STEP_TRAVEL) #フィールド座標系に置ける自身の位置を更新
        elif type(motion_type_or_time) == float:
            self.field_absolute_coordinate_x = self.field_absolute_coordinate_x + math.sin(math.radians(self.field_absolute_angle)) * (motion_control.parameters.FORWARD_1_SECOND_TRAVEL*motion_type_or_time) #フィールド座標系に置ける自身の位置を更新
            self.field_absolute_coordinate_y = self.field_absolute_coordinate_y + math.cos(math.radians(self.field_absolute_angle)) * (motion_control.parameters.FORWARD_1_SECOND_TRAVEL*motion_type_or_time) #フィールド座標系に置ける自身の位置を更新
        else:
            print("coordinate calculation error")
        #print(self.field_absolute_angle)
        #print("X座標: ", self.field_absolute_coordinate_x, "Y座標: ", self.field_absolute_coordinate_y)
        if self.is_plot_required == True:
            self.plot_graph()
            
    def field_absolute_cordinate(self):
        return self.field_absolute_coordinate_x, self.field_absolute_coordinate_y
    
    def set_plot(self):
        self.is_plot_required = True
        self.ax.set_xlim(-1000, 1000)
        self.ax.set_ylim(-1000, 1000)
        self.ax.set_aspect('equal')
        self.x.append(self.field_absolute_coordinate_x)
        self.y.append(self.field_absolute_coordinate_y)
        self.ax.plot(self.x, self.y, color='C0', linestyle='-')
        plt.pause(0.001)
        
    def plot_graph(self):
        self.x.append(self.field_absolute_coordinate_x)
        self.y.append(self.field_absolute_coordinate_y)
        self.ax.plot(self.x, self.y, color='C0', linestyle='-')
        plt.pause(0.001)
        
            


        
        
            