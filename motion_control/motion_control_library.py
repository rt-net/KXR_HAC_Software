import math
import numpy as np
import motion_control.parameters
import adafruit_bno055
import board
import time
import sys
from Rcb4BaseLib import Rcb4BaseLib            #Rcb4BaseLib.pyの中のRcb4BaseLibが使えるように設定

class MotionLibrary:
    X = 0
    Y = 0
    angle = 0
    
    yaw_origin = 0
    pitch_origin = 0
    roll_origin = 0
    
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
        
    def IMU_calib(self): #任意でIMUのキャリブレーションを行う関数
        try:
            self.yaw_origin = self.sensor.euler[0]
            if self.yaw_origin > 180:
                self.yaw_origin = (self.yaw_origin-180)-180
            self.pitch_origin = self.sensor.euler[1]
            self.roll_origin = self.sensor.euler[2]
        except:
            print("sensor callibration failed") 
             
    def get_body_angle(self): #IMUによって機体の絶対角度を得る関数　戻り値は3要素のタプル
        try:
            yaw = self.sensor.euler[0] - self.yaw_origin #Yaw軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
            if yaw > 180: #値が180度を超えている場合
                yaw = (yaw-180)-180 #±180度の値に補正する
                
            pitch = self.sensor.euler[1] -self.pitch_origin #Pitch軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
            roll = self.sensor.euler[2] -self.roll_origin #Roll軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
            body_angle = (yaw, pitch, roll) #3要素を格納
        except: #センサー値の取得が上手くいかなかったとき
            print("sensor error") 
            body_angle = (0, 0, 0) #全ての要素に0を格納
            
        return body_angle #戻り値は3軸の角度のタプル
    
    def touch_ball(self): #ボールを触るモーション
        rcb4.motionPlay(motion_control.parameters.rcb4_touch_ball) #モーション番号3(左横移動)を再生
        while True: #モーションの再生が終わるまで繰り返し
            motion_number = self.rcb4.getMotionPlayNum() #現在再生されているモーション番号を取得
            if motion_number < 0: #モーション番号が0より小さい場合はエラー
                break
            if motion_number == 0: #モーション番号が0のときは再生されていない状態
                break
        
    def walk_forward(self, walk_distance): #指定距離前進
        i = 0 #繰り返しに使うiの初期値設定
        
        step_count = abs(round(walk_distance/motion_control.parameters.walk_step_distance)) #歩行動作を行う回数を確定
        
        while i < step_count:
            self.rcb4.motionPlay(motion_control.parameters.rcb4_walk_forward)
            
            self.angle = self.get_body_angle()[0]
            self.X = self.X +  math.sin(math.radians(self.angle))*(motion_control.parameters.walk_step_distance) #フィールド座標系に置ける自身の位置を更新
            self.Y = self.Y +  math.cos(math.radians(self.angle))*(motion_control.parameters.walk_step_distance) #フィールド座標系に置ける自身の位置を更新
            
            print("X: ", self.X, "Y: ", self.Y)
            
            while True:     #モーションの再生が終わるまで繰り返し
                motion_number = self.rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                if motion_number < 0:                     #モーション番号が0より小さい場合はエラー
                    break
                if motion_number == 0:                    #モーション番号が0のときは再生されていない状態
                    break
                
            i = i+1
            
    def walk_sideway(self, walk_distance): #指定距離横移動
        i = 0 #繰り返しに使うiの初期値設定
        
        step_count = (round(walk_distance/motion_control.parameters.walk_side_step_distance)) #歩行動作を行う回数を確定
        #print(step_count) #歩行動作繰り返し回数を表示
        
        if walk_distance < 0: #移動量が負の時(左へ移動のとき)
            step_count = abs(step_count)
            while i < step_count: #定められた歩行回数まで繰り返し
                self.angle = self.get_body_angle()[0] #IMUの価を読み込む
                
                rcb4.motionPlay(motion_control.parameters.rcb4_walk_left)       #モーション番号3(左横移動)を再生
                
                if body_angle[0] > 0: #Yaw軸周りの角度が正の時
                    self.X = self.X + math.cos(math.radians(self.angle))*(-walk_side_step_distance) #自己位置に[cos(θ)*一歩の移動量 = 負]を加算
                    self.Y = self.Y + math.sin(math.radians(self.angle))*(walk_side_step_distance) #自己位置に[sin(θ)*一歩の移動量]を加算
                else: #Yaw軸周りの角度が負の時
                    self.X = self.X + math.cos(math.radians(self.angle))*(-walk_side_step_distance) #自己位置に[cos(θ)*一歩の移動量 = 負]を加算
                    self.Y = self.Y + math.sin(math.radians(self.angle))*(walk_side_step_distance) #自己位置に[sin(θ)*一歩の移動量 = 負]を加算
                
                while True:     #モーションの再生が終わるまで繰り返し
                    motion_number = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motion_number < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motion_number == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
        else: #移動量が正の時(右へ移動の時)
            while i < step_count: #定められた歩行回数まで繰り返し
                self.angle = self.get_body_angle()[0] #IMUの価を読み込む
                
                rcb4.motionPlay(motion_control.parameters.rcb4_walk_right)     #モーション番号4(右横移動)を再生
                
                if body_angle[0] > 0: #Yaw軸周りの角度が正の時
                    self.X = self.X + math.cos(math.radians(self.angle))*(walk_side_step_distance) #自己位置に[cos(θ)*一歩の移動量]を加算
                    self.Y = self.Y + math.sin(math.radians(self.angle))*(-walk_side_step_distance) #自己位置に[sin(θ)*一歩の移動量 = 負]を加算
                else: #Yaw軸周りの角度が負の時
                    self.X = self.X + math.cos(math.radians(self.angle))*(walk_side_step_distance) #自己位置に[cos(θ)*一歩の移動量]を加算
                    self.Y = self.Y + math.sin(math.radians(self.angle))*(-walk_side_step_distance) #自己位置に[sin(θ)*一歩の移動量]を加算
                    
                while True:     #モーションの再生が終わるまで繰り返し
                    motion_number = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motion_number < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motion_number == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
                
    def turn(self, turn_angle): #指定角度旋回
        i = 0 #繰り返しに使うiの初期値設定
        turn_count = abs(round(turn_angle/motion_control.parameters.turn_step_angle)) #旋回動作を行う回数を確定
        if turn_angle < 0: #旋回角度が負の時(左旋回のとき)
            while i < turn_count: #定められた旋回回数まで繰り返し
                rcb4.motionPlay(motion_control.parameters.rcb4_turn_left)       #モーション番号13(左旋回)を再生
                while True:     #モーションの再生が終わるまで繰り返し
                    motion_number = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motion_number < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motion_number == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
        else: #旋回角度が正の時(右旋回の時)
            while i < turn_count: #定められた旋回回数まで繰り返し
                rcb4.motionPlay(motion_control.parameters.rcb4_turn_right)     #モーション番号14(右旋回)を再生
                while True:     #モーションの再生が終わるまで繰り返し
                    motion_number = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motion_number < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motion_number == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1


        
        
            