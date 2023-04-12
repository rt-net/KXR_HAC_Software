import math
import time
import cv2
import numpy as np
import motion_control.parameters as p
import adafruit_bno055
import board
import time


class MotionLibrary:
    X = 0
    Y = 0
    angle = 0
    
    yaworigin = 0
    pitchorigin = 0
    rollorigin = 0
    
    def __init__(self): 
        print("\n[RCB4初期化中]")
        import sys
        sys.path.append('../Rcb4Lib') #Rcb4Libの検索パスを追加

        from Rcb4BaseLib import Rcb4BaseLib            #Rcb4BaseLib.pyの中のRcb4BaseLibが使えるように設定

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
            self.yaworigin = self.sensor.euler[0] 
            if self.yaworigin > 180:
                self.yaworigin = (self.yaworigin-180)-180
            self.pitchorigin = self.sensor.euler[1]
            self.rollorigin = self.sensor.euler[2]

        except:
            self.yaworigin = 0
            self.pitchorigin = 0
            self.rollorigin = 0
             
    def get_body_angle(self): #IMUによって機体の絶対角度を得る関数　戻り値は3要素のタプル

        try:
            yaw = self.sensor.euler[0] - self.yaworigin #Yaw軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
            if yaw > 180: #値が180度を超えている場合
                yaw = (yaw-180)-180 #±180度の値に補正する
                
            pitch = self.sensor.euler[1] -self.pitchorigin #Pitch軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
            roll = self.sensor.euler[2] -self.rollorigin #Roll軸の値を読み込む　キャリブレーションの補正値が存在する場合は補正を行う
            BodyAngle = (yaw, pitch, roll) #3要素を格納

        except: #センサー値の取得が上手くいかなかったとき
            print("sensor error") 
            BodyAngle = (0, 0, 0) #全ての要素に0を格納
            
        print(yaw)
        return BodyAngle #戻り値は3軸の角度のタプル
        
            
    def walk_forward(self, walkDist):
        i = 0 #繰り返しに使うiの初期値設定
        
        stepCount = abs(round(walkDist/p.walkStepDist)) #旋回動作を行う回数を確定
        
        while i < stepCount:
            #BodyAngle = get_body_angle.main()
            self.rcb4.motionPlay(1)
            
            self.angle = self.get_body_angle()[0]
            self.X = self.X +  math.sin(math.radians(self.angle))*(p.walkStepDist) #フィールド座標系に置ける自身の位置を更新
            self.Y = self.Y +  math.cos(math.radians(self.angle))*(p.walkStepDist) #フィールド座標系に置ける自身の位置を更新
            
            print("X: ", self.X, "Y: ", self.Y)
            
            while True:     #モーションの再生が終わるまで繰り返し
                motionNum = self.rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                    break
                if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                    break
                

            
            i = i+1
            
    def walk_sideway(self, walkDist):
        i = 0 #繰り返しに使うiの初期値設定
        
        stepCount = (round(walkDist/p.walkSideStepDist)) #歩行動作を行う回数を確定
        #print(stepCount) #歩行動作繰り返し回数を表示
        
        if walkDist < 0: #移動量が負の時(左へ移動のとき)
            
            stepCount = abs(stepCount)
            while i < stepCount: #定められた歩行回数まで繰り返し
                self.angle = self.get_body_angle()[0] #IMUの価を読み込む
                
                rcb4.motionPlay(3)       #モーション番号3(左横移動)を再生
                
                if BodyAngle[0] > 0: #Yaw軸周りの角度が正の時
                    self.X = self.X + math.cos(math.radians(self.angle))*(-walkSideStepDist) #自己位置に[cos(θ)*一歩の移動量 = 負]を加算
                    self.Y = self.Y + math.sin(math.radians(self.angle))*(walkSideStepDist) #自己位置に[sin(θ)*一歩の移動量]を加算
                else: #Yaw軸周りの角度が負の時
                    self.X = self.X + math.cos(math.radians(self.angle))*(-walkSideStepDist) #自己位置に[cos(θ)*一歩の移動量 = 負]を加算
                    self.Y = self.Y + math.sin(math.radians(self.angle))*(walkSideStepDist) #自己位置に[sin(θ)*一歩の移動量 = 負]を加算
                
                while True:     #モーションの再生が終わるまで繰り返し
                    motionNum = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                
                i = i+1
            
                
        else: #移動量が正の時(右へ移動の時)
            
            while i < stepCount: #定められた歩行回数まで繰り返し
                self.angle = self.get_body_angle()[0] #IMUの価を読み込む
                
                rcb4.motionPlay(4)     #モーション番号4(右横移動)を再生
                
                if BodyAngle[0] > 0: #Yaw軸周りの角度が正の時
                    self.X = self.X + math.cos(math.radians(self.angle))*(walkSideStepDist) #自己位置に[cos(θ)*一歩の移動量]を加算
                    self.Y = self.Y + math.sin(math.radians(self.angle))*(-walkSideStepDist) #自己位置に[sin(θ)*一歩の移動量 = 負]を加算
                else: #Yaw軸周りの角度が負の時
                    self.X = self.X + math.cos(math.radians(self.angle))*(walkSideStepDist) #自己位置に[cos(θ)*一歩の移動量]を加算
                    self.Y = self.Y + math.sin(math.radians(self.angle))*(-walkSideStepDist) #自己位置に[sin(θ)*一歩の移動量]を加算
                    
                while True:     #モーションの再生が終わるまで繰り返し
                    motionNum = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                
                i = i+1
                
    
    else:  #通信が返ってきていないときはエラー
        print ('checkAcknowledge error')


    
    
        