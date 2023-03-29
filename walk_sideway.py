import sys
sys.path.append('../Rcb4Lib') #Rcb4Libの検索パスを追加
from Rcb4BaseLib import Rcb4BaseLib            #Rcb4BaseLib.pyの中のRcb4BaseLibが使えるように設定

rcb4 = Rcb4BaseLib()      #rcb4をインスタンス(定義)
        
#rcb4.open('/dev/ttyAMA0',115200,1.3)  #(portName,bundrate,timeout(s))
rcb4.open('/dev/ttyUSB0',115200,1.3)

g.walkStepDist = 16 #歩行動作1回あたりの移動量(cm)
walkStepDist = g.walkStepDist
g.X = 0
g.Y = 0

def main(walkDist): #メイン関数　引数は移動量
    i = 0 #繰り返しに使うiの初期値設定
    if rcb4.checkAcknowledge() == True:  #通信が返ってきたとき
        #print ('MotionPlay(1)')
        stepCount = (round(walkDist/walkStepDist)) #歩行動作を行う回数を確定
        print(stepCount)
        if walkDist < 0: #移動量が負の時(左へ移動のとき)
            stepCount = abs(stepCount)
            while i < stepCount: #定められた歩行回数まで繰り返し
                BodyAngle = get_body_angle.main()
                rcb4.motionPlay(3)       #モーション番号3(左横移動)を再生
                if BodyAngle[0] > 0: #Yaw軸周りの角度が正の時
                    g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(-walkStepDist) #自己位置に[cos(θ)*一歩の移動量 = 負]を加算
                    g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(walkStepDist) #自己位置に[sin(θ)*一歩の移動量]を加算
                else: #Yaw軸周りの角度が負の時
                    g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(-walkStepDist) #自己位置に[cos(θ)*一歩の移動量 = 負]を加算
                    g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(walkStepDist) #自己位置に[sin(θ)*一歩の移動量 = 負]を加算
                while True:     #モーションの再生が終わるまで繰り返し
                    motionNum = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
                
        else: #移動量が正の時(右へ移動の時)
            while i < stepCount: #定められた歩行回数まで繰り返し
                rcb4.motionPlay(4)     #モーション番号4(右横移動)を再生
                BodyAngle = get_body_angle.main()
                if BodyAngle[0] > 0: #Yaw軸周りの角度が正の時
                    g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(walkStepDist) #自己位置に[cos(θ)*一歩の移動量]を加算
                    g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(-walkStepDist) #自己位置に[sin(θ)*一歩の移動量 = 負]を加算
                else: #Yaw軸周りの角度が負の時
                    g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(walkStepDist) #自己位置に[cos(θ)*一歩の移動量]を加算
                    g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(-walkStepDist) #自己位置に[sin(θ)*一歩の移動量]を加算
                while True:     #モーションの再生が終わるまで繰り返し
                    motionNum = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
        
    
    else:  #通信が返ってきていないときはエラー
        print ('checkAcknowledge error')
  
