import sys
sys.path.append('../Rcb4Lib') #Rcb4Libの検索パスを追加
from Rcb4BaseLib import Rcb4BaseLib            #Rcb4BaseLib.pyの中のRcb4BaseLibが使えるように設定

rcb4 = Rcb4BaseLib()      #rcb4をインスタンス(定義)
        
#rcb4.open('/dev/ttyAMA0',115200,1.3)  #(portName,bundrate,timeout(s))
rcb4.open('/dev/ttyUSB0',115200,1.3)

walkStepDist = 30 #歩行動作1回あたりの移動量(cm)

def main(walkDist): #メイン関数　引数は移動量
    i = 0 #繰り返しに使うiの初期値設定
    if rcb4.checkAcknowledge() == True:  #通信が返ってきたとき
        #print ('MotionPlay(1)')
        stepCount = abs(round(walkDist/walkStepDist)) #歩行動作を行う回数を確定
        if walkStepDist < 0: #移動量が負の時(左へ移動のとき)
            while i < stepCount: #定められた歩行回数まで繰り返し
                rcb4.motionPlay(3)       #モーション番号3(左横移動)を再生
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
                while True:     #モーションの再生が終わるまで繰り返し
                    motionNum = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
        
    
    else:  #通信が返ってきていないときはエラー
        print ('checkAcknowledge error')
  
