import sys
sys.path.append('../Rcb4Lib') #Rcb4Libの検索パスを追加
from Rcb4BaseLib import Rcb4BaseLib            #Rcb4BaseLib.pyの中のRcb4BaseLibが使えるように設定

rcb4 = Rcb4BaseLib()      #rcb4をインスタンス(定義)
        
#rcb4.open('/dev/ttyAMA0',115200,1.3)  #(portName,bundrate,timeout(s))
rcb4.open('/dev/ttyUSB0',115200,1.3)

turnStepDeg = 25 #旋回動作1回あたりの旋回角度(°)

def main(turnDeg): #メイン関数　引数は旋回角度
    i = 0 #繰り返しに使うiの初期値設定
    if rcb4.checkAcknowledge() == True:  #通信が返ってきたとき
        #print ('MotionPlay(1)')
        turnCount = abs(round(turnDeg/turnStepDeg)) #旋回動作を行う回数を確定
        if turnDeg < 0: #旋回角度が負の時(左旋回のとき)
            while i < turnCount: #定められた旋回回数まで繰り返し
                rcb4.motionPlay(13)       #モーション番号13(左旋回)を再生
                while True:     #モーションの再生が終わるまで繰り返し
                    motionNum = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
                
        else: #旋回角度が正の時(右旋回の時)
            while i < turnCount: #定められた旋回回数まで繰り返し
                rcb4.motionPlay(14)     #モーション番号14(右旋回)を再生
                while True:     #モーションの再生が終わるまで繰り返し
                    motionNum = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                    if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                        break
                    if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                        break
                i = i+1
        
    
    else:  #通信が返ってきていないときはエラー
        print ('checkAcknowledge error')
  
