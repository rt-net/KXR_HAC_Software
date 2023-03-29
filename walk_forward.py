import sys
sys.path.append('../Rcb4Lib') #Rcb4Libの検索パスを追加
from Rcb4BaseLib import Rcb4BaseLib            #Rcb4BaseLib.pyの中のRcb4BaseLibが使えるように設定
import global_value as g
import get_body_angle
import math

rcb4 = Rcb4BaseLib()      #rcb4をインスタンス(定義)
        
#rcb4.open('/dev/ttyAMA0',115200,1.3)  #(portName,bundrate,timeout(s))
rcb4.open('/dev/ttyUSB0',115200,1.3)

walkStepDist = 38

def main(walkDist):
    i = 0 #繰り返しに使うiの初期値設定
    if rcb4.checkAcknowledge() == True: 
        walkCount = abs(round(walkDist/walkStepDist)) #旋回動作を行う回数を確定
        while i < walkCount:
            BodyAngle = get_body_angle.main()
            rcb4.motionPlay(1)
            while True:     #モーションの再生が終わるまで繰り返し
                motionNum = rcb4.getMotionPlayNum()   #現在再生されているモーション番号を取得
                if motionNum < 0:                     #モーション番号が0より小さい場合はエラー
                    break
                if motionNum == 0:                    #モーション番号が0のときは再生されていない状態
                    break
            g.X = g.X +  math.sin(math.radians(BodyAngle[0]))*(walkStepDist)
            g.Y = g.Y +  math.cos(math.radians(BodyAngle[0]))*(walkStepDist)
            
            i = i+1

        
    else:  #通信が返ってきていないときはエラー
        print ('checkAcknowledge error')