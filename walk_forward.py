#coding: UTF-8
import sys
sys.path.append('../Rcb4Lib') #Rcb4Libの検索パスを追加

from Rcb4BaseLib import Rcb4BaseLib            #Rcb4BaseLib.pyの中のRcb4BaseLibが使えるように設定

import time                   #timeが使えるように宣言

rcb4 = Rcb4BaseLib()      #rcb4をインスタンス(定義)
    
#rcb4.open('/dev/ttyAMA0',115200,1.3)  #(portName,bundrate,timeout(s))
rcb4.open('/dev/ttyUSB0',115200,1.3)


def main():
    if rcb4.checkAcknowledge() == True:  #通信が返ってきたとき

        #print('walking forward')
        #KRCの上ボタンを擬似的に押す
        rcb4.setKrrButtonData(Rcb4BaseLib.KRR_BUTTON.UP.value)

    else:  #通信が返ってきていないときはエラー
        print ('checkAcknowledge error')
   
   
    #rcb4.close()

