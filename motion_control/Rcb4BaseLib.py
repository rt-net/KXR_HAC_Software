
#coding: UTF-8

##
##	@file Rcb4BaseLib.py
##	@brief RCB4 base library
##	@author Kondo Kagaku Co.,Ltd.
##			T.Nobuhara 
##	@version 1.0.0B
##	@date	2019/02/04
##	@copyright &copy; Kondo Kagaku Co.,Ltd. 2019
##	@copyright [MIT License](http://opensource.org/licenses/mit-license.php)
##

## @mainpage Rcb4Libの概要
## このライブラリは近藤科学製ロボット用コントロールボード(RCB4)をRaspberryPiやLinux等と通信するためのpythonプログラムになります<br>
## <A HREF= "https://kondo-robot.com/product/hearttoheart4">HeartToHeart4</A> Ver2.2以降に対応しています。<br>
## ロボットの初期設定やモーションはHeartToHeart4を使用して作成してください<br>
## 使い方および詳細は、下記弊社HPをご覧ください。<br>
## <A HREF="http://kondo-robot.com/">http://kondo-robot.com/</A><br>
## 不具合等ありましたら、弊社HPを参照にご連絡ください。<br>




from enum import Enum
import serial
from struct import *
import struct     

##	@class	Rcb4BaseLib
#	@brief	RCB4を動かすため初期設定やコマンドをまとめたクラスです。
class Rcb4BaseLib:

	##	@class	CmdOkType
	#	@brief	コマンドがOKかNGか判定する		
    class CmdOkType(Enum):
        Ok    =  0
        Error = -1

	##	@class	AckType
	#	@brief	返信データのACKの数値の定義		
    class AckType(Enum):
        Ack  = 0x06 
        Nack = 0x15 

	#//コマンドの列挙型
	##	@class	CommandTypes
	#	@brief	コマンドの列挙型		
    class CommandTypes(Enum):
        Move            = 0x00
        Jump            = 0x0B
        Call            = 0x0C
        SingleServo     = 0x0F
        ConstFrameServo = 0x10
        ServoParam      = 0x12
        AckCheck        = 0xFE
        _None            = 0xFF

	##	@class	SubMoveCmd
	##	@brief	Moveのサブコマンドの列挙型
    class SubMoveCmd(Enum):
        RamToCom     = 0x20	##!	RAMの値をCOMへ
        ComToRam     = 0x02	##!	COMからの数値をRAMへ
        DeviceToCom  = 0x21	##!	Device(ICS)の値をCOMへ
        ComToDevice  = 0x12	##!	COMからの数値をDevice(ICS)へ

	##	@class	RamAddr
	#	@brief	RCB4のRAMのアドレス一覧
    class RamAddr(Enum):
        ConfigRamAddress         = 0x0000		#//システム設定RAMアドレス(RAM)（0x00固定）
        ProgramCounterRamAddress = 0x0002		#//プログラムカウンター設定アドレス(RAM)（0x02固定）
        AdcRamAddress            = 0x0022		#//AD変換器0の値のアドレス(0x22固定)
        PioModeAddres	         = 0x0038		#//PIOの入出力設定(2017/07/26追記) 
        PioAddress               = 0x003A		#//PIOポート値
        KrrButtonDataAddress     = 0x0350		#//KRRのボタンデータが記録されているアドレス
        KrrPa1Address            = 0x0352		#//PA1データ(この後2byteずつデータが続く)
        CounterRamAddress        = 0x0457		#//後2byteずつデータが続く)
        UserParmeterRamAddress   = 0x0462		#//(この後2byteずつデータが続く)

	##	@class	RomAddr
	#	@brief	RCB4のROMのアドレス一覧
	
	#//ROMに保存されている先頭アドレス一覧)
	#enum RomAddr: long
    class RomAddr(Enum):
        StartupCmdRomAddress = 0x0444	#//スタートアップモーションを再生するコマンドのアドレス
        MainLoopCmd          = 0x044B	#//メインループが回る最初のアドレス
        MotionRomAddress     = 0x0b80

	##	@class	DeviceAddrOffset
	#	@brief	RCB4のデバイスデータのアドレス一覧
	
	#	//デバイスアドレスのオフセットの定義(基準アドレスからどれくらい離れているか)
	#	enum DeviceAddrOffset:byte
    class DeviceAddrOffset(Enum):
        ategoryAddressOffset      = 0x00		#//システム設定RAMアドレス(RAM)（0x00固定）
        IDAddressOffset            = 0x01		                  #//RAM、ROM上のIDデータの位置
        TrimAddressOffset          = 0x02		                  #//RAM、ROM上のトリムデータの位置
        MotorPositionAddressOffset = 0x04		#//RAM、ROM上の実測値データの位置
        PositionAddressOffset      = 0x06		#//RAM、ROM上のポジションデータの位置
        FrameAddressOffset         = 0x08		#//RAM、ROM上のフレーム数データ（補間速度・スピード）の位置
        Mixing1AddressOffset       = 0x0E		#//(14)RAM、ROM上のミキシング１データの位置
        Mixing1RatioAddressOffset  = 0x10		#//(16)RAM、ROM上のミキシング１倍率データの位置
        Mixing2AddressOffset       = 0x11		#//(17)RAM、ROM上のミキシング2データの位置
        Mixing2RatioAddressOffset  = 0x13						#//(19)RAM、ROM上のミキシング2データの位置

	##	@class	KRR_BUTTON
	##	@brief	KRRから送られてきたボタンの番号一覧
	
	#//KRR用のボタンの定義
	#enum KRR_BUTTON : unsigned short
    class KRR_BUTTON(Enum):
        NONE     = 0x0000		#//何も押されていない
        UP       = 0x0001		#//↑
        DOWN     = 0x0002		#//↓
        RIGHT    = 0x0004		#//→
        LEFT     = 0x0008		#//←
        TRIANGLE = 0x0010		#//△
        CROSS    = 0x0020		#//×
        CIRCLE   = 0x0040		#//○
        SQUARE   = 0x0100		#//□
        S1       = 0x0200		#//シフト1 左手前
        S2       = 0x0400		#//シフト2 左奥
        S3       = 0x0800		#//シフト3 右手前
        S4       = 0x1000		#//シフト4 右奥
        FALSE    = 0xFFFF		#//エラー値(受信失敗等)


	##	@class	ConfigData
	##	@brief	コンフィグデータのデータ一覧
    class ConfigData(Enum):
        EnableSio            =0x0001  #//b0:ICSスイッチ
        EnableRunEeprom      =0x0002  #//b1: EEPROMプログラム実行スイッチ
        EnableServoResponse  =0x0004  #//b2: 補間動作終了メッセージスイッチ
        EnableReferenceTable =0x0008  #// b3:ベクタジャンプスイッチ
        Frame                =0x0030  #// b4, b5 出力周期レジスタ[4:5]
        Baudrate             =0x00c0  #// b6, b7 COMボーレートレジスタ[6:7]※何かとかぶらないか確認
        ZeroFlag             =0x0100  #/// b8 ゼロフラグ
        CarrayFlag           =0x0200  #/// b9 キャリーフラグ
        ProgramError         =0x0400  #/// b10 プログラムエラーフラグ
        RFU                  =0x1800  #//b11,b12 未使用
        IcsBaudrate          =0x6000  #//b13,b14 ICSスイッチボーレートレジスタ[13:14] ※何かとかぶらないか確認
        GreenLED             =0x8000  #/// b15 LEDレジスタ
    ########################################################################################

	##	@class	ServoData
	##	@brief	サーボモータ1つを定義するクラス
    class ServoData:
        
        ##	@brief	サーボモータのIDを格納しておく変数
        Id = 0          #add 2018/10/19
		##	@brief	サーボモータに接続されているSIOの情報を格納しておく変数
		#   @param SIO1-4	1
		#   @param SIO5-8	2
        Sio = 0         #add 2018/10/19
        ##	@brief	データを格納しておく変数
        #	@note	変数はポジションデータだけでなく、スピードやストレッチ等のデータとしても使用できる
        Data = 0        #add 2018/10/19
        
        ##	@brief __init__ コンストラクタ
        #	@param id	サーボモータに設定してあるID
        #	@param sio	RCB4に接続されているSIOの番号(SIO1-4:0x01,SIO5-8:0x02)
        #	@param data	保存しておくデータ
        def __init__(self,id,sio,data):
            self.Id = id
            self.Sio = sio
            self.Data = data
                                                                                                                                                                                                                                                                       #self.IcsNo = id * 2 + (sio -1)
        ##	@brief icsNum2id idとSIOからRCB4のICS番号を返す
        #	@retval	RCB4のICS番号
        def icsNum2id(self):
            return self.Id * 2 + (self.Sio - 1)

        ##	@brief itemAdd ServoDataに対する内部の値をすべて変更する
        #	@param x	サーボモータに設定してあるID
        #	@param y	RCB4に接続されているSIOの番号(SIO1-4:0x01,SIO5-8:0x02)
        #	@param z	保存しておくデータ
        def itemAdd(self,x,y,z):
            self.Id = x
            self.Sio = y
            self.Data = z
            #self.IcsNo = self.Id * 2 + (self.Sio- 1)
         
        def __lt__(self,other):
             return (self.Id * 2 + (self.Sio- 1)) < (other.Id * 2 + (other.Sio -1))
########################################################################################

	##	@brief	バージョン番号
    Version = 220
    
    ##	@brief	AD変換器のチャンネル数
    AdcCount = 11        			
    
    ##	@brief	AD変換器単体のデータバイト数
    AdcSingleDataCount = 2 		
    
    ##	@brief	AD変換器全体のデータバイト数
    AdcDataCount = AdcCount * AdcSingleDataCount
    
	##	@brief	SIO1-4の定義値
    SIO1_4 = 0x01
    
    ##	@brief	SIO5-8の定義値
    SIO5_8 = 0x02	

	##	@brief	1モーション当たりデータ数
    MotionSingleDataCount = 2048	
    
    ##	@brief	モーションの数
    MaxMotionCount = 120	
    
    ##	@brief	モーションデータ全体の最大バイト数
    MotionDataCount = 2048 * 120
    
    ##	@brief	ユーザカウンタのbyte数
    CounterSingleDataCount = 1	
    
    ##	@brief	ユーザカウンタの数
    CounterCount = 10
    
    ##	@brief	ユーザ変数のbyte数
    UserParmeterSingleDataCount = 2
    
    ##	@brief	ユーザ変数の数
    UserParmeterCount = 20
    
    ##	@brief	ICSデバイスの数
    #	@note	デバイスは本当は36個あるが、KRRは含めない
    IcsDeviceSize = 35 #2018/12/27 本当は36だが、KRRは含めない
    
    ##	@brief	ICSデバイスの１つのデータ数(byte)
    IcsDeviceDataSize = 20 #2018/12/27追記
    
    ##	@brief	COMのデバイス名
    com = 0
    
    
    __isSynchronize = False    
    __configData = 0          #2018/10/19


	#///////////////////////////////
	#	ここから関数一覧
	#///////////////////////////////
	
    #////////////////////////////////////////////////////////////////////
    #//		コンストラクタ
    #//
	##	@brief	コンストラクタ
    def _init_(self):
        __isSynchronize = False
        __configData = 0


	#//////////////////////////////////////////////////////////////////////////
	#//	チェックサムを計算
    ##	@brief	チェックサムを計算する
    #	@param	dataBytes	データのbyte列
    #	@retval	計算されたチェックサム
    #	@note	引数のbyte配列はコマンドが入っていて先頭にはデータ数が入っています。
    #			そのデータ数を用いて配列数を判別しています。
    @staticmethod   #add 2018/10/19
    def CheckSum(dataBytes):
        sum = 0
        for  i in range(dataBytes[0] -1):
            sum = sum + dataBytes[i]
        return sum & 0xff

    #////////////////////////////////////////////////////////////////////
    #//	コマンドのチェックサムを計算し代入する
    ##	@brief	コマンドのデータ配列の最後にチェックサムを代入する
    #	@param	dataBytes	送信するデータ配列
    #	@retval	True	挿入成功	
    #	@retval	False	挿入失敗
    #	@warning	送信コマンドのみにしてください	
    def setCheckSum(self,dataBytes):
        if dataBytes[0] == 0:
            return False
        else:
            dataBytes[dataBytes[0] -1 ] = self.CheckSum(dataBytes)
        return True

    #////////////////////////////////////////////////////////////////////
    #//	チェックサムがあっているか確認する
    #//
    ##	@brief	取得したデータのチェックサムを確認する
    #	@param	dataBytes	受信したデータ配列
    #	@retval	True:チェックサムがあっている
    #			False:チェックサムが違っている
    #	@note	取得したデータの最後に付与されているチェックサムがあっているか確認をします
    def __checkCheckSum(self,dataBytes):
        if dataBytes[0] == 0:
            return False
        else:
            if dataBytes[dataBytes[0] -1 ] == self.CheckSum(dataBytes):
                return True
            else:
                return False


    #/////////////////////////////////////////////////////////////////////////////////////////
    #//通信関係
    #/////////////////////////////////////////////////////////////////////////////////////////
    
    #////////////////////////////////////////////////////////////////////
    #//	実際に送受信を行う
    #//
    ##	@brief	実際に送受信を行う
    #	@param	txBuf	送信データの配列
    #	@param	rxLen	受信データ数
    #	@retval	rxbuf	受信データ
    #					配列数0の場合は失敗になります
    #	@retval	rxs		データの受信数
    #	@note	実際に同期した送受信を行います。
    #			失敗した場合は配列0の何も入っていないデータを返します
    #			コマンドを受信し終わった後受信コマンドの合否及びチェックサムの判定もします。
    #
    def synchronize(self,txBuf,  rxLen):
        sendbuf=[256]
        if self.__isSynchronize == False:
            sendbuf.clear()
            
            #送られてきたデータのチェック
            if (len(txBuf) == 0 or rxLen <= 3):
                rxbuf = [] #error
                return	rxbuf
            
            #データのコピーとbyteかどうかの判定
            for i in range(len(txBuf)):
                if txBuf[i] < 0 or 255 < txBuf[i]:
                    rxbuf = [] #error
                    return	rxbuf         
                sendbuf.append(txBuf[i])
            
            
            self.__isSynchronize = True
            
            #print('sendData-->',sendbuf)
            
            self.com.flushInput()#buff clr
            self.com.write(sendbuf)
            rxbuf = self.com.read(rxLen)
            self.com.flushInput()#buff clr
            if len(rxbuf) == rxLen and rxbuf[0] == rxLen:
                if self.__checkCheckSum(rxbuf) == False:
                   rxbuf = [] #error
            else:
                rxbuf = [] #error
            self.__isSynchronize = False
            #print('readData-->',rxbuf)
            return	rxbuf
        else:
            rxbuf = [] #error
            return	rxbuf

    #////////////////////////////////////////////////////////////////////
    #//	Serialポートを設定をして開く
    #//
	##	@brief	SerialポートをRCB-4用に設定をして開く
    #	@param	comName	ポートの名前
    #	@param	bundrate	通信速度
    #	@param	timOut	受信タイムアウト
    #	@retval	True：ポートを開いてコンフィグデータを取得できた
    #	@retval	False:ポートを開けなかったか、コンフィグデータを取得できなかった
    #	@note	Serialポートを開く
    #			通信ができるかどうかACKコマンドを送る
    #			コンフィグデータを取得する
    #			上のどれかが失敗したらエラーを返す
    def open(self,comName,bundrate,timOut):
        if self.com == 0:
            try:
                self.com = serial.Serial(comName,bundrate,parity='E',stopbits =1,timeout=timOut)
                self.com.flushInput()#
                if self.checkAcknowledge() == True:
                    #ACKが成功した場合Configデータを取得
                    confData = self.getConfig()
                    #confDataは0xFFFFの場合はエラー
                    if(confData == 0xFFFF):
                        return False
                    else:
                        self.__configData = confData
                        return True
                else:
                    return False
                                
            except:        
                return False
        else:
            return False

    #////////////////////////////////////////////////////////////////////
    #//	Serialポートを閉じる
    #//
    ##	@brief	Serialポートを閉じる
    #	@retval	Serialポートを閉じるのに失敗したらエラーを返す
    def close(self):
        try:
            self.com.close()
            self.com = 0
            return 0
        except:
            return -1


    #////////////////////////////////////////////////////////////////////
    #//	送受信後ＡＣＫ判定
    #//
    ##	@brief	送受信後ＡＣＫ判定
    #	@param	txData	ACKコマンドが入ったデータ配列
    #	@retval	ACKが正常に返ってきたかどうか判断
    #	@note	ACK限定の送受信関数になります
    def synchronizeAck(self,txData):
        rxbuf = self.synchronize(txData, 4)
        if len(rxbuf) > 3 and self.AckType.Ack.value == rxbuf[2]:
            return True
        else:
            return False



    #/////////////////////////////////////////////////////////////////////////////////////////
    #//	コマンド関係
    #/////////////////////////////////////////////////////////////////////////////////////////
    
    #////////////////////////////////////////////////////////////////////
    #//	ACKコマンドの生成
    #//
    ##	@brief	ACKコマンドの生成
    #	@retval	returnDataSize	ACKコマンドの返信データ数
    #	@retval	txbuf	ACK用のコマンドデータ配列
    #	@note	ACKのコマンドを生成します
    #	@note	静的関数で外部アクセスが可能
    @staticmethod   #add 2018/10/19
    def acknowledgeCmd ():
        txbuf = [0x04, Rcb4BaseLib.CommandTypes.AckCheck.value, Rcb4BaseLib.AckType.Ack.value,0]
        txbuf[3] = Rcb4BaseLib.CheckSum(txbuf)
        
        returnDataSize = 4 #ACKコマンドが返ってくる
        
        return returnDataSize,txbuf


    #////////////////////////////////////////////////////////////////////
    #//	ACKコマンドを送って通信ができているかどうか確認を行う
    #//
    ##	@brief	ACKコマンドを送って通信ができているかどうか確認を行う
    #	@retval	True	ACKコマンドが正常に返ってきた
    #	@retval	False	ACKコマンドが正常でなかった
    def checkAcknowledge(self):
    	reSize,txbuf = self.acknowledgeCmd()
    	rxbuf = self.synchronize(txbuf,reSize)
    	if len(rxbuf) == 0:
    		return False
    	else:
        	return True


    #////////////////////////////////////////////////////////////////////
    #//	COMからRAMに転送するコマンドを作成する
    #//
    ##	@brief	COMからRAMに転送するコマンドを作成する(COM ==> RAM)
    #	@param	destAddr	書き換えるRAMのデータアドレス
    #	@param	destData	書き換えるデータ配列(1byteも可能)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	COM(外部)からRAMにデータを書き込むコマンドを作成します
    #	@note	静的関数で外部アクセスが可能
    @staticmethod   #add 2018/10/19
    def moveComToRamCmd(destAddr, destData):
        txbuf =[]
        if len(destData) > 249:
            return 0,Rcb4BaseLib.CmdOkType.Error.value,buf
#        buf.clear() 
        txbuf.append ((len(destData) + 7) & 0xff)
        txbuf.append ( Rcb4BaseLib.CommandTypes.Move.value)
        txbuf.append ( Rcb4BaseLib.SubMoveCmd.ComToRam.value)
        txbuf.append ( destAddr & 0xff)
        txbuf.append (  (destAddr >> 8 ) & 0xff)
        txbuf.append ( 0x00)
        
        if len(destData) == 1:
            txbuf.append (destData[0])
        else:    
            for i in range(len(destData)):
                txbuf.append (destData[ i ])
        
        txbuf.append (Rcb4BaseLib.CheckSum(txbuf))
        
        returnDataSize = 4 #ACKコマンドが返ってくる
        
        return returnDataSize,txbuf


    #///////////////////////////////////////////////////////////////////////////////////
    #//	moveComToRamCmdからACK受信までの関数
    #//
    ##	@brief	COMからRAMにデータを転送します(COM ==> RAM)
    #	@param	scrAddr	書き換えるRAMの転送アドレス
    #	@param	destData	書き換えるデータ配列(1byteも可能)
    #	@retval	True	データが正常に転送できた
    #	@retval	False	データが正常に転送できなかった
    #	@note	COM(外部)からRAMのデータを書き換えます
    def moveComToRamCmdSynchronize(self,scrAddr, destData):
        
         readSize, sendData = self.moveComToRamCmd(scrAddr ,destData)
         if readSize > 0:
            rxbuf = self.synchronize(sendData,  readSize)
            if len(rxbuf) > 3 and Rcb4BaseLib.AckType.Ack.value ==rxbuf[2]:
                return True
            else:
                return False
         else:
            return False
 
 
    #///////////////////////////////////////////////////////////////////////////////////
    #//RAMの状態をCOMに転送するコマンド　ﾊﾞｯﾌｧｰにセット
    #//
    ##	@brief	RAMからCOMにデータを出力するコマンドを作成します(RAM ==> COM)
    #	@param	scrAddr	取得するデータの先頭アドレス
    #	@param	scrDataSize	取得するデータbyte数
    #	@retval	returnDataSize	受信データ数
    #	@retval	txbuf	送信データコマンドデータ配列
    #	@note	RAM上のデータをCOMに転送するコマンドを作成します
    @staticmethod   #add 2018/10/19
    def moveRamToComCmd(scrAddr, scrDataSize):
        txbuf =[]
        txbuf.append (0x0a)
        txbuf.append ( Rcb4BaseLib.CommandTypes.Move.value)
        txbuf.append ( Rcb4BaseLib.SubMoveCmd.RamToCom.value)
        txbuf.append ( 0x00)
        txbuf.append ( 0x00)
        txbuf.append ( 0x00)
        txbuf.append ( scrAddr & 0xff)
        txbuf.append (  (scrAddr >> 8 ) & 0xff)
        txbuf.append (scrDataSize)
        txbuf.append (Rcb4BaseLib.CheckSum(txbuf))
        
        returnDataSize = scrDataSize + 3
        
        return returnDataSize,txbuf
 
 
    #///////////////////////////////////////////////////////////////////////////////////
    #//	moveRamToComCmdからデータ受信までの関数
    #//
    ##	@brief	RAM上のデータをCOMに出力します(RAM ==> COM)
    #	@param	scrAddr	取得するデータの先頭アドレス
    #	@param	scrDataSize	取得するデータbyte数
    #	@retval	True:正常にデータが返ってきた	False:正常にデータが返ってこなかった
    #	@retval	コマンドを除いた受信したデータ
    #	@note	RAM上のデータをCOMに転送(出力)します(RAM ==> COM)
    #	@note	失敗した場合はFalseと同時に空のデータ配列をかえします。
    def moveRamToComCmdSynchronize(self,scrAddr, scrDataSize):
        readSize, sendData  = self.moveRamToComCmd(scrAddr ,scrDataSize)
        
        #送信データがうまく作れた 
        if readSize > 0:
            rxbuf = self.synchronize(sendData, readSize)
            #正常にデータが返っていないときはエラーを返す
            if len(rxbuf)< readSize:
                rxbuf = []
                return False,rxbuf
            
                        #成功の場合は受信データのみを入れる
            destData = []
            if scrDataSize == 1: #1byte のみのとき                
                destData.append (rxbuf[2])
            
            else:    #複数byteのとき                
                for i in range(scrDataSize):
                    destData.append (rxbuf[i+2])                    
                                       
            return True, bytes(destData) 
            
        #送信データがうまく作れなかった
        else:
            rxbuf =[]
            return False,rxbuf


    #///////////////////////////////////////////////////////////////////////////////////
    #//	COMからDeviceにデータを転送するコマンドを生成する　ﾊﾞｯﾌｧｰにセット
    #//
    ##	@brief	COMからDevice部分にデータを転送するコマンドを生成します(COM ==> Device)
    #	@param	icsNum	書き込むICSデバイスの番号(IDとSIOから計算した値)
    #	@param	offset	書き込むデータのオフセットアドレス	
    #	@param	destData	書き込むデータ
	#	@return	(returnDataSize,txbuf)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	COMから指定したICS番号の指定した部分にデータを転送をするコマンドを生成します(COM ==> Device)
    #	@note	静的関数で外部アクセスが可能
    #	@note	icsNum => icsNum2id()
    #	@note	offset => Rcb4BaseLib.DeviceAddrOffset
    @staticmethod   #add 2018/10/19
    def moveComToDeviceCmd (icsNum, offset, destData):

        buf =[]
        destSize = len(destData)
        if((icsNum < 0)or(Rcb4BaseLib.IcsDeviceSize < icsNum)or (offset < 0) or(Rcb4BaseLib.IcsDeviceDataSize  <(offset+destSize))):
            return -1,buf
 
        buf.append (0x07 + len(destData))
        buf.append ( Rcb4BaseLib.CommandTypes.Move.value)
        buf.append ( Rcb4BaseLib.SubMoveCmd.ComToDevice.value)
        buf.append ( offset)
        buf.append ( icsNum)
        buf.append ( 0x00)
        if len(destData) == 1:
             buf.append (destData[0])
        else:
            for i in range(len(destData)):
                buf.append (destData[ i ])
        buf.append (Rcb4BaseLib.CheckSum(buf))
       # print (txbuf)
        #return len(destData)+7,buf
        return 4,buf #受信数は4byte

 
    #///////////////////////////////////////////////////////////////////////////////////
    #//	COMからDeviceにデータを転送する
    #//
    ##	@brief	COMからDevice部分にデータを転送する
    #	@param	icsNum	書き込むICSデバイスの番号(IDとSIOから計算した値)
    #	@param	offset	書き込むデータのオフセットアドレス	
    #	@param	destData	書き込むデータ
    #	@retval	True:正常にデータが返ってきた	
    #	@retval	False:正常にデータが返ってこなかった
    #	@note	COMから指定したICS番号の指定した部分にデータを転送します(COM ==> Device)
    #	@note	icsNum => icsNum2id()
    #	@note	offset => Rcb4BaseLib.DeviceAddrOffset
    def moveComToDeviceCmdSynchronize(self,icsNum, offset, destData):
        readSize, sendData = self.moveComToDeviceCmd (icsNum, offset, destData)
        if readSize > 0:
            rxbuf = self.synchronize(sendData, readSize)
            #return True,rxbuf
            if len(rxbuf) > 3 and Rcb4BaseLib.AckType.Ack.value ==rxbuf[2]:
                return True
            else:
                return False
        else:
            #rxbuf =[]
            return False


    #///////////////////////////////////////////////////////////////////////////////////
    #//	ICSのデータをCOMに転送するコマンド)　ﾊﾞｯﾌｧｰにセット
    #//
    ##	@brief	Device部分からCOMにデータを転送するコマンドを作成する(Device ==> COM )
    #	@param	icsNum	読み込むICSデバイスの番号(IDとSIOから計算した値)
    #	@param	offset	読み込むデータのオフセットアドレス	
    #	@param	dataSize	読み込むデータ数
	#	@return	(returnDataSize,txbuf)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	Device部分からCOMにデータを転送するコマンドを生成します(Device ==> COM )
    #	@note	静的関数で外部アクセスが可能
    #	@note	icsNum => icsNum2id()
    #	@note	offset => Rcb4BaseLib.DeviceAddrOffset
    @staticmethod   #add 2019/01/09
    def moveDeviceToComCmd(icsNum, offset, dataSize):
        buf =[]
        
        if((icsNum < 0)or(Rcb4BaseLib.IcsDeviceSize < icsNum)or (offset < 0) or(Rcb4BaseLib.IcsDeviceDataSize  <(offset+dataSize))):
            return -1,buf
        
        buf.append (0x0a)
        buf.append ( Rcb4BaseLib.CommandTypes.Move.value)
        buf.append ( Rcb4BaseLib.SubMoveCmd.DeviceToCom.value)
        buf.append ( 0x00)
        buf.append ( 0x00)
        buf.append ( 0x00)
        buf.append ( offset)
        buf.append ( icsNum)
        buf.append (dataSize)
        buf.append (Rcb4BaseLib.CheckSum(buf))
        
        returnDataSize = dataSize+3
        
        return returnDataSize,buf


    #///////////////////////////////////////////////////////////////////////////////////
    #//	moveDeviceToComCmdからデータ受信までの関数
    #//
    ##	@brief	Device部分からCOMにデータを転送する	(Device ==> COM )
    #	@param	icsNum	読み込むICSデバイスの番号(IDとSIOから計算した値)
    #	@param	offset	読み込むデータのオフセットアドレス	
    #	@param	dataSize	読み込むデータ数
    #	@retval	True:正常にデータが返ってきた	False:正常にデータが返ってこなかった
    #	@retval	rxbuf	コマンドを除いた受信したデータ
    #	@note	Device部分からCOMにデータを転送する(Device ==> COM )
    #	@note	失敗した場合はFalseと同時に空のデータ配列をかえします。
    #	@note	icsNum => icsNum2id()
    #	@note	offset => Rcb4BaseLib.DeviceAddrOffset
    def moveDeviceToComCmdSynchronize(self,icsNum, offset, dataSize):

        #moveDeviceToComCmdで送信データを作成
        readSize, sendData  = self.moveDeviceToComCmd(icsNum, offset, dataSize)
        
        #送信データがうまく作れた 
        if readSize > 0:
            rxbuf = self.synchronize(sendData, readSize)
            #正常にデータが返っていないときはエラーを返す
            if len(rxbuf)< readSize:
                rxbuf = []
                return False,rxbuf
            
                        #成功の場合は受信データのみを入れる
            destData = []
            if dataSize == 1: #1byte のみのとき                
                destData.append (rxbuf[2])
            
            else:    #複数byteのとき                
                for i in range(dataSize):
                    destData.append (rxbuf[i+2])                    
                                       
            return True, bytes(destData) 
            
        #送信データがうまく作れなかった
        else:
            rxbuf =[]
            return False,rxbuf



    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定したアドレスにジャンプする　ﾊﾞｯﾌｧｰにセット
    #//
    ##	@brief	指定したROMのアドレスにコールするコマンドを作成する
    #	@param	romAddr	コール先のROMのアドレス
  	#	@return	(returnDataSize,txbuf)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	指定したROMのアドレスにコールするコマンドを作成します
    #	@note	静的関数で外部アクセスが可能
    #	@warning	条件指定なしでコールします
    @staticmethod   #add 2018/10/19
    def callCmd(romAddr):
        buf =[] 
        buf.append (0x07)
        buf.append ( Rcb4BaseLib.CommandTypes.Call.value)
        buf.append ( romAddr & 0xff)
        buf.append ( (romAddr >> 8 ) & 0xff)
        buf.append ( (romAddr >> 16) & 0xff)
        buf.append (0x00)
        buf.append (Rcb4BaseLib.CheckSum(buf))
        return 4,buf


    #///////////////////////////////////////////////////////////////////////////////////////
    #//	サーボを1つだけ動かすコマンドを作成)ﾊﾞｯﾌｧｰにセット
    ##	@brief	サーボモータを１つだけ動かすコマンドを作成します
	#	@param	idNum	サーボモータのID番号
	#	@param	sioNum	SIOのつながっている番号
	#	@param	pos		サーボモータのポジションデータ(0(FREE),0xFFFF(Hold),3500-11500)
	#	@param	frame	フレーム周期
  	#	@return	(returnDataSize,txbuf)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	サーボモータを１つだけ動かします
    #	@note	静的関数で外部アクセスが可能
    #	@note	sioNum	SIO1-4	,	SIO5-8
    @staticmethod   #add 2018/10/19
    def runSingleServoCmd ( idNum, sioNum, pos, frame):
        buf =[] 
        buf.append (0x07)
        buf.append ( Rcb4BaseLib.CommandTypes.SingleServo.value)
        buf.append ( Rcb4BaseLib.icsNum2id(idNum, sioNum))
        buf.append (frame)
        buf.append ( pos  & 0xff)
        buf.append ( (pos >> 8) & 0xff)
        buf.append (Rcb4BaseLib.CheckSum(buf))
        return 4,buf


    #///////////////////////////////////////////////////////////////////////////////////
    #//	サーボモータを複数アクセスするときに必要なデータを作る関数
    #//
    ##	@brief	サーボモータを複数アクセスするときに必要なデータを作る関数
    #	@param	servoDatas	サーボモータ型のデータが入力されている配列
    #	@retval	ret	サーボモータを複数アクセスするときに必要なデータ5byte分
    #	@note	必要なデータは、サーボモータを変更する部分にbitを立てる
    @staticmethod   #add 2018/10/19
    def setServoNo (servoDatas):
      ret = 0;
      
      for idat in servoDatas:
        no = Rcb4BaseLib.icsNum2id(idat.Id, idat.Sio)
        sf = 0x1;
        ret |=  (sf << no);
      return ret << 24;


    #///////////////////////////////////////////////////////////////////////////////////
    #//	サーボデータをソートする
    #//
    ##	@brief	サーボモータ型のデータをソートします
    #	@param	servoDatas	ソート前のデータ
    #	@retval	servoDatas	ソート後のデータ(中身が変わっているだけ)
    #	@note	サーボデータ型をICS番号を基準にソートします
    #	@warning	引数のデータは中身が書き換えられるので注意
    def sortServoDatas (self,servoDatas):
        servoDatas = sorted(servoDatas)
        return servoDatas


    #///////////////////////////////////////////////////////////////////////////////////
    #//	送られてきたサーボデータが重複しないか確認をする
    #//
    ##	@brief	送られてきたservoDataの配列に重複がないか確認をします
    #	@param	servoDatas	servoDataのデータは配列
    #	@retval	True	重複がなかった
    #	@retval	False	重複があったもしくはIDに異常値が入っていた
    def checkServoDatas(servoDatas):
        cheakData = []
            
        #とりあえずデータを作る
        for sData in servoDatas:
            #型があっていないとエラーで返す
            if not (type(sData) is Rcb4BaseLib.ServoData):                
                return False
            cData = sData.icsNum2id()
            #ICS Noの範囲内でないとエラーを返す
            if not (0 <= cData  <= Rcb4BaseLib.IcsDeviceSize):                
                return False
            
            cheakData.append(cData)
        
        #最後に重複データがないかを確認    
        if len(servoDatas) == len(set(cheakData)):
            return True
        else:    
            return False


    #///////////////////////////////////////////////////////////////////////////////////
    #//	複数のサーボを動かす　ﾊﾞｯﾌｧｰにセット
    #//
    ##	@brief	複数のサーボモータを同時に動かすコマンドの作成
    #	@param	servoDatas	サーボモータのデータが保存されている配列
    #	@param	frame	サーボモータを動かすフレーム数
   	#	@return	(returnDataSize,txbuf)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	複数のサーボモータを動かすためのコマンドを作成します
    #	@note	静的関数で外部アクセスが可能
    #	@warning	引数のデータは中身が書き換えられるので注意	
    @staticmethod   #add 2018/10/19
    def runConstFrameServoCmd (servoDatas,frame):
        buf =[]
        
        sDatas = []	
        if type(servoDatas) == Rcb4BaseLib.ServoData:	#サーボモータが単体だった時の処理
            sDatas.append(servoDatas)
        else:
            sDatas = servoDatas
        
        if Rcb4BaseLib.checkServoDatas(sDatas) == False:
            return -1,buf
        
        wk = Rcb4BaseLib.setServoNo (sDatas) >> 24
        buf.append (len(sDatas) * 2 + 9)
        buf.append ( Rcb4BaseLib.CommandTypes.ConstFrameServo.value)
        buf.append (wk & 0xff)
        buf.append ((wk >>  8) & 0xff)
        buf.append ((wk >> 16) & 0xff)
        buf.append ((wk >> 24) & 0xff)
        buf.append ((wk >> 32) & 0xff)
        buf.append (frame)
        servoDatas = sorted(sDatas)
        for idat in servoDatas:
          buf.append(idat.Data & 0xff)
          buf.append((idat.Data >> 8)& 0xff)
        buf.append(Rcb4BaseLib.CheckSum(buf))
       # print(len(txbuf),txbuf)
        return 4,buf


    #///////////////////////////////////////////////////////////////////////////////////
    #//	サーボモータのパラメータを変更するコマンドを生成する
    #//
    ##	@brief	サーボモータのパラメータを変更するコマンドを生成する
    #	@param	servoDatas	サーボモータのデータが保存されている配列
    #	@param	servoParameter	サーボモータのどのパラメータを変更するか指定
   	#	@return	(returnDataSize,txbuf)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	サーボモータのパラメータを変更するコマンドを生成します
    #	@note	静的関数で外部アクセスが可能
    #	@note	servoParameter	0x01:ストレッチ	0x02:スピード
    #	@warning	引数のデータは中身が書き換えられるので注意
    @staticmethod   #add 2018/10/19
    def setParametersBaseCmd (servoDatas,servoParameter):
        buf =[]
    
        sDatas = []
        if type(servoDatas) == Rcb4BaseLib.ServoData:
            sDatas.append(servoDatas)
        else:
            sDatas = servoDatas
    
        if Rcb4BaseLib.checkServoDatas(sDatas) == False:
            return -1,buf
        
        wk = Rcb4BaseLib.setServoNo (sDatas) >> 24
        buf.append (len(sDatas) + 9)
        buf.append ( Rcb4BaseLib.CommandTypes.ServoParam.value)
        buf.append (wk & 0xff)
        buf.append ((wk >>  8) & 0xff)
        buf.append ((wk >> 16) & 0xff)
        buf.append ((wk >> 24) & 0xff)
        buf.append ((wk >> 32) & 0xff)
        buf.append (servoParameter)
        servoDatas = sorted(sDatas)
        for idat in sDatas:
            if 0<idat.Data<128:
                buf.append(idat.Data & 0xff)
            else:
                buf = []
                return 0,buf
        buf.append(Rcb4BaseLib.CheckSum(buf))
     #   print(len(txbuf),txbuf)
        return 4,buf


    #//////////////////////////////////////////////////////////////////////////////////////////
    #//サーボモータ
    #//サーボモータ単体を動かす
    ##	@brief	サーボモータの単体を動かす
	#	@param	id	サーボモータのID番号
	#	@param	sio	SIOのつながっている番号
	#	@param	pos		サーボモータのポジションデータ(0x8000(FREE),0xFFFF(Hold),3500-11500)
	#	@param	frame	フレーム周期
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@note	サーボモータを１つだけ動かします
    #	@note	sioNum	SIO1-4	,	SIO5-8
    def setSingleServo (self,id, sio,pos,frame):
        rxSize,txbuf = self.runSingleServoCmd(id, sio, pos, frame)
        if len(txbuf) == 0:
            return False
        return self.synchronizeAck(txbuf)


    #//一度にサーボモータのスピードを指定する　
    #///////////////////////////////////////////////////////////////////////////////////
    #//	一度にサーボモータのスピードを指定するコマンドの作成
    #//
    ##	@brief	一度にサーボモータのスピードを指定するコマンドの作成
    #	@param	servoDatas	サーボモータのデータが保存されている配列
   	#	@return	(returnDataSize,txbuf)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	setParametersBaseCmdのパラメータを２に設定してコマンドを作成している
    #	@note	静的関数で外部アクセスが可能
    @staticmethod   #add 2018/10/19
    def setSpeedCmd (servoDatas):
      return Rcb4BaseLib.setParametersBaseCmd (servoDatas, 2)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	(一度にサーボモータのストレッチを指定する)　ﾊﾞｯﾌｧｰにセット
    #//
    ##	@brief	一度にサーボモータのストレッチを指定するコマンドの作成
    #	@param	servoDatas	サーボモータのデータが保存されている配列
   	#	@return	(returnDataSize,txbuf)
    #	@retval	returnDataSize	返ってくるデータサイズ
    #	@retval	txbuf	コマンド全体のデータ配列
    #	@note	setParametersBaseCmdのパラメータを1に設定してコマンドを作成している
    #	@note	静的関数で外部アクセスが可能
    @staticmethod   #add 2018/10/19
    def setStretchCmd (servoDatas):
      return  Rcb4BaseLib.setParametersBaseCmd (servoDatas, 1)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定した複数のサーボモータを動かす
    #//	
    ##	@brief	複数のサーボモータを同時に動かす
    #	@param	servoDatas	サーボモータのデータが保存されている配列
    #	@param	frame	サーボモータを動かすフレーム数
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@note	複数のサーボモータを動かすためのコマンドを作成します
    #	@note	静的関数で外部アクセスが可能
    #	@warning	引数のデータは中身が書き換えられるので注意	
    def setServoPos (self,servoDatas,frame):
      rxSize,txbuf = self.runConstFrameServoCmd(servoDatas,frame)
      return  self.synchronizeAck(txbuf)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定した複数のサーボモータをフリーにする
    #//
    ##	@brief	指定した複数のサーボモータをフリー状態にする
    #	@param	servoDatas	サーボモータのデータが保存されている配列(使うのはSIO,IDのみ)
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@note	servoDatasのdata部分を0x8000にしてsetServoPos()に送る
    def setFreePos (self,servoDatas):

      if type(servoDatas) is Rcb4BaseLib.ServoData:#ServoData型単体
          return self.setFreeSingleServo (servoDatas.Id, servoDatas.Sio)
      else:
        buf = []
        for idat in servoDatas:
          #  idat.Data = 0x7fff
          if type(idat) is Rcb4BaseLib.ServoData:#ServoDataかどうかの確認
            #元データを上書きしてしまうので、一度コピー
            buf.append(self.ServoData(idat.Id,idat.Sio,0x8000))
            
        if len(buf) <= 0: #データがなかった場合
          return False
        else:
            return  self.setServoPos(buf,1)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定した複数のサーボモータをホールド状態にする
    #//
    ##	@brief	指定した複数のサーボモータをホールド状態にする
    #	@param	servoDatas	サーボモータのデータが保存されている配列(使うのはSIO,IDのみ)
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@note	servoDatasのdata部分を0x7FFFにしてsetServoPos()に送る
    def setHoldPos (self,servoDatas):
      
      if type(servoDatas) is Rcb4BaseLib.ServoData:#ServoData型単体
          return self.setHoldSingleServo (servoDatas.Id, servoDatas.Sio)
      else:
        buf = []
        for idat in servoDatas:
          #  idat.Data = 0x7fff
          if type(idat) is Rcb4BaseLib.ServoData:#ServoDataかどうかの確認
          #元データを上書きしてしまうので、一度コピー
            buf.append(self.ServoData(idat.Id,idat.Sio,0x7FFF))
            
        if len(buf) <= 0: #データがなかった場合
          return False
        else:
            return  self.setServoPos(buf,1)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定した単体のサーボモータをフリーにする
    #//
    ##	@brief	指定した単体のサーボモータを同時にフリーにする
    #	@param	id	サーボモータのID
    #	@param	sio	サーボモータに接続されているSIO
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@note	ポジション部分を0x8000にしてsetSingleServo()に送る
    def setFreeSingleServo (self,id, sio):
      return  self.setSingleServo (id,sio,0x8000,1)
        
    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定した単体のサーボモータをホールドにする
    #//
    ##	@brief	指定した単体のサーボモータをホールドにする
    #	@param	id	サーボモータのID
    #	@param	sio	サーボモータに接続されているSIO
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@note	ポジション部分を0x7FFFにしてsetSingleServo()に送る
    def setHoldSingleServo (self,id, sio):
      return  self.setSingleServo (id,sio,0x7fff,1)



    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定したサーボモータの角度を取得する
    #//
    ##	@brief	指定したサーボモータの角度を取得する
    #	@param	id	サーボモータのID
    #	@param	sio	サーボモータに接続されているSIO
   	#	@return	(flag,posData)
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@retval	posData	指定されたポジションデータ
	#	@note	DeviceAddrOffset.MotorPositionAddressOffset部分に現在位置が入っているのでそこから2byte読み取る
    def getSinglePos(self,id,sio):
      #retf,retbuf = self.moveDeviceToComCmd(self.icsNum2id(id, sio), Rcb4BaseLib.DeviceAddrOffset.MotorPositionAddressOffset.value,2)
      #rxbuf = self.synchronize(retbuf, 5)
      retf,retbuf = self.moveDeviceToComCmdSynchronize(self.icsNum2id(id, sio), Rcb4BaseLib.DeviceAddrOffset.MotorPositionAddressOffset.value,2)

      if(retf == True) and (len(retbuf) ==2):
        #print(retbuf)
        posData  =  struct.unpack('<h',retbuf)[0]
        #print(paraData)
        #return  retbuf[2] + retbuf[3]*256
        return  True,posData
      
      else:
          return False,-1



    #///////////////////////////////////////////////////////////////////////////////////
    #//	一度にサーボモータのスピードを指定する
    #//
    ##	@brief	一度にサーボモータのスピードを指定する
    #	@param	servoDatas	サーボモータのデータが保存されている配列
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@note	setParametersBaseCmdのパラメータを２に設定してコマンドを作成し、送受信をする
    def setServoSpeed (self,servoDatas):
      rxSize,txbuf = self.setSpeedCmd(servoDatas)
      return self.synchronizeAck(txbuf)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	一度にサーボモータのストレッチを指定する
    #//
    ##	@brief	一度にサーボモータのストレッチを指定する
    #	@param	servoDatas	サーボモータのデータが保存されている配列
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
    #	@note	setParametersBaseCmdのパラメータを1に設定してコマンドを作成し、送受信をする
    def setServoStretch (self,servoDatas):
       rxSize,txbuf = self.setStretchCmd(servoDatas)
       return  self.synchronizeAck(txbuf)

    #/////////////////////////////////////////////////////////////////////////////
    #//IDとSIOからICS番号に変換する
    #///////////////////////////////////////////////////////////////////////////////////
    #//
    ##	@brief	IDとSIOからICS番号に変換する
    #	@param	id	サーボモータのID
    #	@param	sio	サーボモータが接続されているSIO番号(SIO1-4:0x01,SIO5-8:0x02)
    #	@retval	ICS番号
    #	@note	IDとSIOからICS番号に変換します
    #	@note	静的関数で外部アクセスが可能
    #	@note	ServoData.icsNum2id()と挙動は同じです
    @staticmethod   #add 2018/10/19
    def icsNum2id(id,  sio):
        return id * 2 + (sio - 1)




    #///////////////////////////////////////////////////////////////////////////////////
    #// configデータを取得する)
    #///////////////////////////////////////////////////////////////////////////////////
  	##	@brief	コンフィグデータを取得する
	#	@retval	config	Configに書かれているデータ
	#	@retval	0xFFFF	データが正常に受信できなかった
	#	@note	ConfigRamAddressに2byteのデータがあるので、moveRamToComで取得する
    def getConfig(self):
        
        retf,rxbuf = self.moveRamToComCmdSynchronize(Rcb4BaseLib.RamAddr.ConfigRamAddress.value ,2)
        
        if (retf == False) or (len(rxbuf) != 2):
            return 0xFFFF
        else:
            return (rxbuf[1] * 256 + rxbuf[0])


    #///////////////////////////////////////////////////////
    #//PIO関係
    #///////////////////////////////////////////////////////
    
    #///////////////////////////////////////////////////////////////////////////////////
    #//	PIOの現在の状態を取得する
    #//
 	##	@brief	PIOの状態を取得する
	#	@retval	pioData	PIO端子の状態のbitデータの集まり[0:L 1:H]
	#	@retval	0xFFFF(-1)	データが正常に受信できなかった
	#	@note	PioAddressに2byteのデータがあるので、moveRamToComで取得する
	#	@note	Pioのデータは10ピンなのでマスクがかかっています
    def getPio(self):
        retf,rxbuf = self.moveRamToComCmdSynchronize(Rcb4BaseLib.RamAddr.PioAddress.value ,2)
        if(retf == True) and (len(rxbuf) == 2):
            return ((rxbuf[1] & 0x03) * 256) + rxbuf[0] #取得したデータ10bit以外はゴミデータなのでマスクをかける
        else:
            return -1
        

    #//PIOの状態を変更する)
    #///////////////////////////////////////////////////////////////////////////////////
    #//	PIOの現在の状態を変更する
    #//
 	##	@brief	PIOの状態(H/L)を変更する
 	#	@param	pioData	PIO端子の状態のbitデータの集まり[0:L 1:H]
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
	#	@note	PioAddressに2byteのデータがあるので、moveComToRamで書き込む
    def setPio (self,pioData):
        buf=[pioData & 0xff,(pioData >> 8) & 0x03] #送られてきた10bit以外はゴミデータなのでマスクをかける
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.PioAddress.value, buf)

    #///////////////////////////////////////////////////////////////////////////////////
    #//	PIO入出力状態を変更する
    #//
 	##	@brief	PIOの入出力を変更する
 	#	@param	pioModeData	PIOの端子の入出力状態せってするbitデータの集まり[0:L 1:H]
	#	@retval	True	正常にデータを送信
	#	@retval	False	データが正常に送信できなかった
	#	@note	PioModeAddresに2byteのデータがあるので、moveComToRamで書き込む
    def setPioMode (self,pioModeData):
        buf=[pioModeData & 0xff,(pioModeData >> 8) & 0x03] #送られてきた10bit以外はゴミデータなのでマスクをかける
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.PioModeAddres.value, buf)

    #////////////////////////////////////////////////////////
    #//AD関係
    #///////////////////////////////////////////////////////
    
    #ADの測定値アドレスを取得する
    #///////////////////////////////////////////////////////////////////////////////////
    #//	ADの測定値保存されているRAMアドレスを取得する
    #//
    ##	@brief	ADの測定値アドレスを取得する
    #	@param	adPort	ADのポート番号
    #	@retval	取得されたAD値
    #	@retval	-1	データの取得失敗
    #	@note	静的関数で外部アクセスが可能
    @staticmethod   #add 2018/12/21
    def adDataAddr(adPort):
        if adPort >= 0 and adPort < 11:
            return Rcb4BaseLib.RamAddr.AdcRamAddress.value + adPort * 2
        else:
            return  -1
    
    #//ADの状態を取得する
    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定されたADポートの状態を取得する
    #//
    ##	@brief	指定されたADポートの状態を取得する
    #	@param	adPort	接続されているアナログ(AD)ポート
    #	@retval	指定されたADポート
    #	@retval	-1(0xFFFF)	データ取得失敗
    #	@note	RAM上のADの保存されているアドレスから2byte RAM=>COMでデータを取得します。
    #	@note	アドレスは、adDataAddr()から取得します。
    #	@note	ADポート0は電源電圧の分圧分が入力されます。
    #	@note	AD変換は0-5.0Vの10bitのAD変換器で変換されます。
    def getAdData(self,adPort):
        if adPort >= 0 and adPort < 11:
            retf,rxbuf = self.moveRamToComCmdSynchronize(self.adDataAddr(adPort), 2 )
            if (retf == True) and (len(rxbuf) == 2):
                return rxbuf[1] * 256 + rxbuf[0]
            
        #ここに来るときは失敗したとき   
        return 0xffff


    #//すべてのADの状態を取得する
    #///////////////////////////////////////////////////////////////////////////////////
    #//	すべてのADポートの状態を取得する
    #//
    ##	@brief	指定されたADポートの状態を取得する
	#	@return	(retf,redate)
    #	@retval	retf	True	取得成功
    #	@retval	retf	False	取得失敗
    #	@retval	redate	11個分のADデータ
    #	@retval	redate	配列が0の場合は失敗
    #	@note	RAM上のADの保存されているアドレスから2byte * 11分 RAM=>COMでデータを取得します。
    #	@note	ADポート0は電源電圧の分圧分が入力されます。
    #	@note	AD変換は0-5.0Vの10bitのAD変換器で変換されます。
    def getAllAdData(self):
        retf,retbuf = self.moveRamToComCmdSynchronize(Rcb4BaseLib.RamAddr.AdcRamAddress.value  ,self.AdcDataCount)
        redate = []
        if (retf == True) and (len(retbuf) == (self.AdcCount * 2)):
            for i in range(self.AdcCount):
                redate.append( retbuf[1+ i*2] * 256 + retbuf[0+ i*2])
        return retf,redate


    #///////////////////////////////////////////////////////////////////////////////////
    #//	RCB4に入力されている電圧値を取得
    #//
    ##	@brief	RCB4に入力されている電圧値を取得します
    #	@retval	入力された電圧値(V)
 	#	@retval	0	取得失敗
    #	@note	入力された電圧値を取得します。
    #	@note	電圧はADポート0に入力されます
    #	@note	電圧は約1/5に分圧されて0-5.0Vを10bitの分解能のAD変換されます。
    def getRcb4Voltage(self):
        retf, rxbuf = self.moveRamToComCmdSynchronize(self.adDataAddr(0), 2 )
        if retf == True:
            #return (rxbuf[3] * 256 + rxbuf[2]) * 0.169
            battData = (rxbuf[1] * 256 + rxbuf[0]) * 5.0 /1024 #読み取ったデータを電圧値に変換
            battData = battData *49/10 #分圧されいていた実際の値をもとの電圧に戻す
            return battData
        else:
            return 0
            

    #///////////////////////////////////////////////////////////////////////////////////
    #//ボタンデータを送付
    #///////////////////////////////////////////////////////////////////////////////////
    
    
    #///////////////////////////////////////////////////////////////////////////////////
    #//	KRRのすべてのデータを設定する
    #//
    ##	@brief	KRRの疑似データをすべて送信します
    #	@param	buttonData	ボタンデータ(KRR_BUTTON)
    #	@param	adData	PA1－4のデータ(byte型の配列)
    #	@retval	True	送信成功
    #	@retval	False	送信失敗
    #	@warning	KRRが接続されているときは、接続されている側のデータで上書きされるため使えません
    #	@warning	adDataに関しては必ず1byte * 4の配列を渡してください
    #	@note	KRRの受信データ部分にデータをすべて入れます
    #	@note	ボタンデータに関してはKRR_BUTTONから作成します
    #	@note	すべてのデータを書き込むには、KRRのボタンデータのアドレスにCOM=>RAMで6byte分書き込みます
    def setKrrData (self,buttonData, adData):
        #buf=[buttonData & 0xff,(buttonData >> 8) & 0xff,adData[0],adData[1],adData[2],adData[3]] #2018/11修正　ボタンの上位下位が逆
        buf=[(buttonData >> 8) & 0xff,(buttonData) & 0xff,adData[0],adData[1],adData[2],adData[3]]
        
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.KrrButtonDataAddress.value, buf)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	KRRのボタンデータを設定する
    #//
    ##	@brief	KRRの疑似ボタンデータを送信します
    #	@param	buttonData	ボタンデータ(KRR_BUTTON)
    #	@retval	True	送信成功
    #	@retval	False	送信失敗
    #	@warning	KRRが接続されているときは、接続されている側のデータで上書きされるため使えません
    #	@note	KRRの受信データ部分のボタンデータを書き込みます
    #	@note	ボタンデータに関してはKRR_BUTTONから作成します
    def setKrrButtonData (self,buttonData):
        #buf=[buttonData & 0xff,(buttonData >> 8) & 0xff]
        buf=[(buttonData >> 8) & 0xff,(buttonData) & 0xff] #2018/11/07修正　ボタンの上位下位が逆
        
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.KrrButtonDataAddress.value, buf)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	KRRのアナログ(PA)データを書き込む
    #//
    ##	@brief	KRRのアナログ(PA)データを指定したポートに書き込みます
    #	@param	paPort	KRRの指定するポート
    #	@param	adData	指定したポートに書き込むアナログデータ(byte型(8bit))
    #	@retval	True	送信成功
    #	@retval	False	送信失敗
    #	@warning	KRRが接続されているときは、接続されている側のデータで上書きされるため使えません
    #	@warning	adDataに関しては必ず1byte分のデータを入れてください
    #	@note	KRRの受信データ部分の指定したPAポートのデータを書き込みます
 	#	@note	COM=>RAMでPAのポートアドレスにデータを書き込まれます
    def setKrrAdData (self,paPort, adData):
        
        #値が範囲内かチェック
        if (adData < 0 or 255 < adData):
            return False
        
        buf=[adData & 0xff]
        
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.KrrPa1Address.value + (paPort -1), buf)




    #/////////////////////////////////////////////////////////////////////////////
    #//	ミキシング関連
    #///////////////////////////////////////////////////////////////////////////////////

    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定したサーボモータのミキシングを指定したアドレスに設定する
    #//
    ##	@brief	指定したサーボモータのミキシングを設定する
    #	@param	id	サーボモータのID
    #	@param	sio	サーボモータが接続されているSIO番号(SIO1-4:0x01,SIO5-8:0x02)	
	#	@param	mixNum	ミキシングの番号(1or2)
	#	@param	ramAddr	ミキシングをかける指定したRAMアドレス
	#	@param	gain	ramAddrのデータをgain分、かけ合わせる
    #	@retval	True	通信成功
    #	@retval	False	失敗成功
    #	@note	指定したサーボモータにミキシングを指定したアドレスに設定します
    #	@note	サーボの出力データ = サーボモータの指令値 + サーボモータのトリム値 + ミキシング値1 + ミキシング値2
    #	@note	ミキシング値 = ramAddrに保存されているデータ * gain
    #	@warning	ミキシングは1byteもできますが、前提は2byteとして処理を行っています
    def setServoRamAddrMixing(self,id ,sio, mixNum, ramAddr, gain):
                
        buf=[ramAddr & 0xff,(ramAddr >> 8) & 0x0f | 0x40,gain]
        
        offset = Rcb4BaseLib.DeviceAddrOffset.Mixing1AddressOffset.value 
        if mixNum == 2 :
            offset = Rcb4BaseLib.DeviceAddrOffset.Mixing2AddressOffset.value
        
        reFlag = self.moveComToDeviceCmdSynchronize(self.icsNum2id(id, sio), offset, buf)
        #if retbuf[2] == 0x06:
        #    return True
        #else:
        #    return False
        return reFlag
        
   #///////////////////////////////////////////////////////////////////////////////////
    #//	指定したサーボモータのミキシングを指定したデバイスに設定する
    #//
    ##	@brief	指定したサーボモータのミキシングを設定する
    #	@param	id	サーボモータのID
    #	@param	sio	サーボモータが接続されているSIO番号(SIO1-4:0x01,SIO5-8:0x02)	
	#	@param	mixNum	ミキシングの番号(1or2)
	#	@param	mixId	ミキシングをかける指定したサーボモータのID
	#	@param	mixSio	ミキシングをかける指定したサーボモータが接続されているSIO番号(SIO1-4:0x01,SIO5-8:0x02)
	#	@param	devOfeet	ミキシングをデバイスのオフセット値(DeviceAddrOffset)
	#	@param	gain	サーボの指定したデータをgain分、かけ合わせる
    #	@retval	True	通信成功
    #	@retval	False	失敗成功
    #	@note	指定したサーボモータにミキシングを指定したアドレスに設定します
    #	@note	サーボの出力データ = サーボモータの指令値 + サーボモータのトリム値 + ミキシング値1 + ミキシング値2
    #	@note	ミキシング値 = デバイス(サーボモータ)の値 * gain
    #	@note	デバイス(サーボモータ)の値 = (mixIdとmixSioから作られる)ICS番号のdevOfeetされたデータ
    #	@warning	ミキシングは1byteもできますが、前提は2byteとして処理を行っています
    def setServoDeviceMixing(self,id ,sio, mixNum, mixId,mixSio,devOfeet, gain):
        buf=[self.icsNum2id(mixId, mixSio) | 0xc0,devOfeet,gain]
        
        offset = Rcb4BaseLib.DeviceAddrOffset.Mixing1AddressOffset.value 
        if mixNum == 2 :
            offset = Rcb4BaseLib.DeviceAddrOffset.Mixing2AddressOffset.value 
        retf, retbuf =  self.moveComToDeviceCmdSynchronize(self.icsNum2id(id, sio), offset, buf)
        if retbuf[2] == 0x06:
            return True
        else:
            return False


    #///////////////////////////////////////////////////////////////////////////////////
    #//	ミキシングのゲインを設定する
    #//
    ##	@brief	ミキシングのゲインを設定する
    #	@param	id	サーボモータのID
    #	@param	sio	サーボモータが接続されているSIO番号(SIO1-4:0x01,SIO5-8:0x02)	
	#	@param	mixNum	ミキシングの番号(1or2)
    #	@param	gain	サーボの指定したデータをgain分、かけ合わせる
    #	@retval	True	通信成功
    #	@retval	False	失敗成功
    #	@note 指定したサーボモータのミキシングゲインを設定します。
    def setServoMixGain(self,id ,sio, mixNum, gain):
        buf =[gain]
        offset = Rcb4BaseLib.DeviceAddrOffset.Mixing1RatioAddressOffset.value 
        if mixNum == 2 :
            offset = Rcb4BaseLib.DeviceAddrOffset.Mixing2RatioAddressOffset.value 
        retf, retbuf=  self.moveComToDeviceCmdSynchronize(self.icsNum2id(id, sio), offset, buf)
        if retbuf[2] == 0x06:
            return True
        else:
            return False


    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定したサーボのミキシングをOFFにします
    #//
    ##	@brief	指定したサーボモータのミキシングをOFFにします
	#	@param	id	サーボモータのID
    #	@param	sio	サーボモータが接続されているSIO番号(SIO1-4:0x01,SIO5-8:0x02)	
	#	@param	mixNum	ミキシングの番号(1or2)
    #	@retval	True	通信成功
    #	@retval	False	失敗成功
    #	@note	指定したサーボモータのミキシングをOFFにします。
    #	@note	ゲインは0xFFFFが書き込まれます
    def resetServoMixing(self,id ,sio, mixNum):
        buf = [0xff, 0xff]
        offset = Rcb4BaseLib.DeviceAddrOffset.Mixing1AddressOffset.value 
        if mixNum == 2 :
            offset = Rcb4BaseLib.DeviceAddrOffset.Mixing2AddressOffset.value 
        reFlag =  self.moveComToDeviceCmdSynchronize(self.icsNum2id(id, sio), offset, buf)
        #if retbuf[2] == 0x06:
        #    return True
        #else:
        #    return False
        return reFlag 
 
 
    #/////////////////////////////////////////////////////////////////////////////
    #//	ユーザ変数、カウンタ関連
    #/////////////////////////////////////////////////////////////////////////////
 
    #///////////////////////////////////////////////////////////////////////////////////
    #//	カウンターのアドレスを取得する
    #//
    ##	@brief	カウンタのアドレスを取得する
    #	@param	counterNum	カウンタの番号
    #	@retval	カウンタのRAMアドレス
    #	@retval	-1	設定されたアドレスが不正である
    #	@note	静的関数で外部アクセスが可能
    @staticmethod   #add 2018/10/19
    def userCounterAddr(counterNum):
        if (0 < counterNum <= Rcb4BaseLib.CounterCount):
            return Rcb4BaseLib.RamAddr.CounterRamAddress.value + ((counterNum - 1) * Rcb4BaseLib.CounterSingleDataCount) 
        else:
            return -1
    
    
    #///////////////////////////////////////////////////////////////////////////////////
    #//	カウンタの値を書き込む
    #//
    ##	@brief	カウンタ値を書き込む
    #	@param	counterNum	カウンタの番号
    #	@param	data	書き込むデータ(unsigned char ,byte)
    #	@retval	True	通信成功
    #	@retval	False	失敗成功
    #	@note   指定したカウンタに値を書き込みます(1byteのみ)
    #	@note	COM => RAMで、指定したカウンタのアドレスに1byte書き込みます
    #	@warning	データはbyte型(unsigned char)で行ってください
    def setUserCounter(self,counterNum,data):
     
        addr = Rcb4BaseLib.userCounterAddr(counterNum)
        if (addr < 0): #アドレスが範囲外
            return False        
        
        try: #signed byte型で入るかチェック
            sendData = struct.pack('B',data)
        except:
            return False
        
        return self.moveComToRamCmdSynchronize(addr, sendData)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	カウンタの値を読み込む
    #//
    ##	@brief	カウンタ値を読み込みます
    #	@param	counterNum	カウンタの番号
    #	@return	(retf,countData)
    #	@retval	retf	True:通信成功
    #	@retval	retf	False:失敗成功
	#	@retval	countData	取得したカウンタ1byteデータ(unsigned char ,byte)
	#	@retval	countData	-1(0xFF):取得失敗
    #	@note   指定したカウンタの値を読み込みます(1byteのみ)
    #	@note	RAM => COMで、指定したカウンタの値を1byte読み込みます
    #	@warning	データはbyte型(unsigned char)で行ってください 
    def getUserCounter(self,counterNum):
        addr = self.userCounterAddr(counterNum)
        if(addr < 0): #範囲外はFalseを返す
            return False,-1
        else:
            retf, retbuf = self.moveRamToComCmdSynchronize(addr , 1 )
            
            if(retf == False) or (len(retbuf) != 1):
                return False,-1
            else:
                countData  =  struct.unpack_from('B',retbuf,0)[0]
                #countData  = retbuf;
                return True,countData


    #///////////////////////////////////////////////////////////////////////////////////
    #//	ユーザ変数のアドレスを取得する
    #//
    ##	@brief	ユーザ変数のアドレスを取得する
    #	@param	parmeterNum	ユーザ変数の番号
    #	@retval	ユーザ変数のRAMアドレス
    #	@retval	-1	設定されたアドレスが不正である
    #	@note	静的関数で外部アクセスが可能
    @staticmethod   #add 2018/10/19
    def userParmeterAddr(parmeterNum):
        if (0 < parmeterNum <= Rcb4BaseLib.UserParmeterCount ):
            return Rcb4BaseLib.RamAddr.UserParmeterRamAddress.value + ((parmeterNum - 1) * Rcb4BaseLib.UserParmeterSingleDataCount)
        else:
            return -1


    #///////////////////////////////////////////////////////////////////////////////////
    #//	ユーザ変数に値を書き込む
    #//
    ##	@brief	指定したユーザ変数に値を書き込む
    #	@param	parmeterNum	ユーザ変数の番号
    #	@param	data	書き込むデータ(short型)
    #	@retval	True	通信成功
    #	@retval	False	失敗成功
    #	@note   指定したユーザ変数に値を書き込みます(2byte(signed short))
    #	@note	COM => RAMで、指定したユーザ変数に2byte書き込みます
    #	@warning	データはsigned short型で行ってください
    def setUserParmeter(self,parmeterNum,data):
        addr = self.userParmeterAddr(parmeterNum)
        #buf = [data & 0xff,(data >> 8 ) & 0xff]
        try:
            buf = struct.pack('<h',data) #Little short型変換
        except:
            return False


        if not(addr <0):
            return self.moveComToRamCmdSynchronize(addr, buf)
        else:
            return False


    #///////////////////////////////////////////////////////////////////////////////////
    #//	ユーザ変数の値を読み込む
    #//
    ##	@brief	指定したユーザ変数を読み込みます
    #	@param	parmeterNum	ユーザ変数の番号
    #	@return	(retf,paraData)
    #	@retval	retf	True:通信成功
    #	@retval	retf	False:失敗成功
	#	@retval	paraData	取得したユーザ変数2byteデータ(short)
	#	@retval	paraData	-1(0xFFFF):取得失敗
    #	@note   指定したユーザ変数の値を読み込みます(2byteのみ)
    #	@note	RAM => COMで、指定したカウンタの値を2byte読み込みます
    #	@warning	データはbyte型(unsigned char)で行ってください 
    def getUserParmeter(self,parmeterNum):
        addr = self.userParmeterAddr(parmeterNum)
        if not(addr <0):
            retf,retbuf = self.moveRamToComCmdSynchronize(addr , 2 )
            if(retf == True) and (len(retbuf) ==2):
                #print(retbuf)
                paraData  =  struct.unpack('<h',retbuf)[0]
                #print(paraData)
                #return  retbuf[2] + retbuf[3]*256
                return  True,paraData
            else:
                return False,-1
        else:
            return False,-1
 
 
    #/////////////////////////////////////////////////////////////////////////////
    #//	モーション関連
    #/////////////////////////////////////////////////////////////////////////////
 
    #///////////////////////////////////////////////////////////////////////////////////
    #//	現在再生されているモーションの番号を取得します
    #//
    ##	@brief	現在再生されているモーションの番号を取得します。
    #	@retval	モーション番号
    #	@retval	0	どのモーションも再生されていない
    #	@retval	-1	通信失敗
    #	@retval	-2	再生されている場所が異常である
    #	@note	現在再生されているモーション番号を現在のプログラムカウンタとフラグから算出する
    #	@note	モーション内で違うモーションにジャンプをしていたらジャンプしたモーション番号が得られます
    def getMotionPlayNum(self):
        
        retf,retbuf = self.moveRamToComCmdSynchronize(self.RamAddr.ProgramCounterRamAddress.value , 10 )
        
        #print('flag:',retf)
        #通信失敗時はエラーを返す
        if (retf == False) or (len(retbuf) != 10):
            return -1
        
        #フラグ部分の計算
        eflfg = retbuf[3] + retbuf[4] + retbuf[5] + retbuf[6] + retbuf[7]
        
        #現在動いているアドレス
        pcunt = retbuf[0] + (retbuf[1] << 8) + (retbuf[2] << 16)
        
        #アドレスからモーションを逆算
        mno = int(((pcunt -  Rcb4BaseLib.RomAddr.MotionRomAddress.value) / self.MotionSingleDataCount) + 1  )         

        if eflfg == 0:
            return 0
        if pcunt < Rcb4BaseLib.RomAddr.MotionRomAddress.value:
            return 0
        if mno < 0 or mno > 120 :
            return -2
        
        return mno
    


    #///////////////////////////////////////////////////////////////////////////////////
    #//	モーションを一時停止させます
    #//
    ##	@brief	モーションを一時停止させる
    #	@retval	True	通信成功
    #	@retval	False	失敗成功
    #	@note   configデータを変更し、モーションを一時停止させます
    #	@note	EEPROM =>0 ,ベクタージャンプ => 0,サーボの動作レスポンス => 0,ICSスイッチ => 0
    #	@warning	内部のconfigの情報(self.__config)を直接変更するので、初期設定時に読み取る必要があります	
    def suspend(self):
        
        #self.__configData = 0 #del 2018/10/25
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableRunEeprom.value #2018/10/25修正
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableServoResponse.value
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableReferenceTable.value	
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableSio.value
        
        
        txbuf = [self.__configData & 0xff, (self.__configData >> 8) & 0xff]
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.ConfigRamAddress.value, txbuf)



    #///////////////////////////////////////////////////////////////////////////////////
    #//	モーション番号からアドレスを算出します
    #//
    ##	@brief	モーション番号からROMに保存されている先頭アドレスを算出します
    #	@param	motionNum	モーション番号
    #	@retval	モーションのアドレス
    #	@retval	-1	モーションのアドレスが存在しない 
    #	@note   モーション番号からアドレスを算出します
    def motionAddr2motionNum(self,motionNum):
        if 0 < motionNum <= self.MaxMotionCount:
            return (motionNum -1)* self.MotionSingleDataCount + Rcb4BaseLib.RomAddr.MotionRomAddress.value
        else:
            return -1
		


    #///////////////////////////////////////////////////////////////////////////////////
    #//	モーションアドレスにジャンプをする
    #//
    ##	@brief	モーションのアドレスにジャンプします
    #	@param	motionNum	モーション番号
    #	@retval	True	通信成功
    #	@retval	False	失敗成功
    #	@note   指定したモーションのROMのアドレスにジャンプします。
    #	@note   モーションのジャンプにはCallコマンドを使用します。
    def setMotionNum(self,motionNum):
        if 0 < motionNum <= self.MaxMotionCount:
            rxSize,buf = self.callCmd(self.motionAddr2motionNum(motionNum))
            return self.synchronizeAck(buf)
        else:
            return False


    #///////////////////////////////////////////////////////////////////////////////////
    #//	プロフラムカウンタをリセットします
    #//
    ##	@brief	プログラムカウンタをリセットします
	#	@retval	True	通信成功
	#	@retval	False	失敗成功
    #	@note	プログラムカウンタをリセットします。
    #	@note	正確には、プログラムカウンタを初期設定後の部分に変更します。
    #	@note	各種フラグもリセットします。
    def resetProgramCounter(self):
        buf = [Rcb4BaseLib.RomAddr.MainLoopCmd.value  & 0xff, (Rcb4BaseLib.RomAddr.MainLoopCmd.value >> 8) & 0xff,0,0,0,0,0,0,0,0]
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.ProgramCounterRamAddress.value, buf)


    #///////////////////////////////////////////////////////////////////////////////////
    #//	モーションをリスタート(復帰)させます
    #//
    ##	@brief	モーションをリスタートさせます
	#	@retval	True	通信成功
	#	@retval	False	失敗成功
    #	@note	モーションをリスタート(復帰)させます
    #	@note   configデータを変更します
    #	@note	EEPROM =>1 ,ベクタージャンプ => 1,サーボの動作レスポンス => 0,ICSスイッチ => 1
    #	@warning	内部のconfigの情報(self.__config)を直接変更するので、初期設定時に読み取る必要があります	   
    def resume (self):
        #self.__configData = 0  #del 2018/10/19
        self.__configData |= Rcb4BaseLib.ConfigData.EnableRunEeprom.value
        self.__configData &= ~Rcb4BaseLib.ConfigData.EnableServoResponse.value
        self.__configData |= Rcb4BaseLib.ConfigData.EnableReferenceTable.value 	#2019/02/04	ベクタジャンプは有効にしておく
        self.__configData |= Rcb4BaseLib.ConfigData.EnableSio.value
        buf = [ self.__configData & 0xff,(self.__configData >> 8) & 0xff]
        return self.moveComToRamCmdSynchronize(Rcb4BaseLib.RamAddr.ConfigRamAddress.value, buf)



    #///////////////////////////////////////////////////////////////////////////////////
    #//	指定したモーションを再生します
    #//
    ##	@brief	指定したモーションを再生します
    #	@param	motionNum	モーション番号
	#	@retval	True	通信成功
	#	@retval	False	失敗成功
    #	@note   指定したモーションを再生させます
    #	@note	モーションを再生させるには
    #	@note	1.モーションの一時停止
    #	@note	2.プログラムカウンタのリセット
    #	@note	3.モーションのアドレスにジャンプ
    #	@note	4.モーションを復帰させる
    #	@note	以上の手順が必要です
    def motionPlay(self,motionNum):
        
        #モーションが存在さいない
        if (motionNum <= 0)or( self.MaxMotionCount < motionNum ):
            return False
        
        #モーションを一時停止
        if self.suspend() == False:
            return False
        
        #プログラムカウンタをリセット
        if self.resetProgramCounter() == False:
            return False
        
        #モーションアドレスにジャンプ
        if self.setMotionNum(motionNum) == False:
            return False
        
        #モーション再開させる
        return self.resume ()


#/////////////////////////////////////////////////////////////////////////////
#//	ここまで[EOF]
#/////////////////////////////////////////////////////////////////////////////