import numpy as np
import cv2
#各パラメータの記述ファイル

#動画読み込み時の解像度、FPS指定
camWIDTH = 320 #幅
camHEIGHT = 240 #高さ
camFPS = 5 #フレームレート

#--------------------------------------------------------------

#鳥瞰図変換のパラメータ　画角内に長方形を置いた際の各角の位置から得る
BEVtopleft = [46, 0] #左上角
BEVtopright = [252, 24] #右上角
BEVbottomleft = [15, 240] #左下角
BEVbottomright = [315, 220] #右下角

BEVwidth = 345 #画角内に配置できる最大の長方形幅
BEVheight = 395 #画角内に配置できる最大の長方形高さ

#--------------------------------------------------------------

#フィールドの色範囲指定(HSV)
fieldlower = np.array([165,50,50]) #エッジ色の下閾値
fieldupper = np.array([180,255,255]) #エッジ色の上閾値

LineareaTH = 2000 #エッジの存在判定用エッジ色の画素数閾値

#--------------------------------------------------------------

#ボールの色範囲指定(HSV)
balllower = np.array([5,200,180]) #ボール色の下閾値
ballupper = np.array([25,255,255]) #ボール色の上閾値

BallSizeTH = 2000 #ボールの存在判定用ボール色の画素数閾値

#--------------------------------------------------------------

h, w = 66,66 #コーナー部のサイズ(mm)

#コーナーのパターンマッチ元画像の読み込み
fieldcornerleft = cv2.imread("fieldcornerleft.jpg")
fieldcornerleft = cv2.resize(fieldcornerleft, (h, w))
fieldcornerright = cv2.imread("fieldcornerright.jpg")
fieldcornerright = cv2.resize(fieldcornerright, (h, w))
fieldinnercornerleft = cv2.imread("fieldinnercornerleft.png")
fieldinnercornerleft = cv2.resize(fieldinnercornerleft, (h, w))

#--------------------------------------------------------------

ksize = 3 #ぼかしフィルターサイズ
patternmatchTH = 0.8 #パターンマッチの閾値
