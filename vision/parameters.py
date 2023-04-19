import numpy as np
import cv2

#各パラメータの記述ファイル

#動画読み込み時の解像度、FPS指定
camera_width = 320 #幅
camera_height = 240 #高さ
camera_FPS = 5 #フレームレート

#--------------------------------------------------------------

#鳥瞰図変換のパラメータ　画角内に長方形を置いた際の各角の位置から得る
BEV_top_left = [46, 0] #左上角
BEV_top_right = [252, 24] #右上角
BEV_bottom_left = [15, 240] #左下角
BEV_bottom_right = [315, 220] #右下角

BEV_width = 345 #画角内に配置できる最大の長方形幅
BEV_height = 395 #画角内に配置できる最大の長方形高さ

#--------------------------------------------------------------

#フィールドの色範囲指定(HSV)
field_lower = np.array([165,50,50]) #エッジ色の下閾値
field_upper = np.array([180,255,255]) #エッジ色の上閾値

line_area_threshhold = 2000 #エッジの存在判定用エッジ色の画素数閾値

#--------------------------------------------------------------

#ボールの色範囲指定(HSV)
ball_lower = np.array([5,200,180]) #ボール色の下閾値
ball_upper = np.array([25,255,255]) #ボール色の上閾値

ball_size_threshhold = 2000 #ボールの存在判定用ボール色の画素数閾値

#--------------------------------------------------------------

corner_height, corner_width = 66,66 #コーナー部のサイズ(mm)
bwthresh = 30
#コーナーのパターンマッチ元画像の読み込み
field_corner_left = cv2.imread("vision/field_corner_left.jpg")
field_corner_left = cv2.resize(field_corner_left, (corner_height, corner_width))
field_corner_left = cv2.cvtColor(field_corner_left, cv2.COLOR_BGR2GRAY)
ret, field_corner_left = cv2.threshold(field_corner_left, bwthresh, 255, cv2.THRESH_BINARY)

field_corner_right = cv2.imread("vision/field_corner_right.jpg")
field_corner_right = cv2.resize(field_corner_right, (corner_height, corner_width))
field_corner_right = cv2.cvtColor(field_corner_right, cv2.COLOR_BGR2GRAY)
ret, field_corner_right = cv2.threshold(field_corner_right, bwthresh, 255, cv2.THRESH_BINARY)

pattern_match_threshhold = 0.85 #パターンマッチの閾値
#--------------------------------------------------------------

blur_filter_size = 3 #ぼかしフィルターサイズ

