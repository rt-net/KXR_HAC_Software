import numpy as np
import cv2

#各パラメータの記述ファイル

#動画読み込み時の解像度、FPS指定
CAMERA_FRAME_WIDTH = 320 #幅
CAMERA_FRAME_HEIGHT = 240 #高さ
CAMERA_FPS = 5 #フレームレート

#--------------------------------------------------------------

#鳥瞰図変換のパラメータ　画角内に長方形を置いた際の各角の位置から得る
BEV_TOP_LEFT = [46, 0] #左上角
BEV_TOP_RIGHT = [252, 24] #右上角
BEV_BOTTOM_LEFT = [15, 240] #左下角
BEV_BOTTOM_RIGHT = [315, 220] #右下角

BEV_FRAME_WIDTH = 345 #画角内に配置できる最大の長方形幅
BEV_FRAME_HEIGHT = 395 #画角内に配置できる最大の長方形高さ

#--------------------------------------------------------------

#フィールドの色範囲指定(HSV)
FIELD_COLOR_LOWER = np.array([165,50,50]) #エッジ色の下閾値
FIELD_COLOR_UPPER = np.array([180,255,255]) #エッジ色の上閾値

EDGE_PIXEL_AREA_THRESHOLD = 2000 #エッジの存在判定用エッジ色の画素数閾値

#--------------------------------------------------------------

#ボールの色範囲指定(HSV)
BALL_COLOR_LOWER = np.array([5,200,180]) #ボール色の下閾値
BALL_COLOR_UPPER = np.array([25,255,255]) #ボール色の上閾値

BALL_PIXEL_AREA_THRESHOLD = 2000 #ボールの存在判定用ボール色の画素数閾値

#--------------------------------------------------------------

CORNER_TEMPLATE_HEIGHT, CORNER_TEMPLATE_WIDTH = 66,66 #コーナー部のサイズ(mm)
BINARIZATION_THRESHOLD = 30
#コーナーのテンプレートマッチ元画像の読み込み
left_corner_template = cv2.imread("vision/left_corner_template.jpg")
left_corner_template = cv2.resize(left_corner_template, (CORNER_TEMPLATE_HEIGHT, CORNER_TEMPLATE_WIDTH))
left_corner_template = cv2.cvtColor(left_corner_template, cv2.COLOR_BGR2GRAY)
ret, LEFT_CORNER_TEMPLATE = cv2.threshold(left_corner_template, BINARIZATION_THRESHOLD, 255, cv2.THRESH_BINARY)

right_corner_template = cv2.imread("vision/right_corner_template.jpg")
right_corner_template = cv2.resize(right_corner_template, (CORNER_TEMPLATE_HEIGHT, CORNER_TEMPLATE_WIDTH))
right_corner_template = cv2.cvtColor(right_corner_template, cv2.COLOR_BGR2GRAY)
ret, RIGHT_CORNER_TEMPLATE = cv2.threshold(right_corner_template, BINARIZATION_THRESHOLD, 255, cv2.THRESH_BINARY)

TEMPLATE_MATCH_THRESHOLD = 0.85 #パターンマッチの閾値

#--------------------------------------------------------------

BLUR_FILTER_SIZE = 3 #ぼかしフィルターサイズ

