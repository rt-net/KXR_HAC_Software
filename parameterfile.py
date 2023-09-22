import numpy as np
import cv2

#Motion Planning Parameters###################################
FOOT_CENTER_TO_BEV_FRAME_BOTTOM_DISTANCE = 90

WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM = 130
WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM = 120
WALK_PATH_TO_FIELD_EDGE_MAXIMUM_MM = 130

BALL_APPROACH_THRESHOLD = 100

AVOID_CORNER_MM = 200

#Motion Control Parameters#####################################
#一歩当たりの移動量の設定
FORWARD_SINGLE_STEP_TRAVEL = 38 #前進1歩あたりの移動量(mm)
SIDE_SINGLE_STEP_TRAVEL = 50 #横歩行あたりの移動量(mm)
TURN_SINGLE_STEP_ANGLE = 27#9.5#25 #旋回一回あたりの回転角度(度)

FORWARD_1_SECOND_TRAVEL = 75 #1秒あたりの前進量
#--------------------------------------------------------------
#rcb4上でのモーション番号の設定
RCB4_WALK_FORWARD = 9 #RCB4内のモーション番号　前進
RCB4_WALK_LEFT = 11 #RCB4内のモーション番号　左横移動
RCB4_WALK_RIGHT = 12 #RCB4内のモーション番号　右横移動
RCB4_TOUCH_BALL = 18 #RCB4内のモーション番号　ボールタッチ
RCB4_TURN_LEFT = 13 #RCB4内のモーション番号　左旋回
RCB4_TURN_RIGHT = 14 #RCB4内のモーション番号　右旋回
RCB4_STAND_UP = 15 #RCB4内のモーション番号　仰向け起き上がり
#--------------------------------------------------------------
ROBOT_REGULAR_PAUSE = 0.5
ROBOT_LONG_PAUSE = 2
#--------------------------------------------------------------
#グラフ関連のパラメータ
GRAPH_X_AXIS_MAXIMUM = 1000
GRAPH_X_AXIS_MINIMUM = -1000
GRAPH_Y_AXIS_MAXIMUM = 1000
GRAPH_Y_AXIS_MINIMUM = -1000

#Vision Parameters#############################################
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

BEV_FRAME_WIDTH_MM = 345 #画角内に配置できる最大の長方形幅
BEV_FRAME_HEIGHT_MM = 395 #画角内に配置できる最大の長方形高さ

#--------------------------------------------------------------

#フィールドの色範囲指定(HSV)
FIELD_COLOR_MIN_LOW = np.array([0,40,40]) #エッジ色の下閾値
FIELD_COLOR_MAX_LOW = np.array([20,255,255]) #エッジ色の上閾値

FIELD_COLOR_MIN_HIGH = np.array([160,40,40]) #エッジ色の下閾値
FIELD_COLOR_MAX_HIGH = np.array([180,255,255]) #エッジ色の上閾値

EDGE_PIXEL_AREA_THRESHOLD = 2300 #エッジの存在判定用エッジ色の画素数閾値

BLUR_FILTER_SIZE = 3 #ぼかしフィルターサイズ

#--------------------------------------------------------------

#ボールの色範囲指定(HSV)
BALL_COLOR_MIN = np.array([5,200,180]) #ボール色の下閾値
BALL_COLOR_MAX = np.array([25,255,255]) #ボール色の上閾値

BALL_PIXEL_AREA_THRESHOLD = 1500 #ボールの存在判定用ボール色の画素数閾値
BALL_PIXEL_AREA_THRESHOLD_WIDE = 500 #ボールの存在判定用ボール色の画素数閾値

#--------------------------------------------------------------
#ボール位置の指定
BALL_POS_TOLERANCE_MM = 120
BALL_POS_FROM_ROBOT = 80

#--------------------------------------------------------------

#ゴールラインの色範囲指定(HSV)
GOAL_COLOR_MIN = np.array([0, 0, 254]) #ゴールライン色の下閾値
GOAL_COLOR_MAX = np.array([1, 1, 255]) #ゴールライン色の上閾値

BLUR_FILTER_SIZE_GOAL = 9

#--------------------------------------------------------------

CORNER_TEMPLATE_HEIGHT, CORNER_TEMPLATE_WIDTH = 66,66 #コーナー部のサイズ(mm)
BINARIZATION_THRESHOLD = 30
#コーナーのテンプレートマッチ元画像の読み込み
left_corner_template = cv2.imread("tmp/left_corner_template.jpg")
left_corner_template = cv2.resize(left_corner_template, (CORNER_TEMPLATE_HEIGHT, CORNER_TEMPLATE_WIDTH))
left_corner_template = cv2.cvtColor(left_corner_template, cv2.COLOR_BGR2GRAY)
ret, LEFT_CORNER_TEMPLATE = cv2.threshold(left_corner_template, BINARIZATION_THRESHOLD, 255, cv2.THRESH_BINARY)

right_corner_template = cv2.imread("tmp/right_corner_template.jpg")
right_corner_template = cv2.resize(right_corner_template, (CORNER_TEMPLATE_HEIGHT, CORNER_TEMPLATE_WIDTH))
right_corner_template = cv2.cvtColor(right_corner_template, cv2.COLOR_BGR2GRAY)
ret, RIGHT_CORNER_TEMPLATE = cv2.threshold(right_corner_template, BINARIZATION_THRESHOLD, 255, cv2.THRESH_BINARY)

left_corner_template = cv2.imread("tmp/left_corner_template.jpg")
left_corner_template = cv2.resize(left_corner_template, (30, 20))
left_corner_template = cv2.cvtColor(left_corner_template, cv2.COLOR_BGR2GRAY)
ret, LEFT_CORNER_TEMPLATE_WIDE = cv2.threshold(left_corner_template, BINARIZATION_THRESHOLD, 255, cv2.THRESH_BINARY)

right_corner_template = cv2.imread("tmp/right_corner_template.jpg")
right_corner_template = cv2.resize(right_corner_template, (30, 20))
right_corner_template = cv2.cvtColor(right_corner_template, cv2.COLOR_BGR2GRAY)
ret, RIGHT_CORNER_TEMPLATE_WIDE = cv2.threshold(right_corner_template, BINARIZATION_THRESHOLD, 255, cv2.THRESH_BINARY)

TEMPLATE_MATCH_THRESHOLD = 0.68#0.85 #パターンマッチの閾値
TEMPLATE_MATCH_THRESHOLD_WIDE = 0.65#0.85 #パターンマッチの閾値

#--------------------------------------------------------------