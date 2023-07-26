#モーション関連パラメータの記述ファイル

#--------------------------------------------------------
#一歩当たりの移動量の設定
FORWARD_SINGLE_STEP_TRAVEL = 38 #前進1歩あたりの移動量(mm)
SIDE_SINGLE_STEP_TRAVEL = 16 #横歩行あたりの移動量(mm)
TURN_SINGLE_STEP_ANGLE = 25 #旋回一回あたりの回転角度(度)

FORWARD_1_SECOND_TRAVEL = 75 #1秒あたりの前進量
#--------------------------------------------------------
#rcb4上でのモーション番号の設定
RCB4_WALK_FORWARD = 1 #RCB4内のモーション番号　前進
RCB4_WALK_LEFT = 3 #RCB4内のモーション番号　左横移動
RCB4_WALK_RIGHT = 4 #RCB4内のモーション番号　右横移動
RCB4_TOUCH_BALL = 18 #RCB4内のモーション番号　ボールタッチ
RCB4_TURN_LEFT = 13 #RCB4内のモーション番号　左旋回
RCB4_TURN_RIGHT = 14 #RCB4内のモーション番号　右旋回
#--------------------------------------------------------
ROBOT_REGULAR_PAUSE = 0.5
#--------------------------------------------------------
#グラフ関連のパラメータ
GRAPH_X_AXIS_MAXIMUM = 1000
GRAPH_X_AXIS_MINIMUM = -1000
GRAPH_Y_AXIS_MAXIMUM = 1000
GRAPH_Y_AXIS_MINIMUM = -1000