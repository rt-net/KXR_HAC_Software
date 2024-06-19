#vision_libraryの各モジュールの動作確認用
import cv2
import time
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) #一つ上のディレクトリへの検索パスの追加
from vision.vision_library import VisionLibrary

delay = 1

VISION = VisionLibrary()
while True:
    ###カメラ画像の読み込みとキャリブレーション（物体検出の前に必ず行う）###
    VISION.calibrate_img()
    
    ###各要素の検出###
    edge_angle_deg, edge_slope, edge_intercept = VISION.detect_edge_using_numpy_calc() #フィールド端の赤色エッジの検出　エッジ角度，傾き，切片を取得
    ball_coordinate_x, ball_coordinate_y = VISION.detect_ball() #ボールの検出　ボールのx座標，y座標を取得
    corner_type, corner_coordinate_x, corner_coordinate_y = VISION.detect_corner_wide() #コーナーの検出　コーナーの種類，x座標，y座標を取得
    # VISION.detect_ball_line()
    # VISION.detect_goal()
    
    ###結果の表示###
    print("\n##### EDGE DETECTION #####")
    try:
        print("edge_angle: ", edge_angle)
        print("edge_slope: ", edge_slope)
        print("edge_intercept: ", edge_intercept)
    except:
        print("...No edge detected")
        
    print("\n##### BALL DETECTION #####")
    try:
        print("ball_coordinate_x: ", ball_coordinate_x)
        print("ball_coordinate_y: ", ball_coordinate_y)
    except:
        print("...No ball detected")    
        
    print("\n##### CORNER DETECTION #####")
    try:
        print("corner_type: ", corner_type)
        print("corner_coordinate_x: ", corner_coordinate_x)
        print("corner_coordinate_y: ", corner_coordinate_y)
    except:
        print("...No corner detected")  
    
    ###結果画像の表示###
    resultimg = VISION.display_resultimg()
    cv2.imshow("RESULT IMAGE", resultimg)
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break