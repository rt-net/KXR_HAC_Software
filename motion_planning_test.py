import time
import cv2

from vision.vision_library import VisionLibrary
from motion_control.motion_control_library import MotionLibrary
import get_distance_from_the_edge_mm_test

image_display_delay_ms = 1 #画像の表示時間 (ms)

VISION = VisionLibrary() #画像処理ライブラリのインスタンス
MOTION = MotionLibrary() #動作再生ライブラリのインスタンス

MOTION.stop_motion() #動作の再生状態を初期化
MOTION.set_plot() #グラフプロットの設定

distance_from_the_edge_mm = 300 #エッジ沿いを歩行する際のエッジからの距離

start = time.time() #処理時間計測の開始


DISTANCE_FROM_THE_EDGE_MAXIMUM_MM = 300
DISTANCE_FROM_THE_EDGE_MINIMUM_MM = 150
DISTANCE_FROM_THE_EDGE_DEFAULT_MM = 200

while True:
    VISION.calibrate_img()
    angle, slope, intercept = VISION.detect_edge_using_numpy_calc()
    ballx, bally = VISION.detect_ball()
    cornertype, cornerx, cornery = VISION.detect_corner()
    
    result = VISION.display_resultimg()
    cv2.imshow("result", result)
    if cv2.waitKey(image_display_delay_ms) & 0xFF == ord('q'):
        break
    
    if MOTION.button_state() == False:
        MOTION.walk_forward_continue()
    
    if angle != 0:
        distance_from_the_edge_mm =  get_distance_from_the_edge_mm_test.main(slope, intercept)
    else:
        distance_from_the_edge_mm = DISTANCE_FROM_THE_EDGE_MAXIMUM_MM
    
    finish = time.time()
    
    if distance_from_the_edge_mm < DISTANCE_FROM_THE_EDGE_MINIMUM_MM:
        MOTION.stop_motion()
        MOTION.turn(angle)
        if angle > 0:
            MOTION.stop_motion()
            MOTION.walk_sideway(-(distance_from_the_edge_mm-DISTANCE_FROM_THE_EDGE_DEFAULT_MM))
        else:
            MOTION.stop_motion()
            MOTION.walk_sideway((distance_from_the_edge_mm-DISTANCE_FROM_THE_EDGE_DEFAULT_MM))
    
    if MOTION.button_state() == True:
        MOTION.calculate_field_coordinate((finish-start))
    
    start = time.time()
        
    print(MOTION.field_absolute_cordinate())