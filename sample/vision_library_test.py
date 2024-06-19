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
    VISION.calibrate_img()
    # linea, lineb, linec = VISION.detect_edge_using_numpy_calc()
    # ballx, bally = VISION.detect_ball()
    cornertype, cornerx, cornery = VISION.detect_corner()
    
    # print("----------------------")
    # print(linea, lineb)
    # print(ballx, bally)
    print(cornertype, cornerx, cornery)
    # print(VISION.detect_corner_wide())
    #VISION.detect_ball_line()
    
    # VISION.detect_goal()
    resultimg = VISION.display_resultimg()
    cv2.imshow("frame3", resultimg)
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break