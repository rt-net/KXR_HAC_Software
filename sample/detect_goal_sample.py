#vision_libraryを用いたゴール検出の動作確認用（ゴールラインは白色のため，反射等で検出がうまくいかない際にはparameterfile.py中のhsv閾値の調整を行う）

import cv2
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) #一つ上のディレクトリへの検索パスの追加
from vision.vision_library import VisionLibrary

VISION = VisionLibrary()

delay = 1

while True:
    VISION.calibrate_img()
    VISION.detect_goal()
    # result = VISION.detect_goal()
    result = VISION.display_resultimg()
    cv2.imshow("frame2", result)
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break