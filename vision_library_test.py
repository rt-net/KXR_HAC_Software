import cv2
import time

from vision.vision_library import VisionLibrary

delay = 1

VISION = VisionLibrary()
while True:
    start = time.time()
    VISION.calibrate_img()
    # linea, lineb = VISION.detect_edge_using_numpy_calc()
    # ballx, bally = VISION.detect_ball()
    # cornertype, cornerx, cornery = VISION.detect_corner()
    resultimg = VISION.display_resultimg()
    cv2.imshow("frame3", resultimg)
    
    finish = time.time()
    
    print(finish-start)
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break