import cv2

from vision.vision_library import VisionLibrary

delay = 1

VISION = VisionLibrary()
while True:
    VISION.calib_img()
    linea, lineb = VISION.detect_edge()
    ballx, bally = VISION.detect_ball()
    cornertype, cornerx, cornery = VISION.detect_corner()
    resultimg = VISION.display_resultimg()
    cv2.imshow("frame3", resultimg)
    
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break