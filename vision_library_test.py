import cv2

from vision.vision_library import VisionLibrary

delay = 1

VISION = VisionLibrary()
while True:
    #BEV = VISION.calib_img()
    VISION.calib_img()
    linea, lineb = VISION.detect_edge()
    ballx, bally = VISION.detect_ball()
    cornertype, cornerx, cornery = VISION.detect_corner()
    
    # cv2.imshow("frame", edge)
    
    # cv2.imshow("frame2", ball)
    
    # cv2.imshow("frame3", corner)
    
    resultimg = VISION.disp_resultimg()
    #print(linea, lineb)
    cv2.imshow("frame3", resultimg)
    
    
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break