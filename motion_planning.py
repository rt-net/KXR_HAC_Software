import cv2
from vision.vision_library import VisionLibrary
from motion_control.motion_control_library import MotionLibrary
import get_distance_from_the_edge
from vision import parameters as p
import time
import turn
import walk_forward
import walk_sideway
import get_body_angle
import ctypes


import global_value as g
g.X = 0
g.Y = 0

delay = 1

VISION = VisionLibrary()
MOTION = MotionLibrary()

i = 0

while True:
    VISION.calibrate_img()
    linea, lineb = VISION.detect_edge()
    ballx, bally = VISION.detect_ball()
    cornertype, cornerx, cornery = VISION.detect_corner()
    
    result = VISION.display_resultimg()
    # cv2.imshow("result", result)
    # if cv2.waitKey(delay) & 0xFF == ord('q'):
    #     break
    
    
    #a = MOTION.get_body_angle()
    
    
    
    print(MOTION.get_body_angle())
    
    i = i+1
    
    if i == 10:
        MOTION.IMU_calib()
        

    #MOTION.walk_forward(60)
    
    # if linea == 0:
    #     turn.main(-30)
    
    
    # if linea != 0:
    #     dist =  get_distance_from_the_edge.main(linea, lineb)
        
    #     turn.main(angle)
    #     walk_sideway.main(-(dist-150))
    #     MOTION.walk_forward(50)
        
        
        
        