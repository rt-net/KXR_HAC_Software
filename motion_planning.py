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


import global_value as g
g.X = 0
g.Y = 0

delay = 1

VISION = VisionLibrary()
MOTION = MotionLibrary()
delay = 1

while True:
    VISION.calib_img()
    linea, lineb, angle = VISION.detect_edge()
    ballx, bally = VISION.detect_ball()
    cornertype, cornerx, cornery = VISION.detect_corner()
    
    result = VISION.disp_resultimg()
    # cv2.imshow("result", result)
    # if cv2.waitKey(delay) & 0xFF == ord('q'):
    #     break
    
    
    #a = MOTION.get_body_angle()
    
    
    
    print(get_body_angle.main())

    MOTION.walk_forward(60)
    
    # if linea == 0:
    #     turn.main(-30)
    
    
    # if linea != 0:
    #     dist =  get_distance_from_the_edge.main(linea, lineb)
        
    #     turn.main(angle)
    #     walk_sideway.main(-(dist-150))
    #     MOTION.walk_forward(50)
        
        
        
        