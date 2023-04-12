import cv2
from vision.vision_library import VisionLibrary
import get_distance_from_the_edge
from vision import parameters as p
import time
import turn
import walk_forward
import walk_sideway

import global_value as g
g.X = 0
g.Y = 0

delay = 1

VISION = VisionLibrary()
delay = 1

while True:
    VISION.calib_img()
    linea, lineb, angle = VISION.detect_edge()
    ballx, bally = VISION.detect_ball()
    cornertype, cornerx, cornery = VISION.detect_corner()
    
    result = VISION.disp_resultimg()
    cv2.imshow("result", result)
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break
    
    if linea == 0:
        turn.main(-30)
    
    
    if linea != 0:
        dist =  get_distance_from_the_edge.main(linea, lineb)
        
        turn.main(angle)
        walk_sideway.main(-(dist-150))
        walk_forward.main(50)
        
        
        