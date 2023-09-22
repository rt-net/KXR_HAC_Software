import math
import time
import sys

import numpy as np
import cv2

from motion_planning.motion_planning_library import MotionPlanningLibrary

PLANNING = MotionPlanningLibrary()

while True:
    # PLANNING.left_hand_approach()
    # PLANNING.approach_to_ball()
    # print(PLANNING.check_know_ball_pos())
    PLANNING.get_vision_all()
    #PLANNING.get_angle_to_goal()
    cv2.imshow("frame3", PLANNING.display_image())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    # PLANNING.round_corner()
    
    # PLANNING.align_with_field_edge()
    
    # PLANNING.cross_goal()