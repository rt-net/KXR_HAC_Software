import math
import time
import sys

import numpy as np
import cv2

from motion_planning.motion_planning_library import MotionPlanningLibrary

PLANNING = MotionPlanningLibrary()


DISTANCE_FROM_THE_EDGE_MAXIMUM_MM = 300
DISTANCE_FROM_THE_EDGE_MINIMUM_MM = 150
DISTANCE_FROM_THE_EDGE_DEFAULT_MM = 200

while True:
    # PLANNING.left_hand_approach()
    PLANNING.approach_to_ball()
    cv2.imshow("frame3", PLANNING.display_image())
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break