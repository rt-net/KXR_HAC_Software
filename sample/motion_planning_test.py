#motion_planning_library中のそれぞれのモジュールの動作確認用
import math
import time
import sys

import numpy as np
import cv2

from task_execute.task_execute_library import TaskExecuteLibrary

TASKEXECUTE = MotionPlanningLibrary()

while True:
    TASKEXECUTE.left_hand_approach()
    # TASKEXECUTE.approach_to_ball()
    # print(TASKEXECUTE.check_know_ball_pos())
    #TASKEXECUTE.get_vision_all()
    #TASKEXECUTE.get_angle_to_goal()
    # TASKEXECUTE.round_corner()
    # TASKEXECUTE.align_with_field_edge()
    # TASKEXECUTE.cross_goal()
    cv2.imshow("RESULT IMAGE", TASKEXECUTE.display_image())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break