print("start")

import math
import time
import sys

import numpy as np

from motion_planning.motion_planning_library import MotionPlanningLibrary

print("start")

PLANNING = MotionPlanningLibrary()

while True:
    PLANNING.left_hand_approach()