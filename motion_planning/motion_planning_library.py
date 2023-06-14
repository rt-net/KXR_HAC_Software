import math
import time
import sys

import numpy as np

from vision.vision_library import VisionLibrary
from motion_control.motion_control_library import MotionLibrary
from motion_planning import parameters

class MotionPlanningLibrary:
    def __init__(self):
        self.VISION = VisionLibrary()
        self.MOTION = MotionLibrary()
        
        self.distance_from_the_edge = parameters.WALK_PATH_FIELD_EDGE_DISTANCE
        
        print("Motion Planning Initialize")
        
    def get_distance_from_the_edge(self, slope, intercept):
        Ax = parameters.BEV_FRAME_WIDTH/2
        Ay = parameters.BEV_FRAME_HEIGHT + parameters.FOOT_CENTER_TO_BEV_FRAME_BOTTOM_DISTANCE
        
        Bx = (Ay-intercept)/slope
        By = Ay
        
        Cx = Ax
        Cy = Ax*slope+intercept
        
        AC = (Ay - Cy)
        AB = (Ax - Bx)
        BC = (AC**2 + AB**2)**(1/2)
        
        self.distance_from_the_edge = (AB**2 - ((AB**2 - AC**2 +BC**2)/(2*BC))**2)**(1/2)
        
    def get_vision(self):
        self.VISION.calibrate_img()
        self.angle, self.slope, self.intercept = self.VISION.detect_edge_using_numpy_calc()
        self.ball_x_coordinate, self.ball_y_coordinate = self.VISION.detect_ball()
        self.cornertype, self.corner_x_coordinate, self.corner_y_coordinate = self.VISION.detect_corner()
        self.result_image = self.VISION.display_resultimg()
        
    def left_hand_approach(self):
        self.get_vision()
        print(self.angle)
        
        if self.MOTION.button_state() == False:
            self.MOTION.walk_forward_continue()
            
        if self.angle != 0:
            self.MOTION.motion_stop()
            self.MOTION.turn(30)
            #self.get_distance_from_the_edge(self, self.slope, self.intercept)
        
        if self.distance_from_the_edge < parameters.EDGE_APPROACH_THRESHOLD:
            self.MOTION.motion_stop()
            self.MOTION.turn(angle)
            if angle > 0:
                self.MOTION.motion_stop()
                self.MOTION.walk_sideway(-(dist-parameters.WALK_PATH_FIELD_EDGE_DISTANCE))
            else:
                self.MOTION.motion_stop()
                self.MOTION.walk_sideway((dist-parameters.WALK_PATH_FIELD_EDGE_DISTANCE))
        
        
        
        