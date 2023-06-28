import math
import time
import sys

import numpy as np

from vision.vision_library import VisionLibrary
from motion_control.motion_control_library import MotionLibrary
from motion_planning import parameters

BEV_FRAME_WIDTH_MM = 345 #画角内に配置できる最大の長方形幅
BEV_FRAME_HEIGHT_MM = 395 #画角内に配置できる最大の長方形幅

class MotionPlanningLibrary:
    def __init__(self):
        self.VISION = VisionLibrary()
        self.MOTION = MotionLibrary()
        
        self.distance_from_the_edge_mm = parameters.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM
        
        print("[行動計画の初期化成功]")
        
    def calculate_distance_from_the_edge_mm(self, slope, intercept):
        """Calculate current distance from robot-cog to edge
        """
        Ax = BEV_FRAME_WIDTH_MM/2
        Ay = BEV_FRAME_HEIGHT_MM + parameters.FOOT_CENTER_TO_BEV_FRAME_BOTTOM_DISTANCE
        
        Bx = (Ay-intercept)/slope
        By = Ay
        
        Cx = Ax
        Cy = Ax*slope+intercept
        
        AC = (Ay - Cy)
        AB = (Ax - Bx)
        BC = (AC**2 + AB**2)**(1/2)
        
        self.distance_from_the_edge_mm = (AB**2 - ((AB**2 - AC**2 +BC**2)/(2*BC))**2)**(1/2)
        
    def get_vision_all(self):
        """collect all vision data
        """
        self.VISION.calibrate_img()
        self.angle, self.slope, self.intercept = self.VISION.detect_edge_using_numpy_calc()
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball()
        self.cornertype, self.corner_x_coordinate, self.corner_y_coordinate = self.VISION.detect_corner()
        self.result_image = self.VISION.display_resultimg()
        
    def left_hand_approach(self):
        """execute a loop of left hand approach
        """
        self.get_vision_all()
        #print(self.angle)
        
        if self.MOTION.is_button_pressed == False:
            self.MOTION.walk_forward_continue()
            
        if self.angle != 0:
            self.MOTION.stop_motion()
            # self.MOTION.turn(self.angle)
            self.calculate_distance_from_the_edge_mm(self.slope, self.intercept)
            self.MOTION.turn(self.angle)
        
        if self.distance_from_the_edge_mm < parameters.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM or self.distance_from_the_edge_mm > parameters.WALK_PATH_TO_FIELD_EDGE_MAXIMUM_MM:
            self.MOTION.stop_motion()

            if self.angle > 0:
                self.MOTION.stop_motion()
                self.MOTION.walk_sideway(-(self.distance_from_the_edge_mm-parameters.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM))
            else:
                self.MOTION.stop_motion()
                self.MOTION.walk_sideway((self.distance_from_the_edge_mm-parameters.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM))
                
        print("1")
                
    def approach_to_ball(self):
        """execute full ball approach process
        """
        self.get_vision_all()
        
        if self.ball_coordinate_x != 0 and self.ball_coordinate_y != 0:
            #print(self.ball_coordinate_x, self.ball_coordinate_y)
            
            robot_to_ball_distance_x_mm = self.ball_coordinate_x-(BEV_FRAME_WIDTH_MM/2)
            robot_to_ball_distance_y_mm = BEV_FRAME_HEIGHT_MM - self.ball_coordinate_y
            
            #print(robot_to_ball_distance_x_mm)
            
            robot_to_ball_angle = math.degrees(math.tan(robot_to_ball_distance_x_mm/robot_to_ball_distance_y_mm))
            
            self.MOTION.stop_motion()
            self.MOTION.turn(robot_to_ball_angle)
            
            while True:
                self.MOTION.walk_forward_continue()
                self.get_vision_all()
                robot_to_ball_distance_y_mm = BEV_FRAME_HEIGHT_MM - self.ball_coordinate_y
                
                if robot_to_ball_distance_y_mm < parameters.BALL_APPROACH_THRESHOLD:
                    self.MOTION.stop_motion()
                    robot_to_ball_distance_x_mm = self.ball_coordinate_x-(BEV_FRAME_WIDTH_MM/2)
                    self.MOTION.walk_sideway(robot_to_ball_distance_x_mm)
                    self.get_vision_all()
                    robot_to_ball_distance_y_mm = BEV_FRAME_HEIGHT_MM - self.ball_coordinate_y
                    self.MOTION.walk_forward(robot_to_ball_distance_y_mm)                    
                    self.MOTION.touch_ball()
                    #print("touched")
                elif self.ball_coordinate_x == 0 and self.ball_coordinate_y == 0:
                    self.MOTION.stop_motion()
                    break      
                print("3")
            print("2")          
                
        #print("NO BALL")
        
    def display_image(self):
        return self.result_image
    
            
            