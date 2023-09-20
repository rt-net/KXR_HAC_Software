import math
import time
import sys
import os

import numpy as np

from vision.vision_library import VisionLibrary
from motion_control.motion_control_library import MotionLibrary

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import parameterfile

class MotionPlanningLibrary:
    def __init__(self):
        self.VISION = VisionLibrary()
        self.MOTION = MotionLibrary()
        self.distance_from_the_edge_mm = parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM
        
        self.in_goal = False
        self.touched_ball = False
        
        print("[行動計画の初期化成功]")
        
    def calculate_distance_from_the_edge_mm(self, slope, intercept):
        """calculate current distance from robot-cog to edge
        """
        try:
            Ax = parameterfile.BEV_FRAME_WIDTH_MM/2
            Ay = parameterfile.BEV_FRAME_HEIGHT_MM + parameterfile.FOOT_CENTER_TO_BEV_FRAME_BOTTOM_DISTANCE
            
            Bx = (Ay-intercept)/slope
            By = Ay
            
            Cx = Ax
            Cy = Ax*slope+intercept
            
            AC = (Ay - Cy)
            AB = (Ax - Bx)
            BC = (AC**2 + AB**2)**(1/2)
            
            self.distance_from_the_edge_mm = (AB**2 - ((AB**2 - AC**2 +BC**2)/(2*BC))**2)**(1/2)
        except:
            self.distance_from_the_edge_mm = parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM
        
    def get_vision_all(self):
        """collect all vision data
        """
        self.VISION.calibrate_img()
        self.edge_angle, self.edge_slope, self.edge_intercept = self.VISION.detect_edge_using_numpy_calc()
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball()
        self.cornertype, self.corner_x_coordinate, self.corner_y_coordinate = self.VISION.detect_corner()
        self.goalline_angle, self.goalline_slope, self.goalline_intercept = self.VISION.detect_goal()
        self.result_image = self.VISION.display_resultimg()
        
    def align_with_field_edge(self):
        """align robot with field edge
        """
        self.get_vision_all() #画像情報取得
        
        if self.edge_angle != 0: #エッジのアングルが0以外の時→エッジが存在する時
            self.MOTION.turn(round(self.edge_angle)) #エッジと並ぶ角度まで旋回
        
        self.calculate_distance_from_the_edge_mm(self.edge_slope, self.edge_intercept) #エッジから機体中心までの距離を計算
        
        if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM or self.distance_from_the_edge_mm > parameterfile.WALK_PATH_TO_FIELD_EDGE_MAXIMUM_MM:
            if self.edge_angle > 0:
                self.MOTION.walk_sideway(-(self.distance_from_the_edge_mm-parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM))
            else:
                self.MOTION.walk_sideway((self.distance_from_the_edge_mm-parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM))
        
    def left_hand_approach(self):
        """execute a loop of left hand approach
        """
        # self.MOTION.stop_motion() #前進を終了
        # while True:
        self.get_vision_all()

        if self.edge_angle != 0:
            self.calculate_distance_from_the_edge_mm(self.edge_slope, self.edge_intercept)
        
        if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM:
            self.MOTION.stop_motion()
            self.align_with_field_edge()
            
        if self.MOTION.is_button_pressed == False:
            self.MOTION.walk_forward_continue()   
                           
            # if self.cornertype != "NONE":
            #     self.MOTION.stop_motion()
            #     print("Round corner")
            #     self.round_corner()
                
            # if self.ball_coordinate_x != 0:
            #     self.MOTION.stop_motion()
            #     break
                
    def approach_to_ball(self):
        """execute full ball approach process
        """
        self.update_distance_to_ball()        
        print("X", self.distance_to_ball_x_mm, "Y", self.distance_to_ball_y_mm)
        
        self.MOTION.turn(self.angle_to_ball_degrees) #ボールに正対するように旋回
        
        while True:
            if self.MOTION.is_button_pressed == False:
                self.MOTION.walk_forward_continue() #ボールに向けて前進開始
            
            self.update_distance_to_ball() #ボールとの位置関係をアップデート
            
            if self.distance_to_ball_y_mm < parameterfile.BALL_APPROACH_THRESHOLD: #ボールとの距離が閾値以下の時
                self.MOTION.stop_motion() #前進を終了
                self.MOTION.walk_sideway(self.distance_to_ball_x_mm) #ボールを目の前に置くまで横に歩行
                
                self.update_distance_to_ball() #ボールとの距離をもう一度アップデート
                
                print(self.distance_to_ball_y_mm)
                self.MOTION.walk_forward(self.distance_to_ball_y_mm) #ボールまでの残り距離を前進                    
                self.MOTION.touch_ball() #ボールに触れる
                
                if self.is_ball_touched() == True:
                    print("Touched Ball")
                    break
            elif self.ball_coordinate_x == 0 and self.ball_coordinate_y == 0: #ボールを見失ったとき
                self.MOTION.stop_motion() #前進を終了
                print("Ball Lost")
                break          
            
    def turn_to_ball_2(self):
        self.MOTION.stop_motion() #前進を終了
        self.update_distance_to_ball()
        self.MOTION.turn(self.angle_to_ball_degrees) #ボールに正対するように旋回
        
    def walk_to_ball_2(self):
        while True:
            if self.MOTION.is_button_pressed == False:
                self.MOTION.walk_forward_continue() #ボールに向けて前進開始
            
            self.update_distance_to_ball() #ボールとの位置関係をアップデート
            
            if self.distance_to_ball_y_mm < parameterfile.BALL_APPROACH_THRESHOLD: #ボールとの距離が閾値以下の時
                self.MOTION.stop_motion() #前進を終了
                self.MOTION.walk_sideway(self.distance_to_ball_x_mm) #ボールを目の前に置くまで横に歩行
                break
        
    def is_ball_touched(self):
        distance_to_ball_x_mm_old = self.distance_to_ball_x_mm
        distance_to_ball_y_mm_old = self.distance_to_ball_y_mm
        
        self.update_distance_to_ball() #ボールとの位置関係をアップデート
        
    def round_corner(self):
        self.get_vision_all()
        if self.cornertype == "NONE":
            print("None")
        elif self.cornertype == "RIGHT":
            self.MOTION.turn(90)
            print(-(parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM-self.corner_y_coordinate))
            self.MOTION.walk_sideway(-self.corner_y_coordinate)
        elif self.cornertype == "LEFT":
            self.MOTION.turn(-90)
            self.MOTION.walk_sideway(parameterfile.AVOID_CORNER_MM)

            
    def cross_goal(self):
        self.get_vision_all()
        if self.goalline_angle != 0:
            self.calculate_distance_from_the_edge_mm(self.goalline_slope, self.goalline_intercept)
            print("angle", self.goalline_angle)
            print(self.distance_from_the_edge_mm)
            
            if self.goalline_angle > 0:
                self.MOTION.turn(-(90-self.goalline_angle))
            else:
                self.MOTION.turn(90+self.goalline_angle)
            
            self.MOTION.walk_forward_timed(self.distance_from_the_edge_mm+parameterfile.WALK_PATH_TO_FIELD_EDGE_MAXIMUM_MM)
        self.in_goal = True
        
    def update_distance_to_ball(self):
            self.get_vision_all() #画像データ取得
            self.distance_to_ball_x_mm = self.ball_coordinate_x-(parameterfile.BEV_FRAME_WIDTH_MM/2) #ロボット中心からボールへのx方向距離
            self.distance_to_ball_y_mm = parameterfile.BEV_FRAME_HEIGHT_MM - self.ball_coordinate_y #ロボット中心からボールへのy方向距離
            self.angle_to_ball_degrees = math.degrees(math.tan(self.distance_to_ball_x_mm/self.distance_to_ball_y_mm)) #ロボット正面からボールへの角度を計算
            
    def touch_ball(self):
        self.MOTION.stop_motion() #前進を終了
        self.MOTION.touch_ball()
        self.touched_ball = True
        
    def display_image(self):
        return self.result_image

    def check_know_ball_pos(self):
        #print("check know ball pos")
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball()
        if self.ball_coordinate_x != 0:
            return True
        else:
            return False
    
    def check_facing_ball(self):
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball()
        #print("check facing ball")
        if self.ball_coordinate_x > (parameterfile.BEV_FRAME_WIDTH_MM/2-parameterfile.BALL_POS_TOLERANCE_MM) and self.ball_coordinate_x < (parameterfile.BEV_FRAME_WIDTH_MM/2+parameterfile.BALL_POS_TOLERANCE_MM):
            return True
        else:
            return False
    
    def check_near_ball(self):
        #print("check near ball")
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball()
        if self.ball_coordinate_y > parameterfile.BEV_FRAME_HEIGHT_MM-parameterfile.BALL_POS_FROM_ROBOT:
            return True
        else:
            return False
        
    def check_touched_ball(self):
        return self.touched_ball
    
    def check_in_goal(self):
        return False
    
    def check_near_goal(self):
        return False
    
    def check_facing_goal(self):
        self.goalline_angle, self.goalline_slope, self.goalline_intercept = self.VISION.detect_goal()
        if self.goalline_angle != 0:
            return True
        else:
            return False
    
    def in_goal_update(self):
        return self.in_goal