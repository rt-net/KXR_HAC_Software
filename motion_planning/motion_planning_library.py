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
        self.facing_goal = False
        self.is_standing = False
        
        print("[行動計画の初期化成功]")

    ########## データ取得関数 ##########
 
    def get_vision_all(self):
        """collect all vision data
        """
        self.VISION.calibrate_img()
        self.edge_angle, self.edge_slope, self.edge_intercept = self.VISION.detect_edge_using_numpy_calc()
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball()
        self.ball_coordinate_x_wide, self.ball_coordinate_y_wide = self.VISION.detect_ball_wide()
        self.cornertype = self.VISION.detect_corner_wide()
        self.cornertype, self.corner_x_coordinate, self.corner_y_coordinate = self.VISION.detect_corner()
        self.goalline_angle, self.goalline_slope, self.goalline_intercept = self.VISION.detect_goal()
        self.result_image = self.VISION.display_resultimg()
        
    def get_angle_to_goal(self):
        print(self.MOTION.get_body_angle())
        
    ########## 各種位置関係の幾何的計算関数 ##########
     
    def update_distance_to_ball(self):
        self.get_vision_all() #画像データ取得
        if self.ball_coordinate_x == 0 and self.ball_coordinate_y == 0: #鳥瞰図の中にボールが無いとき
            print(self.ball_coordinate_x_wide, self.ball_coordinate_y_wide)
            self.distance_to_ball_x_pixel = self.ball_coordinate_x_wide-(parameterfile.CAMERA_FRAME_WIDTH/2) #ロボット中心からボールへのx方向距離
            self.distance_to_ball_y_pixel = parameterfile.CAMERA_FRAME_HEIGHT - self.ball_coordinate_y_wide #ロボット中心からボールへのy方向距離
            self.angle_to_ball_degrees = 0.7*(math.degrees(math.atan(self.distance_to_ball_x_pixel/self.distance_to_ball_y_pixel))) #ロボット正面からボールへの角度を計算
            self.distance_to_ball_y_mm = 150
            print("BALL FAR")
        else:
            self.distance_to_ball_x_mm = self.ball_coordinate_x-(parameterfile.BEV_FRAME_WIDTH_MM/2) #ロボット中心からボールへのx方向距離
            self.distance_to_ball_y_mm = parameterfile.BEV_FRAME_HEIGHT_MM - self.ball_coordinate_y #ロボット中心からボールへのy方向距離
            self.angle_to_ball_degrees = math.degrees(math.atan(self.distance_to_ball_x_mm/self.distance_to_ball_y_mm)) #ロボット正面からボールへの角度を計算

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

    ########## Primitive Task内でロボットを動かすための関数 ##########
    
    def align_with_field_edge(self): #エッジとの位置関係を正す
        """align robot with field edge
        """
        self.get_vision_all() #画像情報取得
        
        if self.edge_angle != 0: #エッジのアングルが0以外の時→エッジが存在する時
            self.MOTION.turn(round(0.8*self.edge_angle)) #エッジと並ぶ角度まで旋回
        
        self.calculate_distance_from_the_edge_mm(self.edge_slope, self.edge_intercept) #エッジから機体中心までの距離を計算
        
        if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM or self.distance_from_the_edge_mm > parameterfile.WALK_PATH_TO_FIELD_EDGE_MAXIMUM_MM:
            if self.edge_angle > 0:
                self.MOTION.walk_sideway(-(self.distance_from_the_edge_mm-parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM))
            else:
                self.MOTION.walk_sideway((self.distance_from_the_edge_mm-parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM))
                
    def round_corner(self): #コーナーを曲がる（テスト中）
        self.get_vision_all()
        self.MOTION.turn(round(self.edge_angle))
        if self.cornertype == "NONE":
            print("NONE")
        elif self.cornertype == "RIGHT":
            print("RIGHT")
            self.MOTION.turn(80)
            self.MOTION.walk_sideway(-(self.corner_y_coordinate+parameterfile.AVOID_CORNER_MM))
        elif self.cornertype == "LEFT":
            print("LEFT")
            self.MOTION.turn(-80)
            #print(self.corner_y_coordinate-parameterfile.AVOID_CORNER_MM)
            #self.MOTION.walk_sideway(self.corner_y_coordinate-parameterfile.AVOID_CORNER_MM)
        elif self.cornertype == "RIGHT_WIDE":
            print("RIGHT_WIDE")
            self.MOTION.turn(80)
            self.MOTION.walk_sideway(-250)
        elif self.cornertype == "LEFT_WIDE":
            print("LEFT_WIDE")
            self.MOTION.turn(-80)
                    
    ########## Primitive Task ##########
    
    def stand_up(self):
        self.MOTION.stand_up()
        time.sleep(parameterfile.ROBOT_LONG_PAUSE)
        self.MOTION.calibrate_IMU()
        self.init_angle = self.get_angle_to_goal()
        self.is_standing = True
    
    def left_hand_approach(self):
        """execute a loop of left hand approach
        """
        self.get_vision_all()
        if self.edge_angle != 0:
            self.calculate_distance_from_the_edge_mm(self.edge_slope, self.edge_intercept)
        
        if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM:
            self.MOTION.stop_motion()
            self.align_with_field_edge()
            
        if self.MOTION.is_button_pressed == False:
            self.MOTION.walk_forward_continue()   
                           
        if self.cornertype != "NONE":
            self.MOTION.stop_motion()
            print("ROUND CORNER")
            self.round_corner()

    def turn_to_ball(self):
        self.MOTION.stop_motion() #前進を終了
        self.update_distance_to_ball() #ボールとの位置関係をアップデート
        self.MOTION.turn(self.angle_to_ball_degrees) #ボールに正対するように旋回
        
    def walk_to_ball(self): #ボールにアプローチする
        self.update_distance_to_ball() #ボールとの位置関係をアップデート
        
        if self.MOTION.is_button_pressed == False: #前進中でない場合
            self.MOTION.walk_forward_continue() #ボールに向けて前進開始 
               
        if self.distance_to_ball_y_mm < parameterfile.BALL_APPROACH_THRESHOLD: #ボールとの距離が閾値以下の時
            self.MOTION.stop_motion() #前進を終了
            self.MOTION.walk_sideway(self.distance_to_ball_x_mm) #ボールを目の前に置くまで横に歩行
                 
    def touch_ball(self):
        self.MOTION.stop_motion() #前進を終了
        self.MOTION.touch_ball() #ボールに触るモーションを再生
        self.touched_ball = True #ボールに触ったかどうかのステータスを更新
            
    def turn_to_goal(self):
        self.MOTION.stop_motion()
        yaw, pitch, roll = self.MOTION.get_body_angle()
        print(180-yaw)
        self.MOTION.turn(240)#-yaw)
        self.facing_goal=True
    
    def cross_goal(self):
        self.get_vision_all()
        self.MOTION.stop_motion()
        if self.goalline_angle != 0:
            self.calculate_distance_from_the_edge_mm(self.goalline_slope, self.goalline_intercept)
            print("angle", self.goalline_angle)
            print(self.distance_from_the_edge_mm)
            
            if self.goalline_angle > 0:
                self.MOTION.turn(-(90-self.goalline_angle))
            else:
                self.MOTION.turn(90+self.goalline_angle)
            
        self.MOTION.walk_forward_timed(self.distance_from_the_edge_mm+parameterfile.WALK_PATH_TO_FIELD_EDGE_MAXIMUM_MM)
        print("GOAL")
        self.in_goal = True
        
    ########## World State更新関連の関数 ##########
    
    def check_know_ball_pos(self):
        self.get_vision_all() #画像データ取得
        if self.ball_coordinate_x_wide != 0:
            return True
        else:
            return False
    
    def check_facing_ball(self):
        self.get_vision_all() #画像データ取得
        if self.ball_coordinate_x > (parameterfile.BEV_FRAME_WIDTH_MM/2-parameterfile.BALL_POS_TOLERANCE_MM) and self.ball_coordinate_x < (parameterfile.BEV_FRAME_WIDTH_MM/2+parameterfile.BALL_POS_TOLERANCE_MM):
            return True
        elif self.ball_coordinate_x_wide > (parameterfile.CAMERA_FRAME_WIDTH/2-parameterfile.BALL_POS_TOLERANCE_MM) and self.ball_coordinate_x < (parameterfile.CAMERA_FRAME_WIDTH/2+parameterfile.BALL_POS_TOLERANCE_MM):
            return True
        else:
            return False
    
    def check_near_ball(self):
        #print("check near ball")
        self.VISION.calibrate_img()
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball()
        if self.ball_coordinate_y > parameterfile.BEV_FRAME_HEIGHT_MM-parameterfile.BALL_POS_FROM_ROBOT:
            return True
        else:
            return False
        
    def check_touched_ball(self):
        return self.touched_ball
    
    def check_in_goal(self):
        return self.in_goal
    
    def check_near_goal(self):
        self.VISION.calibrate_img()
        self.goalline_angle, self.goalline_slope, self.goalline_intercept = self.VISION.detect_goal()
        self.calculate_distance_from_the_edge_mm(self.goalline_slope, self.goalline_intercept)
        
        if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM:
            return True
        else:
            return False
    
    def check_facing_goal(self):
        return self.facing_goal
        # self.VISION.calibrate_img()
        # self.goalline_angle, self.goalline_slope, self.goalline_intercept = self.VISION.detect_goal()
        
        # if self.goalline_angle != 0 or self.facing_goal == True:
        #     return True
        # else:
        #     return False
        
    def check_standing(self):
        return self.is_standing
        
    def display_image(self):
        return self.result_image
    
    
    # def approach_to_ball(self):
    #     """execute full ball approach process
    #     """
    #     self.update_distance_to_ball()        
    #     print("X", self.distance_to_ball_x_mm, "Y", self.distance_to_ball_y_mm)
        
    #     self.MOTION.turn(self.angle_to_ball_degrees) #ボールに正対するように旋回
        
    #     while True:
    #         if self.MOTION.is_button_pressed == False:
    #             self.MOTION.walk_forward_continue() #ボールに向けて前進開始
            
    #         self.update_distance_to_ball() #ボールとの位置関係をアップデート
            
    #         if self.distance_to_ball_y_mm < parameterfile.BALL_APPROACH_THRESHOLD: #ボールとの距離が閾値以下の時
    #             self.MOTION.stop_motion() #前進を終了
    #             self.MOTION.walk_sideway(self.distance_to_ball_x_mm) #ボールを目の前に置くまで横に歩行
                
    #             self.update_distance_to_ball() #ボールとの距離をもう一度アップデート
                
    #             print(self.distance_to_ball_y_mm)
    #             self.MOTION.walk_forward(self.distance_to_ball_y_mm) #ボールまでの残り距離を前進                    
    #             self.MOTION.touch_ball() #ボールに触れる
                
    #             if self.is_ball_touched() == True:
    #                 print("Touched Ball")
    #                 break
    #         elif self.ball_coordinate_x == 0 and self.ball_coordinate_y == 0: #ボールを見失ったとき
    #             self.MOTION.stop_motion() #前進を終了
    #             print("Ball Lost")
    #             break          
    
            
    # def is_ball_touched(self):
    #     distance_to_ball_x_mm_old = self.distance_to_ball_x_mm
    #     distance_to_ball_y_mm_old = self.distance_to_ball_y_mm
    
    #     self.update_distance_to_ball() #ボールとの位置関係をアップデート