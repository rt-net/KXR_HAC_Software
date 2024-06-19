import math
import time
import sys
import os

import numpy as np

from vision.vision_library import VisionLibrary
from motion_control.motion_control_library import MotionLibrary

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) #一つ上のディレクトリへの検索パスの追加
import parameterfile

class MotionPlanningLibrary: #MotionPlanningLibraryクラス
    def __init__(self):
        """initialize MotionPlanningLibrary class
        """
        self.VISION = VisionLibrary() #VisionLibraryクラスのインスタンス生成
        self.MOTION = MotionLibrary() #MotionLibraryクラスのインスタンス生成
        
        self.distance_from_the_edge_mm = parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM #エッジからの距離をデフォルトのものに設定
        
        #各WorldStateのT/Fの初期化
        self.in_goal = False
        self.touched_ball = False
        self.facing_goal = False
        self.is_standing = False
        
        print("[Motion planning initialized successfully]")

    ########## データ取得関数 ##########
    def get_vision_all(self):
        """collect all vision data
        """
        self.VISION.calibrate_img() #画像のキャリブレーション
        self.edge_angle, self.edge_slope, self.edge_intercept = self.VISION.detect_edge_using_numpy_calc() #フィールドのエッジ検出
        self.ball_coordinate_x_wide, self.ball_coordinate_y_wide = self.VISION.detect_ball_wide() #カメラ画角全体でボール検出
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball() #キャリブレーション後画像でボール検出
        self.cornertype = self.VISION.detect_corner_wide() #カメラ画角全体でフィールドのコーナー検出
        self.cornertype, self.corner_x_coordinate, self.corner_y_coordinate = self.VISION.detect_corner() #キャリブレーション後画像でフィールドのコーナー検出 
        self.goalline_angle, self.goalline_slope, self.goalline_intercept = self.VISION.detect_goal() #ゴールライン検出
        self.result_image = self.VISION.display_resultimg() #結果画像の取得
        
    def get_angle_to_goal(self):
        """get robot's angle towards goal using IMU
        """
        print(self.MOTION.get_body_angle())
        
    ########## 各種位置関係の幾何的計算関数 ##########
    def update_distance_to_ball(self):
        """update distance to ball
        """
        self.get_vision_all() #画像データ取得
        
        if self.ball_coordinate_x == 0 and self.ball_coordinate_y == 0: #鳥瞰図の中にボールが無いとき
            self.distance_to_ball_x_pixel = self.ball_coordinate_x_wide-(parameterfile.CAMERA_FRAME_WIDTH/2) #ロボット中心からボールへのx方向距離
            self.distance_to_ball_y_pixel = parameterfile.CAMERA_FRAME_HEIGHT - self.ball_coordinate_y_wide #ロボット中心からボールへのy方向距離
            self.angle_to_ball_degrees = parameterfile.ROBOT_TO_BALL_ANGLE_CONVERSION*(math.degrees(math.atan(self.distance_to_ball_x_pixel/self.distance_to_ball_y_pixel))) #ロボット正面からボールへの角度を計算
            self.distance_to_ball_y_pixel = parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM #ボールとの距離をデフォルト値に設定
        else:
            self.distance_to_ball_x_mm = self.ball_coordinate_x-(parameterfile.BEV_FRAME_WIDTH_MM/2) #ロボット中心からボールへのx方向距離
            self.distance_to_ball_y_mm = parameterfile.BEV_FRAME_HEIGHT_MM - self.ball_coordinate_y #ロボット中心からボールへのy方向距離
            self.angle_to_ball_degrees = math.degrees(math.atan(self.distance_to_ball_x_mm/self.distance_to_ball_y_mm)) #ロボット正面からボールへの角度を計算

    def calculate_distance_from_the_edge_mm(self, slope, intercept):
        """calculate current distance from robot-cog to field edge
        """
        try: #三角関数を用いて幾何的にエッジとの距離を算出
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
            self.distance_from_the_edge_mm = parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM #計算ができないとき（エッジが水平の時など）、デフォルトの距離を格納

    ########## Primitive Taskにおいてロボットを動かすための関数 ##########
    
    def align_with_field_edge(self): #エッジとの位置関係を正す
        """align robot with field edge
        """
        self.get_vision_all() #全てのカメラ情報を取得
        
        if self.edge_angle != 0: #エッジのアングルが0以外の時　→　エッジが存在する時
            self.MOTION.turn(round(parameterfile.ALIGN_WITH_FIELD_EDGE_CORRECTION_FACTOR*self.edge_angle)) #エッジと並ぶ角度まで旋回
        
        self.calculate_distance_from_the_edge_mm(self.edge_slope, self.edge_intercept) #エッジから機体の投影面積中心までの距離を計算
        
        if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM or self.distance_from_the_edge_mm > parameterfile.WALK_PATH_TO_FIELD_EDGE_MAXIMUM_MM: #エッジとの距離が下閾値より小さい、もしくは上閾値よりも大きい時
            if self.edge_angle > 0: #エッジのアングルが0度以上（ロボットとエッジの位置関係上、エッジが右側に存在）
                self.MOTION.walk_sideway(-(self.distance_from_the_edge_mm-parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM)) #サイドステップでエッジとの位置関係を閾値内に調整
            else: #エッジのアングルが0度以下（ロボットとエッジの位置関係上、エッジが左側に存在）
                self.MOTION.walk_sideway((self.distance_from_the_edge_mm-parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM)) #サイドステップでエッジとの位置関係を閾値内に調整
                
    def round_corner(self): #コーナーを曲がる
        """round corner depending on corner type and position
        """
        self.get_vision_all() #全てのカメラ情報を取得
        
        #cornertypeは文字型　認識結果の文字列の内容によって分岐　右手法（右のエッジに常に沿って移動）がベースになっていることに注意
        if self.cornertype == "NONE": #コーナーが無いとき
            print("NONE") 
        elif self.cornertype == "RIGHT": #右向きのコーナーがあるとき
            print("RIGHT")
            self.MOTION.walk_sideway(-100) #エッジから100mm離れる
            self.MOTION.walk_forward_timed(500) #500mm前進
            self.MOTION.turn(90) #90°右旋回
            #以上のシーケンスでコーナーを右に曲がる
        elif self.cornertype == "LEFT": #左向きのコーナーがあるとき
            print("LEFT")
            self.MOTION.turn(-90) #90°左旋回
            #以上のシーケンスでコーナーを左に曲がる
        elif self.cornertype == "RIGHT_WIDE":
            print("RIGHT_WIDE")
            while True: 
                self.get_vision_all()
                if self.edge_angle != 0:
                    self.calculate_distance_from_the_edge_mm(self.edge_slope, self.edge_intercept)
                
                if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM:
                    self.MOTION.stop_motion()
                    self.align_with_field_edge()
                    
                if self.MOTION.is_button_pressed == False:
                    self.MOTION.walk_forward_timed(100)
                    
                count = count+1
                
                if count > 4:
                    break
            self.MOTION.turn(90)
        elif self.cornertype == "LEFT_WIDE":
            print("LEFT_WIDE")
            self.MOTION.turn(-90)
                    
    ########## Primitive Tasks ##########
    
    def stand_up(self):
        """stand up from laying flat on back
        """
        self.MOTION.stand_up() #仰向けからの起き上がりモーションを再生
        time.sleep(parameterfile.ROBOT_LONG_PAUSE) #ロボットの姿勢が安定するまで待機
        self.MOTION.calibrate_IMU() #IMUのキャリブレーションを行う
        self.init_angle = self.get_angle_to_goal() #ロボットの初期角度を設定
        self.is_standing = True #worldstate関連の変数を変更
    
    def left_hand_approach(self):
        """execute a loop of left hand approach
        """
        self.get_vision_all() #全てのカメラ情報を取得
        
        if self.edge_angle == 0 and self.edge_intercept != 0: #エッジが目の前に真横に走っている時
            MOTION.turn(-90) #左に90°旋回
            
        if self.edge_angle != 0: #エッジのアングルが0度ではないとき　→　エッジに相対しているとき
            self.calculate_distance_from_the_edge_mm(self.edge_slope, self.edge_intercept) #エッジとの距離を計算
        
        if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM: #エッジとの距離が、下閾値を下回るとき
            self.MOTION.stop_motion() #再生中のモーションを停止
            self.align_with_field_edge() #エッジとの位置関係を調整
            
        if self.MOTION.is_button_pressed == False: #モーションがなにも再生されていないとき
            self.MOTION.walk_forward_continue() #前進
                           
        if self.cornertype != "NONE": #コーナーが存在するとき
            self.MOTION.stop_motion() #再生中のモーションを停止
            print("ROUND CORNER") 
            print(self.cornertype) 
            self.round_corner() #コーナーを曲がる

    def turn_to_ball(self): #ボールに正対する
        self.MOTION.stop_motion() #前進を終了
        self.update_distance_to_ball() #ボールとの位置関係を更新
        self.MOTION.turn(self.angle_to_ball_degrees) #ボールに正対するように旋回
        
    def walk_to_ball(self): #ボールに接近する
        self.update_distance_to_ball() #ボールとの位置関係を更新
        
        if self.MOTION.is_button_pressed == False: #前進中でない場合
            self.MOTION.walk_forward_continue() #ボールに向けて前進開始 
               
        if self.distance_to_ball_y_mm < parameterfile.BALL_APPROACH_THRESHOLD: #ボールとの距離が閾値以下の時
            self.MOTION.stop_motion() #前進を終了
            self.MOTION.walk_sideway(self.distance_to_ball_x_mm) #ボールが真正面にくるまでサイドステップ
                 
    def touch_ball(self): #ボールに触る
        self.MOTION.stop_motion() #再生中のモーションを終了
        self.MOTION.touch_ball() #ボールに触るモーションを再生
        self.touched_ball = True #ボールに触ったかどうかのT/Fを更新
            
    def turn_to_goal(self): #ゴールラインに正対する
        self.MOTION.stop_motion() #再生中のモーションを停止
        yaw, pitch, roll = self.MOTION.get_body_angle() #現在のロボット自身の絶対角度を取得（探索開始時の角度を基準とする）
        print(180-yaw) 
        self.MOTION.turn(360-yaw) #ゴールラインに正対するように旋回
        self.facing_goal=True #ゴールに向いているかどうかのT/Fを更新
    
    def cross_goal(self): #ゴールに入る
        self.get_vision_all() #全てのカメラ情報を取得
        self.MOTION.stop_motion() #再生中のモーションを停止
        
        if self.goalline_angle != 0 and self.goalline_intercept != 0: #ゴールの目の前にいるとき
            self.calculate_distance_from_the_edge_mm(self.goalline_slope, self.goalline_intercept) #ゴールラインとの距離を計算
            print("angle", self.goalline_angle)
            print(self.distance_from_the_edge_mm)
            
            #ゴールに正対するように旋回
            if self.goalline_angle > 0:
                self.MOTION.turn(-(90-self.goalline_angle))
            else:
                self.MOTION.turn(90+self.goalline_angle)
        
        count = 0
        
        while True: 
            self.get_vision_all() #全てのカメラ情報を取得
            
            if self.edge_angle != 0:
                self.calculate_distance_from_the_edge_mm(self.edge_slope, self.edge_intercept)
            
            if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_MINIMUM_MM:
                self.MOTION.stop_motion()
                self.align_with_field_edge()
                
            if self.MOTION.is_button_pressed == False:
                self.MOTION.walk_forward_timed(100)
                
            count = count+1
            
            if count > 2:
                break
            
        print("GOAL")
        self.in_goal = True #ゴールに入っているかどうかのT/Fを更新
        
    ########## World State更新関連の関数 ##########
    
    def check_know_ball_pos(self): #ボールの位置を把握しているかどうかのT/Fを返す
        self.get_vision_all() #全てのカメラ情報を取得
        if self.ball_coordinate_x_wide != 0:
            return True
        else:
            return False
    
    def check_facing_ball(self): #ボールに正対しているかどうかのT/Fを返す
        self.get_vision_all() #全てのカメラ情報を取得
        if self.ball_coordinate_x > (parameterfile.BEV_FRAME_WIDTH_MM/2-parameterfile.BALL_POS_TOLERANCE_MM) and self.ball_coordinate_x < (parameterfile.BEV_FRAME_WIDTH_MM/2+parameterfile.BALL_POS_TOLERANCE_MM):
            return True
        else:
            return False
    
    def check_near_ball(self): #ボールに十分近いかどうかのT/Fを返す
        self.VISION.calibrate_img()
        self.ball_coordinate_x, self.ball_coordinate_y = self.VISION.detect_ball()
        if self.ball_coordinate_y > parameterfile.BEV_FRAME_HEIGHT_MM-parameterfile.BALL_POS_FROM_ROBOT:
            return True
        else:
            return False
        
    def check_touched_ball(self): #ボールに触ったかどうかのT/Fを返す
        return self.touched_ball
    
    def check_in_goal(self): #ゴールに入っているかどうかのT/Fを返す
        return self.in_goal
    
    def check_near_goal(self): #ゴールに近いかどうかのT/Fを返す
        self.VISION.calibrate_img()
        self.goalline_angle, self.goalline_slope, self.goalline_intercept = self.VISION.detect_goal()
        if self.goalline_angle != 0 and self.goalline_slope != 0 and self.goalline_intercept != 0:
            self.calculate_distance_from_the_edge_mm(self.goalline_slope, self.goalline_intercept)
            if self.distance_from_the_edge_mm < parameterfile.WALK_PATH_TO_FIELD_EDGE_DEFAULT_MM+200:
                return True
            else:
                return False
        else:
            return False
    
    def check_facing_goal(self): #ゴールの正面にいるかどうかのT/Fを返す
        return self.facing_goal
        
    def check_standing(self): #機体が立っているかどうかのT/Fを返す
        return self.is_standing
        
    def display_image(self): 
        return self.result_image