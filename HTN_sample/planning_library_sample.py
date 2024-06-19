import time
import random

class MotionPlanningLibrarySample:
    def __init__(self): 
        #それぞれのWorldStateに相当する変数を用意　初期値はFalse
        self.standing = False
        self.know_ball_pos = False
        self.facing_ball = False
        self.near_ball = False
        self.touched_ball = False
        self.facing_goal = False
        self.near_goal = False
        self.in_goal = False
    
    ###実行するアクションの関数を用意
    def stand_up(self): 
        print("------STAND UP------") #実行していることをターミナルに示す　実際にロボットを動作する際は，ロボットを動かす関数が入る
        self.standing = random.choice([True, False]) #そのアクションの成功/失敗（＝WorldStateが変わったか）をランダムに想定　実際にロボットを動かす際は，この行は必要ない
        time.sleep(1)

    def walk_in_field(self):
        print("------WALK AROUND------")
        self.know_ball_pos = random.choice([True, False])
        time.sleep(1)

    def turn_to_ball(self):
        print("------TURN TO BALL------")
        self.facing_ball = random.choice([True, False])
        time.sleep(1)
        
    def walk_to_ball(self):
        print("------WALK TO BALL------")
        self.near_ball = random.choice([True, False])
        time.sleep(1)

    def extend_arm(self):
        print("------EXTEND ARM------")
        self.touched_ball = random.choice([True, False])
        time.sleep(1)
        
    def turn(self):
        print("------TURN------")
        self.facing_goal = random.choice([True, False])
        self.near_goal = random.choice([True, False])
        time.sleep(1)
        
    def walk_into_goal(self):
        print("------WALK INTO GOAL------")
        self.in_goal = random.choice([True, False])
        time.sleep(1)
        
    ###WorldState更新のために，周辺環境を確認するための関数
    ###センサなどで確認した周囲の状況，各アクションの実行状況からT/FでWorldStateの状態を返す 
    def check_know_ball_pos(self):
        if self.know_ball_pos==True: #19行目でランダムに成功/失敗を決定したWorldStateのbool値に従って値を返す　実際にロボットを動かす際は，画像認識等でWorldStateの状態を確認する
            return True
        else:
            return False

    def check_facing_ball(self):
        if self.facing_ball==True:
            return True
        else:
            return False

    def check_near_ball(self):
        if self.near_ball==True:
            return True
        else:
            return False

    def check_touched_ball(self):
        if self.touched_ball==True:
            return True
        else:
            return False

    def check_facing_goal(self):
        if self.facing_goal==True:
            return True
        else:
            return False

    def check_near_goal(self):
        if self.near_goal==True:
            return True
        else:
            return False

    def check_in_goal(self):
        if self.in_goal==True:
            return True
        else:
            return False

    def check_standing(self):
        if self.standing==True:
            return True
        else:
            return False