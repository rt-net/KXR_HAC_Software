import time

class MotionPlanningLibrarySample:
    def __init__(self):
        self.standing = False
        self.know_ball_pos = False
        self.facing_ball = False
        self.near_ball = False
        self.touched_ball = False
        self.facing_goal = False
        self.near_goal = False
        self.in_goal = False
    
    #実行するアクションの関数
    
    def stand_up(self): 
        print("------STAND UP------")
        self.standing = True
        time.sleep(1)

    def walk_in_field(self):
        print("------WALK AROUND------")
        self.know_ball_pos = True
        time.sleep(1)

    def turn_to_ball(self):
        print("------TURN TO BALL------")
        self.facing_ball = True
        time.sleep(1)
        
    def walk_to_ball(self):
        print("------WALK TO BALL------")
        self.near_ball = True
        time.sleep(1)

    def extend_arm(self):
        print("------EXTEND ARM------")
        self.touched_ball = True
        time.sleep(1)
        
    def turn(self):
        print("------TURN------")
        self.facing_goal = True
        self.near_goal = True
        time.sleep(1)
        
    def walk_into_goal(self):
        print("------WALK INTO GOAL------")
        self.in_goal = True
        time.sleep(1)
        
    #WorldState更新のために，周辺環境を確認するための関数
    #センサなどで確認した周囲の状況，各アクションの実行状況からT/FでWorldStateの状態を返す
        
    def check_know_ball_pos(self):
        if self.know_ball_pos==True:
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