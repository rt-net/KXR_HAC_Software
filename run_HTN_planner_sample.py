import sys
import HTN_planner
import copy

standing = False,
know_ball_pos=False,
facing_ball=False,
near_ball=False,
touched_ball=False,
facing_goal=False,
near_goal=False,
in_goal=False

####primitive_taskで実行するアクションをそれぞれ関数化####
def stand_up(): 
    print("[STAND UP]")
    standing = True

def walk_in_field():
    print("[WALK AROUND]")
    know_ball_pos = True

def turn_to_ball():
    print("[TURN TO BALL]")
    facing_ball = True
    
def walk_to_ball():
    print("[WALK TO BALL]")
    near_ball = True

def extend_arm():
    print("[EXTEND ARM]")
    touched_ball = True
    
def turn():
    print("[TURN TO GOAL]")
    facing_goal = True
    near_goal = True
    
def walk_into_goal():
    print("[WALK INTO GOAL]")
    in_goal = True

####world_state更新関数をそれぞれ作成####
def check_know_ball_pos():
    return know_ball_pos

def check_facing_ball():
    return facing_ball

def check_near_ball():
    return near_ball

def check_touched_ball():
    return touched_ball

def check_facing_goal():
    return facing_goal

def check_near_goal():
    return near_goal

def check_in_goal():
    return in_goal

def check_standing():
    return True

####WorldState####
world_state = HTN_planner.WorldState(standing = False,
                     know_ball_pos=False,
                     facing_ball=False,
                     near_ball=False,
                     touched_ball=False,
                     facing_goal=False,
                     near_goal=False,
                     in_goal=False) #HTN_planner.WorldStateクラスのインスタンス　初期値には全てFalseが入っている
world_state.set_update_functions(standing = check_standing,
                                 know_ball_pos = check_know_ball_pos,
                                 facing_ball = check_facing_ball,
                                 near_ball = check_near_ball,
                                 touched_ball = check_touched_ball,
                                 facing_goal = check_facing_goal,
                                 near_goal = check_near_goal,
                                 in_goal = check_in_goal) #それぞれのworld_stateについて、更新用の関数をセットする

####PrimitiveTasks####
init_pos = HTN_planner.PrimitiveTask("StandUp") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
init_pos.set_precondition(standing=False, know_ball_pos=False, in_goal=False, touched_ball=False) #辞書型でpreconditionを設定
init_pos.set_effects(standing=True) #辞書型でeffectを設定
init_pos.set_action(stand_up) #アクションの関数を指定

walk_around = HTN_planner.PrimitiveTask("WalkAround") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
walk_around.set_precondition(know_ball_pos=False, in_goal=False, touched_ball=False) #辞書型でpreconditionを設定
walk_around.set_effects(know_ball_pos=True) #辞書型でeffectを設定
walk_around.set_action(walk_in_field) #アクションの関数を指定

face_ball = HTN_planner.PrimitiveTask("FaceBall") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
face_ball.set_precondition(facing_ball=False, know_ball_pos=True, in_goal=False, touched_ball=False) #辞書型でpreconditionを設定
face_ball.set_effects(facing_ball=True) #辞書型でeffectを設定
face_ball.set_action(turn_to_ball) #アクションの関数を指定

approach_ball = HTN_planner.PrimitiveTask("ApproachBall") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
approach_ball.set_precondition(near_ball = False, facing_ball=True, touched_ball=False) #辞書型でpreconditionを設定
approach_ball.set_effects(near_ball=True) #辞書型でeffectを設定
approach_ball.set_action(walk_to_ball) #アクションの関数を指定

touch_ball = HTN_planner.PrimitiveTask("TouchBall") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
touch_ball.set_precondition(near_ball=True, touched_ball=False) #辞書型でpreconditionを設定
touch_ball.set_effects(touched_ball=True) #辞書型でeffectを設定
touch_ball.set_action(extend_arm) #アクションの関数を指定

turn_to_goal = HTN_planner.PrimitiveTask("TurntoGoal") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
turn_to_goal.set_precondition(facing_goal=False, touched_ball=True) #辞書型でpreconditionを設定
turn_to_goal.set_effects(facing_goal=True) #辞書型でeffectを設定
turn_to_goal.set_action(turn) #アクションの関数を指定

walk_to_goal = HTN_planner.PrimitiveTask("WalktoGoal") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
walk_to_goal.set_precondition(facing_goal=True, touched_ball=True) #辞書型でpreconditionを設定
walk_to_goal.set_effects(near_goal=True) #辞書型でeffectを設定
walk_to_goal.set_action(walk_in_field) #アクションの関数を指定

cross_goal = HTN_planner.PrimitiveTask("CrossGoal") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
cross_goal.set_precondition(near_goal=True) #辞書型でpreconditionを設定
cross_goal.set_effects(in_goal=True) #辞書型でeffectを設定
cross_goal.set_action(walk_into_goal) #アクションの関数を指定

####Method####
find_ball = HTN_planner.Method("FindBall") #HTN_planner.Methodクラスのインスタンス生成
find_ball.set_precondition(know_ball_pos=False, in_goal=False) #辞書型でpreconditionを渡す
find_ball.set_subtask(init_pos, walk_around) #含まれるsubtaskをタプルで渡す

go_touch_ball = HTN_planner.Method("GoTouchBall") #HTN_planner.Methodクラスのインスタンス生成
go_touch_ball.set_precondition(know_ball_pos=True, in_goal=False) #辞書型でpreconditionを渡す
go_touch_ball.set_subtask(face_ball, approach_ball, touch_ball) #含まれるsubtaskをタプルで渡す

go_to_goal = HTN_planner.Method("GotoGoal") #HTN_planner.Methodクラスのインスタンス生成
go_to_goal.set_precondition(touched_ball=True, in_goal=False) #辞書型でpreconditionを渡す
go_to_goal.set_subtask(turn_to_goal, walk_to_goal, cross_goal) #含まれるsubtaskをタプルで渡す

####CompoundTasks####
root_task = HTN_planner.CompoundTask("HACStrategy") #HTN_planner.CompoundTaskクラスのインスタンス生成
root_task.set_method(find_ball, go_touch_ball, go_to_goal) #含まれるmethodをタプルで渡す　CompoundTaskにはpreconditionは無い？

# ########### HTNPlanner using Decomposed History ###########
planner = HTN_planner.Planner() #Plannerのインスタンス
world = copy.deepcopy(world_state) #world_stateクラスのコピー

while True: #メインループ
    print('\n\n'+"#"*5+"  Generate Plan With History  "+"#"*5)
    world.update_state_with_sensor_data()
    planner.make_plan([root_task], world) #root_taskについて、現在のworld_stateに基づいてプランを立てる
    planner.show_plan()
    planner.execute_plan(world) #プランを実行する