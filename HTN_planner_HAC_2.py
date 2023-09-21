import sys
import htn_HAC
import copy

from motion_planning.motion_planning_library import MotionPlanningLibrary

PLANNING = MotionPlanningLibrary()

def stand_up():
    PLANNING.stand_up()

def walk_in_field():
    PLANNING.left_hand_approach()

def turn_to_ball():
    PLANNING.turn_to_ball()
    
def walk_to_ball():
    PLANNING.walk_to_ball()

def extend_arm():
    PLANNING.touch_ball()
    
def walk_into_goal():
    PLANNING.cross_goal()
    
def turn():
    PLANNING.turn_to_goal()

def walk_forward():
    pass

def check_know_ball_pos():
    return PLANNING.check_know_ball_pos()

def check_facing_ball():
    return PLANNING.check_facing_ball()

def check_near_ball():
    return PLANNING.check_near_ball()

def check_touched_ball():
    return PLANNING.check_touched_ball()

def check_facing_goal():
    return PLANNING.check_facing_goal()

def check_near_goal():
    return PLANNING.check_near_goal()

def check_in_goal():
    return PLANNING.check_in_goal()

def check_standing():
    return PLANNING.check_standing()

####WorldState####
world_state = htn_HAC.WorldState(standing = False,
                     know_ball_pos=False,
                     facing_ball=False,
                     near_ball=False,
                     touched_ball=False,
                     facing_goal=False,
                     near_goal=False,
                     in_goal=False) #htn_HAC.WorldStateクラスのインスタンス　初期値には全てFalseが入っている
world_state.set_update_functions(standing = check_standing,
                                 know_ball_pos = check_know_ball_pos,
                                 facing_ball = check_facing_ball,
                                 near_ball = check_near_ball,
                                 touched_ball = check_touched_ball,
                                 facing_goal = check_facing_goal,
                                 near_goal = check_near_goal,
                                 in_goal = check_in_goal) #world_state更新関数のセット

####PrimitiveTasks####
init_pos = htn_HAC.PrimitiveTask("StandUp") #htn_HAC.PrimitiveTaskクラスのインスタンス生成
init_pos.set_precondition(standing=False, know_ball_pos=False, in_goal=False, touched_ball=False) #辞書型でpreconditionを設定
init_pos.set_effects(standing=True) #辞書型でeffectを設定
init_pos.set_action(stand_up) #アクションの関数を指定

walk_around = htn_HAC.PrimitiveTask("WalkAround") #htn_HAC.PrimitiveTaskクラスのインスタンス生成
walk_around.set_precondition(know_ball_pos=False, in_goal=False, touched_ball=False) #辞書型でpreconditionを設定
walk_around.set_effects(know_ball_pos=True) #辞書型でeffectを設定
walk_around.set_action(walk_in_field) #アクションの関数を指定

face_ball = htn_HAC.PrimitiveTask("FaceBall") #htn_HAC.PrimitiveTaskクラスのインスタンス生成
face_ball.set_precondition(facing_ball=False, know_ball_pos=True, in_goal=False, touched_ball=False) #辞書型でpreconditionを設定
face_ball.set_effects(facing_ball=True) #辞書型でeffectを設定
face_ball.set_action(turn_to_ball) #アクションの関数を指定

approach_ball = htn_HAC.PrimitiveTask("ApproachBall")
approach_ball.set_precondition(near_ball = False, facing_ball=True, touched_ball=False)
approach_ball.set_effects(near_ball=True)
approach_ball.set_action(walk_to_ball) #アクションの関数を指定

touch_ball = htn_HAC.PrimitiveTask("TouchBall")
touch_ball.set_precondition(near_ball=True, touched_ball=False)
touch_ball.set_effects(touched_ball=True)
touch_ball.set_action(extend_arm) #アクションの関数を指定

turn_to_goal = htn_HAC.PrimitiveTask("TurntoGoal")
turn_to_goal.set_precondition(facing_goal=False, touched_ball=True)
turn_to_goal.set_effects(facing_goal=True)
turn_to_goal.set_action(turn) #アクションの関数を指定

walk_to_goal = htn_HAC.PrimitiveTask("WalktoGoal")
walk_to_goal.set_precondition(facing_goal=True, touched_ball=True)
walk_to_goal.set_effects(near_goal=True)
walk_to_goal.set_action(walk_in_field) #アクションの関数を指定

cross_goal = htn_HAC.PrimitiveTask("CrossGoal")
cross_goal.set_precondition(near_goal=True)
cross_goal.set_effects(in_goal=True)
cross_goal.set_action(walk_into_goal) #アクションの関数を指定

####Method####
find_ball = htn_HAC.Method("FindBall") #htn_HAC.Methodクラスのインスタンス生成
find_ball.set_precondition(know_ball_pos=False, in_goal=False) #辞書型でpreconditionを渡す
find_ball.set_subtask(init_pos, walk_around) #含まれるsubtaskをタプルで渡す

go_touch_ball = htn_HAC.Method("GoTouchBall") #htn_HAC.Methodクラスのインスタンス生成
go_touch_ball.set_precondition(know_ball_pos=True, in_goal=False) #辞書型でpreconditionを渡す
go_touch_ball.set_subtask(face_ball, approach_ball, touch_ball) #含まれるsubtaskをタプルで渡す

go_to_goal = htn_HAC.Method("GotoGoal") #htn_HAC.Methodクラスのインスタンス生成
go_to_goal.set_precondition(touched_ball=True, in_goal=False) #辞書型でpreconditionを渡す
go_to_goal.set_subtask(turn_to_goal, walk_to_goal, cross_goal) #含まれるsubtaskをタプルで渡す

####CompoundTasks####
root_task = htn_HAC.CompoundTask("HACStrategy") #htn_HAC.CompoundTaskクラスのインスタンス生成
root_task.set_method(find_ball, go_touch_ball, go_to_goal) #含まれるmethodをタプルで渡す　CompoundTaskにはpreconditionは無い？

# ########### HTNPlanner using Decomposed History ###########
planner = htn_HAC.Planner() #Plannerのインスタンス
world = copy.deepcopy(world_state) #world_stateクラスのコピー

while True: #メインループ
    print('\n\n'+"#"*5+"  Generate Plan With History  "+"#"*5)
    world.update_state_with_sensor_data()
    planner.make_plan([root_task], world) #root_taskについて、現在のworld_stateに基づいてプランを立てる
    planner.show_plan()
    planner.execute_plan(world)


########### HTNPlanner without using Decomposed History ###########
# planner = htn_HAC.Planner(use_history=False)
# world = copy.deepcopy(world_state)

# print(world.state)

# print("#"*5+"  Generate Plan Without History  "+"#"*5)
# planner.make_plan([root_task], world)
# planner.show_plan()
# planner.execute_plan(world)

# print(world.state)

# print('\n'+'REPlANNING'+'\n')

# planner.make_plan([root_task], world)
# planner.show_plan()
# planner.execute_plan(world)

# print(world.state)

# print('\n'+'REPlANNING'+'\n')

# planner.make_plan([root_task], world)
# planner.show_plan()
# planner.execute_plan(world)

# print(world.state)