import sys
import htn_HAC
import copy

# from vision.vision_library import VisionLibrary
# from motion_control.motion_control_library import MotionLibrary
from motion_planning.motion_planning_library import MotionPlanningLibrary

# VISION = VisionLibrary()
# MOTION = MotionLibrary()
PLANNING = MotionPlanningLibrary()

def walk_in_field():
    PLANNING.left_hand_approach()

def turn_to_ball():
    PLANNING.turn_to_ball_2()
    
def walk_to_ball():
    PLANNING.walk_to_ball_2()

def extend_arm():
    PLANNING.touch_ball()
    
def walk_into_goal():
    PLANNING.cross_goal()

####WorldState####
world_state = htn_HAC.WorldState(know_ball_pos=False,
                     facing_ball=False,
                     near_ball=False,
                     touched_ball=False,
                     facing_goal=False,
                     near_goal=False,
                     in_goal=False) #htn_HAC.WorldStateクラスのインスタンス　初期値には全てFalseが入っている

####PrimitiveTasks####
walk_around = htn_HAC.PrimitiveTask("WalkAround") #htn_HAC.PrimitiveTaskクラスのインスタンス生成
walk_around.set_precondition(know_ball_pos=False) #辞書型でpreconditionを設定
walk_around.set_effects(know_ball_pos=True) #辞書型でeffectを設定
walk_around.set_action(walk_in_field) #アクションの関数を指定

face_ball = htn_HAC.PrimitiveTask("FaceBall") #htn_HAC.PrimitiveTaskクラスのインスタンス生成
face_ball.set_precondition(facing_ball=False, in_goal=False) #辞書型でpreconditionを設定
face_ball.set_effects(facing_ball=True) #辞書型でeffectを設定
face_ball.set_action(turn_to_ball) #アクションの関数を指定

approach_ball = htn_HAC.PrimitiveTask("ApproachBall")
approach_ball.set_precondition(facing_ball=True, in_goal=False)
approach_ball.set_effects(near_ball=True)
approach_ball.set_action(walk_to_ball) #アクションの関数を指定

touch_ball = htn_HAC.PrimitiveTask("TouchBall")
touch_ball.set_precondition(near_ball=True, in_goal=False)
touch_ball.set_effects(touched_ball=True)
touch_ball.set_action(extend_arm) #アクションの関数を指定

turn_to_goal = htn_HAC.PrimitiveTask("TurntoGoal")
turn_to_goal.set_precondition(facing_goal=False)
turn_to_goal.set_effects(facing_goal=True)
turn_to_goal.set_action(extend_arm) #アクションの関数を指定

walk_to_goal = htn_HAC.PrimitiveTask("CrossGoal")
walk_to_goal.set_precondition(facing_goal=True)
walk_to_goal.set_effects(near_goal=True)
walk_to_goal.set_action(extend_arm) #アクションの関数を指定

cross_goal = htn_HAC.PrimitiveTask("CrossGoal")
cross_goal.set_precondition(near_goal=True)
cross_goal.set_effects(in_goal=True)
cross_goal.set_action(walk_into_goal) #アクションの関数を指定

####Method####
find_ball = htn_HAC.Method("FindBall") #htn_HAC.Methodクラスのインスタンス生成
find_ball.set_precondition(know_ball_pos=False, in_goal=False) #辞書型でpreconditionを渡す
find_ball.set_subtask(walk_around) #含まれるsubtaskをタプルで渡す

go_touch_ball = htn_HAC.Method("GoTouchBall") #htn_HAC.Methodクラスのインスタンス生成
go_touch_ball.set_precondition(know_ball_pos=True, in_goal=False) #辞書型でpreconditionを渡す
go_touch_ball.set_subtask(face_ball, approach_ball, touch_ball) #含まれるsubtaskをタプルで渡す

go_to_goal = htn_HAC.Method("GotoGoal") #htn_HAC.Methodクラスのインスタンス生成
go_to_goal.set_precondition(touched_ball=True, in_goal=False) #辞書型でpreconditionを渡す
go_to_goal.set_subtask(turn_to_goal, walk_to_goal, cross_goal) #含まれるsubtaskをタプルで渡す

####CompoundTasks####
root_task = htn_HAC.CompoundTask("HACStrategy") #htn_HAC.CompoundTaskクラスのインスタンス生成
root_task.set_method(find_ball, go_touch_ball, go_to_goal) #含まれるmethodをタプルで渡す　CompoundTaskにはpreconditionは無い？



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



# ########### HTNPlanner using Decomposed History ###########
planner = htn_HAC.Planner()
world = copy.deepcopy(world_state)

print('\n\n'+"#"*5+"  Generate Plan With History  "+"#"*5)
planner.make_plan([root_task], world_state)
planner.show_plan()
planner.execute_plan(world)