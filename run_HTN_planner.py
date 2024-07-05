import sys
import HTN_planner
import copy

from motion_planning.motion_planning_library import MotionPlanningLibrary

PLANNING = MotionPlanningLibrary() #MotionPlanningLibraryのインスタンス

####WorldState####
world_state = HTN_planner.WorldState(WS_standing=False,
                                     WS_know_ball_pos=False,
                                     WS_facing_ball=False,
                                     WS_near_ball=False,
                                     WS_touched_ball=False,
                                     WS_facing_goal=False,
                                     WS_near_goal=False,
                                     WS_in_goal=False) #HTN_planner.WorldStateクラスのインスタンス　初期値には全てFalseが入っている

def check_standing():
    return PLANNING.check_standing()
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

world_state.set_update_functions(WS_standing=check_standing,
                                 WS_know_ball_pos=check_know_ball_pos,
                                 WS_facing_ball=check_facing_ball,
                                 WS_near_ball=check_near_ball,
                                 WS_touched_ball=check_touched_ball,
                                 WS_facing_goal=check_facing_goal,
                                 WS_near_goal=check_near_goal,
                                 WS_in_goal=check_in_goal) #それぞれのworld_stateについて、更新陽の関数をセットする

####PrimitiveTasks####
PT_init_pos = HTN_planner.PrimitiveTask("StandUp") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
PT_init_pos.set_precondition(WS_standing=False, WS_know_ball_pos=False, WS_in_goal=False, WS_touched_ball=False) #辞書型でpreconditionを設定
PT_init_pos.set_effects(WS_standing=True) #辞書型でeffectを設定
PT_init_pos.set_action(PLANNING.stand_up) #アクションの関数を指定

PT_walk_around = HTN_planner.PrimitiveTask("WalkAround") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
PT_walk_around.set_precondition(WS_know_ball_pos=False, WS_in_goal=False, WS_touched_ball=False) #辞書型でpreconditionを設定
PT_walk_around.set_effects(WS_know_ball_pos=True) #辞書型でeffectを設定
PT_walk_around.set_action(PLANNING.left_hand_approach) #アクションの関数を指定

PT_face_ball = HTN_planner.PrimitiveTask("FaceBall") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
PT_face_ball.set_precondition(WS_facing_ball=False, WS_know_ball_pos=True, WS_in_goal=False, WS_touched_ball=False) #辞書型でpreconditionを設定
PT_face_ball.set_effects(WS_facing_ball=True) #辞書型でeffectを設定
PT_face_ball.set_action(PLANNING.turn_to_ball) #アクションの関数を指定

PT_approach_ball = HTN_planner.PrimitiveTask("ApproachBall") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
PT_approach_ball.set_precondition(WS_near_ball = False, WS_facing_ball=True, WS_touched_ball=False) #辞書型でpreconditionを設定
PT_approach_ball.set_effects(WS_near_ball=True) #辞書型でeffectを設定
PT_approach_ball.set_action(PLANNING.walk_to_ball) #アクションの関数を指定

PT_touch_ball = HTN_planner.PrimitiveTask("TouchBall") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
PT_touch_ball.set_precondition(WS_near_ball=True, WS_touched_ball=False) #辞書型でpreconditionを設定
PT_touch_ball.set_effects(WS_touched_ball=True) #辞書型でeffectを設定
PT_touch_ball.set_action(PLANNING.touch_ball) #アクションの関数を指定

PT_turn_to_goal = HTN_planner.PrimitiveTask("TurntoGoal") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
PT_turn_to_goal.set_precondition(WS_facing_goal=False, WS_touched_ball=True) #辞書型でpreconditionを設定
PT_turn_to_goal.set_effects(WS_facing_goal=True) #辞書型でeffectを設定
PT_turn_to_goal.set_action(PLANNING.turn_to_goal) #アクションの関数を指定

PT_walk_to_goal = HTN_planner.PrimitiveTask("WalktoGoal") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
PT_walk_to_goal.set_precondition(WS_facing_goal=True, WS_touched_ball=True) #辞書型でpreconditionを設定
PT_walk_to_goal.set_effects(WS_near_goal=True) #辞書型でeffectを設定
PT_walk_to_goal.set_action(PLANNING.left_hand_approach) #アクションの関数を指定

PT_cross_goal = HTN_planner.PrimitiveTask("CrossGoal") #HTN_planner.PrimitiveTaskクラスのインスタンス生成
PT_cross_goal.set_precondition(WS_near_goal=True) #辞書型でpreconditionを設定
PT_cross_goal.set_effects(WS_in_goal=True) #辞書型でeffectを設定
PT_cross_goal.set_action(PLANNING.cross_goal) #アクションの関数を指定

####Method####
M_find_ball = HTN_planner.Method("FindBall") #HTN_planner.Methodクラスのインスタンス生成
M_find_ball.set_precondition(WS_know_ball_pos=False, WS_in_goal=False) #辞書型でpreconditionを渡す
M_find_ball.set_subtask(PT_init_pos, PT_walk_around) #含まれるsubtaskをタプルで渡す

M_go_touch_ball = HTN_planner.Method("GoTouchBall") #HTN_planner.Methodクラスのインスタンス生成
M_go_touch_ball.set_precondition(WS_know_ball_pos=True, WS_in_goal=False) #辞書型でpreconditionを渡す
M_go_touch_ball.set_subtask(PT_face_ball, PT_approach_ball, PT_touch_ball) #含まれるsubtaskをタプルで渡す

M_go_to_goal = HTN_planner.Method("GotoGoal") #HTN_planner.Methodクラスのインスタンス生成
M_go_to_goal.set_precondition(WS_touched_ball=True, WS_in_goal=False) #辞書型でpreconditionを渡す
M_go_to_goal.set_subtask(PT_turn_to_goal, PT_walk_to_goal, PT_cross_goal) #含まれるsubtaskをタプルで渡す

####CompoundTasks####
CT_root_task = HTN_planner.CompoundTask("HACStrategy") #HTN_planner.CompoundTaskクラスのインスタンス生成
CT_root_task.set_method(M_find_ball, M_go_touch_ball, M_go_to_goal) #含まれるmethodをタプルで渡す　CompoundTaskにはpreconditionは無い？

# ########### HTNPlanner using Decomposed History ###########
planner = HTN_planner.Planner() #Plannerのインスタンス

# a = copy.deepcopy(world_state)
while True: #メインループ
    print('\n\n'+"#"*5+"  Generate Plan With History  "+"#"*5)
    world_state.update_state_with_sensor_data()
    planner.make_plan([CT_root_task], world_state) #CT_root_taskについて、現在のworld_stateに基づいてプランを立てる
    planner.show_plan()
    planner.execute_plan(world_state) #プランを実行する