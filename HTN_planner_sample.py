import sys
import htn
import copy

####WorldState####
world_state = htn.WorldState(near_ball=False,
                         know_ball_pos=False,
                         ball_and_target_on_straight_line=False,
                         ball_in_goal=False) #htn.WorldStateクラスのインスタンス　初期値には全てFalseが入っている

####PrimitiveTasks####
s_ball = htn.PrimitiveTask("SearchBall") #htn.PrimitiveTaskクラスのs_ballインスタンス生成
s_ball.set_precondition(know_ball_pos=False) #辞書型でpreconditionを設定
s_ball.set_effects(know_ball_pos=True) #辞書型でeffectを設定

ap_ball = htn.PrimitiveTask("ApproachBall")
ap_ball.set_precondition(know_ball_pos=True)
ap_ball.set_effects(near_ball=True)

turn = htn.PrimitiveTask("TurnAroundBall")
turn.set_precondition(near_ball=True)
turn.set_effects(ball_and_target_on_straight_line=True)

kick = htn.PrimitiveTask("KickBallToTarget")
kick.set_precondition(near_ball=True, ball_and_target_on_straight_line=True)
kick.set_effects(ball_in_goal=True)


####Method####
f_ball = htn.Method("FarFromBall") #htn.Methodクラスのインスタンス生成
f_ball.set_precondition(near_ball=False) #辞書型でpreconditionを渡す
f_ball.set_subtask(s_ball, ap_ball) #含まれるsubtaskをタプルで渡す

n_ball = htn.Method("NearTheBall")
n_ball.set_precondition(near_ball=True)
n_ball.set_subtask(turn, kick)

####CompoundTasks####
root_task = htn.CompoundTask("SimpleKickStrategy") #htn.CompoundTaskクラスのroot_taskインスタンス生成
root_task.set_method(f_ball, n_ball) #含まれるmethodをタプルで渡す　CompoundTaskにはpreconditionとeffectは無い？

########### HTNPlanner without using Decomposed History ###########
planner = htn.Planner(use_history=False) #htn.plannerのインスタンス　履歴は不使用
world = copy.deepcopy(world_state) #world_stateはhtn.worldstateクラスなので、そのコピーであるworldもクラスになる

print("#"*5+"  Generate Plan Without History  "+"#"*5) #履歴を使わずにプラン生成
planner.make_plan([root_task], world) #root_taskを現在のworldについて実行するプランを生成する
planner.show_plan()
planner.execute_plan(world)

print(world.state)

print('\n'+'REPlANNING'+'\n')

planner.make_plan([root_task], world)
planner.show_plan()
planner.execute_plan(world)

print(world.state)



########### HTNPlanner using Decomposed History ###########
planner = htn.Planner()
world = copy.deepcopy(world_state)

print('\n\n'+"#"*5+"  Generate Plan With History  "+"#"*5)
planner.make_plan([root_task], world_state)
planner.show_plan()
planner.execute_plan(world)