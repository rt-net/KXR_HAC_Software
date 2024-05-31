import copy
import math
import time
import sys

import numpy as np

class WorldState:
    def __init__(self, **kwargs): #world_stateの初期化 **kwargsはキーワード引数を辞書型で受け取る
        self.state = kwargs #辞書型のworld_state初期値をself.stateに格納

    def set_update_functions(self, **kwargs):
        assert len(kwargs) == len(self.state)
        self.state_check_list = kwargs #どの関数をworld_stateのアップデートに使うか指定

    def update_state_for_planner(self, new_state):
        for name, effect in new_state.items():
            self.state[name] = effect #stateをnew_stateに基づいてアップデート
    
    def update_state_with_sensor_data(self): 
        #print("Updating World State")
        for state_name, update_function in self.state_check_list.items():  #state_check_listのそれぞれの内容に従いセンサーデータからstateを更新
            self.state[state_name] = update_function()
            print("WorldState: updating", state_name, ":", self.state[state_name])
        # print(self.state)
        

    def check_if_state_changed(self): #stateが変わったか確認する
        old_state = copy.deepcopy(self.state) #old_stateは呼び出し時のstate
        self.update_state_with_sensor_data() #センサーデータによってstateを更新
        if self.state != old_state:
            return True #stateが変わっていればTrueを返す
        else:
            return False #変わっていなければFalseを返す


class CompoundTask:
    def __init__(self, name):
        self.name = name #CompoundTaskの名称を設定
        self.method_list = None #中身のMethodの初期化

    def set_method(self, *args): 
        self.method_list = args #渡されたmethodをself.method_listに代入 Compound TaskにはMethodしか入らない


class Method:
    def __init__(self, name):
        self.name = name #Methodの名称を設定
        self.preconditions = {} #辞書型でpreconditionsを生成
        self.subtasks = None #subtaskの初期化

    def set_precondition(self, **kwargs):
        self.preconditions = kwargs #preconditionの内容を辞書型で代入 {'near_ball':False}のように入っている　複数入ることもある

    def set_subtask(self, *args):
        self.subtasks = args #渡されたsubtaskをself.subtasksに格納 タプルで渡されている


class PrimitiveTask:
    STATUS_FAILED = 0
    STATUS_ACTIVE = 1
    STATUS_COMPLETE = 2
    STATUS_INITIALIZED = 3

    def __init__(self,name):
        self.name = name #PrimitiveTaskの名称をself.nameに代入　"SearchBall"のように渡されているのでcharで入る
        self.status = PrimitiveTask.STATUS_INITIALIZED
        self.preconditions = {} #辞書型でpreconditionsを生成

    def set_precondition(self, **kwargs):
        self.preconditions = kwargs #preconditionの内容を辞書型で代入 {'near_ball':False}のように入っている　複数個入ることもある

    def set_effects(self, **kwargs):
        self.effects = kwargs #effectの内容を辞書型で代入 {'near_ball':True}のように入っている　複数個入ることもある
        
    def set_action(self, action):
        self.action = action

    def monitor_task_status(self, world):
        world.update_state_with_sensor_data()
        if set(self.effects.items()).issubset(world.state.items()): 
            print("COMPLETE")
            return PrimitiveTask.STATUS_COMPLETE
        elif set(self.preconditions.items()).issubset(world.state.items()):
            print("ACTIVE")
            return PrimitiveTask.STATUS_ACTIVE
        else:
            print("FAILED")
            return PrimitiveTask.STATUS_FAILED
        
    def run_action(self, world):
        self.status = PrimitiveTask.STATUS_ACTIVE
        print("in action----", self.action)
        while self.status == PrimitiveTask.STATUS_ACTIVE:
            self.action()
            self.status = self.monitor_task_status(world)
                        

class PlanningHistory: #計画の履歴を作る
    def reset(self): #履歴をリセットする
        self.f_plan = [] #最終プランの初期化
        self.history = [] #履歴の初期化
        self.world_state = None #world_stateの初期化

    def record(self, task, f_plan, world = None): #履歴を保存する
        self.history.append(task) #履歴に引数であるタスクを付け足す
        self.f_plan = f_plan #最終プランをselfに
        #self.world_state = copy.deepcopy(world)

    def restore_task(self):
        if self.history: #もしself.historyに値が格納されているなら
            return [self.history.pop(-1)] #履歴から末尾の値を消したものを返す
        return [] #空の配列を返す

    def restore_world_state(self):
        return copy.deepcopy(self.world_state) #world_stateの完全なコピーを返す　world_stateの値が更新されても変わらないdeepcopy


class FinalPlan:
    def reset(self):
        self.tasks = [] #tasksのリストを空の状態にする

    def add(self, task):
        self.tasks.append(task) #tasksのリストに引数となっているタスクを付け足す

    def run(self, world):
        #print(self.tasks)
        for task in self.tasks: #taskはクラスとして存在する　class.PrimitiveTaskの別々のインスタンス
            print("\n実行中のタスク: ", task.name)
            task.run_action(world)
            if task.status == PrimitiveTask.STATUS_FAILED:
                break

class Planner:
    def __init__(self, use_history= True): #use_historyの指定が無ければTrueになる　Falseの指定があるならそのまま
        self.working_state = None #working_stateを初期化
        self.f_plan = FinalPlan() #最終プランのインスタンス生成
        self.his = PlanningHistory() #計画履歴のインスタンス生成
        self.use_history = use_history #履歴の使用有無をインスタンス変数にする

    def check_task_precond(self, task): #渡されたタスクのpreconditionを確認
        for precondition in task.preconditions.items(): #渡されたタスクのpreconditionの要素について　items()は辞書オブジェクトのキーと値をforループさせるためのメソッド
            # print(precondition)
            # print(self.working_state.state.items())
            if precondition not in self.working_state.state.items(): #working_stateの中にそのタスクのpreconditionが無かったら
                return False #Falseを返す
        return True #ここまでのループでFalseが返らなかったら（working_stateとpreconditionが合致していたら）Trueを返す

    def make_plan(self, tasks_to_process, world):#, DecomHis): 渡されたタスクと現在のworld_stateについてプランを作成
        print ("*"*10+"CREATING A PLAN"+"*"*10) #プラン生成
        self.f_plan.reset() #最終プランの初期化
        self.his.reset() #計画履歴の初期化
        self.working_state = copy.deepcopy(world) #引数worldからコピーした作業状態をインスタンス変数self.working_stateにする　プランニング作業中に引数のもとが更新されても基の値を保持
        
        while tasks_to_process: #tasks_to_processが存在する間
            current_task = tasks_to_process.pop(0) #tasks_to_processの先頭の値を削除して、それをcurrent_taskに代入 pop(0)はリストの先頭を削除して持ってくる
            if current_task.__class__.__name__ == 'CompoundTask': #__class__.__name__プロパティ属性はクラスの名前を持ってくる　これがもしCompundTaskであれば
                print("current compound task name", current_task.name)
                for method in current_task.method_list: #現在のタスクにおけるメソッドのリストについて繰り返す
                    if self.check_task_precond(method): #そのメソッドについてpreconditionを確認 Trueが返ってきたら（working_stateと合致していたら）
                        print("current method name", method.name)
                        self.his.record(current_task, self.f_plan) #現在のタスクをプランに記録 PlanningHistoryクラスを用いる
                        tasks_to_process = list(method.subtasks) + tasks_to_process #実行するタスクにメソッドのsubtaskを追加
                    elif self.use_history: #それ以外の場合（use_historyは常にTrue）
                        tasks_to_process = tasks_to_process + self.his.restore_task() #tasks_to_processはそのまま
            elif current_task.__class__.__name__ == 'PrimitiveTask': #もしPrimitiveTaskであれば
                print("current primitive task name", current_task.name)
                if self.check_task_precond(current_task): #現在のタスクのpreconditionを確認
                    self.working_state.update_state_for_planner(current_task.effects) #world_stateをコピーしたものであるworking_stateを実行したtaskのeffectにより更新
                    self.f_plan.add(current_task) #現在のタスクをプランに追加
                elif self.use_history: #それ以外の場合（use_historyは常にTrue）
                    tasks_to_process = tasks_to_process + self.his.restore_task() #履歴を一段階戻す
            else:
                print('WARNING: NO PLAN FOUND')

    def show_plan(self):
        #print(self.f_plan.tasks)
        print("-"*10+"Plan Start"+"-"*10)
        for task in self.f_plan.tasks:
            print(task.name)
        print("-"*10+"Plan Finish"+"-"*10)

    def execute_plan(self, world):
        self.f_plan.run(world)