# from vision.vision_library import VisionLibrary
# from motion_control.motion_control_library import MotionLibrary

# vision = VisionLibrary()
# motion = MotionLibrary()

class WorldState:
    def __init__(self, **kwargs):
        self.state = kwargs
    
    def update_state(self, new_state):
        for name, effect in new_state.items():
            self.state[name] = effect
            
class PrimitiveTask:
    def __init__(self,name):
        self.name = name #PrimitiveTaskの名称を設定
        self.preconditions = {} #辞書型でpreconditionsを生成

    def set_precondition(self, **kwargs):
        self.preconditions = kwargs #preconditionの内容を辞書型で代入 {'near_ball':False}のように入っている

    def set_effects(self, **kwargs):
        self.effects = kwargs #effectの内容を辞書型で代入 {'near_ball':True}のように入っている
            
world_state = WorldState(know_ball_pos=False,
                     near_edge=False,
                     near_ball=False,
                     touched_ball=False,
                     in_goal=False)

search_ball = PrimitiveTask("SearchBall") #htn.PrimitiveTaskクラスのインスタンス
search_ball.set_precondition(know_ball_pos=False) #辞書型でpreconditionを渡す
search_ball.set_effects(know_ball_pos=True) #辞書型でeffectを渡す

world_state.update_state(search_ball.effects)

print(search_ball.effects)
print(world_state.state)
