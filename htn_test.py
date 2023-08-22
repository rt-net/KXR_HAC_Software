def print_something():
    print("something")

class PrimitiveTask:
    def __init__(self,name):
        self.name = name #PrimitiveTaskの名称をself.nameに代入　"SearchBall"のように渡されているのでcharで入る
        self.preconditions = {} #辞書型でpreconditionsを生成

    def set_precondition(self, **kwargs):
        self.preconditions = kwargs #preconditionの内容を辞書型で代入 {'near_ball':False}のように入っている　複数個入ることもある

    def set_effects(self, **kwargs):
        self.effects = kwargs #effectの内容を辞書型で代入 {'near_ball':True}のように入っている　複数個入ることもある
        
    def set_action(self, action):
        self.action = action
        
    def do_action(self):
        print(self.action)
        self.action()
        

s_ball = PrimitiveTask("SearchBall") #htn.PrimitiveTaskクラスのs_ballインスタンス生成
s_ball.set_precondition(know_ball_pos=False) #辞書型でpreconditionを設定
s_ball.set_effects(know_ball_pos=True) #辞書型でeffectを設定
s_ball.set_action(print_something)

s_ball.do_action()
