#motion_control_libraryの各モジュールを再生するサンプル
import sys
import os

import pprint
pprint.pprint(sys.path)

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) #一つ上のディレクトリへの検索パスの追加
from motion_control.motion_control_library import MotionLibrary

pprint.pprint(sys.path)

MOTION = MotionLibrary() #MotionLibraryクラスのインスタンス生成
MOTION.enable_plot() #自己位置のプロットを有効化

while True:
    #移動系のモーション
    MOTION.walk_forward(90) #指定距離（mm）一歩ずつ前進　歩幅は固定のため、距離の誤差大
    MOTION.walk_sideway(90) #指定距離（mm）一歩ずつ横に歩行　正で右方向、負で左方向　歩幅は固定のため、距離の誤差大
    MOTION.turn(90) #指定角度（°）一歩ずつ旋回　正で右旋回、負で左旋回　歩幅は固定のため、角度の誤差大
    
    MOTION.walk_forward_continue() #連続した歩行モーションを再生する
    MOTION.stop_motion() #再生中のモーションを停止する
    
    MOTION.walk_forward_timed(90) #指定距離（mm）連続して前進　歩幅は固定のため、距離の誤差大
    
    #その他のモーション
    MOTION.touch_ball() #ボールに触るモーションを再生する
    MOTION.stand_up() #仰向けから立ち上がるモーションを再生する
    