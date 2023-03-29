import walk_forward
import detect_edge
import detect_corner
import time
import stop_krr
import cv2
import global_value as g
import turn
import walk_sideway
import get_body_angle
import math


delay = 1
angleTH = 10
g.framex = 345
g.framey = 395
g.X = 0
g.Y = 0

while True:
    resultimg, angle, xcog, ycog = detect_edge.main()
    
    cv2.imshow("test", resultimg)
    
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break
    BodyAngle = get_body_angle.main()
    
    print(BodyAngle)
    
    if angle != 0:
        if angle <= -angleTH:
            print("turn: ", angle)
            turn.main(angle)
        elif angle >= angleTH:
            print("turn: ", angle)
            turn.main(angle)
            
    if xcog == 0:
        while True:
            print("WalkLeft: ", 50)
            BodyAngle = get_body_angle.main()
            walk_sideway.main(-20)
            turn.main(-20)
=======
            if BodyAngle[0] > 0:
                g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(1)
                g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(1)
            else:
                g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(-1)
                g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(1)
            #walk_sideway.main(-(100-xcog))
>>>>>>> parent of 4efc8e2 (横移動の自己位置推定を追加)
=======
            walk_sideway.main(-(100-xcog))
>>>>>>> parent of 13220ee (前進の自己位置推定の追加)
=======
>>>>>>> parent of ea613d4 (pycacheの削除)
            resultimg, angle, xcog, ycog = detect_edge.main()
            if xcog != 0:
                break
        
    if xcog < 10:
        print("WalkLeft: ", -(50-xcog))
        walk_sideway.main(-20)
        
    elif xcog > 50:
        print("WalkRight: ", xcog-50)            
        walk_sideway.main(20)
        
    else:
        print("WalkForward")
        walk_forward.main(50)
        
    print(g.X, g.Y)