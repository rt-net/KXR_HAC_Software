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