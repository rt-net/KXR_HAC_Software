import walk_forward_2
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
            #turn.main(angle)
        elif angle >= angleTH:
            print("turn: ", angle)
            #turn.main(angle)
            
    if xcog == 0:
        while True:
            print("WalkLeft: ", 50)
            BodyAngle = get_body_angle.main()
            if BodyAngle[0] > 0:
                g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(1)
                g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(1)
            else:
                g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(-1)
                g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(1)
            #walk_sideway.main(-(100-xcog))
            resultimg, angle, xcog, ycog = detect_edge.main()
            if xcog != 0:
                break
        
    if xcog < 10:
        print("WalkLeft: ", -(50-xcog))
        if BodyAngle[0] > 0:
            g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(1)
            g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(1)
        else:
            g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(-1)
            g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(1)
        #walk_sideway.main(-(100-xcog))
    elif xcog > 50:
        print("WalkRight: ", xcog-50)
        if BodyAngle[0] > 0:
            g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(-1)
            g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(-1)
        else:
            g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(1)
            g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(-1)
            
        #walk_sideway.main(xcog-100)
    else:
        print("WalkForward")
        g.X = g.X + math.sin(math.radians(BodyAngle[0]))*(1)
        g.Y = g.Y + math.cos(math.radians(BodyAngle[0]))*(1)
        #walk_forward_2.main()
        
    print(g.X, g.Y)