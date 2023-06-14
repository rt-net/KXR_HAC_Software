import WalkForward2
import detectEdge
import detectCorner
import time
import StopKRR
import cv2
import globalvalue as g
import turn
import walkSideWay
import getBodyAngle
import math

delay = 1
angleTH = 10
g.framex = 345
g.framey = 395
g.X = 0
g.Y = 0

while True:
    resultimg, angle, xcog, ycog = detectEdge.main()
    BodyAngle = getBodyAngle.main()
    
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
            BodyAngle = getBodyAngle.main()
            if BodyAngle[0] > 0:
                g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(1)
                g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(1)
            else:
                g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(-1)
                g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(1)
            #walkSideWay.main(-(100-xcog))
            resultimg, angle, xcog, ycog = detectEdge.main()
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
        #walkSideWay.main(-(100-xcog))
    elif xcog > 50:
        print("WalkRight: ", xcog-50)
        if BodyAngle[0] > 0:
            g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(-1)
            g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(-1)
        else:
            g.X = g.X + math.cos(math.radians(BodyAngle[0]))*(1)
            g.Y = g.Y + math.sin(math.radians(BodyAngle[0]))*(-1)
            
        #walkSideWay.main(xcog-100)
    else:
        print("WalkForward")
        g.X = g.X + math.sin(math.radians(BodyAngle[0]))*(1)
        g.Y = g.Y + math.cos(math.radians(BodyAngle[0]))*(1)
        #WalkForward2.main()
        
    print(g.X, g.Y)