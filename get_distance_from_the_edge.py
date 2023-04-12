import math
import numpy
import global_value as g
from vision import parameters as p


def main(slope, integer):
    
    print("slope", slope)
    print("integer", integer)
    
    Ax = p.BEVwidth/2
    Ay = p.BEVheight + 90
    
    Bx = (Ay-integer)/slope
    By = Ay
    
    Cx = Ax
    Cy = Ax*slope+integer
    
    print("Ax, Ay", Ax, Ay)
    print("Bx, By", Bx, By)
    print("Cx, Cy", Cx, Cy)
    
    AC = (Ay - Cy)
    AB = (Ax - Bx)
    
    BC = (AC**2 + AB**2)**(1/2)
    
    dist = (AB**2 - ((AB**2 - AC**2 +BC**2)/(2*BC))**2)**(1/2)
    
    print(dist)
    
    # framex = p.BEVwidth #画角の大きさ
    # framey = p.BEVheight #画角の大きさ

    # turnoriginx = framex/2 #ロボット回転中心の画角内での位置
    # turnoriginy = framey + 100 #ロボット回転中心の画角内での位置
    
    # print(turnoriginx)
    # print(turnoriginy)

    # xdash = (framey - integer)/slope
    # ydash = slope*(framex/2)+integer

    # #print(turnoriginx, ydash)
    # #print(xdash, turnoriginy)

    # a = abs(turnoriginx-xdash)
    # b = abs(turnoriginy-ydash)
    # c = (a**2 + b**2)**(1/2)

    # x = (b**2-a**2+c**2)/(2*c)

    # dist = (b**2-x**2)**(1/2)

    # #print(a)
    # #print(b)
    # #print(c)
    # #print(x)
    # #print(dist)
    
    return dist




