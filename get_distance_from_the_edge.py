import math
import numpy
import global_value as g


def main(slope, integer):
    
    framex = g.framex #画角の大きさ
    framey = g.framey #画角の大きさ

    turnoriginx = framex/2 #ロボット回転中心の画角内での位置
    turnoriginy = framey + 100 #ロボット回転中心の画角内での位置

    xdash = (framey - integer)/slope
    ydash = slope*(framex/2)+integer

    #print(turnoriginx, ydash)
    #print(xdash, turnoriginy)

    a = abs(turnoriginx-xdash)
    b = abs(turnoriginy-ydash)
    c = (a**2 + b**2)**(1/2)

    x = (b**2-a**2+c**2)/(2*c)

    dist = (b**2-x**2)**(1/2)

    #print(a)
    #print(b)
    #print(c)
    #print(x)
    #print(dist)
    
    return dist




