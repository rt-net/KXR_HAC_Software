import math
import numpy
from vision import parameters


def main(slope, intercept):    
    Ax = parameters.BEV_FRAME_WIDTH/2
    Ay = parameters.BEV_FRAME_HEIGHT + 90
    
    Bx = (Ay-intercept)/slope
    By = Ay
    
    Cx = Ax
    Cy = Ax*slope+intercept
    
    AC = (Ay - Cy)
    AB = (Ax - Bx)
    
    BC = (AC**2 + AB**2)**(1/2)
    
    dist = (AB**2 - ((AB**2 - AC**2 +BC**2)/(2*BC))**2)**(1/2)
    
    return dist

