# -*- coding: utf-8 -*-
"""
Created on Sun Mar 10 22:09:04 2019

@author: xyan
"""

from math import sin,cos,radians

def GenerateSamplePoints(x,y,r):
    Points = []
    StepSize = 10
    Np = round(360/StepSize)
    for i in range(Np):
        angle = i * StepSize
        xi = r * cos(radians(angle))
        yi = r * sin(radians(angle))
        Cx = x + xi
        Cy = y + yi
        Points.append([Cx,Cy])
    return Points

def RefineCirclePoints(pos,x_max,y_max):
    newpos = []
    for x,y in pos:
        if x < 0 or y < 0:
            newx = -min(abs(x),x_max)
            newy = -min(abs(y),y_max)
        else:
            newx = min(abs(x),x_max)
            newy = min(abs(y),y_max)
        newpos.append([newx,newy])
    return newpos