# -*- coding: utf-8 -*-
"""
Created on Tue Mar 12 16:17:08 2019

@author: xyan
"""
from math import cos,sin,radians,atan2,degrees
import matplotlib.pyplot as plt
def SamplePosition(Points, alpha, d,theta):
    x,y = Points[0],Points[1]
    deg = theta - alpha
    detx = d * cos(radians(deg))
    dety = d * sin(radians(deg))
    x = x + detx
    y = y + dety
    
    return [x,y]
    
def FindHeadingAngle(Point1,Point2):
    [x1,y1] = Point1
    [x2,y2] = Point2
    dety = (y2-y1)
    detx = (x2-x1)
    slopeangle = degrees(atan2(dety,detx))
    print(slopeangle)
    Heading = slopeangle + 90
    return Heading
    
Points = [1.5, 2.0]
alpha = 60
d = 1
theta =  45
S_Points = SamplePosition(Points, alpha, d,theta)

plt.scatter(Points[0],Points[1])
plt.scatter(S_Points[0],S_Points[1])

Point1 = [1,2]
Point2 = [2,2.5774]
Angle = FindHeadingAngle(Point1,Point2)