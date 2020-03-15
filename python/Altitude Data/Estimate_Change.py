# -*- coding: utf-8 -*-
"""
Created on Sun Mar 10 11:11:02 2019

@author: xyan
"""

#import numpy as np

from math import cos,sin,radians
import numpy as np
import matplotlib.pyplot as plt

def Estimate_Shape(WinSpeed, Angle, SamplePoints):
    Rate = WinSpeed
    X_Rate = Rate * cos(radians(Angle))
    Y_Rate = Rate * sin(radians(Angle))
    NewSamplePoint = []
    for [x,y] in SamplePoints:
#        [x,y] = convertLatLonToxy(lat,lon)
        x = x + X_Rate
        y = y + Y_Rate
#        [NewLat,NewLon] = convertxyToLatLon(x,y)
        NewSamplePoint.append([x,y])
    return NewSamplePoint
        
#def convertLatLonToxy(self,lat,long):
#    R = 111000
#    a = lat-1.48594
#    b = long-(-132.53555)
#    x = R*a
#    y = R*cos(radians(lat))*b
#    # print(utm.from_latlon(lat, long))
#    return [x,y]
        
#def convertxyToLatLon(self,x,y):
#    R = 111000
#    lat = x/R + 1.48594
#    long = y/(R*cos(radians(lat))) + (-132.53555)
#    return [lat,long]


OriginalPoints = [[1.8,3.5],[2.4,4.2],[2.6,3.5],[3.2,3.5]]
WinSpeed = 10
Angle = -80
NewPoints = Estimate_Shape(WinSpeed, Angle, OriginalPoints)

Data = (OriginalPoints,NewPoints)
Groups = ('1','2')

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, axisbg="1.0")
 
for data, group in zip(Data, Groups):
    for x,y in data:
        ax.scatter(x, y, alpha=0.8, edgecolors='none', s=30, label=group)
 
plt.title('Matplot scatter plot')
plt.legend(loc=2)
plt.show()