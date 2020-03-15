# -*- coding: utf-8 -*-
"""
Created on Mon Mar 18 12:08:40 2019

@author: xyan
"""
import numpy as np
import matplotlib.pyplot as plt
def ComputeSubRegion(BP,Pos):
    Dist = []
    for i in range(len(BP)):
        Dist.append((BP[i][0] - Pos[0])**2+(BP[i][1]-Pos[1])**2)
    Indice = (np.argsort(Dist))
#    print(Indice[0:2][:])
    BelowBP = Indice[0:2][:]
    
    UpperBP = Indice[2:4][:]
#    print(UpperBP)
    SubBP = []
    for j in range(len(BelowBP)):
        NewX1 = Pos[0]
        NewY1 = (BP[BelowBP[j]][1] + Pos[1])/2
        SubBP.append([NewX1,NewY1])
    for k in range(len(UpperBP)):
        NewX2 = (BP[UpperBP[k]][0] + Pos[0])/2
        NewY2 = (BP[UpperBP[k]][1] + Pos[1])/2
        SubBP.append([NewX2,NewY2])
    Center = np.mean(SubBP,axis=0)    
    return SubBP,Center
        

BP = [[1.8,3.5],[1.8,1.5],[3.2,3.5],[3.2,1.5]]
print(BP)

Pos = [2.0,2.8]        

SubBP,SubCenter = ComputeSubRegion(BP,Pos)

for x1,y1 in BP:
    plt.scatter(x1,y1)

for x2,y2 in SubBP:
    plt.scatter(x2,y2)

#plt.scatter(Pos[0],Pos[1])
plt.scatter(SubCenter[0],SubCenter[1])