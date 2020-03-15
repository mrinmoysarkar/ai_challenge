# -*- coding: utf-8 -*-
"""
Created on Fri Mar 22 11:36:36 2019

@author: xyan
"""
from math import inf
import numpy as np
from sklearn.metrics.pairwise import euclidean_distances
def ReAssignPos(pos,Gridmatrix,W,H,Th):
    x,y = pos[0],pos[1]
    dx, dy=10, 20
    [i,j] = MaptoGrid(x,y,dx,dy)
    nr = round(W/dx)
    nc = round(H/dy)
    visited = []
    for k in range(4):
        tempRI_Min = i - nr
        tempRI_Max = i + nr
        tempCI_Min = j - nc
        tempCI_Max = j + nc
        grids = []
        for r_I in range(tempRI_Min,tempRI_Max):
            for c_I in range(tempCI_Min,tempCI_Max):
                grids.append(Gridmatrix[r_I][c_I])
        visited.append(np.sum(grids) / (nr * nc))
    
    MinIndice = np.argmin(visited)
    if visited[MinIndice] < Th:
        
        if MinIndice ==  0:
            X1,Y1 = x,y+H/2
            X2,Y2 = x-W,y+H/2
            X3,Y3 = x-W,y-H/2
            X4,Y4 = x, y-H/2
            return [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
            
        elif MinIndice == 1:
            X1,Y1 = x-W/2,y
            X2,Y2 = x-W/2,y+H
            X3,Y3 = x+W/2,y+H
            X4,Y4 = x+W/2,y
            return [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
        
        elif MinIndice == 2:
            X1,Y1 = x,y-H/2
            X2,Y2 = x-W, y-H/2
            X3,Y3 = x-W,y+H/2
            X4,Y4 = x,y+H/2
            return [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
        
        elif MinIndice == 3:
            X1,Y1 = x+W/2,y-H
            X2,Y2 = x+W/2,y-H
            X3,Y3 = x-W/2,y+H
            X4,Y4 = x-W/2,y+H
        return [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
    
    else:
        x = x 
        y = y
        
        ReAssignPos(pos,Gridmatrix,W,H,Th)
    
    

def MaptoGrid(x,y,dx,dy):
    W = 200
    H = 300
    Center = [50,75]
    X_Offset = 50 - W/2
    Y_Offset = 75 - H/2
    i = round((x - X_Offset) / dx)
    j = round((y - Y_Offset) / dy)
    
    return i,j
        
x,y = -10, -20   
dx,dy = 10, 20
 
[i,j] = MaptoGrid(x,y,dx,dy)

        
        
        
            

                    
                
                    
            
        
    
    