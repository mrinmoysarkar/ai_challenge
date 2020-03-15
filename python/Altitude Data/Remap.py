# -*- coding: utf-8 -*-
"""
Created on Fri Mar 22 11:36:36 2019

@author: xyan
"""
from math import inf
import numpy as np
from sklearn.metrics.pairwise import euclidean_distances
def ReAssignPos(pos,Gridmatrix,W,H,Th):
    # Extract the position into x,y coordinates
    x,y = pos[0],pos[1]

    # Prespecify the grid size
    dx, dy=10, 20
    # Map the current position into grid
    [i,j] = MaptoGrid(x,y,dx,dy)
    # Extract the indice of the current position in the grid matrix
    nr = round((W)/dx)
    nc = round((H)/dy)
    # Initialize the visited vector as empty 
    visited = []
    searchid = []
    BoundIndice = [0,np.shape(Gridmatrix)[0],np.shape(Gridmatrix)[1]]
    if i in BoundIndice or j in BoundIndice:
        if i == BoundIndice[0] and j == BoundIndice[0]:
            searchid = [1,2]
            for k in range(2):
                tempRI_Min = i 
                tempRI_Max = i + nr
                tempCI_Min = j 
                tempCI_Max = j + nc
                grids = []
                for r_I in range(tempRI_Min,tempRI_Max):
                    for c_I in range(tempCI_Min,tempCI_Max):
                        grids.append(Gridmatrix[r_I][c_I])
                    visited.append(np.sum(grids) / (nr * nc))
            MinIndice = searchid[np.argmin(visited)]
        elif i == BoundIndice[0] and j == BoundIndice[2]:
            searchid = [2,3]
            for k in range(2):
                tempRI_Min = i 
                tempRI_Max = i + nr
                tempCI_Min = j - nc
                tempCI_Max = j 
                grids = []
                for r_I in range(tempRI_Min,tempRI_Max):
                    for c_I in range(tempCI_Min,tempCI_Max):
                        grids.append(Gridmatrix[r_I][c_I])
                    visited.append(np.sum(grids) / (nr * nc))
            MinIndice = searchid[np.argmin(visited)]
        elif i == BoundIndice[1] and j == BoundIndice[0]:
            searchid = [0,1]
            for k in range(2):
                tempRI_Min = i - nr
                tempRI_Max = i 
                tempCI_Min = j 
                tempCI_Max = j + nc
                grids = []
                for r_I in range(tempRI_Min,tempRI_Max):
                    for c_I in range(tempCI_Min,tempCI_Max):
                        grids.append(Gridmatrix[r_I][c_I])
                    visited.append(np.sum(grids) / (nr * nc))
            MinIndice = searchid[np.argmin(visited)]
        elif i == BoundIndice[1] and j == BoundIndice[2]:
            searchid = [0,3]
            for k in range(2):
                tempRI_Min = i - nr
                tempRI_Max = i
                tempCI_Min = j - nc
                tempCI_Max = j
                grids = []
                for r_I in range(tempRI_Min,tempRI_Max):
                    for c_I in range(tempCI_Min,tempCI_Max):
                        grids.append(Gridmatrix[r_I][c_I])
                    visited.append(np.sum(grids) / (nr * nc))
            MinIndice = searchid[np.argmin(visited)]
        
    else:        
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
        if MinIndice == 0:
            X1,Y1 = x+W/2,y
            X2,Y2 = x+W/2,y-H
            X3,Y3 = x-W/2,y+H
            X4,Y4 = x-W/2,y
            return [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
        elif MinIndice ==  1:
            X1,Y1 = x,y+H/2
            X2,Y2 = x-W,y+H/2
            X3,Y3 = x-W,y-H/2
            X4,Y4 = x, y-H/2
            return [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
        elif MinIndice == 2:
            X1,Y1 = x,y-H/2
            X2,Y2 = x-W, y-H/2
            X3,Y3 = x-W,y+H/2
            X4,Y4 = x,y+H/2
            return [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
        elif MinIndice == 3:
            X1,Y1 = x-W/2,y
            X2,Y2 = x-W/2,y+H
            X3,Y3 = x+W/2,y+H
            X4,Y4 = x+W/2,y
            return [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
    else:
        if MinIndice== 0:
            x = x + W
            y = y
        elif MinIndice == 1:
            x = x
            y = y + H
        elif MinIndice == 2:
            x = x -W
            y = y
        elif MinIndice == 3:
            x = x
            y = y - H
            
        res = ReAssignPos(pos,Gridmatrix,W,H,Th)
        return res
    

def MaptoGrid(x,y,dx,dy):
    W = 200
    H = 300
    referPoints = [-50,-75]
    X_Offset = referPoints[0] 
    Y_Offset = referPoints[1] 
    if abs(x - X_Offset) >= W or abs(y - Y_Offset) >= H:
        print('Out of the Boundary')
        i = 0
        j = 0
    else:
        i = round(abs(x - X_Offset) / dx)
        j = round(abs(y - Y_Offset) / dy)
        
    return i,j
        
x,y = 400,-50   
dx,dy = 10, 20
 
#[i,j] = MaptoGrid(x,y,dx,dy)


#NextPos = ReAssignPos(pos,Gridmatrix,W,H,Th)        
        
        
            

                    
                
                    
            
        
    
    