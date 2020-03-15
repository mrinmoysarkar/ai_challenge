# -*- coding: utf-8 -*-
"""
Created on Wed Mar 20 18:30:52 2019

@author: xyan
"""
import numpy as np
# Write a function for dividing the entire region into a set of grids
def GridMatrix(x,y,dx,dy,GridList):
    W = 200
    H = 300
    Center = [50,75]
    if not GridList:
        NR = round(W / dx)
        NC = round(H / dy)
        GridList =  np.full((NR,NC),False, dtype=bool)
#    else:
    X_Offset = 50 - W/2
    Y_Offset = 75 - H/2
    i = round((x - X_Offset) / dx)
    j = round((y - Y_Offset) / dy)
    GridList[i][j] = True
        
    return GridList

GridList = []

x,y = -10, -20
dx,dy = 10, 20

GridList = GridMatrix(x,y,dx,dy,GridList)