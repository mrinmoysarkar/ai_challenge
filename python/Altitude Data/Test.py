# -*- coding: utf-8 -*-
"""
Created on Sat Mar  9 12:11:40 2019

@author: xyan
"""

import pandas as pd



altidata1 = pd.read_csv('altidata1.csv',header=None)

altidata1 = altidata1.T

altidata2 = pd.read_csv('altidata2.csv',header=None)

altidata2 = altidata2.T

altidata3 = pd.read_csv('altidata3.csv',header=None)

altidata3 = altidata3.T

altidata4 = pd.read_csv('altidata4.csv',header=None)

altidata4 = altidata4.T


def Altitude_Fetch(lat,lon):

    if (lat - 39.0) <= 1:
        if abs(lon + 122.0) <= 1:
            i = round((lat - 39) * 3600) - 1 
            j = round((lon + 122) * 3600) - 1
            sz = altidata1.shape
            if i >= sz[0]:
                i = sz[0] - 1
            if j >= sz[1]:
                j = sz[1] - 1
            Altitude = altidata1[i][j]  
            
            print('Map: 1', i, j)
        else:
            i = round((lat - 39) * 3600) - 1 
            j = round((lon + 121) * 3600) - 1
            sz = altidata2.shape
            if i >= sz[0]:
                i = sz[0] - 1
            if j >= sz[1]:
                j = sz[1] - 1
            Altitude = altidata2[i][j]
            
            print('Map: 2', i, j)
    else:
        if (lon + 122) <= 1:
            i = round((lat - 40) * 3600) - 1 
            j = round((lon + 122) * 3600) - 1
            sz = altidata3.shape
            if i >= sz[0]:
                i = sz[0] - 1
            if j >= sz[1]:
                j = sz[1] - 1
            Altitude = altidata3[sz[0]-i][sz[1]-j]
            
            print('Map: 3', i, j)
        elif (lon + 122) > 1:
            i = round((lat - 40) * 3600) - 1 
            j = round((lon + 121) * 3600) - 1
            sz = altidata4.shape
            if i >= sz[0]:
                i = sz[0] - 1
            if j >= sz[1]:
                j = sz[1] - 1
            Altitude = altidata4[sz[0]-i][sz[1]-j]
            
            print('Map: 4', i, j)
            
    return Altitude

            
    
    

        
        