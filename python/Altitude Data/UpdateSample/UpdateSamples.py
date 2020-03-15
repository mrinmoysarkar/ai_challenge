# -*- coding: utf-8 -*-
"""
Created on Mon Mar 25 17:38:05 2019

@author: xyan
"""

import numpy as np
from scipy.spatial.distance import pdist,squareform
def UpdateSamples(sample,r,Np):
    sample = np.array(sample)
    points = sample[:,0:2]
    Dist = squareform(pdist(points))
    [Nr,Nc] = np.shape(Dist)
    UpdateSample = []
    marked = []
    for i in range(Nr):
        if i not in marked:
            D = Dist[i][:]
            marked.append(i)
            temp = []
            for j in range(Nc):
                if j not in marked:
                    if D[j] < r:
                        temp.append(sample[j,:])
                        marked.append(j)
            if np.shape(temp)[0]<Np:
                temp = sample[i,:]
                temp = np.array(temp)
                
                UpdateSample.append(temp)
            else:
                temp = np.array(temp)
                sorted_Indice = np.argsort(temp[:,2])
                temp = temp[sorted_Indice,:]
                UpdateSample.append(temp[-Np:,:])
            
    New = UpdateSample   
    print(New)
    return New

#sample = {'1':[100,200],'2':[150,200],'3':[50,200],'4':[450,300],'5':[350,300],'6':[450,280],'7':[500,300]}
sample = [[100,200,1],[150,200,2],[50,200,3],[450,300,4],[350,300,5],[450,280,6],[500,300,7]]
r = 100
Np = 1

UpdateSample = UpdateSamples(sample,r,Np)