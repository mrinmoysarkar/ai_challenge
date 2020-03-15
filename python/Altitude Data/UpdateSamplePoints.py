# -*- coding: utf-8 -*-
"""
Created on Sun Mar 17 11:40:56 2019

@author: xyan
"""
from scipy.spatial.distance import dist
import numpy as np
def UpdateSamplePoints(Samples,K):
    Points =  Samples[:][0:2]
    DistMat = pdist(Points)
    T = Points[:][2]
    NewSample = []
    for i in range(np.shape(Points)[0]):
        Dist = DistMat[i][:]
        Sort = np.argsort(Dist)
        Indices = Sort[1:K+1]
        Neighbors = Points[Indices][:]
        NeiDist = Dist[Indices]
        NeighborTime = Samples[Indices][2]
        for j in range(K):
            if NeighborTime[j]<T[i] or NeiDist[j]>np.mean(NeiDist):
                NewSample.append(Neighbors[j][:])
        
        
        return NewSample