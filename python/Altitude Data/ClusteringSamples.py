# -*- coding: utf-8 -*-
"""
Created on Sat Mar 23 16:35:25 2019

@author: xyan
"""

import numpy as np

from sklearn.cluster import DBSCAN
from sklearn.cluster import MeanShift
from sklearn import metrics
from sklearn.preprocessing import StandardScaler

def ClusteringSamples(X):
    OriginalX = X
    X = StandardScaler().fit_transform(X)
    clustering = DBSCAN(eps=0.5, min_samples=10).fit(X)
#    clustering = MeanShift().fit(X)
#    labels = db.labels_
    labels = clustering.labels_
    uniquelabels = np.unique(labels)
    NewX = {}
    for label in uniquelabels:
#        label = round(label)
        NewX[label] = OriginalX[labels==label]
    return NewX, labels

from sklearn.datasets.samples_generator import make_blobs
centers = [[1, 1], [-1, -1], [1, -1]]
X, labels_true = make_blobs(n_samples=750, centers=centers, cluster_std=0.4,
                            random_state=0)

NewX,labels = ClusteringSamples(X)