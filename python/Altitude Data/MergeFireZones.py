# -*- coding: utf-8 -*-
"""
Created on Fri Mar 22 18:42:46 2019

@author: xyan
"""
import numpy as np
def MergeFireZones(Zones,ZoneCenters):
    Zids = [*Zones]
    Nz=  len(Zids)
    MergeList = {}
    for i in range(Nz):
        
        CurrentZoneFirePoints = Zones[Zids[i]]
        CurrentZoneCenter = ZoneCenters[i]
        CurrentFireCenter = np.mean(CurrentZoneFirePoints,axis=0)
        Connected = []
        for j in range(Nz):
            if j != i:
                NextZoneCenter = ZoneCenters[j][:]
                NextZoneFirePoints = Zones[Zids[j]]
                NextFireCenter = np.mean(NextZoneFirePoints,axis=0)
                D = (CurrentFireCenter[0]-NextFireCenter[0])**2 + (CurrentFireCenter[1]-NextFireCenter[1])**2
                ThresholdD = (CurrentZoneCenter[0]-NextZoneCenter[0])**2 + (CurrentZoneCenter[1]-NextZoneCenter[1])**2
                if D < ThresholdD:
                    Connected.append(Zids[j])
        if not Connected:
            MergeList[Zids[i]] = [Zids[i]]
        else:
            MergeList[Zids[i]] = Connected
    return MergeList

def ConnectZone(MergeList,Zid):
    ConnectedZones = MergeList[Zid]
    if len(ConnectedZones)==1:
        NextConnectZone = ConnectedZones
    else:
        NextConnectZone = ConnectedZones[0]

        
    return ConnectZone(MergeList,NextConnectZone)
        
def DoMerge(MergeList):
    keys = MergeList.keys()
    for zid in keys:
        NextConnectZone = ConnectZone(MergeList,zid)
        

# Initialize the Zone Firepoints 
Zones = {1:[[1,2],[10,12]],2:[[1,3],[2,4],[3,4]],3:[[13,15],[15,16]]}
ZoneCenters = [[5.5,5],[0,6.5],[10,12]]

NewZone = MergeFireZones(Zones,ZoneCenters)