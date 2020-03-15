# -*- coding: utf-8 -*-
"""
Created on Thu Mar 21 09:44:18 2019

@author: xyan
"""
from afrl.cmasi.AirVehicleState import AirVehicleState
import numpy as np
def AssignRecoveryRegion(Airvehicle,RecoveryZone):
#    veicleid = AirVehicleState.ID
    veicleid = Airvehicle[0]
    Goback = False
    AvailableEnergy =  Airvehicle[1]  
    EnergyRate = Airvehicle[2]
#    AvailableEnergy =  AirVehicleState.EnergyAvailable
#    EnergyRate = AirVehicleState.ActualEnergyRate
#    Location = AirVehicleState.get_Location()
#    Speed = AirVehicleState.Airspeed
#    Lat = Location.get_Latitude()
#    Lon = Location.get_Longitude()
#    [x,y] = self.convertLatLonToxy(Lat,Lon)
    x,y = Airvehicle[3],Airvehicle[4]
    Speed = Airvehicle[5]
    Dist = []
    RecoveryPos = [x,y]
    if EnergyRate > 0.0:
        Dist = []
        for i in range(np.shape(RecoveryZone)[0]):
            x2 = RecoveryZone[i][0]
            y2 = RecoveryZone[i][1]
            d = ((x - x2)**2 + (y - y2)**2)**0.5
            Dist.append(d)
        MinIndice = np.argmin(Dist)
        RemainTime =  AvailableEnergy / EnergyRate
        MaximumDistLeft = Speed * RemainTime
        print(MaximumDistLeft,Dist[MinIndice])
        if Dist[MinIndice] >= MaximumDistLeft:
            Goback = True
            RecoveryPos = RecoveryZone[MinIndice][:]       
    return [Goback,RecoveryPos]
    
Airvehicle = [1,100,1,100,200,30]
RX = [[1500,2000],[-1000,-1500],[-2000,500]]
[Goback,RecoveryPos] = AssignRecoveryRegion(Airvehicle,RX)