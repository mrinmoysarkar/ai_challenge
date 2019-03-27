#!/usr/bin/python
# author: Mrinmoy sarkar
# date: 13 FEB 2019
# email: mrinmoy.pol@gmail.com
# team: AM2X

from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.Circle import Circle
from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.Waypoint import Waypoint
from afrl.cmasi.TurnType import TurnType
from afrl.cmasi.VehicleActionCommand import VehicleActionCommand
from afrl.cmasi.LoiterAction import LoiterAction
from afrl.cmasi.LoiterType import LoiterType
from afrl.cmasi.LoiterDirection import LoiterDirection
from afrl.cmasi.CommandStatusType import CommandStatusType
from afrl.cmasi.AltitudeType import AltitudeType
from afrl.cmasi.searchai.HazardZoneDetection import HazardZoneDetection
from afrl.cmasi.searchai.HazardType import HazardType
from afrl.cmasi.Location3D import Location3D
from afrl.cmasi.Rectangle import Rectangle
from afrl.cmasi.FlightDirectorAction import FlightDirectorAction
from afrl.cmasi.SpeedType import SpeedType
from afrl.cmasi.AltitudeType import AltitudeType
from afrl.cmasi.AirVehicleState import AirVehicleState
from afrl.cmasi.MissionCommand import MissionCommand
from afrl.cmasi.SessionStatus import SessionStatus
from afrl.cmasi.GoToWaypointAction import GoToWaypointAction
from afrl.cmasi.GimbalAngleAction import GimbalAngleAction
from lmcp import LMCPFactory
from afrl.cmasi.KeepInZone import KeepInZone
from afrl.cmasi.searchai.HazardZone import HazardZone
from afrl.cmasi.searchai.HazardZoneChangeCommand import HazardZoneChangeCommand
from afrl.cmasi.AirVehicleConfiguration import AirVehicleConfiguration
from afrl.cmasi.WeatherReport import WeatherReport
from math import sin,cos,atan2,pi,radians,sqrt,tan
from random import randint
import pandas as pd
import numpy as np
from afrl.cmasi.searchai.HazardType import HazardType
from afrl.cmasi.NavigationMode import NavigationMode
from afrl.cmasi.EntityConfiguration import EntityConfiguration
from afrl.cmasi.EntityState import EntityState
from afrl.cmasi.searchai.RecoveryPoint import RecoveryPoint
from afrl.cmasi.perceive.EntityPerception import EntityPerception
from afrl.cmasi.RemoveEntities import RemoveEntities
from afrl.cmasi.GimbalScanAction import GimbalScanAction
import time
from sklearn.cluster import DBSCAN
from sklearn.cluster import MeanShift
from sklearn import metrics
from sklearn.preprocessing import StandardScaler

import cv2

# import utm

filePath = '../../altitude_data/'

class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))
    
class SampleHazardDetector(IDataReceived):

    def __init__(self, tcpClient):
        self.__client = tcpClient
        
        
        
        self.__estimatedHazardZone = Polygon()
        self.__keepInZone = Rectangle()
        self.__currentLocationofUAV = {}
        
        
        self.__searchAreaCenterLat = 0
        self.__searchAreaCenterLong = 0
        self.__searchAreaWidth = 0
        self.__searchAreaHeight = 0
        self.__firezonePoints = {}
        self.__firezoneHintLocation = {}
        self.__centerLocation = Location3D()
        self.__gotHint = False
        
        
        
        
        
        self.anchor = []
        
        
        
        
        self.__noOfUAVs = 0
        self.__sendReport = False
        
        self.__maxSpeedofUAV = {}
        
        self.__resulationOfGrid = 1000
        self.__minidel = 500
        
        
        self.__waypoints = {}
        self.__uavsInMission = {}
        self.__MissionReady = False
        self.__noOfZone = 0
        self.__zoneassigned = {}
        
        
        self.__zoneCenter = {}
        self.__zoneboundaryPoints = {}
        
        self.altidata1 = pd.read_csv(filePath+'altidata1.csv',header=None)
        self.altidata1 = self.altidata1.T
        self.altidata2 = pd.read_csv(filePath+'altidata2.csv',header=None)
        self.altidata2 = self.altidata2.T
        self.altidata3 = pd.read_csv(filePath+'altidata3.csv',header=None)
        self.altidata3 = self.altidata3.T
        self.altidata4 = pd.read_csv(filePath+'altidata4.csv',header=None)
        self.altidata4 = self.altidata4.T

        self.__safeHeight = 100 # this value is substracted from the max range of the sensor
        self.__surveySafeHeight = 300
        self.__normalSearchAltitude = 450
        
        self.__initLocationOfUAVs = {}
        
        self.__maxAzimuthangle = {}
        self.__minAzimuthangle = {}
        self.__uavsInSearch = {}
        self.__uavsInSarvey = {}
        self.__uavisHeadingtoSurveylocation = {}
        self.__uavisInsmokeZone = {}
        self.__UAVSurvayingZoneId = {}
        
        
        
        
        self.__previousreportsendTime = 0
        self.__previousWeatherReportTime = 0
        
        
        self.__wspeed = 0
        self.__ditectionTheta = 0
        
        
        
        self.__totalWaypointsassignedToUAV = {}
        self.__previouswaypointNo = {}
        self.__visitedTotalwaypoints = {}
        self.__updateArea = False
        
       
        self.__currentVicleState = {}
        self.__simulationTimemilliSeconds = 0
        self.__hazardSensorStatus = {}
        self.__sensorRefreshrate = 1.0
        self.__recoveryPoints = []
        self.__entityConfigList = []
        self.__airvehicleConfigList = []
        self.__removedairVeicleList = []
        self.__currentEntityState = {}
        self.__uavsInZone = {}
        self.__maxSpeedGlobal = 0
        self.__sensorMaxrange = {}
        self.__windspeedupdateTime = 5000 #in milisecond
        self.__maxsurvayUAVForzone = 3
        self.__maxSpeedForsurvey = 25
        self.__surveyCircleRadius = 1000
        self.__searchCircleRadius = 3000
        self.__uavRecharging = {}

        self.__secondaryMergeThreshold = 0
        self.__globalMap = None
        self.__dgrid = None
        self.__glopbalmaxforpercentarea = 0
        self.__boundaryparameterFornewMission = [0,0,0,0]
        self.__stopRecursion = False
        self.__mapResulotion = 500 #in meter
        self.__initialSmallGridW = 0
        self.__initialSmallGridH = 0
        self.__uavInSmokemisssion = {}
        self.__uavisHeadingtoSmokeSurveylocation = {}
        self.__energyThreshold = 50
        self.__maxSpeedofUAVduringSurvey = {}
        self.__energyconsumptionRate = 0.13
        self.__mapHold = {}
        
    def dataReceived(self, lmcpObject):
        if isinstance(lmcpObject, KeepInZone):
            self.__keepInZone = lmcpObject.Boundary
            centerpoint = lmcpObject.Boundary.get_CenterPoint()
            self.__centerLocation = centerpoint
            self.__searchAreaCenterLat = centerpoint.get_Latitude()
            self.__searchAreaCenterLong = centerpoint.get_Longitude()
            self.__searchAreaWidth = lmcpObject.Boundary.get_Width()/2.0 - 500
            self.__searchAreaHeight = lmcpObject.Boundary.get_Height()/2.0 - 500

            row = int(lmcpObject.Boundary.get_Width()/self.__mapResulotion)
            col = int(lmcpObject.Boundary.get_Height()/self.__mapResulotion)
            self.__globalMap = np.zeros([row,col])
            self.__dgrid = np.zeros([row,col])
            
            print('found keep in zone')     
         
        elif isinstance(lmcpObject, RecoveryPoint):
            self.__recoveryPoints.append(lmcpObject.Boundary)
            
        elif isinstance(lmcpObject, AirVehicleState):
            airVehicleState = lmcpObject
            self.__simulationTimemilliSeconds = airVehicleState.Time
            self.__currentVicleState[airVehicleState.ID] = airVehicleState
            
            if self.__simulationTimemilliSeconds == 0:
                self.__initLocationOfUAVs[airVehicleState.ID] = airVehicleState.Location
                self.__currentLocationofUAV[airVehicleState.ID] = airVehicleState.Location
                    
                
            if self.__simulationTimemilliSeconds > 0: 
                if (self.__simulationTimemilliSeconds - self.__previousWeatherReportTime) > self.__windspeedupdateTime and airVehicleState.WindSpeed > 0:
                    self.__previousWeatherReportTime = self.__simulationTimemilliSeconds
                    self.__wspeed += 0 #airVehicleState.WindSpeed
                    self.__ditectionTheta = 0 #airVehicleState.WindDirection
                    self.__updateArea = True
                # elif airVehicleState.WindSpeed == 0:
                #     self.__wspeed = 0
                #     self.__ditectionTheta = 0

        elif isinstance(lmcpObject, EntityPerception):
            pass

        elif isinstance(lmcpObject, RemoveEntities):
            self.__removedairVeicleList.append(lmcpObject.EntityList[0])
            for airVehicleConfig in self.__airvehicleConfigList:
                if airVehicleConfig.ID == lmcpObject.EntityList[0]:
                    self.__airvehicleConfigList.remove(airVehicleConfig)
                    break

        elif isinstance(lmcpObject, AirVehicleConfiguration):
            airvehicleConfiguration = lmcpObject
            self.__maxSpeedofUAV[airvehicleConfiguration.ID] = airvehicleConfiguration.get_MaximumSpeed()
            payloadconfigList = airvehicleConfiguration.PayloadConfigurationList
            self.__maxAzimuthangle[airvehicleConfiguration.ID] = payloadconfigList[0].MaxAzimuth
            self.__minAzimuthangle[airvehicleConfiguration.ID] = payloadconfigList[0].MinAzimuth
            self.__sensorMaxrange[airvehicleConfiguration.ID] = airvehicleConfiguration.PayloadConfigurationList[2].MaxRange
            self.__airvehicleConfigList.append(airvehicleConfiguration)
            self.sendGimbleScanCommand(airvehicleConfiguration.ID,airvehicleConfiguration.PayloadConfigurationList[0].MaxAzimuthSlewRate)
             
        elif isinstance(lmcpObject, EntityConfiguration):
            self.__entityConfigList.append(lmcpObject)
            
        elif isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            detectedLocation = hazardDetected.get_DetectedLocation()
            detectingEntity = hazardDetected.get_DetectingEnitiyID()
            # vid = detectingEntity
            fireZoneType = hazardDetected.get_DetectedHazardZoneType()
            
            
            self.__hazardSensorStatus[detectingEntity] = time.time()

            if fireZoneType == HazardType.Fire:
                self.__maxSpeedofUAVduringSurvey[detectingEntity] = self.__maxSpeedofUAV[detectingEntity] if self.__maxSpeedofUAV[detectingEntity] <= self.__maxSpeedForsurvey else self.__maxSpeedForsurvey ## play here

                self.__uavsInSarvey[detectingEntity] = True
                self.__gotHint = True
            
                [x,y] = self.convertLatLonToxy(detectedLocation.get_Latitude(),detectedLocation.get_Longitude())
                zid = self.getZoneId([x,y])
                if not zid in self.__uavsInZone:
                    self.__uavsInZone[zid] = [detectingEntity]
                else:
                    if not detectingEntity in self.__uavsInZone[zid]:
                        self.__uavsInZone[zid].append(detectingEntity)

                self.__firezoneHintLocation[zid] = detectedLocation
                self.__UAVSurvayingZoneId[detectingEntity] = zid
                if not self.__firezonePoints or not zid in self.__firezonePoints:
                    self.__firezonePoints[zid] = [[x,y]]
                else:
                    self.__firezonePoints[zid].append([x,y])
            elif fireZoneType == HazardType.Smoke:
                # print('smoke detected')
                self.__uavisInsmokeZone[detectingEntity] = True
            #     pass
    
    def sendMissionCommand(self,veicleid,veicleLocation):
        missionCommand = MissionCommand()
        missionCommand.set_VehicleID(veicleid)
        missionCommand.set_Status(CommandStatusType.Pending)
        missionCommand.set_CommandID(1)

        zid,locid = self.getNearestZone(veicleLocation,veicleid)
        vstate = self.getAirVeicleState(veicleid)
        safeHeight = abs(self.__sensorMaxrange[veicleid] * sin(radians(vstate.PayloadStateList[0].Elevation))) - self.__safeHeight
        
        missionCommand.set_FirstWaypoint(locid)
        
        i = 0
        waypoints = self.__waypoints[zid]
        waypointaltimap,waypointconnectingmap = self.mapaltiwithwaypointnumber(waypoints)
        for waypoint in waypoints:
            i = i+1
            alti = max(waypointaltimap[waypoint.get_Number()], waypointaltimap[waypoint.get_NextWaypoint()], waypointaltimap[waypointconnectingmap[waypoint.get_NextWaypoint()]])
            waypoint.set_Altitude(alti + safeHeight)
            waypoint.set_Speed(self.__maxSpeedofUAV[veicleid])
            missionCommand.get_WaypointList().append(waypoint)

        self.__totalWaypointsassignedToUAV[veicleid] = i
        self.__visitedTotalwaypoints[veicleid] = 0
        self.__previouswaypointNo[veicleid] = locid
        self.__uavsInMission[veicleid] = True

        self.__client.sendLMCPObject(missionCommand)     
    
    def sendWaypoint(self,veicleid,initLocation,endLocation,radius=0):
        if radius == 0:
            radius = self.__searchCircleRadius
        missionCommand = MissionCommand()
        missionCommand.set_FirstWaypoint(1)
        missionCommand.set_VehicleID(veicleid)
        missionCommand.set_Status(CommandStatusType.Pending)
        missionCommand.set_CommandID(1)

        vstate = self.getAirVeicleState(veicleid)
        safeHeight = abs(self.__sensorMaxrange[veicleid] * sin(radians(vstate.PayloadStateList[0].Elevation))) - self.__safeHeight
        

        waypoints = self.getwaypointsBetweenLocations(initLocation,endLocation,veicleid,radius)
        waypointaltimap,waypointconnectingmap = self.mapaltiwithwaypointnumber(waypoints)

        i = 0
        for waypoint in waypoints:
            i += 1
            alti = max(waypointaltimap[waypoint.get_Number()], waypointaltimap[waypoint.get_NextWaypoint()], waypointaltimap[waypointconnectingmap[waypoint.get_NextWaypoint()]])
            waypoint.set_Altitude(alti + safeHeight)
            waypoint.set_Speed(self.__maxSpeedofUAV[veicleid])
            missionCommand.get_WaypointList().append(waypoint)

        self.__totalWaypointsassignedToUAV[veicleid] = i
        self.__visitedTotalwaypoints[veicleid] = 0
        self.__previouswaypointNo[veicleid] = 1
        self.__uavsInMission[veicleid] = True 
        self.__client.sendLMCPObject(missionCommand)
        
    def sendOnlywaypoints(self,vstate,waypoints,firstWaypoint=1):
        veicleid = vstate.ID
        missionCommand = MissionCommand()
        missionCommand.set_FirstWaypoint(firstWaypoint)
        missionCommand.set_VehicleID(veicleid)
        missionCommand.set_Status(CommandStatusType.Pending)
        missionCommand.set_CommandID(1)

        safeHeight = abs(self.__sensorMaxrange[vstate.ID] * sin(radians(vstate.PayloadStateList[0].Elevation))) - self.__safeHeight
        
        waypointaltimap,waypointconnectingmap = self.mapaltiwithwaypointnumber(waypoints)

        i = 0
        for waypoint in waypoints:
            i += 1
            alti = max(waypointaltimap[waypoint.get_Number()], waypointaltimap[waypoint.get_NextWaypoint()], waypointaltimap[waypointconnectingmap[waypoint.get_NextWaypoint()]])
            waypoint.set_Altitude(alti + safeHeight)
            waypoint.set_Speed(self.__maxSpeedofUAV[veicleid])
            missionCommand.get_WaypointList().append(waypoint)

        self.__totalWaypointsassignedToUAV[veicleid] = i
        self.__visitedTotalwaypoints[veicleid] = 0
        self.__previouswaypointNo[veicleid] = firstWaypoint
        self.__uavsInMission[veicleid] = True
        self.__client.sendLMCPObject(missionCommand)

    def sendGimbleScanCommand(self,veicleid,slewRate):
        #Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(veicleid)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)
         
        
        gimbalScanAction = GimbalScanAction()
        gimbalScanAction.set_PayloadID(1)
        gimbalScanAction.set_AzimuthSlewRate(60)
        gimbalScanAction.set_StartAzimuth(90)
        gimbalScanAction.set_EndAzimuth(-90)
        gimbalScanAction.set_ElevationSlewRate(10)
        gimbalScanAction.set_StartElevation(-45)
        gimbalScanAction.set_EndElevation(-45)
        
        vehicleActionCommand.get_VehicleActionList().append(gimbalScanAction)
        
        self.__client.sendLMCPObject(vehicleActionCommand)     

    def sendEstimateReport(self,zid):
        #Setting up the mission to send to the UAV
        hazardZoneEstimateReport = HazardZoneEstimateReport()
        hazardZoneEstimateReport.set_EstimatedZoneShape(self.__estimatedHazardZone)
        hazardZoneEstimateReport.set_UniqueTrackingID(zid)
        hazardZoneEstimateReport.set_EstimatedGrowthRate(0)
        hazardZoneEstimateReport.set_PerceivedZoneType(HazardType.Fire)
        hazardZoneEstimateReport.set_EstimatedZoneDirection(0)
        hazardZoneEstimateReport.set_EstimatedZoneSpeed(0)

        #Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(hazardZoneEstimateReport)
        
    def sendinitialMission(self):
        maxSpeed = max(self.__maxSpeedofUAV.values())
        self.__maxSpeedGlobal = maxSpeed
        zi = 0
        for id in self.__maxSpeedofUAV.keys():
            if self.__maxSpeedofUAV[id] == maxSpeed:
                self.sendMissionCommand(id,self.__initLocationOfUAVs[id])
                self.__uavsInMission[id] = True
                self.__uavsInSearch[id] = True
                zi += 1
            if zi == self.__noOfZone:
                break
        
        for zid in range(1,self.__noOfZone+1): 
            endLoc = self.__zoneCenter[zid]
            mind = 10e10
            minid = 0
            minLoc = Location3D()
            for id in self.__maxSpeedofUAV.keys():
                if not id in self.__uavsInMission:
                    startLoc =  self.__initLocationOfUAVs[id]
                    d = self.getdistance(startLoc,endLoc)
                    if d < mind:
                        mind = d
                        minid = id
                        minLoc = startLoc
            if minid != 0:            
                self.__uavsInMission[minid] = True
                self.sendWaypoint(minid,minLoc,endLoc)                
        
    def sendSmokeZonemission(self,vstate):
        # if self.__maxSpeedofUAV[vstate.ID] < self.__maxSpeedGlobal:
        # print('sendSmokeZonemission',vstate.ID)
        if vstate.EnergyAvailable > 95:
            if (not vstate.ID in self.__uavsInSarvey or (vstate.ID in self.__uavsInSarvey and not self.__uavsInSarvey[vstate.ID])) and (not vstate.ID in self.__uavisHeadingtoSurveylocation):
                waypoints,firstWaypoint = self.smokeZoneMission(vstate)
                self.sendOnlywaypoints(vstate,waypoints,firstWaypoint=firstWaypoint)
                self.__uavInSmokemisssion[vstate.ID] = True
        else:
            # call a available low speed uav
            self.callUAVSForSmokeZoneSurvey(vstate)
            # assign a mission
            waypoints,firstWaypoint = self.smokeZoneMission(vstate)
            self.sendOnlywaypoints(vstate,waypoints,firstWaypoint=firstWaypoint)
            self.__uavInSmokemisssion[vstate.ID] = True

    def smokeZoneMission(self,vstate): # needs to be debugged
        [xc,yc] = self.convertLatLonToxy(vstate.Location.get_Latitude(),vstate.Location.get_Longitude()) 
        r = self.__searchCircleRadius + 1000
        direction = 0
        points = self.GenerateSamplePointsOnACircleforSurvey(xc,yc,r,vstate.Heading,direction)
        vid = vstate.ID
        safeHeight = abs(self.__sensorMaxrange[vid] * sin(radians(vstate.PayloadStateList[0].Elevation))) - self.__safeHeight
        waypoints = []
        for i in range(1,len(points)+1):
            p = points[i-1]
            x = p[0]
            y = p[1]
            [lat,lon] = self.convertxyToLatLon(x,y)
            waypoint = Waypoint()
            waypoint.set_Latitude(lat)
            waypoint.set_Longitude(lon)
            alti = self.getAltitudeLatLon(lat,lon) 
            # if alti < self.__normalSearchAltitude:
            #     waypoint.set_Altitude(self.__normalSearchAltitude)
            # else:
            waypoint.set_Altitude(alti + safeHeight) #self.__safeHeight)
            waypoint.set_AltitudeType(AltitudeType.MSL)
            waypoint.set_Number(i)
            if i == len(points):
                waypoint.set_NextWaypoint(1)
            else:
                waypoint.set_NextWaypoint(i+1)
            waypoint.set_Speed(self.__maxSpeedofUAV[vid])
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(0)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
        minima = 1e10
        minLocid = 1
        minLoc = Location3D()
        
        for i in range(len(waypoints)):
            loc = waypoints[i]
            d = self.getdistance(loc,vstate.Location)
            if d < minima:
                minima = d
                minLoc = loc
                minLocid = i+1
        
        if sqrt(minima) < 1000:
            return waypoints,minLocid
        
        waypoints1,minLocid = self.getBetweenLatLon(vstate.Location,minLoc,waypointNumber,minima,minLocid,vstate.ID)
        waypoints = waypoints1 + waypoints
        return waypoints,minLocid

    def callUAVSForSmokeZoneSurvey(self,vstate):   
        for uav in self.__airvehicleConfigList:
            if uav.ID in self.__uavisHeadingtoSmokeSurveylocation or uav.ID in self.__uavInSmokemisssion:
                vstate1 = self.getAirVeicleState(uav.ID)
                d = self.getdistance(vstate.Location,vstate1.Location)
                if d**0.5 < 10000:
                    return
        print('call uav for survey smoke')
        mind = 10e20
        vid = -1
        for uav in self.__airvehicleConfigList:
            if (not uav.ID in self.__uavsInSearch) and (not uav.ID in self.__uavsInSarvey) and (not uav.ID in self.__uavisHeadingtoSurveylocation) and (not uav.ID in self.__uavisHeadingtoSmokeSurveylocation) and (self.__maxSpeedGlobal > uav.MaximumSpeed):
                vstate1 = self.getAirVeicleState(uav.ID)
                d = self.getdistance(vstate.Location,vstate1.Location)
                if d < mind:
                    mind = d
                    vid = vstate1.ID
                    minLoc = vstate1.Location
        if vid != -1:
            self.__uavisHeadingtoSmokeSurveylocation[vid] = True
            self.sendWaypoint(vid,minLoc,vstate.Location) 
        else:
            return

    def sendServeyCommand(self,vstate,direction,r=0,speed=0):
        veicleid = vstate.ID
        [xc,yc] = self.convertLatLonToxy(vstate.Location.Latitude,vstate.Location.Longitude)
        if r==0:
            r = self.__surveyCircleRadius
        if speed == 0:
            speed = self.__maxSpeedofUAV[veicleid]
        points = self.GenerateSamplePointsOnACircleforSurvey(xc,yc,r,vstate.Heading,direction)
        vid = vstate.ID
        safeHeight = abs(self.__sensorMaxrange[vid] * sin(radians(vstate.PayloadStateList[0].Elevation))) - self.__safeHeight
        missionCommand = MissionCommand()
        missionCommand.set_FirstWaypoint(1)
        missionCommand.set_VehicleID(vid)
        missionCommand.set_Status(CommandStatusType.Pending)
        missionCommand.set_CommandID(1)
        for i in range(1,len(points)+1):
            p = points[i-1]
            x = p[0]
            y = p[1]
            [lat,lon] = self.convertxyToLatLon(x,y)
            waypoint = Waypoint()
            waypoint.set_Latitude(lat)
            waypoint.set_Longitude(lon)
            alti = self.getAltitudeLatLon(lat,lon) 
            # if alti < self.__normalSearchAltitude:
            #     waypoint.set_Altitude(self.__normalSearchAltitude)
            # else:
            waypoint.set_Altitude(alti + safeHeight) #self.__safeHeight)
            waypoint.set_AltitudeType(AltitudeType.MSL)
            waypoint.set_Number(i)
            if i == len(points):
                waypoint.set_NextWaypoint(1)
            else:
                waypoint.set_NextWaypoint(i+1)
            waypoint.set_Speed(speed)
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(0)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            missionCommand.get_WaypointList().append(waypoint)
    
        self.__visitedTotalwaypoints[vstate.ID] = 0
        self.__previouswaypointNo[vstate.ID] = 1  
        self.__totalWaypointsassignedToUAV[vstate.ID] = i
        self.__uavsInMission[veicleid] = True

        self.__client.sendLMCPObject(missionCommand)

    def callUAVSForSurvey(self,vstate):
        while True:
            mind = 10e20
            vid = -1
            zid = self.__UAVSurvayingZoneId[vstate.ID]
            noofsurvayUAVinZone = len(self.__uavsInZone[zid])
            if noofsurvayUAVinZone >= self.__maxsurvayUAVForzone:
                return 

            for uav in self.__airvehicleConfigList:
                if (not uav.ID in self.__uavsInSearch) and (not uav.ID in self.__uavsInSarvey) and (not uav.ID in self.__uavisHeadingtoSurveylocation) and (self.__maxSpeedGlobal > uav.MaximumSpeed):
                    vstate1 = self.getAirVeicleState(uav.ID)
                    d = self.getdistance(vstate.Location,vstate1.Location)
                    if d < mind:
                        mind = d
                        vid = vstate1.ID
                        minLoc = vstate1.Location
            if vid != -1:
                self.__uavisHeadingtoSurveylocation[vid] = True
                self.__UAVSurvayingZoneId[vid] = zid
                self.__uavsInZone[zid].append(vid)
                self.sendWaypoint(vid,minLoc,vstate.Location) 
            else:
                return

    def convertLatLonToxy(self,lat,long):
        R = 111000
        a = lat-self.__searchAreaCenterLat
        b = long-self.__searchAreaCenterLong
        x = R*a
        y = R*cos(radians(lat))*b
        return [x,y]
        
    def convertxyToLatLon(self,x,y):
        R = 111000
        lat = x/R + self.__searchAreaCenterLat
        long = y/(R*cos(radians(lat))) + self.__searchAreaCenterLong
        return [lat,long]
     
    def GenerateSamplePointsOnACircle(self,x,y,r):
        Points = []
        StepSize = 30
        Np = round(360/StepSize)
        for i in range(Np):
            angle = 360 - i * StepSize
            xi = r * cos(radians(angle))
            yi = r * sin(radians(angle))
            Cx = x + xi
            Cy = y + yi
            Points.append([Cx,Cy])
        return Points
    
    def GenerateSamplePointsOnACircleforSurvey(self,xc,yc,r,headingangle,direction):
        # xc = xc + r*cos(radians(90-headingangle))
        # yc = yc - r*sin(radians(90-headingangle))
        # headingangle += 90
        Points = []
        Points.append([xc,yc])
        StepSize = 45
        Np = round(360/StepSize)
        for i in range(Np):
            if direction == 0:
                angle = (headingangle+180)%360 - i * StepSize
                angle = angle if angle >= 0 else angle + 360
            elif direction == 1:
                angle = ((headingangle+180)%360 + i * StepSize)%360
            xi = r * cos(radians(angle))
            yi = r * sin(radians(angle))
            Cx = xc + xi
            Cy = yc + yi
            Points.append([Cx,Cy])
        return Points

    def calculateGridCoordinate(self):
        self.__MissionReady = True
        self.__zoneCenter = {}
        
        self.__waypoints = {}
        w = self.__searchAreaWidth*2
        h = self.__searchAreaHeight*2
        self.__noOfUAVs = len(self.__airvehicleConfigList)
        maxSpeed = []
        for airvehicle in self.__airvehicleConfigList:
            maxSpeed.append(airvehicle.MaximumSpeed)

        self.__noOfZone = maxSpeed.count(max(maxSpeed))
        self.__noOfZone = self.__noOfZone if self.__noOfZone%2 == 0 else self.__noOfZone - 1 
        wSeg,hSeg = self.getBig2Factor(self.__noOfZone)

        dw = w/wSeg
        dh = h/hSeg
        currCenterx = -w/2
        currCentery = -h/2
        self.__secondaryMergeThreshold = max(dw,dh)
        for ws in range(wSeg):
            for hs in range(hSeg):
                zoneid = ws*hSeg + hs + 1

                waypointNumber = 1

                x = currCenterx
                y = currCentery

                [lat,lon] = self.convertxyToLatLon(x,y)
                waypoint = Waypoint()
                waypoint.set_Latitude(lat)
                waypoint.set_Longitude(lon)
                alti = self.getAltitudeLatLon(lat,lon) 
                if alti < self.__normalSearchAltitude:
                    waypoint.set_Altitude(self.__normalSearchAltitude)
                else:
                    waypoint.set_Altitude(alti + self.__safeHeight)
                waypoint.set_AltitudeType(AltitudeType.MSL)
                waypoint.set_Number(waypointNumber)
                waypoint.set_NextWaypoint(waypointNumber+1)
                waypoint.set_Speed(30)
                waypoint.set_SpeedType(SpeedType.Airspeed)
                waypoint.set_ClimbRate(15)
                waypoint.set_TurnType(TurnType.TurnShort)
                waypoint.set_ContingencyWaypointA(0)
                waypoint.set_ContingencyWaypointB(0)

                waypoints = []
                waypoints.append(waypoint)
                if ws%2==0:
                    if hs%2 == 0:
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx,currCentery,currCenterx+dw,currCentery+dh,2,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx+dw,currCentery+dh,currCenterx,currCentery+dh,waypointNumber,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx,currCentery+dh,currCenterx+dw,currCentery,waypointNumber,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx+dw,currCentery,currCenterx,currCentery,waypointNumber,1)
                        waypoints = waypoints + wpoints
                    else:
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx,currCentery,currCenterx+dw,currCentery+dh,2,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx+dw,currCentery+dh,currCenterx+dw,currCentery,waypointNumber,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx+dw,currCentery,currCenterx,currCentery+dh,waypointNumber,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx,currCentery+dh,currCenterx,currCentery,waypointNumber,1)
                        waypoints = waypoints + wpoints
                else:
                    if hs%2 != 0:
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx,currCentery,currCenterx+dw,currCentery+dh,2,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx+dw,currCentery+dh,currCenterx,currCentery+dh,waypointNumber,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx,currCentery+dh,currCenterx+dw,currCentery,waypointNumber,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx+dw,currCentery,currCenterx,currCentery,waypointNumber,1)
                        waypoints = waypoints + wpoints
                    else:
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx,currCentery,currCenterx+dw,currCentery+dh,2,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx+dw,currCentery+dh,currCenterx+dw,currCentery,waypointNumber,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx+dw,currCentery,currCenterx,currCentery+dh,waypointNumber,0)
                        waypoints = waypoints + wpoints
                        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(currCenterx,currCentery+dh,currCenterx,currCentery,waypointNumber,1)
                        waypoints = waypoints + wpoints

                self.__waypoints[zoneid] = waypoints

                zlocation = Location3D()
                [lat,lon] = self.convertxyToLatLon(currCenterx+dw/2,currCentery+dh/2)
                zlocation.set_Latitude(lat)
                zlocation.set_Longitude(lon)
                zlocation.set_Altitude(450)
                self.__zoneCenter[zoneid] = zlocation
                self.__zoneboundaryPoints[zoneid] = [[currCenterx,currCentery],[currCenterx+dw,currCentery],[currCenterx,currCentery+dh],[currCenterx+dw,currCentery+dh]]

                currCentery += dh
            currCenterx += dw
            currCentery = -h/2
       
    def getClosestPoint(self,points,refPoint):
        mind = 10e10
        closestPoint = []
        for i in len(points):
            d = self.distance(points[i],refPoint)
            if d < mind:
                mind = d
                closestPoint = points[i]
        return closestPoint
    
    def getNearestZone(self,location,vid):
        minima = 1e10
        zoneid = 0
        minLocid = 1
        minLoc = Location3D()
        for z in range(1,self.__noOfZone+1):
            if not z in self.__zoneassigned:
                waypoints = self.__waypoints[z]
                for i in range(len(waypoints)):
                    loc = waypoints[i]
                    d = self.getdistance(loc,location)
                    if d < minima:
                        minima = d
                        zoneid = z
                        minLocid = i+1
                        minLoc = loc
        self.__zoneassigned[zoneid]=True
        if sqrt(minima) < 1000:
            return zoneid,minLocid
        
        waypoints,minLocid = self.getBetweenLatLon(location,minLoc,len(self.__waypoints[zoneid]),minima,minLocid,vid)
        self.__waypoints[zoneid] = waypoints + self.__waypoints[zoneid]
        return zoneid,minLocid
    
    def getwaypointsBetweenLocations(self,startLoc,endLoc,vid,radius): 
        vstate = self.getAirVeicleState(vid)
        safeHeight = abs(self.__sensorMaxrange[vid] * sin(radians(vstate.PayloadStateList[0].Elevation))) - self.__safeHeight
        d = self.getdistance(startLoc,endLoc)
        [xs,ys] = self.convertLatLonToxy(startLoc.get_Latitude(),startLoc.get_Longitude())
        [xe,ye] = self.convertLatLonToxy(endLoc.get_Latitude(),endLoc.get_Longitude())
        # radius = self.__searchCircleRadius
        xc = xe
        yc = ye
        points = self.GenerateSamplePointsOnACircle(xc,yc,radius)
        mind = 10e20
        startIndx = 0
        i = 0
        for p in points:
            d = (xs-p[0])**2 + (ys-p[1])**2
            if d < mind:
                mind = d
                xe = p[0]
                ye = p[1]
                startIndx = i
            i += 1

        delx = xe-xs
        dely = ye-ys
        m = dely/delx
        ii = int(round(sqrt(d)/self.__minidel))
        delx /= ii
        ii = ii
        x = xs
        waypointNumber = 1
        waypoints = []
        x += delx
        for i in range(ii):
            y = ys + (x-xs)*m
            [lat,lon] = self.convertxyToLatLon(x,y)
            x += delx
            waypoint = Waypoint()
            waypoint.set_Latitude(lat)
            waypoint.set_Longitude(lon)
            alti = self.getAltitudeLatLon(lat,lon) 
            # if alti < self.__normalSearchAltitude:
            #     waypoint.set_Altitude(self.__normalSearchAltitude)
            # else:
            waypoint.set_Altitude(alti + safeHeight) #self.__safeHeight)
            waypoint.set_AltitudeType(AltitudeType.MSL)
            waypoint.set_Number(waypointNumber)
            waypoint.set_NextWaypoint(waypointNumber+1)
            waypoint.set_Speed(self.__maxSpeedofUAV[vid])
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        
        wpointnumber = waypointNumber

        i = 0
        while i != (len(points)-1):
            startIndx = startIndx  % (len(points)-1)
            p = points[startIndx+1]
            i += 1
            startIndx = (startIndx + 1)
            x = p[0]
            y = p[1]
            [lat,lon] = self.convertxyToLatLon(x,y)
            waypoint = Waypoint()
            waypoint.set_Latitude(lat)
            waypoint.set_Longitude(lon)
            alti = self.getAltitudeLatLon(lat,lon) 
            # if alti < self.__normalSearchAltitude:
            #     waypoint.set_Altitude(self.__normalSearchAltitude)
            # else:
            waypoint.set_Altitude(alti + safeHeight) #self.__safeHeight)
            waypoint.set_AltitudeType(AltitudeType.MSL)
            waypoint.set_Number(waypointNumber)
            if i == len(points)-1:
                waypoint.set_NextWaypoint(wpointnumber)
            else:
                waypoint.set_NextWaypoint(waypointNumber+1)
            waypoint.set_Speed(self.__maxSpeedofUAV[vid])
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        return waypoints
    
    def getBetweenLatLon(self,startLoc,endLoc,startwaypointId,d,connectingwaypointId,vid):
        vstate = self.getAirVeicleState(vid)
        safeHeight = abs(self.__sensorMaxrange[vid] * sin(radians(vstate.PayloadStateList[0].Elevation))) - self.__safeHeight
        [xs,ys] = self.convertLatLonToxy(startLoc.get_Latitude(),startLoc.get_Longitude())
        [xe,ye] = self.convertLatLonToxy(endLoc.get_Latitude(),endLoc.get_Longitude())
        delx = xe-xs
        dely = ye-ys
        m = dely/delx
        ii = int(round(sqrt(d)/self.__resulationOfGrid))
        delx /= ii
        ii = ii 
        x = xs
        waypointNumber = startwaypointId+1
        waypoints = []
        x += delx
        for i in range(ii):
            y = ys + (x-xs)*m
            [lat,lon] = self.convertxyToLatLon(x,y)
            x += delx
            waypoint = Waypoint()
            waypoint.set_Latitude(lat)
            waypoint.set_Longitude(lon)
            alti = self.getAltitudeLatLon(lat,lon) 
            # if alti < self.__normalSearchAltitude:
            #     waypoint.set_Altitude(self.__normalSearchAltitude)
            # else:
            waypoint.set_Altitude(alti + safeHeight) #self.__safeHeight)
            waypoint.set_AltitudeType(AltitudeType.MSL)
            waypoint.set_Number(waypointNumber)
            if i == ii-1:
                waypoint.set_NextWaypoint(connectingwaypointId)
            else:
                waypoint.set_NextWaypoint(waypointNumber+1)
            waypoint.set_Speed(self.__maxSpeedofUAV[vid])
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        waypointNumber -= 1
        return waypoints, startwaypointId+1
    
    def getBetweenLatLonwithoutVIDAlt(self,xs,ys,xe,ye,waypointNumber,option):
        delx = xe-xs
        dely = ye-ys
        d = delx**2 + dely**2
        if delx != 0:
            m = dely/delx
        ii = int(round(sqrt(d)/self.__resulationOfGrid))
        delx /= ii
        dely /= ii
        ii = ii - 1
        x = xs
        y = ys
        waypoints = []
        x += delx
        for i in range(ii):
            if delx == 0:
                y += dely
            else:
                y = ys + (x-xs)*m 
            [lat,lon] = self.convertxyToLatLon(x,y)
            x += delx
            waypoint = Waypoint()
            waypoint.set_Latitude(lat)
            waypoint.set_Longitude(lon)
            alti = self.getAltitudeLatLon(lat,lon) 
            if alti < self.__normalSearchAltitude:
                waypoint.set_Altitude(self.__normalSearchAltitude)
            else:
                waypoint.set_Altitude(alti + self.__safeHeight)
            waypoint.set_AltitudeType(AltitudeType.MSL)
            waypoint.set_Number(waypointNumber)
            if option==1 and i == ii-1:
                waypoint.set_NextWaypoint(1)
            else:
                waypoint.set_NextWaypoint(waypointNumber+1)
            waypoint.set_Speed(30)
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        
        return waypoints,waypointNumber
    
    def mapaltiwithwaypointnumber(self,waypoints):
        map = {}
        wpointconnectmap = {}
        for waypoint in waypoints:
            alti = self.getAltitudeLatLon(waypoint.get_Latitude(),waypoint.get_Longitude())
            map[waypoint.get_Number()] = alti
            wpointconnectmap[waypoint.get_Number()] = waypoint.get_NextWaypoint()
        return map,wpointconnectmap

    def getAltitudeLatLon(self,lat,lon):
        if (lat - 39.0) <= 1:
            if abs(lon + 122.0) <= 1:
                i = round((lat - 39) * 3600) 
                j = round((lon + 122) * 3600)
                sz = self.altidata1.shape
                if i >= sz[0]:
                    i = sz[0] - 1
                elif i < 0:
                    i = 0
                if j >= sz[1]:
                    j = sz[1] - 1
                elif j<0:
                    j=0
                Altitude = self.altidata1[i][j]
            else:
                i = round((lat - 39) * 3600)
                j = round((lon + 121) * 3600)
                sz = self.altidata2.shape
                if i >= sz[0]:
                    i = sz[0] - 1
                elif i < 0:
                    i = 0
                if j >= sz[1]:
                    j = sz[1] - 1
                elif j < 0:
                    j = 0
                Altitude = self.altidata2[i][j]
        else:
            if (lon + 122) <= 1:
                i = round((lat - 40) * 3600)
                j = round((lon + 122) * 3600) 
                sz = self.altidata3.shape
                if i >= sz[0]:
                    i = sz[0] - 1
                elif i < 0:
                    i = 0
                if j >= sz[1]:
                    j = sz[1] - 1
                elif j < 0:
                    j = 0
                Altitude = self.altidata3[i][j] 
            else:
                i = round((lat - 40) * 3600)
                j = round((lon + 121) * 3600)
                sz = self.altidata4.shape
                if i >= sz[0]:
                    i = sz[0] - 1
                elif i < 0:
                    i = 0
                if j >= sz[1]:
                    j = sz[1] - 1
                elif j < 0:
                    j = 0
                Altitude = self.altidata4[i][j]
        return Altitude
    
    def getAltitude(self,location):
        lat = location.get_Latitude()
        lon = location.get_Longitude()
        if (lat - 39.0) <= 1:
            if abs(lon + 122.0) <= 1:
                i = round((lat - 39) * 3600)
                j = round((lon + 122) * 3600)
                sz = self.altidata1.shape
                if i >= sz[0]:
                    i = sz[0] - 1
                elif i < 0:
                    i = 0
                if j >= sz[1]:
                    j = sz[1] - 1
                elif j<0:
                    j=0
                Altitude = self.altidata1[i][j]
            else:
                i = round((lat - 39) * 3600)
                j = round((lon + 121) * 3600)
                sz = self.altidata2.shape
                if i >= sz[0]:
                    i = sz[0] - 1
                elif i < 0:
                    i = 0
                if j >= sz[1]:
                    j = sz[1] - 1
                elif j < 0:
                    j = 0
                Altitude = self.altidata2[i][j]
        else:
            if (lon + 122) <= 1:
                i = round((lat - 40) * 3600)
                j = round((lon + 122) * 3600)
                sz = self.altidata3.shape
                if i >= sz[0]:
                    i = sz[0] - 1
                elif i < 0:
                    i = 0
                if j >= sz[1]:
                    j = sz[1] - 1
                elif j < 0:
                    j = 0
                Altitude = self.altidata3[i][j] 
            else:
                i = round((lat - 40) * 3600)
                j = round((lon + 121) * 3600)
                sz = self.altidata4.shape
                if i >= sz[0]:
                    i = sz[0] - 1
                elif i < 0:
                    i = 0
                if j >= sz[1]:
                    j = sz[1] - 1
                elif j < 0:
                    j = 0
                Altitude = self.altidata4[i][j]
        return Altitude
        
    def isinKeepInZone(self,location):
        xyposition = self.convertLatLonToxy(location.get_Latitude(),location.get_Longitude())
        if abs(xyposition[0]) <= self.__searchAreaWidth and abs(xyposition[1]) <= self.__searchAreaHeight:
            return True
        return False

    def getZoneIdLocation(self,loc):
        zlocation = loc
        mind =10e20
        zid = 0
        for id in self.__zoneCenter.keys():
            zc = self.__zoneCenter[id]
            d = self.getdistance(zlocation,zc)
            if d < mind:
                mind = d
                zid = id
        return zid
        
    def getZoneId(self,point):
        zlocation = Location3D()
        [lat,lon] = self.convertxyToLatLon(point[0],point[1])
        zlocation.set_Latitude(lat)
        zlocation.set_Longitude(lon)
        mind = 10e20
        zid = 0
        for id in self.__zoneCenter.keys():
            zc = self.__zoneCenter[id]
            d = self.getdistance(zlocation,zc)
            if d < mind:
                mind = d
                zid = id
        return zid
    
    def getdistance(self,loc1,loc2):
        loc1XY = self.convertLatLonToxy(loc1.get_Latitude(),loc1.get_Longitude())
        loc2XY = self.convertLatLonToxy(loc2.get_Latitude(),loc2.get_Longitude())
        d = (loc1XY[0] - loc2XY[0])**2 + (loc1XY[1] - loc2XY[1])**2

        return d
            
    def graham_scan(self,points): 
        #print('points', points)
        min_idx=None
        for i,(x,y) in enumerate(points):
            if min_idx==None or y<points[min_idx][1]:
                min_idx=i
            if y==points[min_idx][1] and x<points[min_idx][0]:
                min_idx=i
        self.anchor=points[min_idx]
        sorted_pts=self.quicksort(points)
        del sorted_pts[sorted_pts.index(self.anchor)]
        hull=[self.anchor,sorted_pts[0]]
        for s in sorted_pts[1:]:
            while self.det(hull[-2],hull[-1],s)<=0:
                del hull[-1] 
            hull.append(s)
        return hull

    def det(self,p1,p2,p3):
        return   (p2[0]-p1[0])*(p3[1]-p1[1]) \
                -(p2[1]-p1[1])*(p3[0]-p1[0])        
    
    def quicksort(self,a):
        if len(a)<=1: return a
        smaller,equal,larger=[],[],[]
        piv_ang=self.polar_angle(a[randint(0,len(a)-1)]) # select random pivot
        for pt in a:
            pt_ang=self.polar_angle(pt) # calculate current point angle
            if   pt_ang<piv_ang:  smaller.append(pt)
            elif pt_ang==piv_ang: equal.append(pt)
            else: 				  larger.append(pt)
        return   self.quicksort(smaller) \
                +sorted(equal,key=self.distance) \
                +self.quicksort(larger)    
     
    def distance(self,p0,p1=None):
        if p1==None: p1=self.anchor
        y_span=p0[1]-p1[1]
        x_span=p0[0]-p1[0]
        return y_span**2 + x_span**2
            
    def polar_angle(self,p0,p1=None):
        if p1==None: p1=self.anchor
        y_span=p0[1]-p1[1]
        x_span=p0[0]-p1[0]
        return atan2(y_span,x_span)

    def secondaryMerge(self,data):
        #print('before secondary merge')
        #print(data.keys())
        #loop = len(data.keys())-1
        #for lp in range(loop):
        while True:
            #print('in loop',lp)
            flag = True
            keys = list(data.keys())
            # newData = {}
            # newKey = 0
            # print(keys)
            for i in range(len(keys)):
                for j in range(i+1,len(keys)):
                    p1 = np.mean(np.array(data[keys[i]]),axis=0)
                    p2 = np.mean(np.array(data[keys[j]]),axis=0)
                    d = np.linalg.norm(p1-p2)
                    #print(p1,p2,d,self.__secondaryMergeThreshold)
                    if d <= self.__secondaryMergeThreshold:# and not self.checksubset(data[keys[j]][:],data[keys[i]][:]):
                        data[keys[i]] = data[keys[i]] + data[keys[j]]
                        del data[keys[j]]
                        # print('one data merged')
                        # newKey += 1
                        flag = False
                        break
                if not flag:
                    break
            if flag:
                break        
            # if newKey != 0:
            #     data = dict(newData)
            # print(data)
            # print("**********************")
        
        # print('data after secondary merge done phase 1')
        # while True:
        #     keys = list(data.keys())
        #     flag = True
        #     for i in range(len(keys)):
        #         for j in range(i+1,len(keys)):
        #             if self.checksubset(data[keys[j]],data[keys[i]]):
        #                 flag = False
        #                 del data[keys[j]]
        #                 print('***************deleting data*******************')
        #                 break
        #         if not flag:
        #             break
        #     if flag:
        #         break
        #print('data after secondary merge')
        #print(data.keys())
        return dict(data)

    def mergeFireZones(self, allxypoints):
        originalxypoints = np.array(allxypoints)
        allxypoints = StandardScaler().fit_transform(allxypoints)
        clustering = MeanShift().fit(allxypoints)
        labels = np.array(clustering.labels_)
        uniquelabels = np.unique(labels)
        # print(uniquelabels)
        pointsInZones = {}
        for label in uniquelabels:
            condition = labels==label
            data = originalxypoints[condition]
            # print('**********')
            # print(data)
            pointsInZones[label] = data
        # print(pointsInZones)
        return dict(pointsInZones)

    def findBoundaryandSendReport(self):        
        if self.__firezonePoints:
            reset = False
            allxypoints = []
            for key in self.__firezonePoints.keys():
                points = self.__firezonePoints[key]
                # self.__firezonePoints[key] = []
                
                allxypoints += points
                   
            if len(allxypoints) >= 3:
                pointsInZones = self.mergeFireZones(allxypoints[:])
                data = {}
                flag = False
                for key in pointsInZones.keys():
                    allxypoints = pointsInZones[key]
                    # print('points')
                    # print(allxypoints)
                    if len(allxypoints) >= 3:
                        boundarypoints = self.graham_scan(list(allxypoints[:]))
                        data[key] = boundarypoints
                        flag = True
                if flag:
                    data = self.secondaryMerge(dict(data))
                else:
                    data = self.secondaryMerge(dict(pointsInZones))
                #print('data returned',data)
                for key in data.keys():
                    allxypoints = data[key]
                    # print('points')
                    # print(allxypoints)
                    if len(allxypoints) >= 3:
                        #print('what is that 2',allxypoints)
                        boundarypoints = self.graham_scan(list(allxypoints[:]))
                        
                        #print('boundary points')
                        # print(boundarypoints)
                        # self.__firezonePoints[key] = boundarypoints
                        if self.__wspeed != 0:
                            boundarypoints = self.translateEstimatedShape(self.__wspeed,self.__ditectionTheta,boundarypoints[:])
                            # boundarypoints = self.__firezonePoints[key]
                            reset = True
                        for xypoint in boundarypoints:
                            [lat,lon]=self.convertxyToLatLon(xypoint[0],xypoint[1])
                            locationpoint = Location3D()
                            locationpoint.set_Latitude(lat)
                            locationpoint.set_Longitude(lon)
                            self.__estimatedHazardZone.get_BoundaryPoints().append(locationpoint)             
                        self.sendEstimateReport(key)
                        self.__estimatedHazardZone = Polygon()
            # if reset:
            #     self.__wspeed = 0
            #     self.__ditectionTheta = 0

    def getSendReportStatus(self):
        # return self.__sendReport
        return self.__gotHint
    
    def setSendReportStatus(self,status):
        self.__sendReport = status

    def getupdateAreaStatus(self):
        return self.__updateArea
    
    def setupdateAreaStatus(self,status):
        self.__updateArea = status
        
    def updateEstimatedArea(self):
        if self.__firezonePoints:
            reset = False
            for key in self.__firezonePoints.keys():
                points = self.__firezonePoints[key]
                allxypoints = points
                   
                if len(allxypoints) >= 3:
                    boundarypoints = self.graham_scan(allxypoints)
                    self.__firezonePoints[key] = boundarypoints
                    if self.__wspeed != 0:
                        self.translateEstimatedShape(self.__wspeed,self.__ditectionTheta)
                        reset = True
            if reset:
                self.__wspeed = 0
                self.__ditectionTheta = 0
        
    def translateEstimatedShape(self, WinSpeed, Angle, firepoints):
        Rate = WinSpeed
        X_Rate = Rate * cos(radians(Angle))
        Y_Rate = Rate * sin(radians(Angle))
        # if self.__firezonePoints:
        #     for key in self.__firezonePoints.keys():
        #         points = self.__firezonePoints[key]
        #         SamplePoints = points
        NewSamplePoint = []
        if firepoints:
            for [x,y] in firepoints:
                x = x - X_Rate
                y = y - Y_Rate
                NewSamplePoint.append([x,y])
        else:
            NewSamplePoint = firepoints
        # self.__firezonePoints[key] = NewSamplePoint
        return list(NewSamplePoint)
    
    def getBig2Factor(self,num):
        ii = int(num/2)+1
        for i in range(ii):
            if i>0 and num%i == 0 and num/i <= i:
                return i,int(num/i)
                
    def checksubset(self,y,x):
        #test y is a subset of x or not
        counter = 0
        print('check subset')
        for i in range(len(y)):
            for j in range(len(x)):
                # print(x[j] == y[i])
                # print('*************')
                if sum(abs(x[j] - y[i])<[0.001,0.001])==len(y[i]):
                    counter += 1
        return counter == len(y)

    def getSimTime(self):
        return self.__simulationTimemilliSeconds

    def getMissionReadyStatus(self):
        return self.__MissionReady

    def getAirVeicleState(self,vid):
        # if not vid in self.__hazardSensorStatus:
        #     self.__hazardSensorStatus[vid] = 0
        # dt = (time.time()-self.__hazardSensorStatus[vid])
        if not vid in self.__currentVicleState:
            return None
        return self.__currentVicleState[vid]#,  1 if dt < self.__sensorRefreshrate else 0

    def getNoOfUAVs(self):
        return self.__noOfUAVs

    def getSurveyStatus(self,vid):
        if not vid in self.__uavsInSarvey:
            return False
        return self.__uavsInSarvey[vid]
    
    def setSurveyStatus(self,vid,status):
        self.__uavsInSarvey[vid] = status

    def getSmokeZoneStatus(self,vstate):
        if not vstate.ID in self.__uavisInsmokeZone:
            return False
        return self.__uavisInsmokeZone[vstate.ID]

    def setSmokeZoneStatus(self,vstate,status):
        self.__uavisInsmokeZone[vstate.ID] = status

    def getSmokeMissionStatus(self,vstate):
        if not vstate.ID in self.__uavInSmokemisssion:
            return False
        return self.__uavInSmokemisssion[vstate.ID]

    def getAirveicleConfigList(self):
        return self.__airvehicleConfigList

    def getSurveyDirection(self,vid):
        zid = self.__UAVSurvayingZoneId[vid]
        uavlist = self.__uavsInZone[zid]
        indx = uavlist.index(vid)
        if indx%2 == 0:
            return 0
        return 1

    def isMissionComplete(self,vstate):
        veicleid = vstate.ID
        if vstate.Mode == NavigationMode.Waypoint:
            currentwaypointNo = vstate.CurrentWaypoint
            if not vstate.ID in self.__previouswaypointNo:
                self.__previouswaypointNo[vstate.ID] = currentwaypointNo
                return False

            if currentwaypointNo != self.__previouswaypointNo[vstate.ID]:
                self.__previouswaypointNo[vstate.ID] = currentwaypointNo
                if not vstate.ID in self.__visitedTotalwaypoints:
                    self.__visitedTotalwaypoints[vstate.ID] = 0
                self.__visitedTotalwaypoints[vstate.ID] += 1
                if self.__visitedTotalwaypoints[vstate.ID] >= self.__totalWaypointsassignedToUAV[vstate.ID]:
                    self.__visitedTotalwaypoints[vstate.ID] = 0
                    # print('uav ',vstate.ID, ' finished its mission')
                    self.__uavsInMission[vstate.ID] = False
                    return True
        else:
            return True

        if veicleid in self.__uavRecharging and not self.__uavRecharging[veicleid] and veicleid in self.__uavsInSearch and not self.__uavsInSearch[veicleid]:
                self.__uavsInSarvey[veicleid] = False
                return True
                
        return False

    def assignNewMission(self,vstate):
        vid = vstate.ID
        if vid in self.__uavsInSarvey and self.__uavsInSarvey[vid]:
            #increase the radius of survey by 1km
            direction = self.getSurveyDirection(vstate.ID)
            self.sendServeyCommand(vstate,direction,r=(self.__surveyCircleRadius+1000),speed=self.__maxSpeedofUAVduringSurvey[vid])
        elif (self.__maxSpeedofUAV[vid] < self.__maxSpeedGlobal) and vid in self.__uavisHeadingtoSurveylocation:
            self.sendServeyCommand(vstate,0,r=(self.__searchCircleRadius+1000))
        elif (self.__maxSpeedofUAV[vid] < self.__maxSpeedGlobal) and vid in self.__uavInSmokemisssion:
            self.sendServeyCommand(vstate,0,r=(self.__searchCircleRadius+1000))
        elif (not (self.__maxSpeedofUAV[vid] < self.__maxSpeedGlobal)) and ((vid in self.__uavsInSearch and not self.__uavsInSearch[vid]) or (not vid in self.__uavsInSearch) or (not vid in self.__uavsInMission) or (vid in self.__uavsInMission and not self.__uavsInMission[vid])):
            self.getNewAreaforSearch(vstate)
            self.__uavsInSearch[vid] = True

    def checkpowerStatus(self,AirVehicleState):
        veicleid = AirVehicleState.ID
        Goback = False
        AvailableEnergy =  AirVehicleState.EnergyAvailable
        if AvailableEnergy > self.__energyThreshold:
            self.__uavRecharging[veicleid] = False
            return False
        if veicleid in self.__uavRecharging and self.__uavRecharging[veicleid]:
            self.__mapHold[veicleid] = []
            return True

        EnergyRate = AirVehicleState.ActualEnergyRate
        Location = AirVehicleState.Location
        Speed = self.__maxSpeedofUAV[veicleid]
        Lat = Location.get_Latitude()
        Lon = Location.get_Longitude()
        [x,y] = self.convertLatLonToxy(Lat,Lon)
        Dist = []
        RecoveryPos = [x,y]
        if EnergyRate > 0.0:
            Dist = []
            for rzone in self.__recoveryPoints:
                [x2,y2] = self.convertLatLonToxy(rzone.CenterPoint.get_Latitude(),rzone.CenterPoint.get_Longitude())
                d = ((x - x2)**2 + (y - y2)**2)**0.5
                Dist.append(d)
            MinIndice = np.argmin(Dist)
            RemainTime =  AvailableEnergy / self.__energyconsumptionRate#EnergyRate
            MaximumDistLeft = Speed * RemainTime
            # if veicleid == 4:
            #     print('uav',veicleid, AvailableEnergy,'max disthat can be traveled ', MaximumDistLeft, 'recovery zone area', Dist[MinIndice])
            if Dist[MinIndice] >= MaximumDistLeft:
                Goback = True
                RecoveryPos = self.__recoveryPoints[MinIndice].CenterPoint 
                radius = self.__recoveryPoints[MinIndice].Radius
                radius = radius*0.95
                self.__uavRecharging[veicleid] = True
                self.__uavsInSearch[veicleid] = False
                self.__uavsInSarvey[veicleid] = False
                self.sendWaypoint(veicleid, Location, RecoveryPos, radius=radius)
                # print('uav',veicleid,'recharging')
                self.__mapHold[veicleid] = []
                return True

        return False #Goback,RecoveryPos

    def updateGlobalMap(self,vstate):
        [x,y] = self.convertLatLonToxy(vstate.Location.get_Latitude(),vstate.Location.get_Longitude())

        x = abs(-x + self.__searchAreaWidth)
        y = abs(y + self.__searchAreaHeight)
        i = int(x/self.__mapResulotion)
        j = int(y/self.__mapResulotion)

        i = i if i < self.__globalMap.shape[0] else self.__globalMap.shape[0] - 1
        j = j if j < self.__globalMap.shape[1] else self.__globalMap.shape[1] - 1

        self.__globalMap[i][j] = 1
        # print(self.__globalMap.shape)

    def getNewAreaforSearch(self,vstate): # needs work 
        [x,y] = self.convertLatLonToxy(vstate.Location.get_Latitude(),vstate.Location.get_Longitude())

        x = abs(-x + self.__searchAreaWidth)
        y = abs(y + self.__searchAreaHeight)
        ic = int(x/self.__mapResulotion)
        jc = int(y/self.__mapResulotion)

        ic = ic if ic < self.__globalMap.shape[0] else self.__globalMap.shape[0] - 1
        jc = jc if jc < self.__globalMap.shape[1] else self.__globalMap.shape[1] - 1

        w = int(10000/self.__mapResulotion)
        h = int(10000/self.__mapResulotion)

        print('center of search',ic,jc)

        self.__dgrid = np.zeros(self.__dgrid.shape)
        self.__glopbalmaxforpercentarea = 0
        self.__boundaryparameterFornewMission = [0,0,0,0]
        self.__stopRecursion = False
        
        
        flag,self.__boundaryparameterFornewMission = self.recursiveSearch(ic,jc,w,h)

        self.__mapHold[vstate.ID] = self.__boundaryparameterFornewMission

        print(self.__boundaryparameterFornewMission)
        if flag:
            x1 = -self.__boundaryparameterFornewMission[0]*self.__mapResulotion + self.__searchAreaWidth
            y1 = self.__boundaryparameterFornewMission[2]*self.__mapResulotion - self.__searchAreaHeight

            x2 = -self.__boundaryparameterFornewMission[1]*self.__mapResulotion + self.__searchAreaWidth
            y2 = self.__boundaryparameterFornewMission[2]*self.__mapResulotion - self.__searchAreaHeight

            x3 = -self.__boundaryparameterFornewMission[1]*self.__mapResulotion + self.__searchAreaWidth
            y3 = self.__boundaryparameterFornewMission[3]*self.__mapResulotion - self.__searchAreaHeight

            x4 = -self.__boundaryparameterFornewMission[0]*self.__mapResulotion + self.__searchAreaWidth
            y4 = self.__boundaryparameterFornewMission[3]*self.__mapResulotion - self.__searchAreaHeight

            waypointNumber = 1

            x = x1
            y = y1

            [lat,lon] = self.convertxyToLatLon(x,y)
            waypoint = Waypoint()
            waypoint.set_Latitude(lat)
            waypoint.set_Longitude(lon)
            alti = self.getAltitudeLatLon(lat,lon) 
            if alti < self.__normalSearchAltitude:
                waypoint.set_Altitude(self.__normalSearchAltitude)
            else:
                waypoint.set_Altitude(alti + self.__safeHeight)
            waypoint.set_AltitudeType(AltitudeType.MSL)
            waypoint.set_Number(waypointNumber)
            waypoint.set_NextWaypoint(waypointNumber+1)
            waypoint.set_Speed(30)
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)

            waypoints = []
            waypoints.append(waypoint)
            
            wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(x1,y1,x3,y3,2,0)
            waypoints = waypoints + wpoints
            wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(x3,y3,x2,y2,waypointNumber,0)
            waypoints = waypoints + wpoints
            wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(x2,y2,x4,y4,waypointNumber,0)
            waypoints = waypoints + wpoints
            wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(x4,y4,x1,y1,waypointNumber,1)
            waypoints = waypoints + wpoints

            minima = 1e10
            minLocid = 1
            minLoc = Location3D()
            
            for i in range(len(waypoints)):
                loc = waypoints[i]
                d = self.getdistance(loc,vstate.Location)
                if d < minima:
                    minima = d
                    minLoc = loc
                    minLocid = i+1
            
            if sqrt(minima) < 1000:
                self.sendOnlywaypoints(vstate,waypoints,firstWaypoint=minLocid)
                return waypoints,minLocid
            
            waypoints1,minLocid = self.getBetweenLatLon(vstate.Location,minLoc,waypointNumber,minima,minLocid,vstate.ID)
            waypoints = waypoints1 + waypoints
            self.sendOnlywaypoints(vstate,waypoints,firstWaypoint=minLocid)
            return waypoints,minLocid

    def recursiveSearch(self,i,j,w,h):
        # print('in the loop')
        gridw = self.__globalMap.shape[0]
        gridh = self.__globalMap.shape[1]
        
        l = int(gridw/w)
        m = int(gridh/h)
        uavlist = self.getAirveicleConfigList()

        for i in range(l-1):
            for j in range(m-1):
                flag = True
                area1 = self.__globalMap[i*w:(i+1)*w,j*h:(j+1)*h]
                p = sum(sum(area1))/float(area1.shape[0]*area1.shape[1])
                if (1-p)>0.6:
                    for uav in uavlist:
                        if uav.ID in self.__mapHold and self.__mapHold[uav.ID] and i*w == self.__mapHold[uav.ID][0] and (i+1)*w == self.__mapHold[uav.ID][1] and j*h == self.__mapHold[uav.ID][2] and (j+1)*h == self.__mapHold[uav.ID][3]:
                            flag = False
                            break
                    if flag:
                        return True, [i*w,(i+1)*w,j*h,(j+1)*h]
        return False,[]

    def saveMAP(self):
         cv2.imwrite("globalMap.png", self.__globalMap*255)

    def getsurveyspeed(self,vstate):
        return self.__maxSpeedofUAVduringSurvey[vstate.ID]

    def getrechargeStatus(self,vstate):
        return self.__uavRecharging[vstate.ID]


#################
## Main
#################

if __name__ == '__main__':
    myHost = 'localhost'
    myPort = 5555
    amaseClient = AmaseTCPClient(myHost, myPort)
    #amaseClient.addReceiveCallback(PrintLMCPObject())
    smpleHazardDetector = SampleHazardDetector(amaseClient)
    amaseClient.addReceiveCallback(smpleHazardDetector)

    try:
        # make a threaded client, listen until a keyboard interrupt (ctrl-c)
        #start client thread
        amaseClient.start()

        dt = 1
        sensorState = {}
        savetime = time.time()
        previousReportingtime = 0
        reportingTime = 10
        while True:
            #wait for keyboard interrupt
            if smpleHazardDetector.getSimTime() > 0:
                if smpleHazardDetector.getSendReportStatus() and (time.time() - previousReportingtime) > reportingTime:
                    smpleHazardDetector.findBoundaryandSendReport()
                    previousReportingtime = time.time()
            
                if not smpleHazardDetector.getMissionReadyStatus():
                    smpleHazardDetector.calculateGridCoordinate()
                    smpleHazardDetector.sendinitialMission()

                uavlist = smpleHazardDetector.getAirveicleConfigList()
                for uav in uavlist:
                    vstate = smpleHazardDetector.getAirVeicleState(uav.ID)
                    if vstate is not None:
                        #update global map
                        smpleHazardDetector.updateGlobalMap(vstate)
                        #check power
                        if not smpleHazardDetector.checkpowerStatus(vstate):
                            #check if mission complete or not
                            if smpleHazardDetector.isMissionComplete(vstate):
                                smpleHazardDetector.assignNewMission(vstate)
                                
                            if smpleHazardDetector.getSurveyStatus(uav.ID):
                                if not uav.ID in sensorState:
                                    direction = smpleHazardDetector.getSurveyDirection(uav.ID)
                                    smpleHazardDetector.sendServeyCommand(vstate,direction,speed=smpleHazardDetector.getsurveyspeed(vstate))
                                    smpleHazardDetector.callUAVSForSurvey(vstate)
                                    sensorState[uav.ID] = 1
                                elif vstate.Mode == NavigationMode.Waypoint and vstate.CurrentWaypoint > 2:
                                    direction = smpleHazardDetector.getSurveyDirection(uav.ID)
                                    smpleHazardDetector.sendServeyCommand(vstate,direction,speed=smpleHazardDetector.getsurveyspeed(vstate))
                                    smpleHazardDetector.callUAVSForSurvey(vstate)
                                smpleHazardDetector.setSurveyStatus(uav.ID,False)
                            elif smpleHazardDetector.getSmokeZoneStatus(vstate) and not smpleHazardDetector.getSmokeMissionStatus(vstate):
                                smpleHazardDetector.sendSmokeZonemission(vstate)
                                smpleHazardDetector.setSmokeZoneStatus(vstate,False)

                            

            time.sleep(dt)

            if (time.time() - savetime) > 700:
                savetime = time.time()
                smpleHazardDetector.saveMAP()
           
    except KeyboardInterrupt as ki:
        print("Stopping amase tcp client")
    except Exception as ex:
        print('exception')
        print(ex)
        # print(ex.args)
    smpleHazardDetector.saveMAP()
    amaseClient.stop()
