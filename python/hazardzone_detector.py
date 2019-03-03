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
from math import sin,cos,atan2,pi,radians,sqrt
from random import randint
import pandas as pd
# import utm


class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))
    
class SampleHazardDetector(IDataReceived):

    def __init__(self, tcpClient):
        self.__client = tcpClient
        self.__uavsLoiter = {}
        self.__uavsgothint = {}
        self.__uavsisinfirezone = {}
        self.__estimatedHazardZone = Polygon()
        self.__keepInZone = Rectangle()
        self.__currentLocationofUAV = {}
        self.__currentHeadingAngleUAV = {}
        self.__pastHeadingAngleUAV = {}
        self.__searchAreaCenterLat = 0
        self.__searchAreaCenterLong = 0
        self.__searchAreaWidth = 0
        self.__searchAreaHeight = 0
        self.__firezonePoints = {}
        self.__firezoneHintLocation = Location3D()
        self.__centerLocation = Location3D()
        self.__gotHint = False
        self.__simulationTime = 0
        self.__isInMinorAxis = {}
        self.__turnlocation = {}
        self.__turntime = {}
        self.__counter = 0
        self.anchor = []
        self.__lastfireZonelocation = {}
        self.__option = [0,0,0,0]
        self.__keepoutOption = [0,0,0,0]
        self.__keepoutoptionChangetime = [0,0,0,0]
        self.__noOfUAVs = 4
        self.__sendReport = False
        self.__boundaryPoints = []
        self.__maxSpeedofUAV = {}
        self.__insideFireZoneLastTime = {}
        self.__timethreshold = [1.3,2.5]
        self.__lastLocation = Location3D()
        self.__flag = False
        
        self.__simulationTimeSeconds = 0
        self.__resulationOfGrid = 5000
        self.__minidel = 500
        self.__maxdel = 10000
        self.__allGridLocation = []
        self.__waypoints = {}
        self.__uavsInMission = {}
        self.__MissionReady = False
        self.__noOfZone = 4
        self.__zoneassigned = [False,False,False,False]
        self.__closesrWaypointID = {}
        self.__gotowaypointReady = [False,False,False,False]
        self.__zoneCenter = {}
        
        self.__altidata = pd.read_csv('altidata.csv',header=None)
        self.__altidata = self.__altidata.T
        self.__latBias = 39
        self.__lonBias = -122
        self.__delAlevation = 3600
        
        self.__safeHeight = 450
        self.__normalSearchAltitude = 450
        
        self.__initLocationOfUAVs = {}
        
        self.__maxAzimuthangle = {}
        self.__minAzimuthangle = {}

    def dataReceived(self, lmcpObject):
        #print(lmcpObject)
        if isinstance(lmcpObject,KeepInZone):
            #print(lmcpObject.toXMLStr(""))
            self.__keepInZone = lmcpObject.Boundary
            centerpoint = lmcpObject.Boundary.get_CenterPoint()
            self.__centerLocation = centerpoint
            self.__searchAreaCenterLat = centerpoint.get_Latitude()
            self.__searchAreaCenterLong = centerpoint.get_Longitude()
            self.__searchAreaWidth = lmcpObject.Boundary.get_Width()/2.0
            self.__searchAreaHeight = lmcpObject.Boundary.get_Height()/2.0
            self.calculateGridCoordinate()
            self.__MissionReady = True
            print('found keep in zone')
            # print(self.getAltitude(centerpoint))
            # print(self.__allGridLocation)
        elif isinstance(lmcpObject, AirVehicleState):
            airVehicleState = lmcpObject
            self.__simulationTime = airVehicleState.Time/(1000*60)
            self.__simulationTimeSeconds = airVehicleState.Time/1000
            veicleid = airVehicleState.ID
            veicleLocation = airVehicleState.Location
            
            if self.__simulationTimeSeconds == 0:
                self.__initLocationOfUAVs[veicleid] = veicleLocation
                
            if self.__simulationTimeSeconds > 3 and not veicleid in self.__uavsInMission:
                    self.__uavsInMission[veicleid] = False
                    self.sendGimbleCommand(veicleid,45)
                    print('gimble command send')
             
            if False:#self.__simulationTimeSeconds > 0:
                self.__currentLocationofUAV[veicleid] = veicleLocation
                # hazardSensorState = airVehicleState.PayloadStateList[2]
                # footprint = hazardSensorState.Footprint
                # center = hazardSensorState.Centerpoint     
                
                if self.__MissionReady:# and not veicleid in self.__uavsInMission:
                    self.__MissionReady = False
                    maxSpeed = max(self.__maxSpeedofUAV.values())
                    zi = 0
                    for id in self.__maxSpeedofUAV.keys():
                        if self.__maxSpeedofUAV[id] == maxSpeed:
                            self.sendMissionCommand(id,self.__initLocationOfUAVs[id])
                            self.__uavsInMission[id] = True
                            zi += 1
                        if zi == self.__noOfZone:
                            break
                    for zid in range(self.__noOfZone): 
                        endLoc = self.__zoneCenter[zid+1]
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
                                # self.sendLoiterCommand(id,self.__zoneCenter[zid+1],3000,self.__maxSpeedofUAV[id])
                                # break
                            
                            
                # if self.__gotHint and (self.__simulationTime > 18 and self.__simulationTime < 21) or (self.__simulationTime > 36 and self.__simulationTime < 40) or self.__simulationTime > 55:
                    # self.__sendReport = True
                
        elif isinstance(lmcpObject, AirVehicleConfiguration):
            airvehicleConfiguration = lmcpObject
            self.__maxSpeedofUAV[airvehicleConfiguration.ID] = airvehicleConfiguration.get_MaximumSpeed()
            payloadconfigList = airvehicleConfiguration.PayloadConfigurationList
            self.__maxAzimuthangle[airvehicleConfiguration.ID] = payloadconfigList[0].MaxAzimuth
            self.__minAzimuthangle[airvehicleConfiguration.ID] = payloadconfigList[0].MinAzimuth
            print("found AirVehicleConfiguration") 
        # elif isinstance(lmcpObject, SessionStatus):
            # #print("found session status")
            # #print(lmcpObject.toXMLStr(""))
            # pass
        # elif isinstance(lmcpObject, WeatherReport):
            # #print("found WeatherReport")
            # #print(lmcpObject.toXMLStr(""))
            # pass
        # elif isinstance(lmcpObject, HazardZone):
            # #print("found HazardZone")
            # pass
        # elif isinstance(lmcpObject, HazardZoneChangeCommand):
            # #print("found HazardZoneChangeCommand")
            # pass
        elif isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            detectedLocation = hazardDetected.get_DetectedLocation()
            detectingEntity = hazardDetected.get_DetectingEnitiyID()
            self.__firezoneHintLocation = detectedLocation
            self.__gotHint = True
            self.__lastfireZonelocation[detectingEntity] = detectedLocation
            self.__insideFireZoneLastTime[detectingEntity] = self.__simulationTimeSeconds
            if not detectingEntity in self.__uavsisinfirezone:
                self.__uavsisinfirezone[detectingEntity] = True
                # self.__currentHeadingAngleUAV[detectingEntity] = 3
                # self.turn(detectingEntity)
                
            if not self.__firezonePoints or not detectingEntity in self.__firezonePoints or not self.__firezonePoints[detectingEntity]:
                [x,y] = self.convertLatLonToxy(detectedLocation.get_Latitude(),detectedLocation.get_Longitude())
                self.__firezonePoints[detectingEntity] = [[x,y]]
            else:
                [x,y] = self.convertLatLonToxy(detectedLocation.get_Latitude(),detectedLocation.get_Longitude())
                self.__firezonePoints[detectingEntity].append([x,y])
    
    def sendMissionCommand(self,veicleid,veicleLocation):
        missionCommand = MissionCommand()
        missionCommand.set_VehicleID(veicleid)
        missionCommand.set_Status(CommandStatusType.Pending)
        missionCommand.set_CommandID(1)
        
        zid,locid = self.getNearestZone(veicleLocation,veicleid)##
        
        missionCommand.set_FirstWaypoint(locid)
        
        for waypoint in self.__waypoints[zid]:
            missionCommand.get_WaypointList().append(waypoint)
        self.__client.sendLMCPObject(missionCommand)
          
    def gotoWaypoint(self,veicleid):
        gotoWaypointAction = GoToWaypointAction()  
        vehicleActionCommand = VehicleActionCommand()
        flightDirectorAction = FlightDirectorAction();
        
        vehicleActionCommand.set_VehicleID(veicleid)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)
        gotoWaypointAction.set_WaypointNumber(self.__closesrWaypointID[veicleid])
        vehicleActionCommand.get_VehicleActionList().append(gotoWaypointAction)
        
        self.__client.sendLMCPObject(vehicleActionCommand) 
    
    def sendWaypoint(self,veicleid,initLocation,endLocation):
        missionCommand = MissionCommand()
        missionCommand.set_FirstWaypoint(1)
        missionCommand.set_VehicleID(veicleid)
        missionCommand.set_Status(CommandStatusType.Pending)
        missionCommand.set_CommandID(1)
        
        waypoints = self.getwaypointsBetweenLocations(initLocation,endLocation,veicleid)
         
        for waypoint in waypoints:
            missionCommand.get_WaypointList().append(waypoint)
        
        self.__client.sendLMCPObject(missionCommand)
    
    def sendHeadingAngleCommand(self,veicleid,headingangle):
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(veicleid)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)
        
        flightDirectorAction = FlightDirectorAction();
        flightDirectorAction.set_Speed(self.__maxSpeedofUAV[veicleid])
        flightDirectorAction.set_SpeedType(SpeedType.Airspeed)
        flightDirectorAction.set_Heading(headingangle)
        flightDirectorAction.set_Altitude(100)
        flightDirectorAction.set_AltitudeType(AltitudeType.MSL)
        flightDirectorAction.set_ClimbRate(0)
        
        vehicleActionCommand.get_VehicleActionList().append(flightDirectorAction)
        
        self.__client.sendLMCPObject(vehicleActionCommand)
    
    def sendGimbleCommand(self, veicleid, azimuthangle):
        #Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(veicleid)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)
         
        if azimuthangle >=  self.__minAzimuthangle[veicleid] and azimuthangle <=  self.__maxAzimuthangle[veicleid]:
            azimuthangle = azimuthangle
        elif azimuthangle <= self.__minAzimuthangle[veicleid]:
            azimuthangle = self.__minAzimuthangle[veicleid]
        else:
            azimuthangle = self.__maxAzimuthangle[veicleid]
        
        gimbleAngleAction = GimbalAngleAction()
        gimbleAngleAction.set_PayloadID(1)
        gimbleAngleAction.set_Azimuth(azimuthangle)
        gimbleAngleAction.set_Elevation(-45)
        gimbleAngleAction.set_Rotation(0)
        
        vehicleActionCommand.get_VehicleActionList().append(gimbleAngleAction)
        
        self.__client.sendLMCPObject(vehicleActionCommand)
        
    
    def sendLoiterCommand(self, veicleid, location, radius, speed):
        #Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(veicleid)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)

        #Setting up the loiter action
        loiterAction = LoiterAction()
        loiterAction.set_LoiterType(LoiterType.Circular)
        loiterAction.set_Radius(radius)
        loiterAction.set_Axis(0)
        loiterAction.set_Length(0)
        loiterAction.set_Direction(LoiterDirection.Clockwise)
        loiterAction.set_Duration(100000)
        loiterAction.set_Airspeed(speed)

        #Creating a 3D location object for the stare point
        loiterAction.set_Location(location)

        #Adding the loiter action to the vehicle action list
        vehicleActionCommand.get_VehicleActionList().append(loiterAction)

        #Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicleActionCommand)

    def sendEstimateReport(self):
        #Setting up the mission to send to the UAV
        hazardZoneEstimateReport = HazardZoneEstimateReport()
        hazardZoneEstimateReport.set_EstimatedZoneShape(self.__estimatedHazardZone)
        hazardZoneEstimateReport.set_UniqueTrackingID(1)
        hazardZoneEstimateReport.set_EstimatedGrowthRate(0)
        hazardZoneEstimateReport.set_PerceivedZoneType(HazardType.Fire)
        hazardZoneEstimateReport.set_EstimatedZoneDirection(0)
        hazardZoneEstimateReport.set_EstimatedZoneSpeed(0)

        #Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(hazardZoneEstimateReport)
        
    def isLeft(self,location,center):
        R = 111000
        a = location.get_Latitude() - center.get_Latitude()
        b = location.get_Longitude() - center.get_Longitude()
        # x = R*a
        # y = R*radians(lat)*b
        if a > 0 and b > 0:
            return True
        elif a < 0 and b > 0:
            return False
        elif a < 0 and b < 0:
            return True
        elif a > 0 and b < 0:
            return False
        
    def convertLatLonToxy(self,lat,long):
        R = 111000
        a = lat-self.__searchAreaCenterLat
        b = long-self.__searchAreaCenterLong
        x = R*a
        y = R*cos(radians(lat))*b
        # print(utm.from_latlon(lat, long))
        return [x,y]
        
    def convertxyToLatLon(self,x,y):
        R = 111000
        lat = x/R + self.__searchAreaCenterLat
        long = y/(R*cos(radians(lat))) + self.__searchAreaCenterLong
        return [lat,long]
     
    def calculateGridCoordinate(self):
        self.__zoneCenter = {}
        self.__allGridLocation = []
        self.__waypoints = {}
        w = self.__searchAreaWidth
        h = self.__searchAreaHeight
        A = (w*h*4)/self.__noOfZone
        a = sqrt(A)
        zone = int(sqrt(self.__noOfZone))
        minidel = self.__minidel
        maxdel = self.__maxdel
        rowseg = int(maxdel/minidel)
        row = int(round(a/maxdel))*2+1
        col = int(round(a/minidel))
         
        
        # print(row,col,a)
        
        delta = w/zone
        delxp = 0
        delxn = 0
        for z1 in range(zone):
            if z1%2==0:
                delxp += delta
                zx = delxp
            else:
                delxn -= delta
                zx = delxn
            delyp = 0
            delyn = 0
            for z2 in range(zone):
                if z2%2==0:
                    delyp += delta
                    zy = delyp
                else:
                    delyn -= delta
                    zy = delyn
                waypoints = []
                
                waypointNumber = 1
                
                x = 0
                y = 0
            
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
                waypoints.append(waypoint)
                waypointNumber += 1
            
                for i in range(row):
                    if i%2 == 0:
                        if z2 == 0:
                            if y == 0:
                                option = 1
                            else:
                                option = 2
                        else:
                            if y == 0:
                                option = 2
                            else:
                                option = 1
                            
                        for j in range(col):
                            if option == 1:
                                y += minidel
                            else:
                                y -= minidel
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
                            if i == row-1 and j == col-1:
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
                                
                    else:
                        for j in range(rowseg):
                            if z1 == 0:
                                x += minidel
                            else:
                                x -= minidel
                            [lat,lon] = self.convertxyToLatLon(x,y)
                            
                            waypoint = Waypoint()
                            waypoint.set_Latitude(lat)
                            waypoint.set_Longitude(lon)
                            alti = self.getAltitudeLatLon(lat,lon) + self.__safeHeight
                            if alti < self.__normalSearchAltitude:
                                waypoint.set_Altitude(self.__normalSearchAltitude)
                            else:
                                waypoint.set_Altitude(alti)
                            waypoint.set_AltitudeType(AltitudeType.MSL)
                            waypoint.set_Number(waypointNumber)
                            if i == row-1 and j == rowseg-1:
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
                            
                    # x = w-i*self.__resulationOfGrid - z1*(w)
                    # self.__allGridLocation.append([])
                    # for j in range(col):
                        # if i%2 == 0:
                            # y = h-j*self.__resulationOfGrid - z2*(h)
                        # else:
                            # y = j*self.__resulationOfGrid - z2*(h)
                        # # print(x,y)
                        # location = Location3D()
                        # [lat,lon] = self.convertxyToLatLon(x,y)
                        # location.set_Latitude(lat)
                        # location.set_Longitude(lon)
                        # location.set_Altitude(1000)
                        # self.__allGridLocation[i].append(location)
                        
                        
                        
                        
                self.__waypoints[z1*zone+z2+1] = waypoints
                zlocation = Location3D()
                [lat,lon] = self.convertxyToLatLon(zx,zy)
                zlocation.set_Latitude(lat)
                zlocation.set_Longitude(lon)
                zlocation.set_Altitude(450)
                self.__zoneCenter[z1*zone+z2+1] = zlocation
    
    def getNearestZone(self,location,vid):
        minima = 1e10
        zoneid = 0
        minLocid = 1
        minLoc = Location3D()
        for z in range(self.__noOfZone):
            if not self.__zoneassigned[z]:
                waypoints = self.__waypoints[z+1]
                for i in range(len(waypoints)):
                    loc = waypoints[i]
                    d = self.getdistance(loc,location)
                    if d < minima:
                        minima = d
                        zoneid = z+1
                        minLocid = i+1
                        minLoc = loc
        self.__zoneassigned[zoneid-1]=True
        if sqrt(minima) < 5000:
            return zoneid,minLocid
        waypoints,minLocid = self.getBetweenLatLon(location,minLoc,len(self.__waypoints[zoneid]),minima,minLocid,vid)
        self.__waypoints[zoneid] += waypoints
        return zoneid,minLocid
    
    def getwaypointsBetweenLocations(self,startLoc,endLoc,vid):
        d = self.getdistance(startLoc,endLoc)
        [xs,ys] = self.convertLatLonToxy(startLoc.get_Latitude(),startLoc.get_Longitude())
        [xe,ye] = self.convertLatLonToxy(endLoc.get_Latitude(),endLoc.get_Longitude())
        delx = xe-xs
        dely = ye-ys
        m = dely/delx
        ii = int(round(sqrt(d)/self.__minidel))
        delx /= ii
        ii = ii - 2
        x = xs
        waypointNumber = 1
        waypoints = []
        x += 2*delx
        for i in range(ii):
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
            # if i == ii-1:
                # waypoint.set_NextWaypoint(waypointNumber)
            # else:
            waypoint.set_NextWaypoint(waypointNumber+1)
            waypoint.set_Speed(self.__maxSpeedofUAV[vid])
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        
        y += 2*delx
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
        waypoint.set_Speed(self.__maxSpeedofUAV[vid])
        waypoint.set_SpeedType(SpeedType.Airspeed)
        waypoint.set_ClimbRate(15)
        waypoint.set_TurnType(TurnType.TurnShort)
        waypoint.set_ContingencyWaypointA(0)
        waypoint.set_ContingencyWaypointB(0)
        waypoints.append(waypoint)
        waypointNumber += 1
        
        y -= 4*delx
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
        waypoint.set_NextWaypoint(waypointNumber-2)
        waypoint.set_Speed(self.__maxSpeedofUAV[vid])
        waypoint.set_SpeedType(SpeedType.Airspeed)
        waypoint.set_ClimbRate(15)
        waypoint.set_TurnType(TurnType.TurnShort)
        waypoint.set_ContingencyWaypointA(0)
        waypoint.set_ContingencyWaypointB(0)
        waypoints.append(waypoint)
    
        return waypoints
    
    def getBetweenLatLon(self,startLoc,endLoc,startwaypointId,d,connectingwaypointId,vid):
        [xs,ys] = self.convertLatLonToxy(startLoc.get_Latitude(),startLoc.get_Longitude())
        [xe,ye] = self.convertLatLonToxy(endLoc.get_Latitude(),endLoc.get_Longitude())
        delx = xe-xs
        dely = ye-ys
        m = dely/delx
        ii = int(round(sqrt(d)/1000))
        delx /= ii
        ii = ii - 3
        x = xs
        waypointNumber = startwaypointId+1
        waypoints = []
        x += 3*delx
        for i in range(ii):
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
            if i == ii-1:
                waypoint.set_NextWaypoint(connectingwaypointId)
            else:
                waypoint.set_NextWaypoint(waypointNumber+1)
            waypoint.set_Speed(self.__maxSpeedofUAV[vid])
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(0)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        waypointNumber -= 1
        return waypoints, startwaypointId+1
    
    def getAltitudeLatLon(self,lat,lon):
        row = int(round((lat - self.__latBias)*self.__delAlevation))
        col = int(round((lon - self.__lonBias)*self.__delAlevation))
        sz = self.__altidata.shape
        if row >= sz[0]:
            row = sz[0]-1
        if col >= sz[1]:
            col = sz[1]-1
        # print(self.__altidata[row][col])
        return self.__altidata[row][col]
    
    def getAltitude(self,location):
        row = int(round((location.get_Latitude() - self.__latBias)*self.__delAlevation))
        col = int(round((location.get_Longitude() - self.__lonBias)*self.__delAlevation))
        sz = self.__altidata.shape
        # print(sz)
        if row >= sz[0]:
            row = sz[0]-1
        if col >= sz[1]:
            col = sz[1]-1
        # print(self.__altidata[row][col])
        return self.__altidata[row][col]
    
    def getNextLoiterCenter(self,veicleid,refLocation):
        if veicleid == 2:
            [xr,yr] = self.convertLatLonToxy(refLocation.get_Latitude(),refLocation.get_Longitude())
            x = xr + (self.__searchAreaWidth-abs(xr))/2
            [lat,lon] = self.convertxyToLatLon(x,yr)
            location = Location3D()
            location.set_Latitude(lat)
            location.set_Longitude(lon)
            location.set_Altitude(refLocation.get_Altitude())
            return location
        elif veicleid == 3:
            [xr,yr] = self.convertLatLonToxy(refLocation.get_Latitude(),refLocation.get_Longitude())
            y = yr - (self.__searchAreaHeight-abs(yr))/2
            [lat,lon] = self.convertxyToLatLon(xr,y)
            location = Location3D()
            location.set_Latitude(lat)
            location.set_Longitude(lon)
            location.set_Altitude(refLocation.get_Altitude())
            # print(lat,lon,'  x,y ',xr,y)
            return location
        if veicleid == 4:
            [xr,yr] = self.convertLatLonToxy(refLocation.get_Latitude(),refLocation.get_Longitude())
            y = yr + (self.__searchAreaHeight-abs(yr))/2
            [lat,lon] = self.convertxyToLatLon(xr,y)
            location = Location3D()
            location.set_Latitude(lat)
            location.set_Longitude(lon)
            location.set_Altitude(refLocation.get_Altitude())
            # print(lat,lon,'  x,y ',xr,y)
            return location
        return refLocation
    
    def isinKeepInZone(self,location):
        xyposition = self.convertLatLonToxy(location.get_Latitude(),location.get_Longitude())
        # print(xyposition[0],' width: ', self.__searchAreaWidth, xyposition[1], ' height: ',self.__searchAreaHeight)
        if abs(xyposition[0]) <= self.__searchAreaWidth and abs(xyposition[1]) <= self.__searchAreaHeight:
            return True
        return False

    def isLeavingFireZone(self,veicleid,location):
        if self.__lastfireZonelocation:
            if veicleid in self.__lastfireZonelocation:
                # print(self.__firezonePoints)
                lastFireLocation =  self.__lastfireZonelocation[veicleid]
                lastFireLocationXY = self.convertLatLonToxy(lastFireLocation.get_Latitude(),lastFireLocation.get_Longitude())
                locationXY = self.convertLatLonToxy(location.get_Latitude(),location.get_Longitude())
                d = (locationXY[0] - lastFireLocationXY[0])**2 + (locationXY[1] - lastFireLocationXY[1])**2
                if d > 250000:
                    return True
            
        return False

    def getdistance(self,loc1,loc2):
        loc1XY = self.convertLatLonToxy(loc1.get_Latitude(),loc1.get_Longitude())
        loc2XY = self.convertLatLonToxy(loc2.get_Latitude(),loc2.get_Longitude())
        d = (loc1XY[0] - loc2XY[0])**2 + (loc1XY[1] - loc2XY[1])**2
        # print(d)
        return d
    
    def turn(self,veicleid,left):
        delang = 15
        if left == 0:
            if self.__currentHeadingAngleUAV[veicleid] < delang:
                self.__currentHeadingAngleUAV[veicleid] = 360 - delang
            else:
                self.__currentHeadingAngleUAV[veicleid] -= delang
        elif left == 1:
            self.__currentHeadingAngleUAV[veicleid] = (self.__currentHeadingAngleUAV[veicleid]+delang)%360
        self.sendHeadingAngleCommand(veicleid,self.__currentHeadingAngleUAV[veicleid])
        
    def turn(self,veicleid):
        headingAngle = 0
        if veicleid == 1:
            if self.__currentHeadingAngleUAV[veicleid] == 0:
                self.__currentHeadingAngleUAV[veicleid] = 1
                if self.__keepoutOption[veicleid-1] == 0:
                    headingAngle = 90
                else:
                    headingAngle = 270
            elif self.__currentHeadingAngleUAV[veicleid] == 1:
                self.__currentHeadingAngleUAV[veicleid] = 2
                headingAngle = 180
            elif self.__currentHeadingAngleUAV[veicleid] == 2:
                self.__currentHeadingAngleUAV[veicleid] = 3
                if self.__keepoutOption[veicleid-1] == 0:
                    headingAngle = 90
                else:
                    headingAngle = 270
            elif self.__currentHeadingAngleUAV[veicleid] == 3:
                self.__currentHeadingAngleUAV[veicleid] = 0
                headingAngle = 0
        elif veicleid  == 2 :
            if self.__currentHeadingAngleUAV[veicleid] == 0:
                self.__currentHeadingAngleUAV[veicleid] = 1
                if self.__keepoutOption[veicleid-1] == 0:
                    headingAngle = 270
                else:
                    headingAngle = 90
            elif self.__currentHeadingAngleUAV[veicleid] == 1:
                self.__currentHeadingAngleUAV[veicleid] = 2
                headingAngle = 180
            elif self.__currentHeadingAngleUAV[veicleid] == 2:
                self.__currentHeadingAngleUAV[veicleid] = 3
                if self.__keepoutOption[veicleid-1] == 0:
                    headingAngle = 270
                else:
                    headingAngle = 90
            elif self.__currentHeadingAngleUAV[veicleid] == 3:
                self.__currentHeadingAngleUAV[veicleid] = 0
                headingAngle = 0
        elif veicleid == 3:
            if self.__currentHeadingAngleUAV[veicleid] == 0:
                self.__currentHeadingAngleUAV[veicleid] = 1
                if self.__keepoutOption[veicleid-1] == 0:
                    headingAngle = 180
                else:
                    headingAngle = 0
            elif self.__currentHeadingAngleUAV[veicleid] == 1:
                self.__currentHeadingAngleUAV[veicleid] = 2
                headingAngle = 270
            elif self.__currentHeadingAngleUAV[veicleid] == 2:
                self.__currentHeadingAngleUAV[veicleid] = 3
                if self.__keepoutOption[veicleid-1] == 0:
                    headingAngle = 180
                else:
                    headingAngle = 0
            elif self.__currentHeadingAngleUAV[veicleid] == 3:
                self.__currentHeadingAngleUAV[veicleid] = 0
                headingAngle = 90
        elif veicleid == 4:
            if self.__currentHeadingAngleUAV[veicleid] == 0:
                self.__currentHeadingAngleUAV[veicleid] = 1
                if self.__keepoutOption[veicleid-1] == 0:
                    headingAngle = 0
                else:
                    headingAngle = 180
            elif self.__currentHeadingAngleUAV[veicleid] == 1:
                self.__currentHeadingAngleUAV[veicleid] = 2
                headingAngle = 270
            elif self.__currentHeadingAngleUAV[veicleid] == 2:
                self.__currentHeadingAngleUAV[veicleid] = 3
                if self.__keepoutOption[veicleid-1] == 0:
                    headingAngle = 0
                else:
                    headingAngle = 180
            elif self.__currentHeadingAngleUAV[veicleid] == 3:
                self.__currentHeadingAngleUAV[veicleid] = 0
                headingAngle = 90
        

        print('turning',veicleid,' heading', headingAngle)
        self.sendHeadingAngleCommand(veicleid,headingAngle)
            
    def graham_scan(self,points): 
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

    def findBoundaryandSendReport(self):
        allxypoints = self.__boundaryPoints
        if self.__firezonePoints:
            for key in self.__firezonePoints.keys():
                points = self.__firezonePoints[key]
                self.__firezonePoints[key] = []
                # print(points)
                allxypoints += points
                    
        # print(len(allxypoints))
        boundarypoints = self.graham_scan(allxypoints)
        self.__boundaryPoints = boundarypoints
        # print(len(boundarypoints))
        # print(boundarypoints)
        for xypoint in boundarypoints:
            [lat,lon]=self.convertxyToLatLon(xypoint[0],xypoint[1])
            locationpoint = Location3D()
            locationpoint.set_Latitude(lat)
            locationpoint.set_Longitude(lon)
            self.__estimatedHazardZone.get_BoundaryPoints().append(locationpoint)  
        self.sendEstimateReport()
        self.__estimatedHazardZone = Polygon()

    def getSendReportStatus(self):
        return self.__sendReport
    def setSendReportStatus(self,status):
        self.__sendReport = status

     
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

        while True:
            #wait for keyboard interrupt
            if smpleHazardDetector.getSendReportStatus():
                smpleHazardDetector.findBoundaryandSendReport()
                smpleHazardDetector.setSendReportStatus(False)
            # pass
    except KeyboardInterrupt as ki:
        print("Stopping amase tcp client")
    except Exception as ex:
        print(ex)
    amaseClient.stop()
