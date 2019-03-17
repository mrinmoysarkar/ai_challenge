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
from afrl.cmasi.searchai.HazardType import HazardType
from afrl.cmasi.NavigationMode import NavigationMode
# import utm

filePath = '../../altitude_data/'

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
        self.__firezoneHintLocation = {}
        self.__centerLocation = Location3D()
        self.__gotHint = False
        self.__simulationTime = 0
        self.__isInMinorAxis = {}
        self.__turnlocation = {}
        self.__turntime = {}
        self.__counter = [0,0,0,0,0,0,0,0]
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
        self.__zoneboundaryPoints = {}
        
        self.altidata1 = pd.read_csv(filePath+'altidata1.csv',header=None)
        self.altidata1 = self.altidata1.T
        self.altidata2 = pd.read_csv(filePath+'altidata2.csv',header=None)
        self.altidata2 = self.altidata2.T
        self.altidata3 = pd.read_csv(filePath+'altidata3.csv',header=None)
        self.altidata3 = self.altidata3.T
        self.altidata4 = pd.read_csv(filePath+'altidata4.csv',header=None)
        self.altidata4 = self.altidata4.T

        self.__latBias = 39
        self.__lonBias = -122
        self.__delAlevation = 3600
        
        self.__safeHeight = 450
        self.__surveySafeHeight = 300
        self.__normalSearchAltitude = 450
        
        self.__initLocationOfUAVs = {}
        
        self.__maxAzimuthangle = {}
        self.__minAzimuthangle = {}
        self.__uavsInSearch = {}
        self.__uavsInSarvey = {}
        self.__uavisHeadingtoSurveylocatio = {}
        self.__uavisInsmokeZone = {}
        self.__UAVSurvayingZoneId = {}
        
        self.__changedirection = {}
        
        self.__lasttime = [0,0,0,0,0,0,0,0]
        self.__lasttime1 = [0,0,0,0,0,0,0,0]
        self.__previousreportsendTime = 0
        self.__previousWeatherReportTime = 0
        self.__desiredheading = [0,0,0,0,0,0,0,0]
        self.__LastheadingAngleSendtime = [0,0,0,0,0,0,0,0]
        
        self.__veicleStrategiId = [0,0,0,0,0,0,0,0]
        self.__NoofUAVinZone = [0,0,0,0]
        
        self.__wspeed = 0
        self.__ditectionTheta = 0
        
        self.__changeHangle = [True,True,True,True,True,True,True,True]
        
        self.__totalWaypointsassignedToUAV = {}
        self.__previouswaypointNo = {}
        self.__visitedTotalwaypoints = [0,0,0,0,0,0,0,0,0]
        self.__updateArea = False
        
        ####experimental
        self.__NewSTGt1 = [0,0,0,0,0,0,0,0]
        self.__NewSTGt2 = [0,0,0,0,0,0,0,0]
        self.__NewSTGt3 = [0,0,0,0,0,0,0,0]
        self.__NewSTGt4 = [0,0,0,0,0,0,0,0]
        self.__NewSTGoption = [0,0,0,0,0,0,0,0]
        self.__NewSTGdt = [2,2,2,2,2,2,2,2]
        self.__NewSTGleft = [0,0,0,0,0,0,0,0]
        self.__NewSTGforward = [0,0,0,0,0,0,0,0]
        self.__NewSTGdtaction = [5,5,5,5,5,5,5,5]
        self.__NewSTGheadingangle = [0,0,0,0,0,0,0,0]
        self.__NewSTGfirst = [0,0,0,0,0,0,0,0]
        self.__NewSTGrefHeading = [0,0,0,0,0,0,0,0]
        
    def dataReceived(self, lmcpObject):
        
        if isinstance(lmcpObject,KeepInZone):
            #print(lmcpObject.toXMLStr(""))
            self.__keepInZone = lmcpObject.Boundary
            centerpoint = lmcpObject.Boundary.get_CenterPoint()
            self.__centerLocation = centerpoint
            self.__searchAreaCenterLat = centerpoint.get_Latitude()
            self.__searchAreaCenterLong = centerpoint.get_Longitude()
            self.__searchAreaWidth = lmcpObject.Boundary.get_Width()/2.0
            self.__searchAreaHeight = lmcpObject.Boundary.get_Height()/2.0
            # self.calculateGridCoordinate()
            # self.calculateGridCoordinateAlt()
            self.calculateGridCoordinateAlt1()
            self.__MissionReady = True
            print('found keep in zone')
            
        elif isinstance(lmcpObject, AirVehicleState):
            airVehicleState = lmcpObject
            self.__simulationTime = airVehicleState.Time/(1000*60)
            self.__simulationTimeSeconds = airVehicleState.Time/1000
            veicleid = airVehicleState.ID
            veicleLocation = airVehicleState.Location
            
            if self.__simulationTimeSeconds == 0:
                self.__initLocationOfUAVs[veicleid] = veicleLocation
             
            if self.__simulationTimeSeconds > 0:
                self.__currentLocationofUAV[veicleid] = veicleLocation
                self.__currentHeadingAngleUAV[veicleid] = airVehicleState.Heading
                     
                if self.__MissionReady:
                    self.sendinitialMission()
                    
                # if airVehicleState.Mode == NavigationMode.Waypoint:
                    # currentwaypointNo = airVehicleState.CurrentWaypoint
                    # if currentwaypointNo != self.__previouswaypointNo[veicleid]:
                        # self.__previouswaypointNo[veicleid] = currentwaypointNo
                        # self.__visitedTotalwaypoints[veicleid-1] += 1
                        # if self.__visitedTotalwaypoints[veicleid-1] >= self.__totalWaypointsassignedToUAV[veicleid]:
                            # print('uav ',veicleid, ' finished its mission')
                
                self.checkSurveyStatus(airVehicleState)
                
                if self.__gotHint and (self.__simulationTime - self.__previousreportsendTime) > 0.3:# and ((self.__simulationTime > 28 and self.__simulationTime < 31) or (self.__simulationTime > 38 and self.__simulationTime < 41) or self.__simulationTime > 58):# or (self.__simulationTime > 55 and (self.__simulationTime - self.__previousreportsendTime) > 1)):
                    self.__previousreportsendTime = self.__simulationTime
                    self.__sendReport = True
                
                # if (self.__simulationTimeSeconds - self.__previousWeatherReportTime) > 2 and airVehicleState.WindSpeed > 0:
                    # self.__previousWeatherReportTime = self.__simulationTimeSeconds
                    # self.__wspeed += airVehicleState.WindSpeed
                    # self.__ditectionTheta = airVehicleState.WindDirection
                    # self.__updateArea = True
                # elif airVehicleState.WindSpeed == 0:
                    # self.__wspeed = 0
                    # self.__ditectionTheta = 0
                    
        elif isinstance(lmcpObject, AirVehicleConfiguration):
            airvehicleConfiguration = lmcpObject
            self.__maxSpeedofUAV[airvehicleConfiguration.ID] = airvehicleConfiguration.get_MaximumSpeed()
            payloadconfigList = airvehicleConfiguration.PayloadConfigurationList
            self.__maxAzimuthangle[airvehicleConfiguration.ID] = payloadconfigList[0].MaxAzimuth
            self.__minAzimuthangle[airvehicleConfiguration.ID] = payloadconfigList[0].MinAzimuth
            print("found AirVehicleConfiguration") 
            
        elif isinstance(lmcpObject, WeatherReport):
            print("found WeatherReport")
            print(lmcpObject.toXMLStr(""))
            wreport = lmcpObject
            wspeed = wreport.WindSpeed
            ditectionTheta = wreport.WindDirection
 
        elif isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            detectedLocation = hazardDetected.get_DetectedLocation()
            detectingEntity = hazardDetected.get_DetectingEnitiyID()
            vid = detectingEntity
            fireZoneType = hazardDetected.get_DetectedHazardZoneType()
            #self.__uavsInSarvey[detectingEntity] = True
            self.__maxSpeedofUAV[detectingEntity] = 18 ## play here
            
            if fireZoneType == HazardType.Fire:
                self.__uavsInSarvey[detectingEntity] = True
                if self.__NewSTGoption[vid-1] == 1:
                    print('found forward',vid)
                    self.__NewSTGforward[vid-1] = 1
                elif self.__NewSTGoption[vid-1] == 2:
                    print('found left',vid)
                    self.__NewSTGleft[vid-1] = 1
                    
                    
                if self.__changeHangle[vid-1]:
                    self.__changeHangle[vid-1] = False
                    self.sendGimbleCommand(vid,0,-45)
                # print('change direction issued')
                self.__changedirection[detectingEntity] = True
                self.__gotHint = True
                self.__lastfireZonelocation[detectingEntity] = detectedLocation
                self.__insideFireZoneLastTime[detectingEntity] = self.__simulationTimeSeconds
                if not detectingEntity in self.__uavsisinfirezone:
                    self.__uavsisinfirezone[detectingEntity] = True
                [x,y] = self.convertLatLonToxy(detectedLocation.get_Latitude(),detectedLocation.get_Longitude())
                zid = self.getZoneId([x,y])
                self.__firezoneHintLocation[zid] = detectedLocation
                self.__UAVSurvayingZoneId[vid] = zid
                if not self.__firezonePoints or not zid in self.__firezonePoints:
                    self.__firezonePoints[zid] = [[x,y]]
                else:
                    self.__firezonePoints[zid].append([x,y])
            elif fireZoneType == HazardType.Smoke:
                #print('smoke detected')
                # self.__uavisInsmokeZone[vid] = True
                # self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__currentHeadingAngleUAV[vid],self.__currentLocationofUAV[vid])
                pass
    
    def sendMissionCommand(self,veicleid,veicleLocation):
        missionCommand = MissionCommand()
        missionCommand.set_VehicleID(veicleid)
        missionCommand.set_Status(CommandStatusType.Pending)
        missionCommand.set_CommandID(1)
        self.__visitedTotalwaypoints[veicleid-1] = 0
        zid,locid = self.getNearestZone(veicleLocation,veicleid)##
        
        missionCommand.set_FirstWaypoint(locid)
        self.__previouswaypointNo[veicleid] = locid
        i = 0
        for waypoint in self.__waypoints[zid]:
            i += 1
            waypoint.set_Speed(self.__maxSpeedofUAV[veicleid])
            missionCommand.get_WaypointList().append(waypoint)
        self.__totalWaypointsassignedToUAV[veicleid] = i
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
        
        self.__visitedTotalwaypoints[veicleid-1] = 0
        self.__previouswaypointNo[veicleid] = 1
        waypoints = self.getwaypointsBetweenLocations(initLocation,endLocation,veicleid)
        i = 0
        for waypoint in waypoints:
            i += 1
            waypoint.set_Speed(self.__maxSpeedofUAV[veicleid])
            missionCommand.get_WaypointList().append(waypoint)
        self.__totalWaypointsassignedToUAV[veicleid] = i
        self.__client.sendLMCPObject(missionCommand)
    
    def sendHeadingAngleCommandwithcurrentlocation(self,veicleid,headingangle,currentlocation):
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(veicleid)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)
        
        flightDirectorAction = FlightDirectorAction();
        flightDirectorAction.set_Speed(self.__maxSpeedofUAV[veicleid])
        flightDirectorAction.set_SpeedType(SpeedType.Airspeed)
        flightDirectorAction.set_Heading(headingangle)
        flightDirectorAction.set_Altitude(self.getAltitude(currentlocation)+self.__surveySafeHeight)
        flightDirectorAction.set_AltitudeType(AltitudeType.MSL)
        flightDirectorAction.set_ClimbRate(0)
        
        vehicleActionCommand.get_VehicleActionList().append(flightDirectorAction)
        
        self.__client.sendLMCPObject(vehicleActionCommand)
    
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
    
    def sendGimbleCommand(self, veicleid, azimuthangle,elevationangle):
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
        gimbleAngleAction.set_Elevation(elevationangle)
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
        self.__MissionReady = False
        maxSpeed = max(self.__maxSpeedofUAV.values())
        zi = 0
        for id in self.__maxSpeedofUAV.keys():
            if self.__maxSpeedofUAV[id] == maxSpeed:
                self.sendGimbleCommand(id,0,-75)
                self.sendMissionCommand(id,self.__initLocationOfUAVs[id])
                self.__uavsInMission[id] = True
                self.__uavsInSearch[id] = True
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
    
    def checkSurveyStatus(self,vstate):
        airVehicleState = vstate
        veicleid = vstate.ID
        veicleLocation = vstate.Location
        if veicleid in self.__uavsInSarvey and self.__uavsInSarvey[veicleid]:
            #self.surveyStrategy(veicleid,airVehicleState,veicleLocation)
            self.surveyNewStrategy(airVehicleState)
        elif not veicleid in self.__uavisHeadingtoSurveylocatio:
            zid = self.getZoneIdLocation(veicleLocation)
            if self.__firezoneHintLocation:
                if zid in self.__firezoneHintLocation:
                    self.sendGimbleCommand(veicleid,0,-45)
                    self.sendWaypoint(veicleid,veicleLocation,self.__firezoneHintLocation[zid])
                    self.__uavisHeadingtoSurveylocatio[veicleid] = True
                    self.__UAVSurvayingZoneId[veicleid] = zid
                    
                    self.__NoofUAVinZone[zid-1] += 1
                    if self.__NoofUAVinZone[zid-1]%2 != 0:
                        self.__veicleStrategiId[veicleid-1] = 1
                else:
                    for zid in self.__firezoneHintLocation.keys():
                        for i in range(3):
                            if list(self.__UAVSurvayingZoneId.values()).count(zid) >= 3:
                                continue
                            minLoc = Location3D()
                            mind = 10e20
                            minvid = 10e20
                            for vid in self.__currentLocationofUAV.keys():
                                if not vid in self.__uavsInSearch and not vid in self.__uavisHeadingtoSurveylocatio and not vid in self.__uavsInSarvey:
                                    loc = self.__firezoneHintLocation[zid]
                                    d = self.getdistance(loc,self.__currentLocationofUAV[vid])
                                    if d < mind:
                                        mind = d
                                        minLoc = loc
                                        minvid = vid
                            if mind != 10e20:
                                self.sendGimbleCommand(minvid,0,-45)
                                self.sendWaypoint(minvid,self.__currentLocationofUAV[minvid],minLoc)
                                self.__uavisHeadingtoSurveylocatio[minvid] = True
                                self.__UAVSurvayingZoneId[minvid] = zid
                                 
                                self.__NoofUAVinZone[zid-1] += 1
                                if self.__NoofUAVinZone[zid-1]%2 != 0:
                                    self.__veicleStrategiId[minvid-1] = 1
    
    def surveyNewStrategy(self,airVehicleState):
        vid = airVehicleState.ID
        currentlocation = airVehicleState.Location
        leftstg = self.__veicleStrategiId[vid-1]
        print("survey new strategy", vid)
        if leftstg == 0:#left
            if self.__NewSTGfirst[vid-1] == 0:
                self.__NewSTGfirst[vid-1] = 1
                self.__NewSTGrefHeading[vid-1] = airVehicleState.Heading
                self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading + 90)%360
                self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
            else:
                headingangleError = abs(self.__NewSTGheadingangle[vid-1]-airVehicleState.Heading)
                headingangleError = headingangleError if headingangleError < 180 else (360-headingangleError)
                print("survey new strategy left", vid, headingangleError)
                if headingangleError < 15:
                    if self.__NewSTGoption[vid-1] == 0 and (self.__simulationTimeSeconds-self.__NewSTGt3[vid-1]) > self.__NewSTGdt[vid-1]:
                        print('#look forward',vid)
                        self.__NewSTGforward[vid-1] = 0
                        self.sendGimbleCommand(vid,0,-45)
                        self.__NewSTGoption[vid-1] = 1
                        self.__NewSTGt1[vid-1] = self.__simulationTimeSeconds
                    elif self.__NewSTGoption[vid-1] == 1 and (self.__simulationTimeSeconds-self.__NewSTGt1[vid-1]) > self.__NewSTGdt[vid-1]:
                        print('#look left',vid)
                        if self.__NewSTGforward[vid-1] == 0:
                            self.__NewSTGforward[vid-1] = 2
                        self.__NewSTGleft[vid-1] = 0
                        self.sendGimbleCommand(vid,-90,-45)
                        self.__NewSTGoption[vid-1] = 2
                        self.__NewSTGt2[vid-1] = self.__simulationTimeSeconds
                    elif self.__NewSTGoption[vid-1] == 2 and (self.__simulationTimeSeconds-self.__NewSTGt2[vid-1]) > self.__NewSTGdt[vid-1]:
                        print('# take action',vid)
                        if self.__NewSTGleft[vid-1] == 0:
                            self.__NewSTGleft[vid-1] = 2
                        self.sendGimbleCommand(vid,0,-45)
                        self.__NewSTGoption[vid-1] = 0
                        self.__NewSTGt3[vid-1] = self.__simulationTimeSeconds
                        
                        if self.__NewSTGleft[vid-1] == 1 and self.__NewSTGforward[vid-1] == 1:
                            print('#take right',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading + 90)%360
                            self.__NewSTGdtaction[vid-1] = 7
                            self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
                          
                        elif self.__NewSTGleft[vid-1] == 1 and self.__NewSTGforward[vid-1] == 2:
                            print('#go straignt',vid)
                            self.__NewSTGheadingangle[vid-1] = airVehicleState.Heading
                            self.__NewSTGdtaction[vid-1] = 7
                            
                        elif self.__NewSTGleft[vid-1] == 2 and self.__NewSTGforward[vid-1] == 1:
                            print('#take hard right',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading + 135)%360
                            self.__NewSTGdtaction[vid-1] = 7
                            self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
                               
                        elif self.__NewSTGleft[vid-1] == 2 and self.__NewSTGforward[vid-1] == 2:
                            print('# take hard left',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading - 30)
                            self.__NewSTGheadingangle[vid-1] = self.__NewSTGheadingangle[vid-1] if self.__NewSTGheadingangle[vid-1] >= 0 else (self.__NewSTGheadingangle[vid-1] + 360)
                            self.__NewSTGdtaction[vid-1] = 10
                            self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
                        
                if (self.__simulationTimeSeconds - self.__NewSTGt3[vid-1]) > self.__NewSTGdtaction[vid-1]:
                    self.__NewSTGt3[vid-1] = self.__simulationTimeSeconds
                    self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
        elif leftstg == 1: #right
            if self.__NewSTGfirst[vid-1] == 0:
                self.__NewSTGfirst[vid-1] = 1
                self.__NewSTGrefHeading[vid-1] = airVehicleState.Heading
                self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading - 90)
                self.__NewSTGheadingangle[vid-1] = self.__NewSTGheadingangle[vid-1] if self.__NewSTGheadingangle[vid-1] >= 0 else (self.__NewSTGheadingangle[vid-1] + 360)

                self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
            else:
                headingangleError = abs(self.__NewSTGheadingangle[vid-1]-airVehicleState.Heading)
                headingangleError = headingangleError if headingangleError < 180 else (360-headingangleError)
                print("survey new strategy right", vid, headingangleError)
                if headingangleError < 15:
                    if self.__NewSTGoption[vid-1] == 0 and (self.__simulationTimeSeconds-self.__NewSTGt3[vid-1]) > self.__NewSTGdt[vid-1]:
                        print('#look forward',vid)
                        self.__NewSTGforward[vid-1] = 0
                        self.sendGimbleCommand(vid,0,-45)
                        self.__NewSTGoption[vid-1] = 1
                        self.__NewSTGt1[vid-1] = self.__simulationTimeSeconds
                    elif self.__NewSTGoption[vid-1] == 1 and (self.__simulationTimeSeconds-self.__NewSTGt1[vid-1]) > self.__NewSTGdt[vid-1]:
                        print('#look right',vid)
                        if self.__NewSTGforward[vid-1] == 0:
                            self.__NewSTGforward[vid-1] = 2
                        self.__NewSTGleft[vid-1] = 0
                        self.sendGimbleCommand(vid,90,-45)
                        self.__NewSTGoption[vid-1] = 2
                        self.__NewSTGt2[vid-1] = self.__simulationTimeSeconds
                    elif self.__NewSTGoption[vid-1] == 2 and (self.__simulationTimeSeconds-self.__NewSTGt2[vid-1]) > self.__NewSTGdt[vid-1]:
                        print('# take action',vid)
                        if self.__NewSTGleft[vid-1] == 0:
                            self.__NewSTGleft[vid-1] = 2
                        self.sendGimbleCommand(vid,0,-45)
                        self.__NewSTGoption[vid-1] = 0
                        self.__NewSTGt3[vid-1] = self.__simulationTimeSeconds
                        
                        if self.__NewSTGleft[vid-1] == 1 and self.__NewSTGforward[vid-1] == 1:
                            print('#take left',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading - 90)
                            self.__NewSTGheadingangle[vid-1] = self.__NewSTGheadingangle[vid-1] if self.__NewSTGheadingangle[vid-1] >= 0 else (self.__NewSTGheadingangle[vid-1] + 360)

                            self.__NewSTGdtaction[vid-1] = 7
                            self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
                          
                        elif self.__NewSTGleft[vid-1] == 1 and self.__NewSTGforward[vid-1] == 2:
                            print('#go straignt',vid)
                            self.__NewSTGheadingangle[vid-1] = airVehicleState.Heading
                            self.__NewSTGdtaction[vid-1] = 7
                            
                        elif self.__NewSTGleft[vid-1] == 2 and self.__NewSTGforward[vid-1] == 1:
                            print('#take hard left',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading - 135)
                            self.__NewSTGheadingangle[vid-1] = self.__NewSTGheadingangle[vid-1] if self.__NewSTGheadingangle[vid-1] >= 0 else (self.__NewSTGheadingangle[vid-1] + 360)

                            self.__NewSTGdtaction[vid-1] = 7
                            self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
                               
                        elif self.__NewSTGleft[vid-1] == 2 and self.__NewSTGforward[vid-1] == 2:
                            print('# take hard right',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading + 30)%360
                            self.__NewSTGdtaction[vid-1] = 10
                            self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
                        
                if (self.__simulationTimeSeconds - self.__NewSTGt3[vid-1]) > self.__NewSTGdtaction[vid-1]:
                    self.__NewSTGt3[vid-1] = self.__simulationTimeSeconds
                    self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__NewSTGheadingangle[vid-1],currentlocation)
        
    def surveyStrategy(self,veicleid,airVehicleState,veicleLocation): # need works
        if self.__veicleStrategiId[veicleid-1] == 0:
            # # Right direction strategy
            if veicleid in self.__changedirection and self.__changedirection[veicleid] and (self.__simulationTimeSeconds - self.__lasttime[veicleid-1])>20: 
                self.__desiredheading[veicleid-1] = airVehicleState.Heading
                if self.__counter[veicleid-1] == 0:
                    self.__lasttime[veicleid-1] = self.__simulationTimeSeconds
                    self.__lasttime1[veicleid-1] = self.__simulationTimeSeconds
                    self.__desiredheading[veicleid-1] = (airVehicleState.Heading + 145)%360
                    self.__counter[veicleid-1] = 1
                elif self.__counter[veicleid-1] == 1:
                    self.__desiredheading[veicleid-1] = (airVehicleState.Heading - 145)
                    self.__desiredheading[veicleid-1] = self.__desiredheading[veicleid-1] if self.__desiredheading[veicleid-1] > 0 else self.__desiredheading[veicleid-1]+360
                    self.__changedirection[veicleid] = False
                    self.__counter[veicleid-1] = 0 
                self.sendHeadingAngleCommandwithcurrentlocation(veicleid,self.__desiredheading[veicleid-1],veicleLocation)
                
            elif veicleid in self.__changedirection and not self.__changedirection[veicleid] and (self.__simulationTimeSeconds - self.__lasttime1[veicleid-1])>60:
                print('hard turn Left vid', veicleid)
                self.__lasttime1[veicleid-1] = self.__simulationTimeSeconds
                self.__desiredheading[veicleid-1] = (airVehicleState.Heading - 45)
                self.__desiredheading[veicleid-1] = self.__desiredheading[veicleid-1] if self.__desiredheading[veicleid-1] > 0 else self.__desiredheading[veicleid-1]+360
                self.sendHeadingAngleCommandwithcurrentlocation(veicleid,self.__desiredheading[veicleid-1],veicleLocation)
                
            elif (self.__simulationTimeSeconds - self.__LastheadingAngleSendtime[veicleid-1]) > 5 and veicleid in self.__changedirection:
                self.__LastheadingAngleSendtime[veicleid-1] = self.__simulationTimeSeconds
                self.sendHeadingAngleCommandwithcurrentlocation(veicleid,self.__desiredheading[veicleid-1],veicleLocation)
        elif  self.__veicleStrategiId[veicleid-1] == 1:      
            # #Left direction strategy
            if veicleid in self.__changedirection and self.__changedirection[veicleid] and (self.__simulationTimeSeconds - self.__lasttime[veicleid-1])>20: 
                self.__desiredheading[veicleid-1] = airVehicleState.Heading
                if self.__counter[veicleid-1] == 0:
                    self.__lasttime[veicleid-1] = self.__simulationTimeSeconds
                    self.__lasttime1[veicleid-1] = self.__simulationTimeSeconds
                    self.__desiredheading[veicleid-1] = (airVehicleState.Heading - 145)
                    self.__desiredheading[veicleid-1] = self.__desiredheading[veicleid-1] if self.__desiredheading[veicleid-1] > 0 else self.__desiredheading[veicleid-1]+360
                    self.__counter[veicleid-1] = 1
                elif self.__counter[veicleid-1] == 1:
                    self.__desiredheading[veicleid-1] = (airVehicleState.Heading + 145)%360
                    self.__changedirection[veicleid] = False
                    self.__counter[veicleid-1] = 0
                self.sendHeadingAngleCommandwithcurrentlocation(veicleid,self.__desiredheading[veicleid-1],veicleLocation)
            
            elif veicleid in self.__changedirection and not self.__changedirection[veicleid] and (self.__simulationTimeSeconds - self.__lasttime1[veicleid-1])>60:
                print('hard turn Right vid', veicleid)
                self.__lasttime1[veicleid-1] = self.__simulationTimeSeconds
                self.__desiredheading[veicleid-1] = (airVehicleState.Heading + 45)%360
                self.sendHeadingAngleCommandwithcurrentlocation(veicleid,self.__desiredheading[veicleid-1],veicleLocation)
            elif (self.__simulationTimeSeconds - self.__LastheadingAngleSendtime[veicleid-1]) > 5 and veicleid in self.__changedirection:
                self.__LastheadingAngleSendtime[veicleid-1] = self.__simulationTimeSeconds
                self.sendHeadingAngleCommandwithcurrentlocation(veicleid,self.__desiredheading[veicleid-1],veicleLocation)
        
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
        StepSize = 10
        Np = round(360/StepSize)
        for i in range(Np):
            angle = 360 - i * StepSize
            xi = r * cos(radians(angle))
            yi = r * sin(radians(angle))
            Cx = x + xi
            Cy = y + yi
            Points.append([Cx,Cy])
        return Points
    
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
                waypoint.set_Speed(35)
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
                            
                            # else:
                            waypoint.set_NextWaypoint(waypointNumber+1)
                            waypoint.set_Speed(35)
                            waypoint.set_SpeedType(SpeedType.Airspeed)
                            waypoint.set_ClimbRate(15)
                            waypoint.set_TurnType(TurnType.TurnShort)
                            waypoint.set_ContingencyWaypointA(0)
                            waypoint.set_ContingencyWaypointB(0)
                            waypoints.append(waypoint)
                            if i == row-1 and j == col-1:
                                wpoints = self.getBetweenLatLonwithoutVID(x,y,0,0,waypointNumber,1)
                                waypoints = waypoints + wpoints
                                
                            else:
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
                            
                            waypoint.set_NextWaypoint(waypointNumber+1)
                            waypoint.set_Speed(35)
                            waypoint.set_SpeedType(SpeedType.Airspeed)
                            waypoint.set_ClimbRate(15)
                            waypoint.set_TurnType(TurnType.TurnShort)
                            waypoint.set_ContingencyWaypointA(0)
                            waypoint.set_ContingencyWaypointB(0)
                            
                            waypoints.append(waypoint)
                            if i == row-1 and j == rowseg-1:
                                wpoints = self.getBetweenLatLonwithoutVID(x,y,0,0,waypointNumber,1)
                                waypoints += wpoints
                                
                            else:
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
    
    def calculateGridCoordinateAlt(self):
        self.__zoneCenter = {}
        self.__allGridLocation = []
        self.__waypoints = {}
        w = self.__searchAreaWidth
        h = self.__searchAreaHeight
        A = (w*h*4)/self.__noOfZone
        a = sqrt(A)


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

        # zone 1
        waypoints = []
        waypoints.append(waypoint)
                    
        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(0,0,a,a,2,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(a,a,0,a,waypointNumber,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(0,a,a,0,waypointNumber,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(a,0,0,0,waypointNumber,1)
        waypoints = waypoints + wpoints
        self.__waypoints[1] = waypoints
                
        zlocation = Location3D()
        [lat,lon] = self.convertxyToLatLon(a/2,a/2)
        zlocation.set_Latitude(lat)
        zlocation.set_Longitude(lon)
        zlocation.set_Altitude(450)
        self.__zoneCenter[1] = zlocation
        self.__zoneboundaryPoints[1] = [[0,0],[a,a],[0,a],[a,0]]

        # zone 2
        waypoints = []
        waypoints.append(waypoint)
          
        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(0,0,-a,a,2,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(-a,a,-a,0,waypointNumber,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(-a,0,0,a,waypointNumber,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(0,a,0,0,waypointNumber,1)
        waypoints = waypoints + wpoints
        self.__waypoints[2] = waypoints
                
        zlocation = Location3D()
        [lat,lon] = self.convertxyToLatLon(-a/2,a/2)
        zlocation.set_Latitude(lat)
        zlocation.set_Longitude(lon)
        zlocation.set_Altitude(450)
        self.__zoneCenter[2] = zlocation
        self.__zoneboundaryPoints[2] = [[0,0],[-a,a],[0,a],[-a,0]]

        # zone 3
        waypoints = []
        waypoints.append(waypoint)
                    
        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(0,0,-a,-a,2,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(-a,-a,-a,0,waypointNumber,0)#
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(-a,0,0,-a,waypointNumber,0)#
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(0,-a,0,0,waypointNumber,1)#
        waypoints = waypoints + wpoints
        self.__waypoints[3] = waypoints
                
        zlocation = Location3D()
        [lat,lon] = self.convertxyToLatLon(-a/2,-a/2)
        zlocation.set_Latitude(lat)
        zlocation.set_Longitude(lon)
        zlocation.set_Altitude(450)
        self.__zoneCenter[3] = zlocation
        self.__zoneboundaryPoints[3] = [[0,0],[-a,-a],[0,-a],[-a,0]]

        # zone 4
        waypoints = []
        waypoints.append(waypoint)
                    
        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(0,0,a,-a,2,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(a,-a,a,0,waypointNumber,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(a,0,0,-a,waypointNumber,0)
        waypoints = waypoints + wpoints

        wpoints,waypointNumber = self.getBetweenLatLonwithoutVIDAlt(0,-a,0,0,waypointNumber,1)
        waypoints = waypoints + wpoints
        self.__waypoints[4] = waypoints
                
        zlocation = Location3D()
        [lat,lon] = self.convertxyToLatLon(a/2,-a/2)
        zlocation.set_Latitude(lat)
        zlocation.set_Longitude(lon)
        zlocation.set_Altitude(450)
        self.__zoneCenter[4] = zlocation
        self.__zoneboundaryPoints[4] = [[0,0],[a,-a],[0,-a],[a,0]]
    
    def calculateGridCoordinateAlt1(self):
        self.__zoneCenter = {}
        self.__allGridLocation = []
        self.__waypoints = {}
        w = self.__searchAreaWidth*2
        h = self.__searchAreaHeight*2


        wSeg,hSeg = self.getBig2Factor(self.__noOfZone)

        dw = w/wSeg
        dh = h/hSeg
        currCenterx = -w/2
        currCentery = -h/2

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
        

    def getFourcenter(self,arbitararyCenter): # have to work
        zid = 0
        mind = 10e10
        i=0
        for zcenter in self.__zoneCenter:
            d = self.getdistance(arbitararyCenter,zcenter)
            i += 1
            if d < mind:
                mind = d
                zid = i
        
        [x,y] = self.convertLatLonToxy(arbitararyCenter.get_Latitude(),arbitararyCenter.get_Longitude())
        cornerpoints = self.__zoneboundaryPoints[zid]
        xc1,yc1 = (cornerpoints[0][0]+x)/2,(cornerpoints[0][1]+y)/2
        xc2,yc2 = (cornerpoints[1][0]+x)/2,(cornerpoints[1][1]+y)/2
        xc3,yc3 = (cornerpoints[2][0]+x)/2,(cornerpoints[2][1]+y)/2
        xc4,yc4 = (cornerpoints[3][0]+x)/2,(cornerpoints[3][1]+y)/2
        
        points = [[xc1,yc1],[xc2,yc2],[xc3,yc3],[xc4,yc4]]
        
        for vid in self.__currentLocationofUAV.keys():
            if vid in self.__uavsInSearch and not vid in self.__uavsInSarvey:
                mind = 10e10
                i=0
                zid1 = 0
                for zcenter in self.__zoneCenter:
                    d = self.getdistance(arbitararyCenter,self.__currentLocationofUAV[vid])
                    i += 1
                    if d < mind:
                        mind = d
                        zid1 = i
                if zid1 == zid:
                    refPoint = self.convertLatLonToxy(self.__currentLocationofUAV[vid].get_Latitude(),self.__currentLocationofUAV[vid].get_Longitude())
                    destipoint =  getClosestPoint(self,points,refPoint)
    
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
            waypoint.set_NextWaypoint(waypointNumber+1)
            waypoint.set_Speed(self.__maxSpeedofUAV[vid])
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        
        
        radius = 3000
        xc = x
        yc = y
        points = self.GenerateSamplePointsOnACircle(xc,yc,radius)
        wpoints,wpointnumber = self.getBetweenLatLonwithoutVIDAlt(xc,yc,points[0][0],points[0][1],waypointNumber,0)
        waypoints += wpoints
        waypointNumber = wpointnumber
        for i in range(len(points)-1):
            p = points[i+1]
            x = p[0]
            y = p[1]
            [lat,lon] = self.convertxyToLatLon(x,y)
            waypoint = Waypoint()
            waypoint.set_Latitude(lat)
            waypoint.set_Longitude(lon)
            alti = self.getAltitudeLatLon(lat,lon) 
            if alti < self.__normalSearchAltitude:
                waypoint.set_Altitude(self.__normalSearchAltitude)
            else:
                waypoint.set_Altitude(alti + self.__safeHeight)##
            waypoint.set_AltitudeType(AltitudeType.MSL)
            waypoint.set_Number(waypointNumber)
            if i == len(points)-2:
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
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        waypointNumber -= 1
        return waypoints, startwaypointId+1
    
    def getBetweenLatLonwithoutVID(self,xs,ys,xe,ye,startwaypointId,connectingwaypointId):
        delx = xe-xs
        dely = ye-ys
        d = delx**2 + dely**2
        m = dely/delx
        ii = int(round(sqrt(d)/500))
        delx /= ii
        ii = ii - 1
        x = xs
        waypointNumber = startwaypointId+1
        waypoints = []
        x += delx
        for i in range(ii):
            y = ys + (x-xs)*m + 300
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
            waypoint.set_Speed(35)
            waypoint.set_SpeedType(SpeedType.Airspeed)
            waypoint.set_ClimbRate(15)
            waypoint.set_TurnType(TurnType.TurnShort)
            waypoint.set_ContingencyWaypointA(0)
            waypoint.set_ContingencyWaypointB(0)
            waypoints.append(waypoint)
            waypointNumber += 1
        
        return waypoints
    
    def getBetweenLatLonwithoutVIDAlt(self,xs,ys,xe,ye,waypointNumber,option):
        delx = xe-xs
        dely = ye-ys
        d = delx**2 + dely**2
        if delx != 0:
            m = dely/delx
        ii = int(round(sqrt(d)/300))
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
            return location
        if veicleid == 4:
            [xr,yr] = self.convertLatLonToxy(refLocation.get_Latitude(),refLocation.get_Longitude())
            y = yr + (self.__searchAreaHeight-abs(yr))/2
            [lat,lon] = self.convertxyToLatLon(xr,y)
            location = Location3D()
            location.set_Latitude(lat)
            location.set_Longitude(lon)
            location.set_Altitude(refLocation.get_Altitude())
            return location
        return refLocation
    
    def isinKeepInZone(self,location):
        xyposition = self.convertLatLonToxy(location.get_Latitude(),location.get_Longitude())
        if abs(xyposition[0]) <= self.__searchAreaWidth and abs(xyposition[1]) <= self.__searchAreaHeight:
            return True
        return False

    def isLeavingFireZone(self,veicleid,location):
        if self.__lastfireZonelocation:
            if veicleid in self.__lastfireZonelocation:
                lastFireLocation =  self.__lastfireZonelocation[veicleid]
                lastFireLocationXY = self.convertLatLonToxy(lastFireLocation.get_Latitude(),lastFireLocation.get_Longitude())
                locationXY = self.convertLatLonToxy(location.get_Latitude(),location.get_Longitude())
                d = (locationXY[0] - lastFireLocationXY[0])**2 + (locationXY[1] - lastFireLocationXY[1])**2
                if d > 250000:
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
        mind =10e20
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
        # print(d)
        return d
            
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
        # allxypoints = self.__boundaryPoints
        if self.__firezonePoints:
            reset = False
            for key in self.__firezonePoints.keys():
                points = self.__firezonePoints[key]
                # self.__firezonePoints[key] = []
                # print(points)
                allxypoints = points
                   
                if len(allxypoints) >= 3:
                    boundarypoints = self.graham_scan(allxypoints)
                    self.__firezonePoints[key] = boundarypoints
                    if self.__wspeed != 0:
                        self.translateEstimatedShape(self.__wspeed,self.__ditectionTheta)
                        boundarypoints = self.__firezonePoints[key]
                        reset = True
                    for xypoint in boundarypoints:
                        [lat,lon]=self.convertxyToLatLon(xypoint[0],xypoint[1])
                        locationpoint = Location3D()
                        locationpoint.set_Latitude(lat)
                        locationpoint.set_Longitude(lon)
                        self.__estimatedHazardZone.get_BoundaryPoints().append(locationpoint)             
                    self.sendEstimateReport(key)
                    self.__estimatedHazardZone = Polygon()
            if reset:
                self.__wspeed = 0
                self.__ditectionTheta = 0

    def getSendReportStatus(self):
        return self.__sendReport
    
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
        
    def translateEstimatedShape(self, WinSpeed, Angle):
        Rate = WinSpeed
        X_Rate = Rate * cos(radians(Angle))
        Y_Rate = Rate * sin(radians(Angle))
        if self.__firezonePoints:
            for key in self.__firezonePoints.keys():
                points = self.__firezonePoints[key]
                SamplePoints = points
                NewSamplePoint = []
                if SamplePoints:
                    for x,y in SamplePoints:
                        x = x - X_Rate
                        y = y - Y_Rate
                        NewSamplePoint.append([x,y])
                else:
                    NewSamplePoint = SamplePoints
                self.__firezonePoints[key] = NewSamplePoint
    
    def getBig2Factor(self,num):
        ii = int(num/2)+1
        for i in range(ii):
            if i>0 and num%i == 0 and num/i <= i:
                return i,int(num/i)
                
                
                
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
            # elif smpleHazardDetector.getupdateAreaStatus():
                # smpleHazardDetector.updateEstimatedArea()
                # smpleHazardDetector.setupdateAreaStatus(False)
            # pass
    except KeyboardInterrupt as ki:
        print("Stopping amase tcp client")
    except Exception as ex:
        print(ex)
    amaseClient.stop()
