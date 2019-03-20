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
        self.__gotHint = [False,False,False,False]
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
            self.calculateGridCoordinateAlt()
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

                self.__NewSTGheadingangle[veicleid-1] = airVehicleState.Heading
                self.sendHeadingAngleCommand(veicleid,self.__NewSTGheadingangle[veicleid-1])
            
            if self.__simulationTimeSeconds > 0:
                self.__currentLocationofUAV[veicleid] = veicleLocation
                self.__currentHeadingAngleUAV[veicleid] = airVehicleState.Heading
                
                self.surveyNewStrategy(airVehicleState)   
                    
        elif isinstance(lmcpObject, AirVehicleConfiguration):
            airvehicleConfiguration = lmcpObject
            self.__maxSpeedofUAV[airvehicleConfiguration.ID] = airvehicleConfiguration.get_MaximumSpeed()
            payloadconfigList = airvehicleConfiguration.PayloadConfigurationList
            self.__maxAzimuthangle[airvehicleConfiguration.ID] = payloadconfigList[0].MaxAzimuth
            self.__minAzimuthangle[airvehicleConfiguration.ID] = payloadconfigList[0].MinAzimuth
            print("found AirVehicleConfiguration") 
 
        elif isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            detectedLocation = hazardDetected.get_DetectedLocation()
            detectingEntity = hazardDetected.get_DetectingEnitiyID()
            vid = detectingEntity
            fireZoneType = hazardDetected.get_DetectedHazardZoneType()
            self.__uavsInSarvey[detectingEntity] = True
            self.__maxSpeedofUAV[detectingEntity] = 15 ## play here
            
            if fireZoneType == HazardType.Fire:
                
                self.__gotHint[vid-1] = True
                if self.__NewSTGoption[vid-1] == 1:
                    print('found forward',vid)
                    self.__NewSTGforward[vid-1] = 1
                elif self.__NewSTGoption[vid-1] == 2:
                    print('found left',vid)
                    self.__NewSTGleft[vid-1] = 1
                        
            elif fireZoneType == HazardType.Smoke:
                #print('smoke detected')
                # self.__uavisInsmokeZone[vid] = True
                # self.sendHeadingAngleCommandwithcurrentlocation(vid,self.__currentHeadingAngleUAV[vid],self.__currentLocationofUAV[vid])
                pass
   
    
    
    def sendHeadingAngleCommand(self,veicleid,headingangle):
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(veicleid)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)
        
        flightDirectorAction = FlightDirectorAction();
        flightDirectorAction.set_Speed(15)
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
        
        gimbleAngleAction = GimbalAngleAction()
        gimbleAngleAction.set_PayloadID(1)
        gimbleAngleAction.set_Azimuth(azimuthangle)
        gimbleAngleAction.set_Elevation(elevationangle)
        gimbleAngleAction.set_Rotation(0)
        
        vehicleActionCommand.get_VehicleActionList().append(gimbleAngleAction)
        
        self.__client.sendLMCPObject(vehicleActionCommand)
        
    def surveyNewStrategy(self,airVehicleState):
        vid = airVehicleState.ID
        leftstg = 1
        if self.__gotHint[vid-1] and leftstg == 0:#left
            if self.__NewSTGfirst[vid-1] == 0:
                self.__NewSTGfirst[vid-1] = 1
                self.__NewSTGrefHeading[vid-1] = airVehicleState.Heading
                self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading + 90)%360
                self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
            else:
                headingangleError = abs(self.__NewSTGheadingangle[vid-1]-airVehicleState.Heading)
                headingangleError = headingangleError if headingangleError < 180 else (360-headingangleError)
                if headingangleError < 5:
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
                            self.__NewSTGdtaction[vid-1] = 5
                            self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
                          
                        elif self.__NewSTGleft[vid-1] == 1 and self.__NewSTGforward[vid-1] == 2:
                            print('#go straignt',vid)
                            self.__NewSTGheadingangle[vid-1] = airVehicleState.Heading
                            self.__NewSTGdtaction[vid-1] = 5
                            
                        elif self.__NewSTGleft[vid-1] == 2 and self.__NewSTGforward[vid-1] == 1:
                            print('#take hard right',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading + 135)%360
                            self.__NewSTGdtaction[vid-1] = 5
                            self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
                               
                        elif self.__NewSTGleft[vid-1] == 2 and self.__NewSTGforward[vid-1] == 2:
                            print('# take hard left',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading - 30)
                            self.__NewSTGheadingangle[vid-1] = self.__NewSTGheadingangle[vid-1] if self.__NewSTGheadingangle[vid-1] >= 0 else (self.__NewSTGheadingangle[vid-1] + 360)
                            self.__NewSTGdtaction[vid-1] = 10
                        
                if (self.__simulationTimeSeconds - self.__NewSTGt3[vid-1]) > self.__NewSTGdtaction[vid-1]:
                    self.__NewSTGt3[vid-1] = self.__simulationTimeSeconds
                    self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
        elif self.__gotHint[vid-1] and leftstg == 1: #right
            if self.__NewSTGfirst[vid-1] == 0:
                self.__NewSTGfirst[vid-1] = 1
                self.__NewSTGrefHeading[vid-1] = airVehicleState.Heading
                self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading - 90)
                self.__NewSTGheadingangle[vid-1] = self.__NewSTGheadingangle[vid-1] if self.__NewSTGheadingangle[vid-1] >= 0 else (self.__NewSTGheadingangle[vid-1] + 360)

                self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
            else:
                headingangleError = abs(self.__NewSTGheadingangle[vid-1]-airVehicleState.Heading)
                headingangleError = headingangleError if headingangleError < 180 else (360-headingangleError)
                if headingangleError < 5:
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

                            self.__NewSTGdtaction[vid-1] = 5
                            self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
                          
                        elif self.__NewSTGleft[vid-1] == 1 and self.__NewSTGforward[vid-1] == 2:
                            print('#go straignt',vid)
                            self.__NewSTGheadingangle[vid-1] = airVehicleState.Heading
                            self.__NewSTGdtaction[vid-1] = 5
                            
                        elif self.__NewSTGleft[vid-1] == 2 and self.__NewSTGforward[vid-1] == 1:
                            print('#take hard left',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading - 135)
                            self.__NewSTGheadingangle[vid-1] = self.__NewSTGheadingangle[vid-1] if self.__NewSTGheadingangle[vid-1] >= 0 else (self.__NewSTGheadingangle[vid-1] + 360)

                            self.__NewSTGdtaction[vid-1] = 5
                            self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
                               
                        elif self.__NewSTGleft[vid-1] == 2 and self.__NewSTGforward[vid-1] == 2:
                            print('# take hard right',vid)
                            self.__NewSTGheadingangle[vid-1] = (airVehicleState.Heading + 30)%360
                            self.__NewSTGdtaction[vid-1] = 10
                            self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
                        
                if (self.__simulationTimeSeconds - self.__NewSTGt3[vid-1]) > self.__NewSTGdtaction[vid-1]:
                    self.__NewSTGt3[vid-1] = self.__simulationTimeSeconds
                    self.sendHeadingAngleCommand(vid,self.__NewSTGheadingangle[vid-1])
     
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
            # if smpleHazardDetector.getSendReportStatus():
                # smpleHazardDetector.findBoundaryandSendReport()
                # smpleHazardDetector.setSendReportStatus(False)
            # elif smpleHazardDetector.getupdateAreaStatus():
                # smpleHazardDetector.updateEstimatedArea()
                # smpleHazardDetector.setupdateAreaStatus(False)
            pass
    except KeyboardInterrupt as ki:
        print("Stopping amase tcp client")
    except Exception as ex:
        print(ex)
    amaseClient.stop()
