#!/usr/bin/python
# author: Mrinmoy sarkar
# date: 13 FEB 2019
# email: mrinmoy.pol@gmail.com

from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.Circle import Circle
from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.Waypoint import Waypoint
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
from lmcp import LMCPFactory
from afrl.cmasi.KeepInZone import KeepInZone
from afrl.cmasi.searchai.HazardZone import HazardZone
from afrl.cmasi.searchai.HazardZoneChangeCommand import HazardZoneChangeCommand
from afrl.cmasi.AirVehicleConfiguration import AirVehicleConfiguration
from afrl.cmasi.WeatherReport import WeatherReport
from math import sin,cos,atan2,pi,radians
from random import randint


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
        self.__counter = 0
        self.anchor = []
        self.__lastfireZonelocation = {}
        

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
            print('found keep in zone')
        elif isinstance(lmcpObject, AirVehicleState):
            airVehicleState = lmcpObject
            self.__simulationTime = airVehicleState.Time/(1000*60)
            #print("found air vehicle state")
            #print(lmcpObject.toXMLStr(""))
            veicleid = airVehicleState.ID
            veicleLocation = airVehicleState.Location
            self.__currentLocationofUAV[veicleid] = veicleLocation
            
            # if self.__counter < 50:
                # self.__counter = self.__counter + 1
                # if not self.__firezonePoints or (self.__firezonePoints and not veicleid in self.__firezonePoints):
                    # self.__firezonePoints[veicleid] = [veicleLocation]
                # else:
                    # self.__firezonePoints[veicleid].append(veicleLocation)
                    
                    
            
            # if self.__counter == 50:
                # self.findBoundaryandSendReport()
                # self.__counter = self.__counter + 1
            # if not self.isinKeepInZone(veicleLocation):
                # print(veicleid, 'out the zone')
                
            if veicleid in self.__uavsisinfirezone and self.isLeavingFireZone(veicleid,veicleLocation):
                # # print('leaving fire zone')
                
                if not self.__isInMinorAxis:
                    self.turn(veicleid)
                    self.__isInMinorAxis[veicleid] = True
                    self.__turnlocation[veicleid] = veicleLocation
                elif veicleid in self.__isInMinorAxis:
                    if self.__isInMinorAxis[veicleid] and self.getdistance(veicleLocation,self.__turnlocation[veicleid])>20000:
                        self.__isInMinorAxis[veicleid] = False
                        self.turn(veicleid)
                        self.__turnlocation[veicleid] = veicleLocation
            if self.__isInMinorAxis and veicleid in self.__isInMinorAxis:
                if not self.__isInMinorAxis[veicleid] and self.getdistance(veicleLocation,self.__turnlocation[veicleid])>250000:
                    self.__isInMinorAxis[veicleid] = True
                    self.turn(veicleid)
                    self.__turnlocation[veicleid] = veicleLocation
            if not veicleid in self.__uavsLoiter:
                self.__uavsLoiter[veicleid] = True
                self.sendLoiterCommand(veicleid,self.__centerLocation,5000*veicleid)
                # self.sendHeadingAngleCommand(veicleid,0)
                # self.__currentHeadingAngleUAV[veicleid] = 0
                # locinxy = self.convertLatLonToxy(veicleLocation.get_Latitude(),veicleLocation.get_Longitude())
                # print(veicleid,' X: ',locinxy[0],' Y: ', locinxy[1])
                # self.__estimatedHazardZone.get_BoundaryPoints().append(vehicleLocation)

                #Send out the estimation report to draw the polygon
                # self.sendEstimateReport()
            if not veicleid in self.__uavsgothint and self.__gotHint and not veicleid in self.__uavsisinfirezone:
                self.__uavsgothint[veicleid] = True
                self.sendLoiterCommand(veicleid, self.__firezoneHintLocation,1000*veicleid)
            
            if self.__gotHint and self.__simulationTime > 18 and self.__simulationTime < 21:
                self.findBoundaryandSendReport()
            elif self.__gotHint and self.__simulationTime > 38 and self.__simulationTime < 43:
                self.findBoundaryandSendReport()
            elif self.__gotHint and self.__simulationTime > 55:
                self.findBoundaryandSendReport()
        elif isinstance(lmcpObject, AirVehicleConfiguration):
            #print("found AirVehicleConfiguration") 
            pass
        elif isinstance(lmcpObject, SessionStatus):
            #print("found session status")
            #print(lmcpObject.toXMLStr(""))
            pass
        elif isinstance(lmcpObject, WeatherReport):
            #print("found WeatherReport")
            #print(lmcpObject.toXMLStr(""))
            pass
        elif isinstance(lmcpObject, HazardZone):
            print("found HazardZone")
        elif isinstance(lmcpObject, HazardZoneChangeCommand):
            print("found HazardZoneChangeCommand")
        elif isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            detectedLocation = hazardDetected.get_DetectedLocation()
            detectingEntity = hazardDetected.get_DetectingEnitiyID()
            self.__firezoneHintLocation = detectedLocation
            self.__gotHint = True
            self.__lastfireZonelocation[detectingEntity] = detectedLocation
            if not detectingEntity in self.__uavsisinfirezone:
                self.__uavsisinfirezone[detectingEntity] = True
                self.__currentHeadingAngleUAV[detectingEntity] = 3
                self.turn(detectingEntity)
                
            if not self.__firezonePoints or not detectingEntity in self.__firezonePoints:
                self.__firezonePoints[detectingEntity] = [detectedLocation]
                # print('in fire zone 1')
            else:
                self.__firezonePoints[detectingEntity].append(detectedLocation)
                # print(self.__firezonePoints)
            
            if not detectingEntity in self.__uavsLoiter:
                #Send the loiter command
                #self.sendLoiterCommand(detectingEntity, detectedLocation)

                #Note: Polygon points must be in clockwise or counter-clockwise order to get a shape without intersections
                #self.__estimatedHazardZone.get_BoundaryPoints().append(detectedLocation)

                #Send out the estimation report to draw the polygon
                #self.sendEstimateReport()

                #self.__uavsLoiter[detectingEntity] = True
                #print('UAV' + str(detectingEntity) + ' detected hazard at ' + str(detectedLocation.get_Latitude()) + ',' + str(detectedLocation.get_Longitude()) + '. Sending loiter command.');
                pass
    
    def sendWaypoint(self,veicleid,location):
        missionCommand = MissionCommand()
        missionCommand.set_FirstWaypoint(1)
        missionCommand.set_VehicleID(veicleid)
        missionCommand.set_Status(CommandStatusType.Pending)
        missionCommand.set_CommandID(1)
        
        waypoints = []
        waypoint1 = Waypoint()
        waypoint1.setLatitude(1.505)
        waypoint1.setLongitude(-132.539)
        waypoint1.setAltitude(100)
        waypoint1.setAltitudeType(AltitudeType.MSL)
        waypoint1.setNumber(1)
        waypoint1.setNextWaypoint(2)
        waypoint1.setSpeed(30)
        waypoint1.setSpeedType(SpeedType.Airspeed)
        waypoint1.setClimbRate(0)
        waypoint1.setTurnType(TurnType.TurnShort)
        waypoint1.setContingencyWaypointA(0)
        waypoint1.setContingencyWaypointB(0)
         
        
        waypoint2 = Waypoint()
        waypoint2.setLatitude(1.52)
        waypoint2.setLongitude(-132.51)
        waypoint2.setAltitude(100)
        waypoint2.setAltitudeType(AltitudeType.MSL)
        waypoint2.setNumber(2)
        waypoint2.setNextWaypoint(1)
        waypoint2.setSpeed(30)
        waypoint2.setSpeedType(SpeedType.Airspeed)
        waypoint2.setClimbRate(0);
        waypoint2.setTurnType(TurnType.TurnShort)
        waypoint2.setContingencyWaypointA(0)
        waypoint2.setContingencyWaypointB(0)
         
        
        waypoints.append(waypoint1)
        waypoints.append(waypoint2)
         
        
        missionCommand.get_WaypointList().append(waypoints)
        self.__client.sendLMCPmessage(LMCPFactory.packMessage(missionCommand,True))
    
    def sendHeadingAngleCommand(self,vehicleId,headingangle):
        vehicleActionCommand = VehicleActionCommand()
        flightDirectorAction = FlightDirectorAction();
        
        vehicleActionCommand.set_VehicleID(vehicleId)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)
   
        flightDirectorAction.set_Speed(30)
        flightDirectorAction.set_SpeedType(SpeedType.Airspeed)
        flightDirectorAction.set_Heading(headingangle)
        flightDirectorAction.set_Altitude(100)
        flightDirectorAction.set_AltitudeType(AltitudeType.MSL)
        flightDirectorAction.set_ClimbRate(0)
        
        vehicleActionCommand.get_VehicleActionList().append(flightDirectorAction)
        
        self.__client.sendLMCPmessage(LMCPFactory.packMessage(vehicleActionCommand,True))
    
    def sendLoiterCommand(self, vehicleId, location, radius):
        #Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(vehicleId)
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
        loiterAction.set_Airspeed(30)

        #Creating a 3D location object for the stare point
        loiterAction.set_Location(location)

        #Adding the loiter action to the vehicle action list
        vehicleActionCommand.get_VehicleActionList().append(loiterAction)

        #Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPmessage(LMCPFactory.packMessage(vehicleActionCommand,True))

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
        self.__client.sendLMCPmessage(LMCPFactory.packMessage(hazardZoneEstimateReport,True))
        
    def convertLatLonToxy(self,lat,long):
        R = 111000
        a = lat-self.__searchAreaCenterLat
        b = long-self.__searchAreaCenterLong
        x = R*a
        y = R*radians(lat)*b
        return [x,y]
        
    def convertxyToLatLon(self,x,y):
        R = 111000
        lat = x/R + self.__searchAreaCenterLat
        long = y/(R*radians(lat)) + self.__searchAreaCenterLong
        return [lat,long]
        
    def isinKeepInZone(self,location):
        xyposition = self.convertLatLonToxy(location.get_Latitude(),location.get_Longitude())
        # print(xyposition[0],' width: ', self.__searchAreaWidth, xyposition[1], ' height: ',self.__searchAreaHeight)
        if abs(xyposition[0]) <= self.__searchAreaWidth and abs(xyposition[1]) <= self.__searchAreaHeight:
            return True
        return False

    def isLeavingFireZone(self,vehicleid,location):
        if self.__lastfireZonelocation:
            if vehicleid in self.__lastfireZonelocation:
                # print(self.__firezonePoints)
                lastFireLocation =  self.__lastfireZonelocation[vehicleid]
                lastFireLocationXY = self.convertLatLonToxy(lastFireLocation.get_Latitude(),lastFireLocation.get_Longitude())
                locationXY = self.convertLatLonToxy(location.get_Latitude(),location.get_Longitude())
                d = (locationXY[0] - lastFireLocationXY[0])**2 + (locationXY[1] - lastFireLocationXY[1])**2
                if d > 10000:
                    return True
            
        return False

    def getdistance(self,loc1,loc2):
        loc1XY = self.convertLatLonToxy(loc1.get_Latitude(),loc1.get_Longitude())
        loc2XY = self.convertLatLonToxy(loc2.get_Latitude(),loc2.get_Longitude())
        d = (loc1XY[0] - loc2XY[0])**2 + (loc1XY[1] - loc2XY[1])**2
        # print(d)
        return d
    
    def turn(self,veicleid):
        headingAngle = 0
        if self.__currentHeadingAngleUAV[veicleid] == 0:
            self.__currentHeadingAngleUAV[veicleid] = 1
            headingAngle = 270
        elif self.__currentHeadingAngleUAV[veicleid] == 1:
            self.__currentHeadingAngleUAV[veicleid] = 2
            headingAngle = 180
        elif self.__currentHeadingAngleUAV[veicleid] == 2:
            self.__currentHeadingAngleUAV[veicleid] = 3
            headingAngle = 270
        elif self.__currentHeadingAngleUAV[veicleid] == 3:
            self.__currentHeadingAngleUAV[veicleid] = 0
            headingAngle = 0
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
        allxypoints = []
        if self.__firezonePoints:
            for key in self.__firezonePoints.keys():
                points = self.__firezonePoints[key]
                for point in points:
                    [x,y] = self.convertLatLonToxy(point.get_Latitude(),point.get_Longitude())
                    allxypoints.append([x,y])
        # print(len(allxypoints))
        boundarypoints = self.graham_scan(allxypoints)
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



     
#################
## Main
#################

if __name__ == '__main__':
    myHost = 'localhost'
    myPort = 5555
    amaseClient = AmaseTCPClient(myHost, myPort)
    #amaseClient.addReceiveCallback(PrintLMCPObject())
    amaseClient.addReceiveCallback(SampleHazardDetector(amaseClient))

    try:
        # make a threaded client, listen until a keyboard interrupt (ctrl-c)
        #start client thread
        amaseClient.start()

        while True:
            #wait for keyboard interrupt
            pass
    except KeyboardInterrupt as ki:
        print("Stopping amase tcp client")
    except Exception as ex:
        print(ex)
    amaseClient.stop()
