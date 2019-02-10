#!/usr/bin/python
# author: Mrinmoy sarkar
# date: 1 FEB 2019
# email: mrinmoy.pol@gmail.com


from amase import AmaseTCPClient
from amase import IDataReceived
from afrl.cmasi import AirVehicleState
from afrl.cmasi import SessionStatus
from afrl.cmasi import AirVehicleConfiguration
from afrl.cmasi.searchai import HazardZoneDetection
from afrl.cmasi.searchai import HazardZone
from afrl.cmasi.searchai import HazardZoneChangeCommand

from afrl.cmasi import VehicleActionCommand
from afrl.cmasi import LoiterAction
from afrl.cmasi import CommandStatusType
from afrl.cmasi import LoiterType
from afrl.cmasi import LoiterDirection
from afrl.cmasi import Location3D
from lmcp import LMCPFactory
from afrl.cmasi.searchai import HazardZoneEstimateReport
from afrl.cmasi import Polygon
from afrl.cmasi.searchai import HazardType
import afrl
from afrl.cmasi import FlightDirectorAction 
from afrl.cmasi import SpeedType
from afrl.cmasi import AltitudeType




class PrintLMCPObject(IDataReceived):
    
    def __init__(self):
        self.uavs = [True,True,True,True]
        self.estimatedHazardZone =  Polygon.Polygon()
        self.turn = True
        
    def dataReceived(self, lmcpObject):
        #print("message starts::::::")
        #print(lmcpObject.toXMLStr(""))
        #print(lmcpObject)
        if self.turn:
            vehicleId = 2
            #self.sendHeadingAngleCommand(vehicleId)
            self.turn = False
        
        if isinstance(lmcpObject, AirVehicleState.AirVehicleState):
            print("found air vehicle state")
            pass
        elif isinstance(lmcpObject, SessionStatus.SessionStatus):
            #print("found session status")
            pass
        elif isinstance(lmcpObject, AirVehicleConfiguration.AirVehicleConfiguration):
            #print("found AirVehicleConfiguration") 
            pass
        elif isinstance(lmcpObject, HazardZoneDetection.HazardZoneDetection):
            print("found HazardZoneDetection")
            location = lmcpObject.get_DetectedLocation()
            vehicleId = lmcpObject.get_DetectingEnitiyID()
            if self.uavs[vehicleId-1]:
                self.sendLoiterCommand(vehicleId, location)
                self.estimatedHazardZone.get_BoundaryPoints().append(location)
                self.sendEstimatedReport(self.estimatedHazardZone)
                self.uavs[vehicleId-1] =  False
        elif isinstance(lmcpObject, HazardZone.HazardZone):
            print("found HazardZone")
        elif isinstance(lmcpObject, HazardZoneChangeCommand.HazardZoneChangeCommand):
            print("found HazardZoneChangeCommand")
            
            
        #print("message ends::::::::", myHost)
    
    def sendHeadingAngleCommand(self,vehicleId):
        o = VehicleActionCommand.VehicleActionCommand()
        flightDirectorAction = FlightDirectorAction.FlightDirectorAction();
        
        o.set_VehicleID(vehicleId)
        o.set_Status(CommandStatusType.CommandStatusType.Pending)
        o.set_CommandID(1)
   
        flightDirectorAction.set_Speed(30)
        flightDirectorAction.set_SpeedType(SpeedType.SpeedType.Airspeed)
        flightDirectorAction.set_Heading(10)
        flightDirectorAction.set_Altitude(100)
        flightDirectorAction.set_AltitudeType(AltitudeType.AltitudeType.MSL)
        flightDirectorAction.set_ClimbRate(0)
        
        o.get_VehicleActionList().append(flightDirectorAction)
        
        amaseClient.sendLMCPmessage(LMCPFactory.packMessage(o, True))
    
    def sendLoiterCommand(self,vehicleId,location):
        o = VehicleActionCommand.VehicleActionCommand()
        loiterAction = LoiterAction.LoiterAction();
        
        o.set_VehicleID(vehicleId)
        o.set_Status(CommandStatusType.CommandStatusType.Pending)
        o.set_CommandID(1)
   
        
        loiterAction.set_LoiterType(LoiterType.LoiterType.Circular);
        loiterAction.set_Radius(250);
        loiterAction.set_Axis(0);
        loiterAction.set_Length(0);
        loiterAction.set_Direction(LoiterDirection.LoiterDirection.Clockwise);
        loiterAction.set_Duration(100000);
        loiterAction.set_Airspeed(15);
        loiterAction.set_Location(location);
        
        o.get_VehicleActionList().append(loiterAction)
        
        amaseClient.sendLMCPmessage(LMCPFactory.packMessage(o, True))
        
    def sendEstimatedReport(self, estimatedShape):
        o = HazardZoneEstimateReport.HazardZoneEstimateReport();
        o.set_EstimatedZoneShape(estimatedShape);
        o.set_UniqueTrackingID(1);
        o.set_EstimatedGrowthRate(0);
        o.set_PerceivedZoneType(HazardType.HazardType.Fire);
        o.set_EstimatedZoneDirection(0);
        o.set_EstimatedZoneSpeed(0);
        
        amaseClient.sendLMCPmessage(LMCPFactory.packMessage(o, True))
        
if __name__ == '__main__':
    myHost = 'localhost'
    myPort = 5555
    amaseClient = AmaseTCPClient(myHost, myPort)
    amaseClient.addReceiveCallback(PrintLMCPObject())
    

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