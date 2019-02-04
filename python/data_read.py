#!/usr/bin/python
# author: Mrinmoy sarkar
# date: 1 FEB 2019
# email: mrinmoy.pol@gmail.com


from amase import AmaseTCPClient
from amase import IDataReceived
from afrl.cmasi import AirVehicleState
import afrl




class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print("message starts::::::")
        #print(lmcpObject.toXMLStr(""))
        print(lmcpObject)
        
        if isinstance(lmcpObject,afrl.cmasi.AirVehicleState.AirVehicleState):
            print("found air vehicle state")
        elif isinstance(lmcpObject,afrl.cmasi.SessionStatus.SessionStatus):
            print("found session status")
        elif isinstance(lmcpObject,afrl.cmasi.AirVehicleConfiguration.AirVehicleConfiguration):
            print("found AirVehicleConfiguration") 
        elif isinstance(lmcpObject,afrl.cmasi.searchai.HazardZoneDetection.HazardZoneDetection):
            print("found HazardZoneDetection")
        elif isinstance(lmcpObject,afrl.cmasi.searchai.HazardZone.HazardZone):
            print("found HazardZone")
        elif isinstance(lmcpObject,afrl.cmasi.searchai.HazardZoneChangeCommand.HazardZoneChangeCommand):
            print("found HazardZoneChangeCommand")
            
        print("message ends::::::::")

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