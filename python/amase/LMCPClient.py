import socket
from lmcp import LMCPFactory

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

from afrl.cmasi import AbstractGeometry
from afrl.cmasi import KeyValuePair
from afrl.cmasi import Location3D
from afrl.cmasi import PayloadAction
from afrl.cmasi import PayloadConfiguration
from afrl.cmasi import PayloadState
from afrl.cmasi import VehicleAction
from afrl.cmasi import Task
from afrl.cmasi import SearchTask
from afrl.cmasi import AbstractZone
from afrl.cmasi import EntityConfiguration
from afrl.cmasi import FlightProfile
from afrl.cmasi import AirVehicleConfiguration
from afrl.cmasi import EntityState
from afrl.cmasi import AirVehicleState
from afrl.cmasi import Wedge
from afrl.cmasi import AreaSearchTask
from afrl.cmasi import CameraAction
from afrl.cmasi import CameraConfiguration
from afrl.cmasi import GimballedPayloadState
from afrl.cmasi import CameraState
from afrl.cmasi import Circle
from afrl.cmasi import GimbalAngleAction
from afrl.cmasi import GimbalConfiguration
from afrl.cmasi import GimbalScanAction
from afrl.cmasi import GimbalStareAction
from afrl.cmasi import GimbalState
from afrl.cmasi import GoToWaypointAction
from afrl.cmasi import KeepInZone
from afrl.cmasi import KeepOutZone
from afrl.cmasi import LineSearchTask
from afrl.cmasi import NavigationAction
from afrl.cmasi import LoiterAction
from afrl.cmasi import LoiterTask
from afrl.cmasi import Waypoint
from afrl.cmasi import MissionCommand
from afrl.cmasi import MustFlyTask
from afrl.cmasi import OperatorSignal
from afrl.cmasi import OperatingRegion
from afrl.cmasi import AutomationRequest
from afrl.cmasi import PointSearchTask
from afrl.cmasi import Polygon
from afrl.cmasi import Rectangle
from afrl.cmasi import RemoveTasks
from afrl.cmasi import ServiceStatus
from afrl.cmasi import SessionStatus
from afrl.cmasi import VehicleActionCommand
from afrl.cmasi import VideoStreamAction
from afrl.cmasi import VideoStreamConfiguration
from afrl.cmasi import VideoStreamState
from afrl.cmasi import AutomationResponse
from afrl.cmasi import RemoveZones
from afrl.cmasi import RemoveEntities
from afrl.cmasi import FlightDirectorAction
from afrl.cmasi import WeatherReport
from afrl.cmasi import FollowPathCommand
from afrl.cmasi import PathWaypoint
from afrl.cmasi import StopMovementAction
from afrl.cmasi import WaypointTransfer
from afrl.cmasi import PayloadStowAction
from afrl.cmasi.perceive import EntityPerception
from afrl.cmasi.perceive import TrackEntityAction
from afrl.cmasi.perceive import TrackEntityTask
from afrl.cmasi.searchai import HazardZone
from afrl.cmasi.searchai import HazardZoneDetection
from afrl.cmasi.searchai import HazardZoneEstimateReport
from afrl.cmasi.searchai import RecoveryPoint
from afrl.cmasi.searchai import HazardZoneChangeCommand
from afrl.cmasi.searchai import HazardSensorConfiguration
from afrl.cmasi.searchai import HazardSensorState


s = socket.socket()
host = socket.gethostname()
port = 11041
s.connect((host, port))
buf = bytearray()

#Pack AbstractGeometry
obj = AbstractGeometry.AbstractGeometry()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack KeyValuePair
obj = KeyValuePair.KeyValuePair()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Location3D
obj = Location3D.Location3D()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadAction
obj = PayloadAction.PayloadAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadConfiguration
obj = PayloadConfiguration.PayloadConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadState
obj = PayloadState.PayloadState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VehicleAction
obj = VehicleAction.VehicleAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Task
obj = Task.Task()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SearchTask
obj = SearchTask.SearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AbstractZone
obj = AbstractZone.AbstractZone()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityConfiguration
obj = EntityConfiguration.EntityConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack FlightProfile
obj = FlightProfile.FlightProfile()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AirVehicleConfiguration
obj = AirVehicleConfiguration.AirVehicleConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityState
obj = EntityState.EntityState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AirVehicleState
obj = AirVehicleState.AirVehicleState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Wedge
obj = Wedge.Wedge()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AreaSearchTask
obj = AreaSearchTask.AreaSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CameraAction
obj = CameraAction.CameraAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CameraConfiguration
obj = CameraConfiguration.CameraConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimballedPayloadState
obj = GimballedPayloadState.GimballedPayloadState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack CameraState
obj = CameraState.CameraState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Circle
obj = Circle.Circle()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalAngleAction
obj = GimbalAngleAction.GimbalAngleAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalConfiguration
obj = GimbalConfiguration.GimbalConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalScanAction
obj = GimbalScanAction.GimbalScanAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalStareAction
obj = GimbalStareAction.GimbalStareAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GimbalState
obj = GimbalState.GimbalState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack GoToWaypointAction
obj = GoToWaypointAction.GoToWaypointAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack KeepInZone
obj = KeepInZone.KeepInZone()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack KeepOutZone
obj = KeepOutZone.KeepOutZone()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack LineSearchTask
obj = LineSearchTask.LineSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack NavigationAction
obj = NavigationAction.NavigationAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack LoiterAction
obj = LoiterAction.LoiterAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack LoiterTask
obj = LoiterTask.LoiterTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Waypoint
obj = Waypoint.Waypoint()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack MissionCommand
obj = MissionCommand.MissionCommand()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack MustFlyTask
obj = MustFlyTask.MustFlyTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack OperatorSignal
obj = OperatorSignal.OperatorSignal()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack OperatingRegion
obj = OperatingRegion.OperatingRegion()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AutomationRequest
obj = AutomationRequest.AutomationRequest()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PointSearchTask
obj = PointSearchTask.PointSearchTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Polygon
obj = Polygon.Polygon()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack Rectangle
obj = Rectangle.Rectangle()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RemoveTasks
obj = RemoveTasks.RemoveTasks()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack ServiceStatus
obj = ServiceStatus.ServiceStatus()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack SessionStatus
obj = SessionStatus.SessionStatus()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VehicleActionCommand
obj = VehicleActionCommand.VehicleActionCommand()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VideoStreamAction
obj = VideoStreamAction.VideoStreamAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VideoStreamConfiguration
obj = VideoStreamConfiguration.VideoStreamConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack VideoStreamState
obj = VideoStreamState.VideoStreamState()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack AutomationResponse
obj = AutomationResponse.AutomationResponse()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RemoveZones
obj = RemoveZones.RemoveZones()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RemoveEntities
obj = RemoveEntities.RemoveEntities()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack FlightDirectorAction
obj = FlightDirectorAction.FlightDirectorAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack WeatherReport
obj = WeatherReport.WeatherReport()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack FollowPathCommand
obj = FollowPathCommand.FollowPathCommand()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PathWaypoint
obj = PathWaypoint.PathWaypoint()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack StopMovementAction
obj = StopMovementAction.StopMovementAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack WaypointTransfer
obj = WaypointTransfer.WaypointTransfer()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack PayloadStowAction
obj = PayloadStowAction.PayloadStowAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack EntityPerception
obj = EntityPerception.EntityPerception()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TrackEntityAction
obj = TrackEntityAction.TrackEntityAction()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack TrackEntityTask
obj = TrackEntityTask.TrackEntityTask()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack HazardZone
obj = HazardZone.HazardZone()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack HazardZoneDetection
obj = HazardZoneDetection.HazardZoneDetection()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack HazardZoneEstimateReport
obj = HazardZoneEstimateReport.HazardZoneEstimateReport()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack RecoveryPoint
obj = RecoveryPoint.RecoveryPoint()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack HazardZoneChangeCommand
obj = HazardZoneChangeCommand.HazardZoneChangeCommand()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack HazardSensorConfiguration
obj = HazardSensorConfiguration.HazardSensorConfiguration()
buf.extend(LMCPFactory.packMessage(obj, True))
#Pack HazardSensorState
obj = HazardSensorState.HazardSensorState()
buf.extend(LMCPFactory.packMessage(obj, True))


s.send(buf)


