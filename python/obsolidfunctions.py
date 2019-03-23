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


elif isinstance(lmcpObject, EntityState):
            self.__currentEntityState[lmcpObject.ID] = lmcpObject
            print(self.__currentEntityState)
            print('entity state')

self.__timethreshold = [1.3,2.5]

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


# for vid in range(1,noOfUAVs):
                #     vState[vid],sensorStateFront[vid] = smpleHazardDetector.getAirVeicleState(vid)
                #     smpleHazardDetector.sendGimbleCommand(vid,-sensorRotationAngle,-45)
                # time.sleep(dt)
                # for vid in range(1,noOfUAVs):
                #     vState[vid],sensorStateLeft[vid] = smpleHazardDetector.getAirVeicleState(vid)
                #     smpleHazardDetector.sendGimbleCommand(vid,sensorRotationAngle,-45)
                # time.sleep(dt)
                # for vid in range(1,noOfUAVs):
                #     vState[vid],sensorStateRight[vid] = smpleHazardDetector.getAirVeicleState(vid)
                #     smpleHazardDetector.sendGimbleCommand(vid,0,-45)
                # for vid in range(1,noOfUAVs):
                #     if smpleHazardDetector.getSurveyStatus(vid):
                #         if not sensorStateLeft[vid] and not sensorStateFront[vid] and not sensorStateRight[vid]:
                #             print('hard left')
                #             headingangle = (vState[vid].Heading - 90)
                #             headingangle = headingangle if headingangle>0 else headingangle+360
                #             smpleHazardDetector.sendHeadingAngleCommandwithcurrentlocation(vid,headingangle,vState[vid].Location)
                #         elif (sensorStateLeft[vid] and sensorStateFront[vid] and not sensorStateRight[vid]):
                #             print('soft right')
                #             headingangle = (vState[vid].Heading + 45) % 360
                #             smpleHazardDetector.sendHeadingAngleCommandwithcurrentlocation(vid,headingangle,vState[vid].Location)
                #         elif (not sensorStateLeft[vid] and sensorStateFront[vid] and not sensorStateRight[vid]):
                #             print('right')
                #             headingangle = (vState[vid].Heading + 90) % 360
                #             smpleHazardDetector.sendHeadingAngleCommandwithcurrentlocation(vid,headingangle,vState[vid].Location)
                #         elif sensorStateLeft[vid] and sensorStateFront[vid] and sensorStateRight[vid]:
                #             print('hard right')
                #             headingangle = (vState[vid].Heading + 135) % 360
                #             smpleHazardDetector.sendHeadingAngleCommandwithcurrentlocation(vid,headingangle,vState[vid].Location)
                #         else:
                #             print('straight')

                # time.sleep(5*dt)

                    # if sensorState:
                    #     print(vid,sensorState)
                    #     smpleHazardDetector.sendGimbleCommand(vid,-45,-45)
                    #     time.sleep(0.5)
                    #     vState,sensorState = smpleHazardDetector.getAirVeicleState(vid)
                    #     if sensorState:
                    #         smpleHazardDetector.sendGimbleCommand(vid,0,-45)
                    #         headingangle = (vState.Heading + 45) % 360
                    #         smpleHazardDetector.sendHeadingAngleCommandwithcurrentlocation(vid,headingangle,vState.Location)
            # time.sleep(0.1)



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


def checkSurveyStatus(self,vstate):
        airVehicleState = vstate
        veicleid = vstate.ID
        veicleLocation = vstate.Location
        if veicleid in self.__uavsInSarvey and self.__uavsInSarvey[veicleid]:
            #self.surveyStrategy(veicleid,airVehicleState,veicleLocation)
            self.surveyNewStrategy(airVehicleState)
        elif not veicleid in self.__uavisHeadingtoSurveylocation:
            zid = self.getZoneIdLocation(veicleLocation)
            if self.__firezoneHintLocation:
                if zid in self.__firezoneHintLocation:
                    self.sendGimbleCommand(veicleid,0,-45)
                    self.sendWaypoint(veicleid,veicleLocation,self.__firezoneHintLocation[zid])
                    self.__uavisHeadingtoSurveylocation[veicleid] = True
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
                                if not vid in self.__uavsInSearch and not vid in self.__uavisHeadingtoSurveylocation and not vid in self.__uavsInSarvey:
                                    loc = self.__firezoneHintLocation[zid]
                                    d = self.getdistance(loc,self.__currentLocationofUAV[vid])
                                    if d < mind:
                                        mind = d
                                        minLoc = loc
                                        minvid = vid
                            if mind != 10e20:
                                self.sendGimbleCommand(minvid,0,-45)
                                self.sendWaypoint(minvid,self.__currentLocationofUAV[minvid],minLoc)
                                self.__uavisHeadingtoSurveylocation[minvid] = True
                                self.__UAVSurvayingZoneId[minvid] = zid
                                 
                                self.__NoofUAVinZone[zid-1] += 1
                                if self.__NoofUAVinZone[zid-1]%2 != 0:
                                    self.__veicleStrategiId[minvid-1] = 1


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