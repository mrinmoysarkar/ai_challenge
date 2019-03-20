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