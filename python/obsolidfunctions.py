# def getAltitudeLatLon(self,lat,lon):
        # row = int(round((lat - self.__latBias)*self.__delAlevation))
        # col = int(round((lon - self.__lonBias)*self.__delAlevation))
        # sz = self.__altidata.shape
        # if row >= sz[0]:
            # row = sz[0]-1
        # if col >= sz[1]:
            # col = sz[1]-1
        # # print(self.__altidata[row][col])
        # return self.__altidata[row][col]
    
    # def getAltitude(self,location):
        # row = int(round((location.get_Latitude() - self.__latBias)*self.__delAlevation))
        # col = int(round((location.get_Longitude() - self.__lonBias)*self.__delAlevation))
        # sz = self.__altidata.shape
        # # print(sz)
        # if row >= sz[0]:
            # row = sz[0]-1
        # if col >= sz[1]:
            # col = sz[1]-1
        # # print(self.__altidata[row][col])
        # return self.__altidata[row][col]
		
		
		
    # def turn(self,veicleid,left):
        # delang = 15
        # if left == 0:
            # if self.__currentHeadingAngleUAV[veicleid] < delang:
                # self.__currentHeadingAngleUAV[veicleid] = 360 - delang
            # else:
                # self.__currentHeadingAngleUAV[veicleid] -= delang
        # elif left == 1:
            # self.__currentHeadingAngleUAV[veicleid] = (self.__currentHeadingAngleUAV[veicleid]+delang)%360
        # self.sendHeadingAngleCommand(veicleid,self.__currentHeadingAngleUAV[veicleid])
        
    # def turn(self,veicleid):
        # headingAngle = 0
        # if veicleid == 1:
            # if self.__currentHeadingAngleUAV[veicleid] == 0:
                # self.__currentHeadingAngleUAV[veicleid] = 1
                # if self.__keepoutOption[veicleid-1] == 0:
                    # headingAngle = 90
                # else:
                    # headingAngle = 270
            # elif self.__currentHeadingAngleUAV[veicleid] == 1:
                # self.__currentHeadingAngleUAV[veicleid] = 2
                # headingAngle = 180
            # elif self.__currentHeadingAngleUAV[veicleid] == 2:
                # self.__currentHeadingAngleUAV[veicleid] = 3
                # if self.__keepoutOption[veicleid-1] == 0:
                    # headingAngle = 90
                # else:
                    # headingAngle = 270
            # elif self.__currentHeadingAngleUAV[veicleid] == 3:
                # self.__currentHeadingAngleUAV[veicleid] = 0
                # headingAngle = 0
        # elif veicleid  == 2 :
            # if self.__currentHeadingAngleUAV[veicleid] == 0:
                # self.__currentHeadingAngleUAV[veicleid] = 1
                # if self.__keepoutOption[veicleid-1] == 0:
                    # headingAngle = 270
                # else:
                    # headingAngle = 90
            # elif self.__currentHeadingAngleUAV[veicleid] == 1:
                # self.__currentHeadingAngleUAV[veicleid] = 2
                # headingAngle = 180
            # elif self.__currentHeadingAngleUAV[veicleid] == 2:
                # self.__currentHeadingAngleUAV[veicleid] = 3
                # if self.__keepoutOption[veicleid-1] == 0:
                    # headingAngle = 270
                # else:
                    # headingAngle = 90
            # elif self.__currentHeadingAngleUAV[veicleid] == 3:
                # self.__currentHeadingAngleUAV[veicleid] = 0
                # headingAngle = 0
        # elif veicleid == 3:
            # if self.__currentHeadingAngleUAV[veicleid] == 0:
                # self.__currentHeadingAngleUAV[veicleid] = 1
                # if self.__keepoutOption[veicleid-1] == 0:
                    # headingAngle = 180
                # else:
                    # headingAngle = 0
            # elif self.__currentHeadingAngleUAV[veicleid] == 1:
                # self.__currentHeadingAngleUAV[veicleid] = 2
                # headingAngle = 270
            # elif self.__currentHeadingAngleUAV[veicleid] == 2:
                # self.__currentHeadingAngleUAV[veicleid] = 3
                # if self.__keepoutOption[veicleid-1] == 0:
                    # headingAngle = 180
                # else:
                    # headingAngle = 0
            # elif self.__currentHeadingAngleUAV[veicleid] == 3:
                # self.__currentHeadingAngleUAV[veicleid] = 0
                # headingAngle = 90
        # elif veicleid == 4:
            # if self.__currentHeadingAngleUAV[veicleid] == 0:
                # self.__currentHeadingAngleUAV[veicleid] = 1
                # if self.__keepoutOption[veicleid-1] == 0:
                    # headingAngle = 0
                # else:
                    # headingAngle = 180
            # elif self.__currentHeadingAngleUAV[veicleid] == 1:
                # self.__currentHeadingAngleUAV[veicleid] = 2
                # headingAngle = 270
            # elif self.__currentHeadingAngleUAV[veicleid] == 2:
                # self.__currentHeadingAngleUAV[veicleid] = 3
                # if self.__keepoutOption[veicleid-1] == 0:
                    # headingAngle = 0
                # else:
                    # headingAngle = 180
            # elif self.__currentHeadingAngleUAV[veicleid] == 3:
                # self.__currentHeadingAngleUAV[veicleid] = 0
                # headingAngle = 90
        

        # print('turning',veicleid,' heading', headingAngle)
        # self.sendHeadingAngleCommand(veicleid,headingAngle)
        
        
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