



# delta = 1
# delxp = 0
# delxn = 0
# for i in range(4):
    # if i%2==0:
        # delxp += delta
        # x = delxp
    # else:
        # delxn -= delta
        # x = delxn
    # delyp = 0
    # delyn = 0
    # for j in range(4):
        # if j%2==0:
            # delyp += delta
            # y = delyp
        # else:
            # delyn -= delta
            # y = delyn
        # print(x,y)
        
# minidel = 1
# maxdel = 2
# x = 0
# y = 0
# for i in range(6):
#     if i%2 == 0:
#         if y == 0:
#             option = 1
#         else:
#             option = 2
#         for j in range(6):
#             #if j != 5:
#             if option == 1:
#                 y += minidel
#             else:
#                 y -= minidel
#             print(x,y)
#     else:
#         for j in range(2):
#             x += minidel
#             print(x,y)
        



# w = 100
# h = 100

# wSeg = 2
# hSeg = 2

# dw = w/wSeg
# dh = h/hSeg
# currCenterx = -w/2
# currCentery = -h/2

# for ws in range(wSeg):
#     for hs in range(hSeg):
#         zoneid = ws*hSeg+hs + 1
#         if zoneid%2==0:
#             print('region:',zoneid)
#             print(currCenterx,currCentery,'->',currCenterx+dw,currCentery+dh)
#             print(currCenterx+dw,currCentery+dh,'->',currCenterx,currCentery+dh)
#             print(currCenterx,currCentery+dh,'->',currCenterx+dw,currCentery)
#             print(currCenterx+dw,currCentery,'->',currCenterx,currCentery)
#         else:
#             print('region:',zoneid)
#             print(currCenterx,currCentery,'->',currCenterx+dw,currCentery+dh)
#             print(currCenterx+dw,currCentery+dh,'->',currCenterx+dw,currCentery)
#             print(currCenterx+dw,currCentery,'->',currCenterx,currCentery+dh)
#             print(currCenterx,currCentery+dh,'->',currCenterx,currCentery)
#         currCentery += dh
#     currCenterx += dw
#     currCentery = -h/2


# if not 0:
#     print('zero')
# if not 1:
#     print('one')

# x=[1,1,1,3,4,4,4]
# print(x.count(max(x)))


import matplotlib.pyplot as plt
from math import atan2,cos,sin,radians,degrees


x = 4
y = 5

theta = atan2(y,x)
print(theta)

d = 10

xc = x + d*cos(radians(90-degrees(theta)))
yc = y - d*sin(radians(90-degrees(theta)))

plt.scatter([x,xc],[y,yc])





