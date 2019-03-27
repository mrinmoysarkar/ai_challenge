



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


# import matplotlib.pyplot as plt
# from math import atan2,cos,sin,radians,degrees


# x = 4
# y = 5

# theta = atan2(y,x)
# print(theta)

# d = 10

# xc = x + d*cos(radians(90-degrees(theta)))
# yc = y - d*sin(radians(90-degrees(theta)))

# plt.scatter([x,xc],[y,yc])

# print(max(5,6,9))
# x = [2,3,4]
# y = [7,8]

# x = y + x
# print(x)
# print(y)

# globalList = {1:[2,3],2:[1],3:[1]}

# def mergeThetree(linkdickey,treeLeaf):
#     if treeLeaf:
#         if linkdickey in treeLeaf:
#             print(treeLeaf)
#             return treeLeaf
#     for value in globalList[linkdickey]:
#         mergeThetree(value,treeLeaf + [linkdickey])




# print(mergeThetree(2,[]))

import numpy as np

for i in range(10):
    x=np.array([[1,2],[2,2],[3,3],[4,4],[4,5],[4,6]])
    labels = np.array([1,2,1,2,2,1])
    condition = labels==1
    # print(x[condition])



# def callfunc(lst):
#     lst += [[2,3]]
#     print(lst)

# lst = [[1,1],[2,2]]

# callfunc(lst[:])
# print(lst)
from math import tan,radians

x = 3
y = 5
a = 7
b = 8
eps = 10e-5
theta = radians(40+eps)
tn = tan(theta)
x1 = x + y*tn
y1 = y + x/tn

xs1 = (x+x1)/2
ys1 = y/2

xs2 = x/2
ys2 = (y+y1)/2

x2 = xs2+(b-ys2)/tn
y2 = ys1+(a-xs1)*tn

xs3 = (xs2+x2)/2
ys3 = (ys2+b)/2

xs4 = (xs1+a)/2
ys4 = (ys1+y2)/2

import sys

print(sys.version)
print(sys.executable)



# d = {1:[[1,2],[2,3]],2:[[1,2],[2,3]],3:[[1,2],[2,3]]}

# print(d)
# del d[2]
# print(d)

# # from sets import Set
# x = np.array([[1,2],[2,3]])
# y = np.array([[2,1]])

# #test y is a subset of x or not
# counter = 0
# for i in range(len(y)):
#     for j in range(len(x)):
#         if sum(x[j] == y[i])==len(y[i]):
#             counter += 1
# print(counter == len(y))

# # print(np.unique(x))
# # condition = np.isin(np.array(d[1]),np.array([[1,2]]))

# # print(y<=x)

# print(np.mean(x,axis=0))

datagrid = np.random.randint(0,2,[100,100])

ic = np.random.randint(0,100)
jc = np.random.randint(0,100)

w=3
h=3

x=np.array([[1,2,3],[4,5,6]])
print(x[:,0:2])
