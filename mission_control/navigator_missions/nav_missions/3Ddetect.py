#!/usr/bin/env python
import txros
import rospy
import std_srvs.srv
from navigator_msgs.msg import DockShape
import numpy as np
from geometry_msgs.msg import Point
import sensor_msgs.msg
import nav_msgs.msg
from navigator_tools import rosmsg_to_numpy
import cv2

square = np.array([[0,0,0],[0,1,0],[1,0,0],[1,1,0]]).astype(np.float32)
#~ K = np.array([[400,0,200],[0,400,200],[0,0,1]]).astype(np.float32)
K = np.array([[1412.201315555317, 0.0, 958.1808049309507],[ 0.0, 1407.186287430534, 595.3024122623201], [0.0, 0.0, 1.0]]).astype(np.float32)

dist = np.array([-0.1639911928403439, 0.09983660562976353, -0.0006125709787662186, 0.0005491094123992009, 0.0])


def draw(img, corner, imgpts):
    #~ corner = np.array([500,200])
    
    cv2.line(img, tuple(corner), tuple(imgpts[0].ravel()), (255,0,0), 5)
    cv2.line(img, tuple(corner), tuple(imgpts[1].ravel()), (0,255,0), 5)
    cv2.line(img, tuple(corner), tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

#~ def getBoundingTrapCross(points):
    #~ for i in range(len(points)):
def getCenter(points):
    cx = 0
    cy = 0
    for i in range(len(points)):
        cx += points[i][0]
        cy += points[i][1]
    cx = cx/len(points)
    cy = cy/len(points)
    return np.array([cx,cy])
        

def getMax(points, what):
    max_val = 0
    second_max = 0
    if(points[0][what] > points[1][what]):
        second_max = points[1];
        max_val = points[0];
    else:
        second_max = points[0];
        max_val = points[1];
    
    for i in range(2,len(points)):
        if(points[i][what] >= max_val[what]):
            second_max=max_val;
            max_val=points[i];          
        elif(points[i][what] > second_max[what]):
            second_max=points[i];
    
    return max_val[:-1], second_max[:-1]

def getMin(points, what):
    min_val = 2000
    second_min = 2000
    if(points[0][what] < points[1][what]):
        second_min = points[1];
        min_val = points[0];
    else:
        second_min = points[0];
        min_val = points[1];
    
    for i in range(2,len(points)):
        if(points[i][what] <= min_val[what]):
            second_min=min_val;
            min_val=points[i];          
        elif(points[i][what] < second_min[what]):
            second_min=points[i];
    
    return min_val[:-1], second_min[:-1]

def perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

def seg_intersect(a1,a2, b1,b2):
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )
    return (num / denom.astype(float))*db + b1

def getPlanarBoundingRect(points):
    bigY = getMax(points, 1)
    bigX = getMax(points, 0)
    
    smallY = getMin(points,1)
    smallX = getMin(points,0)
    
    point1=tuple(seg_intersect(bigX[0], bigX[1], bigY[0], bigY[1]).astype(int))
    point2=tuple(seg_intersect(smallX[0], smallX[1], smallY[0], smallY[1]).astype(int))
    point3=tuple(seg_intersect(smallX[0], smallX[1], bigY[0], bigY[1]).astype(int))
    point4=tuple(seg_intersect(bigX[0], bigX[1], smallY[0], smallY[1]).astype(int))

    return point1, point2, point3, point4
    

def boundingRect(points):
    maxX = 0
    minX = 2000
    maxY = 0
    minY = 2000
    for i in range(len(points)):
        if(points[i][0] > maxX):
            maxX = points[i][0];
        if(points[i][1] > maxY):
            maxY = points[i][1];
        if(points[i][0] < minX):
            minX = points[i][0];
        if(points[i][1] < minY):
            minY = points[i][1];
    return np.array([[maxX, maxY], [minX, minY],[maxX,minY],[minX,maxY]])

@txros.util.cancellableInlineCallbacks
def main(navigator):
    img = cv2.imread("/home/daniel/navigator_ws/bluecross.png") 
    cv2.imshow("img",img)
    
    #~ resp = yield navigator.vision_request("get_shape")

    
    points = np.array([[598.0,
    459.0,
    0.0],
    
    [597.0,
    484.0,
    0.0],
     
    [569.0,
    489.0,
    0.0],
    
    [570.0,
    512.0,
    0.0],
    
    [599.0,
    513.0,
    0.0],
     
    [604.0,
    541.0,
    0.0],
     
    [629.0,
    540.0,
    0.0],
     
    [628.0,
    515.0,
    0.0],
     
    [655.0,
    510.0,
    0.0],
    
    [654.0,
    487.0,
    0.0],
     
    [627.0,
    485.0,
    0.0],
     
    [625.0,
    458.0,
    0.0]])
    #~ while(resp.found):
        #~ points = resp.symbol.points
        
    rectp = getPlanarBoundingRect(points)

    rvecs, tvecs, inliers = cv2.solvePnPRansac(square, np.array(rectp).astype(np.float32), K, dist)
    print "Rotation vec: ", rvecs
    print "Traslation vec: ", tvecs
    print "Inliers", inliers
    
    axis = np.float32([[1,0,0], [0,1,0], [0,0,-1]]).reshape(-1,3)
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, K, dist)
    
    center = getCenter(points).astype(int)
    img2 = draw(img, tuple(center), imgpts)

    for i in range(len(points)):
        cv2.circle(img2, tuple(points[i][:-1].astype(int)), 5, (0,255,0), -1)
    
    rectp = getPlanarBoundingRect(points)    

    cv2.line(img2,rectp[0],rectp[2],(255,0,0),5)
    cv2.line(img2,rectp[0],rectp[3],(255,0,0),5)
    cv2.line(img2,rectp[1],rectp[2],(255,0,0),5)
    cv2.line(img2,rectp[1],rectp[3],(255,0,0),5)

    cv2.imshow("img2",img2)
    #~ resp = yield navigator.vision_request("get_shape")

    cv2.waitKey()
    yield 
