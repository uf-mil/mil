#!/usr/bin/env python
import txros
import rospy
import std_srvs.srv
from navigator_msgs.msg import DockShape
import numpy
from geometry_msgs.msg import Point
import sensor_msgs.msg
import nav_msgs.msg

import cv2

from twisted.internet import defer



error_threshold = 0.01
move_per_iteration = 1.0;
#~ square = numpy.array([Point(-1,-1,0), Point(-1,1,0), Point(1,1,0), Point(1,-1,0)])
#~ square = numpy.array([-1,-1,0,-1,1,0,1,1,0,1,-1,0])
square = numpy.array([[0, 0, 0],[ 1, 0, 0], [0, 1, 0],[ 1, 1, 0]]).astype(numpy.float64)
#~ K = numpy.array(numpy.mat("1 0 0;0 1 0;0 0 1"))
K = numpy.array([[1,0,0],[0,1,0],[0,0,1]]).astype(numpy.float64)


def boundingRect(points):
    maxX = 0
    minX = 2000
    maxY = 0
    minY = 2000
    for i in range(len(points)):
        if(points[i].x > maxX):
            maxX = points[i].x;
        if(points[i].y > maxY):
            maxY = points[i].y;
        if(points[i].x < minX):
            minX = points[i].x;
        if(points[i].y < minY):
            minY = points[i].y;
    return numpy.array([maxX, maxY, minX, minY, minX, maxY, maxX, minY])
    #~ return [Point(minX, maxY, 0), Point(maxX, minY,0), Point(maxX, maxY,0), Point(minX, minY,0)]

def callback(data):
    #~ K = numpy.reshape(data.K, (3,3))
    print K
    
@txros.util.cancellableInlineCallbacks
def listener(navigator):
    camerainfoSub = yield navigator.nh.subscribe("/side_camera/camera_info", sensor_msgs.msg.CameraInfo)
    caminfo = yield b.get_next_message()
    #~ K = caminfo.K
    #print caminfo.K
    defer.returnValue(caminfo.K)
    #~ rospy.spin()

@txros.util.cancellableInlineCallbacks
def main(navigator):
    
    #~ shooterFire = rospy.ServiceProxy("/shooter/fire", std_srvs.srv.Trigger)
    #~ 
    #~ cameraInfo = rospy.ServiceProxy("/side_camera/camera_info", sensor_msgs.msg.CameraInfo)
    #~ K = yield listener(navigator)
    #~ yield navigator.nh.sleep(1)
    position, orientiation = navigator.pose
    
    print orientiation
    
    print K
    try:    
        resp = yield navigator.vision_request("get_shape")
        error = float(resp.symbol.CenterX)/resp.symbol.img_width - 0.5
        boundingRectPoints = numpy.reshape(boundingRect(resp.symbol.points), (4,2)).astype(numpy.float64)

        retval, rvec, tvec=cv2.solvePnP(square, boundingRectPoints, K, numpy.array([2,1,3,2]).astype(numpy.float64))

    
        rvec1, rjac = cv2.Rodrigues(rvec);

    
        p1 = boundingRectPoints[0]
        p1 = numpy.insert(p1, 2, 0)
    
        p2 = boundingRectPoints[1]
        p2 = numpy.insert(p2, 2, 0)
    
        p3 = boundingRectPoints[2]
        p3 = numpy.insert(p3, 2, 0)
    
        p4 = boundingRectPoints[3]
        p4 = numpy.insert(p4, 2, 0)

        pointA = numpy.dot(p1, rvec1) + numpy.matrix.transpose(tvec)
        pointB = numpy.dot(p2, rvec1) + numpy.matrix.transpose(tvec)
        pointC = numpy.dot(p3, rvec1) + numpy.matrix.transpose(tvec)
        pointD = numpy.dot(p4, rvec1) + numpy.matrix.transpose(tvec)
    

        cross= numpy.cross(pointA-pointB, pointA-pointC)
        cross = cross/numpy.linalg.norm(cross)
        print "Perpindicular: ", cross
 
        parallel = (pointA - pointB)/numpy.linalg.norm(pointA-pointB)
    
        print "Parrallel: ", parallel
    
    
        print "Test: ", numpy.dot(parallel[0],cross[0])
    
        print "Yaw: ", numpy.arctan2(parallel[0][0], parallel[0][1]) * 180 / 3.1415
        print error
    except:
        print "error excepted"
        error = 1
    

    #~ print boundingRectPoints[0]
    #~ print boundingRectPoints[1]
    yield
    #~ while abs(error) > error_threshold:
        #~ try:
            #~ resp = yield navigator.vision_request("get_shape")
            #~ error = float(resp.symbol.CenterX)/resp.symbol.img_width - 0.5
            #~ print error
            #~ if error < 0:
                #~ yield navigator.move.forward(move_per_iteration).go()
                #~ print "Moving Forward"
            #~ elif error > 0:
                #~ yield navigator.move.backward(move_per_iteration).go()
                #~ print "Moving Backward"
        #~ except:
            #~ print "Erorr excepted"
    #~ yield navigator.move.go()
    #~ shooterFire()
