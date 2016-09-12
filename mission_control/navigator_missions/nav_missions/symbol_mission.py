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

from twisted.internet import defer

error_threshold = 0.1  # how small center error can be to be considered centered
# what proportion of the image width should be taken by the symbol
width_proportion = 0.30
# how small width error can be to be considered correct distance
width_error_threshold = 0.2

def bounding_rect(points):
    maxX = 0
    minX = 2000
    maxY = 0
    minY = 2000
    for i in range(len(points)):
        if(points[i].x > maxX):
            maxX = points[i].x
        if(points[i].y > maxY):
            maxY = points[i].y
        if(points[i].x < minX):
            minX = points[i].x
        if(points[i].y < minY):
            minY = points[i].y
    return np.array([maxX, maxY, minX, minY])


def area_of_rect(rect):
    return (rect[1] - rect[2]) * (rect[1] - rect[3])


@txros.util.cancellableInlineCallbacks
def main(navigator):
  shooterLoad = rospy.ServiceProxy("/shooter/load", std_srvs.srv.Trigger)
  shooterFire = rospy.ServiceProxy("/shooter/fire", std_srvs.srv.Trigger)
  
  resp = yield navigator.vision_request("get_shape")
  while not (resp.found and resp.symbol.img_width != 0):
    yield navigator.move.yaw_left(0.15).go();
    resp = yield navigator.vision_request("get_shape")

  print "Found"
  print "Loading"
  shooterLoad()

  resp = yield navigator.vision_request("get_shape")
  center = float(resp.symbol.CenterX) / resp.symbol.img_width
  error = center - 0.5
  while abs(error) > error_threshold:
    if error < 0:
        print "Turning Left"
        yield navigator.move.yaw_left(0.1).go()
    elif error > 0:
        print "Turning Right"
        yield navigator.move.yaw_right(0.1).go()
    resp = yield navigator.vision_request("get_shape")
    center = float(resp.symbol.CenterX) / resp.symbol.img_width
    error = center - 0.5
    print "Center Proportion: ", center
  print "Centerted"
  
  resp = yield navigator.vision_request("get_shape")
  rect = bounding_rect(resp.symbol.points)
  width = (rect[0] - rect[2]) / resp.symbol.img_width
  width_error = width - width_proportion
  while abs(width_error) > width_error_threshold:
    if width_error < 0:
      print "Moving Towards"
      yield navigator.move.right(1).go()
      #yield navigator.move.forward(1).go()
    elif width_error > 0:
      print "Moving Away"
      yield navigator.move.left(1).go()
      #yield navigator.move.backward(1).go()
    resp = yield navigator.vision_request("get_shape")
    rect = bounding_rect(resp.symbol.points)
    width = (rect[0] - rect[2]) / resp.symbol.img_width
    width_error = width - width_proportion
    print "Width Proportion", width

    
  print "Correct Distance"
  yield navigator.move.go()
  
  print "Shooting"
  shooterFire()
