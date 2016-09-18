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
import time
from twisted.internet import defer

class DetectDeliverMission:
  shooterLoad = rospy.ServiceProxy("/shooter/load", std_srvs.srv.Trigger)
  shooterFire = rospy.ServiceProxy("/shooter/fire", std_srvs.srv.Trigger)
  center_error_threshold = 0.1
  width_proportion = 0.15 #Desired proportion of frame width symbol is present in
  width_error_threshold = 0.03
  resp = None
  def __init__(self,navigator)
    self.center_error = 1;
    self.width_error = 1;
    self.navigator = navigator
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
  @txros.util.cancellableInlineCallbacks
  def isFound():
    self.resp = yield self.navigator.vision_request("get_shape")
    return self.resp.found and self.resp.symbol.img_width != 0
  @txros.util.cancellableInlineCallbacks
  def isCentered():
    if (!self.isFound()) return False
    center = float(self.resp.symbol.CenterX) / self.resp.symbol.img_width
    self.center_error = center - 0.5
    print "Center ", center
    return abs(self.center_error) < self.error_threshold:
  @txros.util.cancellableInlineCallbacks
  def isCorrectDistance():
    if (!self.isFound()) return False
    rect = self.bounding_rect(self.resp.symbol.points)
    width = (rect[0] - rect[2]) / self.resp.symbol.img_width
    self.width_error = width - self.width_proportion
    print "Width ", width
    return abs(self.width_error) < self.width_error_threshold:
  @txros.util.cancellableInlineCallbacks
  def shootAllBalls():
    for i in range(0,3):
      time.sleep(3)
      self.shooterLoad()
      time.sleep(5)
      self.shooterFire()
  @txros.util.cancellableInlineCallbacks
  def findAndShoot():
    if not self.isFound():
      self.navigator.move.yaw_left(360, "deg").go()
      while not self.isFound()
    print "Found"
    yield self.navigator.move.go() #Stop moving once shape seen
    if not self.isCentered()
       if self.center_error < 0:
          print "Turning Left"
          self.navigator.move.yaw_left(180,"deg").go()
      elif self.center_error > 0:
          print "Turning Right"
          self.navigator.move.yaw_right(180,"deg").go()
      while not self.isCentered()
    print "Centered"
    yield self.navigator.move.go() #Stop moving once shape seen
    if not self.isCorrectDistance()
      if self.width_error < 0:
        print "Moving Towards"
        self.navigator.move.right(50).go()
      elif self.width_error > 0:
        print "Moving Away"
        self.navigator.move.left(50).go()
      while not self.isCorrectDistance()
    yield self.navigator.move.go() #Stop moving once centered
    print "Correct Distance"
    yield self.navigator.move.go()
    
    print "Shooting"
    self.shootAllBalls()

@txros.util.cancellableInlineCallbacks
def main(navigator):
  mission = DetectDeliverMission(navigator)
