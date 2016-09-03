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

@txros.util.cancellableInlineCallbacks
def main(navigator):
  #shooterFire = rospy.ServiceProxy("/shooter/fire", std_srvs.srv.Trigger)
  resp = yield navigator.vision_request("get_shape")
  while not resp.found:
    yield navigator.move.yaw_left(0.15).go();
    resp = yield navigator.vision_request("get_shape")

  resp = yield navigator.vision_request("get_shape")
  error = float(resp.symbol.CenterX)/resp.symbol.img_width - 0.5
  while abs(error) > error_threshold:
      try:
          resp = yield navigator.vision_request("get_shape")
          error = float(resp.symbol.CenterX)/resp.symbol.img_width - 0.5
          print error
          if error < 0:
              yield navigator.move.yaw_left(0.1).go()
              print "Turning Left"
          elif error > 0:
              yield navigator.move.yaw_right(0.1).go()
              print "Turning Right"
      except:
          print "Erorr excepted"
  yield navigator.move.go()
  #shooterFire()
