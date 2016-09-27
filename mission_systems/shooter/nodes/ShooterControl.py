#!/usr/bin/env python
from Sabertooth2x12 import Sabertooth2x12
import roslib
import rospy
import actionlib

#import navigator_msgs.msg
from navigator_msgs.msg import ShooterDoAction

class ShooterFireServer():
  def __init__(self,filename):
    #self.motor_controller = Sabertooth2x12(filename)
    self.server = actionlib.SimpleActionServer('/shooter/fire', ShooterDoAction, self.execute, False)
    self.server.start()
  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('do_dishes_server')
  fireServer = ShooterFireServer("/dev/ttyUSB0")
  rospy.spin()
