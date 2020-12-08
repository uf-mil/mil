'''this is an abstract class that all sensor spaces must inherit from so they can be
interfaces with by UnifiedPerception
'''

import rospy

from . import *


def SensorSpaceFactory(sensor_type):
  return eval(sensor_type)





