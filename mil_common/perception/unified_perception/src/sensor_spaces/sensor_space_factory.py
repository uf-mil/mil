'''this is an abstract class that all sensor spaces must inherit from so they can be
interfaces with by UnifiedPerception
'''

import rospy

from camera import Camera


def SensorSpaceFactory(sensor_type):
  space_mapping = {
    'Camera': Camera,
  }
  return space_mapping[sensor_type]





