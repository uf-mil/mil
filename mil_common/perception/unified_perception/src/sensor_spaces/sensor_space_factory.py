from . import *

def SensorSpaceFactory(sensor_type):
  return eval(sensor_type)
