#!/usr/bin/env python

'''this is a ros server that is made to dynamically save sensor data for run time world modeling
* the data that is saved is based on statistical ditrubtions of percieved objects from perception algorithms
*  a dimention is applied on top to each sensor space which is:
  the sum of all unique pdfs which are projectable onto that sensor space
* what is a "unique pdf"?
  when a pdf is signaled, an id (which is a str) is associated with it. If there are multiple reports of a pdf
    with the same pdf within a quamtum, they are multplied together to get an effective intersection of the most likely distribution on the percived object.
'''
import rospy
from rospy.numpy_msg import numpy_msg
from mil_msgs.msg import GaussianDistribution, GaussianDistribution2D
from threading import Lock, Thread

from sensor_spaces.sensor_space_factory import SensorSpaceFactory


class UnifiedPerceptionServer:
  def __init__(self):
    self.inited = False
    self.id = 0
    rospy.init_node('unified_perception_server')
    # create a sensor space instance for each sensor
    self.sensors = {}
    self.min_score = rospy.get_param('min_score')
    sensors = rospy.get_param('sensors')
    for i in sensors:
      self.sensors[i[1]] = SensorSpaceFactory(i[0])(i[1])
    self.distributions = {}
    gaussian_sub = rospy.Subscriber(rospy.get_param('gaussian_topic'),
                                 numpy_msg(GaussianDistribution),
                                 self.gaussian_cb, 10)
    gaussian_2d_sub = rospy.Subscriber(rospy.get_param('gaussian_2d_topic'),
                                    numpy_msg(GaussianDistribution2D),
                                    self.gaussian_cb, 10)
    self.timer = rospy.Timer(rospy.Duration(rospy.get_param('quantum')), self.quantum_cb)
    self.lock = Lock()
    self.inited = True


  def gaussian_cb(self, msg, count):
    if self.inited != True:
      return
    self.lock.acquire()
    # project from sensor space to the world space
    dist = self.sensors[msg.sensor_name].project_to_world_frame(msg)
    # add to the distributions dict
    if msg.id != '':
      if msg.id in self.distributions:
        self.distributions[msg.id] *= dist
      else:
        self.distributions[msg.id] = dist
    else:
      self.distrubutions[str(self.id)] = dist
      self.id += 1
    self.lock.release()
    return

  def quantum_cb(self, event):
    self.lock.acquire()
    # may need to remove old gaussians that are not valid anymore
    world_model = {}
    world_model['gaussians'] = self.distributions
    world_model['data'] = {}
    # apply gaussians to the data in the buffers
    threads = []
    for sensor in self.sensors.values():
      threads.append(Thread(target=sensor.apply_distributions,
                            args=[self.min_score, self.distributions]))
      threads[-1].start()
    for t in threads:
      t.join()

    for name, sensor in self.sensors.items():
      world_model['data'][name] = sensor.msg_buffer
      sensor.clear_msg_buffer()
    # publish that data along with the gaussians as the world model
    # publish gaussians in RVIZ as poses with cov
    markers = []
    for i in world_model['gaussians'].values():
      markers.append(i.to_rviz_marker())
      
    self.lock.release()


if __name__ == '__main__':
  a = UnifiedPerceptionServer()
  rospy.spin()
