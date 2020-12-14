#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2

from mil_msgs.msg import GaussianDistributionStamped
from sensor_space import SensorSpace

from distributions.gaussian import Gaussian

class Lidar(SensorSpace):
  def __init__(self, name):
    topic = rospy.get_param(name + '/point_cloud_topic')
    super(Lidar, self).__init__(name, topic, PointCloud2)
    self.sub = rospy.Subscriber(self.topic, numpy_msg(PointCloud2), self.callback)
    self.frame = rospy.get_param(self.name + '/frame_id')


  def project_to_world_frame(self, msg):
    if isinstance(msg, GaussianDistributionStamped):
      g = Gaussian(len(msg.distribution.mu), msg.distribution.mu,
                    msg.distribution.cov, msg.header.frame_id,
                   classification=msg.distribution.classification)
      return g.transform_to_frame(self.world_frame, self.listener)[0]
    else:
      raise Exception(self.name + ' was given ' + str(type(msg)) + ', which is not supported')


  def apply_distributions_to(self, msg, dists, min_score):
    #TODO
    if isinstance(msg, PointCloud2):
      pass
    else:
      raise Exception(self.name + ': was given a ' + str(type(msg)) + ' which is not supported')
    return


  def project_from_world_frame(self, dist):
    g = dist.transform_to_frame(self.frame, self.listener, time=rospy.Time.now())[0]
    return g
