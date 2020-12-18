#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import numpy as np
from mil_msgs.msg import GaussianDistributionStamped
from sensor_space import SensorSpace
from nav_msgs.msg import Odometry
from distributions.gaussian import Gaussian


class Lidar(SensorSpace):
  def __init__(self, name):
    topic = rospy.get_param(name + '/point_cloud_topic')
    super(Lidar, self).__init__(name, topic, PointCloud2)
    self.sub = rospy.Subscriber(self.topic, numpy_msg(PointCloud2), self.callback)
    self.frame = rospy.get_param(self.name + '/frame_id')
    self.safety_radius = rospy.get_param(self.name + '/safety_radius')
    self.odom_topic = rospy.get_param(self.name + '/odom_topic')


  def project_to_world_frame(self, msg):
    if isinstance(msg, GaussianDistributionStamped):
      g = Gaussian(len(msg.distribution.mu), msg.distribution.mu,
                    msg.distribution.cov, msg.header.frame_id,
                   classification=msg.distribution.classification)
      return g.transform_to_frame(self.world_frame, self.listener)[0]
    else:
      raise Exception(self.name + ' was given ' + str(type(msg)) + ', which is not supported')


  def apply_score_to(self, msg, new_msgs, idx, dists, min_score):
    if not isinstance(msg, PointCloud2):
      raise Exception(self.name + ': was given a ' + \
            str(type(msg)) + ' which is not supported')
    points = np.array(pc2.read_points_list(msg, skip_nans=True))
    score = np.zeros(points.shape[0])
    for dist in dists:
      score += dist.pdf(points[:,:3])
    distance = np.linalg.norm(points[:,:2], axis=1)
    # take only the points with above a minimum score
    points_to_keep = np.where(score > min_score)[0]
    unsafe_points = np.where(distance < self.safety_radius)[0]
    points_to_keep = np.concatenate((points_to_keep, unsafe_points))
    points = points[points_to_keep]
    new_msgs[idx] = pc2.create_cloud(msg.header, msg.fields, points)
    return

  def evaluate_distributions(self, dists):
    # we simply pass the distributions back up to where they are applied to the data
    # this is bcause the lidar field is very 'sparse'
    return dists.values()


  def project_from_world_frame(self, dist):
    g = dist.transform_to_frame(self.frame, self.listener, time=rospy.Time.now())[0]
    return g
