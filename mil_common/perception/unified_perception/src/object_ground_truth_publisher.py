#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from gazebo_msgs.msg import LinkStates
import tf
import tf2_ros
from mil_msgs.msg import GaussianDistribution

from distributions.gaussian import Gaussian

base_link = 'wamv::base_link'


links = ['robotx_navigation_challenge::gate_' + i + '::link'
          for i in ['start_red',
                    'start_green',
                    'end_red',
                    'end_green']]
rospy.init_node('ground_truth_publisher')

listener = tf.TransformListener()
listener.waitForTransform('base_link', 'front_left_camera_link_optical', rospy.Time(0), rospy.Duration(10.0))

pub = rospy.Publisher(rospy.get_param('gaussian_topic'), numpy_msg(GaussianDistribution),
                      queue_size=10)


gd = numpy_msg(GaussianDistribution)()
gd.header.frame_id = 'front_left_camera_link_optical'
gd.sensor_name = 'front_left_camera'

def cb(msg, count):
  # get the base link pose and the bouys' pose
  base_link_idx = msg.name.index(base_link)
  trans = np.array([-msg.pose[base_link_idx].position.x,
                    -msg.pose[base_link_idx].position.y,
                    -msg.pose[base_link_idx].position.z])
  quat = np.array([msg.pose[base_link_idx].orientation.x,
                   msg.pose[base_link_idx].orientation.y,
                   msg.pose[base_link_idx].orientation.z,
                   -msg.pose[base_link_idx].orientation.w])

  links_idx = [msg.name.index(i) for i in links]
  poses = [msg.pose[i] for i in links_idx]
  # transform them into the wamv's frame
  for i in range(len(links)):
    pos = poses[i].position
    v = np.array([pos.x +  trans[0], pos.y + trans[1], pos.z + trans[2]])
    v_u = np.hstack((tf.transformations.unit_vector(v), np.zeros(1,)))
    mu = np.linalg.norm(v) * tf.transformations.quaternion_multiply(
                              tf.transformations.quaternion_multiply(quat, v_u),
                              tf.transformations.quaternion_conjugate(quat))[:3]
    # project to the camera's optical_frame
    cov = np.array([[1,0,0],
                    [0,1,0],
                    [0,0,1]])
    try:
      g = Gaussian(3, mu, cov, 'base_link').transform_to_frame(gd.header.frame_id,
                                                               listener,
                                                               time=rospy.Time.now())[0]
    except Exception as e:
      return
    # if the object is in front of the camera and < 100m away
    if g.mu[0] > 0 and np.linalg.norm(g.mu) < 100:
      gd.mu = g.mu
      gd.cov = g.cov
      gd.id = links[i]
      gd.header.seq += 1
      gd.header.stamp = rospy.Time.now()
      pub.publish(gd)
    rospy.sleep(0.1)



sub = rospy.Subscriber('/gazebo/link_states', numpy_msg(LinkStates), cb, 1)
rospy.spin()
