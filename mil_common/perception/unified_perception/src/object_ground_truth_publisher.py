#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from gazebo_msgs.msg import LinkStates
import tf
from visualization_msgs.msg import MarkerArray
from mil_msgs.msg import GaussianDistribution, GaussianDistributionStamped

from distributions.gaussian import Gaussian

base_link = 'wamv::base_link'


links = ['robotx_navigation_challenge::gate_' + i + '::link'
          for i in ['start_red',
                    'start_green',
                    'end_red',
                    'end_green']] + \
        ['robotx_dock_2016::robotx_dock_2016_base::dock_block_2_1::link',
         'robotx_dock_2016::robotx_dock_2016_base::dock_block_2_2::link',
         'robotx_dock_2016::robotx_dock_2016_base::dock_block_2_4::link',
         'robotx_dock_2016::robotx_dock_2016_base::dock_block_2_5::link',
         'robotx_dock_2016::robotx_dock_2016_base::dock_block_2_7::link',
         'robotx_dock_2016::robotx_dock_2016_base::dock_block_2_8::link'
]

rospy.init_node('ground_truth_publisher')

g_topic = rospy.get_param('distributions')['topics']['mil_msgs.msg.GaussianDistributionStamped']

pub = rospy.Publisher(g_topic, numpy_msg(GaussianDistributionStamped), queue_size=10)

debug_pub = rospy.Publisher('debug/indicated_gaussians', numpy_msg(MarkerArray), queue_size=1)

gd = GaussianDistributionStamped()
gd.header.seq = 0
gd.header.frame_id = 'base_link'
gd.distribution.sensor_name = 'front_left_camera'

seq = [0]

def cb(msg):
  seq[0] += 1
  if seq[0] % 3000 == 0:
    gd.header.seq += 1
    stamp = rospy.Time.now()
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
    marker = MarkerArray()
    # transform them into the wamv's frame
    for i in range(len(links)):
      pos = poses[i].position
      v = np.array([pos.x +  trans[0], pos.y + trans[1], pos.z + trans[2]])
      v_u = np.hstack((tf.transformations.unit_vector(v), np.zeros(1,)))
      quat_p = tf.transformations.quaternion_conjugate(quat)
      mu = np.linalg.norm(v) * tf.transformations.quaternion_multiply(
                               tf.transformations.quaternion_multiply(quat, v_u),
                               quat_p)[:3]
      # project to the camera's optical_frame
      cov = np.array([[1,0,0],
                      [0,1,0],
                      [0,0,1]])

      g = Gaussian(3, mu, cov, 'base_link')
      # if the object is in front of the camera and < 100m away
      if g.mu[0] > 0 and np.linalg.norm(g.mu) < 100:
        if i > 3:
          g.mu += np.array([0,0, 2])
        marker.markers.append(g.to_rviz_marker())
        marker.markers[-1].id = i
        gd.distribution.mu = g.mu
        gd.distribution.cov = np.ravel(g.cov)
        #gd.distribution.id = links[i]
        gd.distribution.classification = links[i]
        gd.header.seq += 1
        gd.header.stamp = stamp
        pub.publish(gd)
    debug_pub.publish(marker)

sub = rospy.Subscriber('/gazebo/link_states', numpy_msg(LinkStates), cb, queue_size=1)
rospy.spin()
