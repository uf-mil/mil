#!/usr/bin/env python
'''This node is for taking input from a Spacenav (3D Mouse)
    [1] http://www.3dconnexion.com/products/spacemouse.html
'''
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, WrenchStamped
import nav_msgs.msg as nav_msgs
import sub8_ros_tools as sub8_utils


class Spacenav(object):
    def __init__(self):
        rospy.init_node('spacenav_remap')

        self.linear_gain = 500
        self.angular_gain = 500

        self.behavior = rospy.get_param('~behavior', 'wrench')
        behaviors = ['wrench', 'velocity', 'position']
        assert self.behavior in behaviors, "Parameter 'behavior' invalid, must be in {}".format(
            behaviors)

        if self.behavior == 'wrench':
            self.wrench_pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=1)
        else:
            raise(NotImplementedError("Velocity and position control by mouse are not yet supported"))

        self.transformer = tf.TransformerROS()
        self.world_to_body = np.eye(3)
        self.odom_sub = rospy.Subscriber('/truth/odom', nav_msgs.Odometry, self.odom_cb, queue_size=1)
        self.twist_sub = rospy.Subscriber('/spacenav/twist', Twist, self.twist_cb, queue_size=1)

    def odom_cb(self, msg):
        '''HACK: Intermediate hack until we have tf set up'''
        pose, twist, _, _ = sub8_utils.odometry_to_numpy(msg)
        position, orientation = pose
        self.world_to_body = self.transformer.fromTranslationRotation(position, orientation)[:3, :3]

    def twist_cb(self, msg):
        linear = sub8_utils.rosmsg_to_numpy(msg.linear)
        angular = sub8_utils.rosmsg_to_numpy(msg.angular)

        if self.behavior == 'wrench':
            # Directly transcribe linear and angular 'velocities' to torque
            # TODO: Setup actual TF!

            self.wrench_pub.publish(
                sub8_utils.make_wrench_stamped(
                    force=self.linear_gain * self.world_to_body.dot(linear),
                    torque=self.angular_gain * self.world_to_body.dot(angular)
                )
            )
        else:
            raise(NotImplementedError("Velocity and position control by mouse are not yet supported"))


if __name__ == '__main__':
    sn = Spacenav()
    rospy.spin()
