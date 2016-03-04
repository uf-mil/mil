#!/usr/bin/env python
import rospy
import nav_msgs.msg as nav_msgs
import sub8_ros_tools as sub8_utils
import tf
import numpy as np
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import Joy
from scipy import linalg

import actionlib
import uf_common.msg as uf_common_msgs
import geometry_msgs.msg as geom_msgs


class Spacenav(object):
    _position_gain = np.array([0.01, 0.01, 0.01])
    _orientation_gain = np.array([0.01, 0.01, 0.01])

    def __init__(self):
        '''
        - Allow the user to move a "ghost" of the sub in Rviz, and then command a pose and orientation

        When determining left and right, the wire faces away from you'''
        rospy.init_node('spacenav_positioning')

        self.transformer = tf.TransformerROS()
        self.world_to_body = np.eye(3)

        self.cur_position = None
        self.cur_orientation = None

        self.target_position = np.zeros(3)
        self.target_orientation = np.eye(3)
        self.target_orientation_quaternion = np.array([0.0, 0.0, 0.0, 1.0])

        self.client = actionlib.SimpleActionClient('/moveto', uf_common_msgs.MoveToAction)
        # self.client.wait_for_server()

        self.odom_sub = rospy.Subscriber('/odom', nav_msgs.Odometry, self.odom_cb, queue_size=1)
        self.twist_sub = rospy.Subscriber('/spacenav/twist', Twist, self.twist_cb, queue_size=1)
        self.joy_sub = rospy.Subscriber('/spacenav/joy', Joy, self.joy_cb, queue_size=1)
        self.target_pose_pub = rospy.Publisher('/posegoal', PoseStamped, queue_size=1)

    def odom_cb(self, msg):
        '''HACK: Intermediate hack until we have tf set up'''
        pose, twist, _, _ = sub8_utils.odometry_to_numpy(msg)
        position, orientation = pose
        self.world_to_body = self.transformer.fromTranslationRotation(position, orientation)[:3, :3]

        self.cur_position = position
        cur_orientation = tf.transformations.quaternion_matrix(orientation)
        self.cur_orientation = cur_orientation[:3, :3]

    def twist_cb(self, msg):
        if self.cur_orientation is None or self.cur_position is None:
            return
        linear = sub8_utils.rosmsg_to_numpy(msg.linear)
        angular = sub8_utils.rosmsg_to_numpy(msg.angular)
        self.target_position += self.target_orientation.dot(self._position_gain * linear)

        gained_angular = self._orientation_gain * angular
        skewed = sub8_utils.skew_symmetric_cross(gained_angular)
        rotation = linalg.expm(skewed)

        # TODO: Better
        # self.target_orientation = self.cur_orientation
        # self.target_orientation = rotation.dot(self.target_orientation)
        self.target_orientation = self.target_orientation.dot(rotation)

        blank = np.eye(4)
        blank[:3, :3] = self.target_orientation
        self.target_orientation_quaternion = tf.transformations.quaternion_from_matrix(blank)
        self.publish_target_pose(self.target_position, self.target_orientation_quaternion)

    def publish_target_pose(self, position, orientation):
        self.target_pose_pub.publish(
            PoseStamped(
                header=sub8_utils.make_header('/map'),
                pose=Pose(
                    position=sub8_utils.numpy_to_point(position),
                    orientation=sub8_utils.numpy_to_quaternion(orientation)
                )
            )
        )

    def joy_cb(self, msg):
        left, right = msg.buttons
        if right:
            self.target_orientation = self.cur_orientation
            self.target_position = self.cur_position

        if left:
            print 'left'
            if self.target_orientation_quaternion is not None:
                self.moveto_action(self.target_position, self.target_orientation_quaternion)
            rospy.sleep(1.0)

    def moveto_action(self, position, orientation):
        self.client.cancel_goal()
        rospy.logwarn("going to waypoint")
        rospy.logwarn("Found server")

        goal = uf_common_msgs.MoveToGoal(
            header=sub8_utils.make_header('/map'),
            posetwist=uf_common_msgs.PoseTwist(
                pose=geom_msgs.Pose(
                    position=sub8_utils.numpy_to_point(position),
                    orientation=sub8_utils.numpy_to_quaternion(orientation)
                )
            ),
            speed=0.2,
            linear_tolerance=0.1,
            angular_tolerance=0.1
        )
        self.client.send_goal(goal)
        # self.client.wait_for_result()
        rospy.logwarn("Got to waypoint")


if __name__ == '__main__':
    s = Spacenav()
    rospy.sleep(1.0)
    # s.moveto_action(np.array([0.0, 0.0, -3.0]), np.array([0.0, 0.0, 0.0, 1.0]))
    rospy.spin()
