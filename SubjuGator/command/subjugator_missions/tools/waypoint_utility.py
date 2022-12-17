#!/usr/bin/env python3
import sys

import actionlib
import geometry_msgs.msg as geom_msgs
import mil_msgs.msg as mil_msgs
import mil_ros_tools
import nav_msgs.msg as nav_msgs
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist
from scipy import linalg
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker


class Spacenav:
    _position_gain = np.array([0.03, 0.03, 0.03])
    _orientation_gain = np.array([0, 0, 0.03])

    def __init__(self):
        """
        - Allow the user to move a "ghost" of the sub in Rviz, and then command a pose and orientation

        When determining left and right, the wire faces away from you"""
        rospy.init_node("spacenav_positioning")

        if sys.argv[1] == "2d" or sys.argv[1] == "3d":
            self.mode = sys.argv[1]
        else:
            rospy.loginfo("Invalid mode - Defaulting to 2D")
            self.mode = "2d"

        self.distance_marker = Marker()
        self.distance_marker.type = self.distance_marker.TEXT_VIEW_FACING
        self.distance_marker.color.r = 1
        self.distance_marker.color.b = 1
        self.distance_marker.color.g = 1
        self.distance_marker.color.a = 1
        self.distance_marker.scale.z = 0.1

        self.transformer = tf.TransformerROS()
        self.world_to_body = np.eye(3)

        self.cur_position = None
        self.cur_orientation = None

        self.diff_position = 0
        self.target_depth = 0
        self.target_distance = 0

        self.target_position = np.zeros(3)
        self.target_orientation = np.eye(3)
        self.target_orientation_quaternion = np.array([0.0, 0.0, 0.0, 1.0])

        self.client = actionlib.SimpleActionClient("/moveto", mil_msgs.MoveToAction)
        # self.client.wait_for_server()

        self.target_pose_pub = rospy.Publisher("/posegoal", PoseStamped, queue_size=1)
        self.target_distance_pub = rospy.Publisher(
            "/pose_distance", Marker, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            "/odom", nav_msgs.Odometry, self.odom_cb, queue_size=1
        )
        self.twist_sub = rospy.Subscriber(
            "/spacenav/twist", Twist, self.twist_cb, queue_size=1
        )
        self.joy_sub = rospy.Subscriber("/spacenav/joy", Joy, self.joy_cb, queue_size=1)

    def odom_cb(self, msg: nav_msgs.Odometry) -> None:
        """HACK: Intermediate hack until we have tf set up"""
        pose, twist, _, _ = mil_ros_tools.odometry_to_numpy(msg)
        position, orientation = pose
        self.world_to_body = self.transformer.fromTranslationRotation(
            position, orientation
        )[:3, :3]

        self.cur_position = position
        cur_orientation = tf.transformations.quaternion_matrix(orientation)
        self.cur_orientation = cur_orientation[:3, :3]

    def twist_cb(self, msg: Twist) -> None:
        if self.cur_orientation is None or self.cur_position is None:
            return
        linear = mil_ros_tools.rosmsg_to_numpy(msg.linear)
        angular = mil_ros_tools.rosmsg_to_numpy(msg.angular)
        self.target_position += self.target_orientation.dot(
            self._position_gain * linear
        )
        self.diff_position = np.subtract(self.cur_position, self.target_position)

        gained_angular = self._orientation_gain * angular
        skewed = mil_ros_tools.skew_symmetric_cross(gained_angular)
        rotation = linalg.expm(skewed)

        # TODO: Better
        # self.target_orientation = self.cur_orientation
        # self.target_orientation = rotation.dot(self.target_orientation)

        self.target_orientation = self.target_orientation.dot(rotation)
        self.target_distance = round(
            np.linalg.norm(np.array([self.diff_position[0], self.diff_position[1]])), 3
        )
        self.target_depth = round(self.diff_position[2], 3)
        self.target_orientation = self.target_orientation.dot(rotation)

        blank = np.eye(4)
        blank[:3, :3] = self.target_orientation
        self.target_orientation_quaternion = tf.transformations.quaternion_from_matrix(
            blank
        )
        self.publish_target_pose(
            self.target_position, self.target_orientation_quaternion
        )

    def publish_target_pose(self, position: np.ndarray, orientation):
        self.target_pose_pub.publish(
            PoseStamped(
                header=mil_ros_tools.make_header("map"),
                pose=Pose(
                    position=mil_ros_tools.numpy_to_point(position),
                    orientation=mil_ros_tools.numpy_to_quaternion(orientation),
                ),
            )
        )
        self.distance_marker.header = mil_ros_tools.make_header("map")
        self.distance_marker.text = (
            "XY: "
            + str(self.target_distance)
            + "m\n"
            + "Z: "
            + str(self.target_depth)
            + "m"
        )
        self.distance_marker.pose = Pose(
            position=mil_ros_tools.numpy_to_point(position),
            orientation=mil_ros_tools.numpy_to_quaternion(orientation),
        )
        self.target_distance_pub.publish(self.distance_marker)

    def joy_cb(self, msg: Joy):
        left, right = msg.buttons
        if right:
            self.target_orientation = self.cur_orientation
            self.target_position = self.cur_position

        if left:
            if self.target_orientation_quaternion is not None:
                self.moveto_action(
                    self.target_position, self.target_orientation_quaternion
                )
            rospy.sleep(1.0)

    def moveto_action(self, position: np.ndarray, orientation: np.ndarray):
        self.client.cancel_goal()
        rospy.logwarn("Going to waypoint")
        rospy.logwarn("Found server")

        # Stay under the water
        if position[2] > 0.3:
            rospy.logwarn("Waypoint set too high!")
            position[2] = -0.5

        goal = mil_msgs.MoveToGoal(
            header=mil_ros_tools.make_header("map"),
            posetwist=mil_msgs.PoseTwist(
                pose=geom_msgs.Pose(
                    position=mil_ros_tools.numpy_to_point(position),
                    orientation=mil_ros_tools.numpy_to_quaternion(orientation),
                )
            ),
            speed=0.2,
            linear_tolerance=0.1,
            angular_tolerance=0.1,
        )
        self.client.send_goal(goal)
        # self.client.wait_for_result()
        rospy.logwarn("Go to waypoint")


if __name__ == "__main__":
    s = Spacenav()
    rospy.sleep(1.0)
    rospy.spin()
