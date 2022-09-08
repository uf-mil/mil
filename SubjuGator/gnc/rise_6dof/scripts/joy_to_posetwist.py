#!/usr/bin/env python3

import numpy
import roslib
import rospy
from c3_trajectory_generator.srv import SetDisabled
from geometry_msgs.msg import Twist, Vector3
from mil_msgs.msg import PoseTwist, PoseTwistStamped
from mil_msgs.orientation_helpers import PoseEditor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

current = None


def update_current(odom):
    global current
    current = odom


def cb(msg: Joy) -> None:
    enabled = msg.buttons[0] or msg.buttons[1]
    set_trajgen_disabled(enabled)
    if not enabled:
        return
    pub.publish(
        PoseTwistStamped(
            header=Header(
                stamp=msg.header.stamp,
                frame_id=current.header.frame_id,
            ),
            posetwist=PoseTwist(
                pose=PoseEditor.from_Odometry(current).zero_roll_and_pitch().as_Pose(),
                twist=Twist(
                    linear=Vector3(
                        *numpy.array(msg.axes[:3])
                        / joy_max
                        * [1, 1, 1 if msg.buttons[1] else 0]
                    ),
                    angular=Vector3(*numpy.array(msg.axes[3:]) / joy_max * [0, 0, 1]),
                ),
            ),
        )
    )


if __name__ == "__main__":
    roslib.load_manifest("rise_6dof")
    roslib.load_manifest("c3_trajectory_generator")
    rospy.init_node("joy_to_posetwist")

    current = rospy.wait_for_message("/odom", Odometry)
    rospy.Subscriber("/odom", Odometry, update_current)
    pub = rospy.Publisher("/trajectory", PoseTwistStamped)

    set_trajgen_disabled = rospy.ServiceProxy(
        "/c3_trajectory_generator/set_disabled", SetDisabled
    )

    joy_max = 0.68359375
    max_vel = 1
    max_angvel = 1

    rospy.Subscriber("spacenav/joy", Joy, cb)

    rospy.spin()
