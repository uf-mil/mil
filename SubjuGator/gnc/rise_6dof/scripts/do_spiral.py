#!/usr/bin/env python3

import math

import numpy
import roslib
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mil_msgs import orientation_helpers
from mil_msgs.msg import PoseTwist, PoseTwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf import transformations

RADIUS = 4 * 0.75  # meter
SPEED = 0.55 * 0.75  # meter/second
PERIOD = 2 * math.pi * RADIUS / SPEED  # second
DESCENT = 0.5 * 0.75  # meter/revolution
dt = 1e-3  # second (used for numerical derivatives)


def position_generator_orig(t):
    r = RADIUS  # *(1 - math.e**(-t/30))
    w = 2 * math.pi / PERIOD
    return numpy.array(
        [r * math.sin(w * t), r * -math.cos(w * t), -DESCENT * t / PERIOD],
    )


def position_generator(t):
    return start + position_generator_orig(t) - position_generator_orig(0)


def positionvelocity_generator(t):
    pos = position_generator(t)
    vel = (position_generator(t + dt / 2) - position_generator(t - dt / 2)) / dt
    return pos, vel


def positionorientationvelocity_generator(t):
    pos, vel = positionvelocity_generator(t)
    orient = orientation_helpers.lookat(vel)
    return pos, orient, vel


def positionorientationvelocityangularvelocity_generator(t):
    pos, orient, vel = positionorientationvelocity_generator(t)

    orient1 = positionorientationvelocity_generator(t - dt / 2)[1]
    orient2 = positionorientationvelocity_generator(t + dt / 2)[1]
    angvel = (
        orientation_helpers.quat_to_rotvec(
            transformations.quaternion_multiply(
                orient2,
                transformations.quaternion_inverse(orient1),
            ),
        )
        / dt
    )

    return pos, orient, vel, angvel


if __name__ == "__main__":
    print("waiting for current position...")
    current = rospy.wait_for_message("/odom", Odometry)
    start = orientation_helpers.xyz_array(current.pose.pose.position)
    print("...got", start)
    start[2] = -0.25

    roslib.load_manifest("rise_6dof")
    rospy.init_node("do_spiral")

    pub = rospy.Publisher("/trajectory", PoseTwistStamped)

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        rospy.sleep(0.01)

        now_time = rospy.Time.now()

        t = (now_time - start_time).to_sec()
        pos, orient, vel, angvel = positionorientationvelocityangularvelocity_generator(
            t,
        )
        if pos[2] < -5:
            break

        world_from_body = transformations.quaternion_matrix(orient)[:3, :3]

        pub.publish(
            PoseTwistStamped(
                header=Header(
                    stamp=now_time,
                    frame_id=current.header.frame_id,
                ),
                posetwist=PoseTwist(
                    pose=Pose(
                        position=Point(*pos),
                        orientation=Quaternion(*orient),
                    ),
                    twist=Twist(
                        linear=Vector3(*world_from_body.T.dot(vel)),
                        angular=Vector3(*world_from_body.T.dot(angvel)),
                    ),
                ),
            ),
        )
