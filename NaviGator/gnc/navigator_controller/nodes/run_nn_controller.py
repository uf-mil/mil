#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from mil_msgs.msg import PoseTwistStamped
from neural_control.nn_controller import NN_controller
from geometry_msgs.msg import PoseStamped

def odom_callback(odom_msg: Odometry) -> None:
    """
    Serves as the callback method for the subscriber to
    the /odom topic in the NN controller.

    Args:
        odom_msg: Odometry - The message passed to the
          callback by the subscriber.
    """
    controller.give_new_state(
        odom_msg.pose.pose, odom_msg.twist.twist, odom_msg.header.stamp.to_sec()
    )

def reference_callback(ref_msg: PoseTwistStamped) -> None:
    """
    Serves as the callback method for the subscriber to
    the /trajectory topic in the NN controller.

    Args:
        ref_msg: PoseTwistStamped - The message passed
          to the callback by the subscriber.
    """
    pose_ref_pub.publish(
        PoseStamped(header=ref_msg.header, pose=ref_msg.posetwist.pose)
    )
    controller.give_new_reference(ref_msg.posetwist.pose, ref_msg.posetwist.twist)

controller = NN_controller(
    dof = 3,
    kp = [1000, 1000, 5600],
    kd = [1200, 1200, 6000],
    kv = 2,
    kw = 2,
    N = 10,
    sig = "tanh",
    nn_limit = [10**10] * 3,
    wrench_topic = "/wrench/autonomous",
    neuralwrench_topic = "/adaptation",
)

rospy.init_node("controller")
rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.Subscriber("/trajectory", PoseTwistStamped, reference_callback)
pose_ref_pub = rospy.Publisher("/pose_ref", PoseStamped, queue_size=100)

rospy.spin()
