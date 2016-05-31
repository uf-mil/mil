from txros import util, tf
import numpy as np
import rospy
from sub8_msgs.srv import TBDetectionSwitch
# from sub8_perception.srv import TBDetectionSwitch
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion

@util.cancellableInlineCallbacks
def align_to_board(msg, sub):
    yield sub.move.set_position(target_position).zero_roll_and_pitch().go()

@util.cancellableInlineCallbacks
def run(sub):

    # dive to mission start depth
    mission_start_depth = float(input("Entered the desired depth for the 'set course' mission: ")) # For pool testing
    # mission_start_depth = 2.15        # meters
    print "descending to set course mission depth"
    sub.to_height(mission_start_depth, 0.5)

    # Turn on board detection
    detection_switch = rospy.ServiceProxy('/torpedo_board/detection_activation_switch', TBDetectionSwitch)
    detection_switch(True)
    print "Torpedo Board detection activated"

    # TODO: Rotate in place so board gets in view
    print "Executing search pattern"
    yield sub.move.yaw_left(3.0).zero_roll_and_pitch().go()

    # TODO: once the board moves out of our field of view, stop rotating
    print "Found torpedo board"
    torp_pose_sub = rospy.Subscriber("/torpedo_board/pose", Pose, callback=align_to_board, callback_args=sub)
    rospy.sleep(10)
    torp_pose_sub.unregister()

    # TODO: move sub to closest point on the ray eminating from the board centroid normal to the board

    # TODO: align x-axis of sub with z axis of torpedo board
    print "aligning to torpedo board"

    # TODO: approach board until it occupies most of our field of view, save this position as the best obserbation position
    print "Approaching board"

    # TODO: Determine which of the targets has the sliding door covering it
    print "Examining targets"

    # TODO: While moving in the plane parallel to the board align bottom paddle to a position on the board slightly to the right of the door handle
    print "Aligning to door removal position"

    # TODO: move directly torwards the board until the bottom pattle is aligned with the handle
    print "Approaching board"

    # TODO: move to the left for about one foot
    print "Removing target cover"

    # TODO:
