#!/usr/bin/env python
from txros import util
import itertools
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
from mil_tools import numpy_quat_pair_to_pose


@util.cancellableInlineCallbacks
def main(navigator):
    waypoints = []
    poses = []

    joy = yield navigator.nh.subscribe("/joy", Joy)
    waypoint_pub = yield navigator.nh.advertise("/mission_waypoints", PoseArray)

    last_set = False
    print "Waiting for input. RB to set waypoint, right D-Pad to go."
    navigator.change_wrench("rc")
    while True:
        joy_msg = yield joy.get_next_message()
        b_set = bool(joy_msg.buttons[5])  # RB
        b_go = bool(joy_msg.buttons[12])  # Right D-Pad

        if b_set and b_set != last_set:
            # Set a waypoint at the present location
            waypoints.append(navigator.move)

            # For displaying a pose array
            poses.append(numpy_quat_pair_to_pose(*navigator.pose))
            pa = PoseArray(header=Header(frame_id="enu"), poses=poses)
            print "SET"
            yield waypoint_pub.publish(pa)

        last_set = b_set

        if b_go and len(waypoints) > 1:
            print "GOING"
            break

    for waypoint in itertools.cycle(waypoints):
        yield waypoint.go()
        print "Arrived!"
        yield navigator.nh.sleep(5)
