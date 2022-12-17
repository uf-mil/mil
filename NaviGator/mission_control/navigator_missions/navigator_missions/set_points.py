#!/usr/bin/env python3
import asyncio
import itertools

from geometry_msgs.msg import PoseArray
from mil_tools import numpy_quat_pair_to_pose
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

from .navigator import NaviGatorMission


async def main(navigator: NaviGatorMission):
    waypoints = []
    poses = []

    joy = navigator.nh.subscribe("/joy", Joy)
    waypoint_pub = navigator.nh.advertise("/mission_waypoints", PoseArray)
    await asyncio.gather(joy.setup(), waypoint_pub.setup())

    last_set = False
    print("Waiting for input. RB to set waypoint, right D-Pad to go.")
    await navigator.change_wrench("rc")
    while True:
        joy_msg = await joy.get_next_message()
        b_set = bool(joy_msg.buttons[5])  # RB
        b_go = bool(joy_msg.buttons[12])  # Right D-Pad

        if b_set and b_set != last_set:
            # Set a waypoint at the present location
            waypoints.append(navigator.move)

            # For displaying a pose array
            poses.append(numpy_quat_pair_to_pose(*navigator.pose))
            pa = PoseArray(header=Header(frame_id="enu"), poses=poses)
            print("SET")
            waypoint_pub.publish(pa)

        last_set = b_set

        if b_go and len(waypoints) > 1:
            print("GOING")
            break

    for waypoint in itertools.cycle(waypoints):
        await waypoint.go()
        print("Arrived!")
        await navigator.nh.sleep(5)
