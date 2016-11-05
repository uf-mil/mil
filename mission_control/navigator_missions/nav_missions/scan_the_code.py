#!/usr/bin/env python
import txros
import numpy as np
from navigator_scan_the_code import ScanTheCodeMission
from geometry_msgs.msg import PoseStamped
import navigator_tools as nt


def publish_pose(pub, pose):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "enu"
    pose = nt.numpy_quat_pair_to_pose(pose.pose[0], pose.pose[1])
    pose_stamped.pose = pose
    pub.publish(pose_stamped)


@txros.util.cancellableInlineCallbacks
def main(navigator):
    pub = yield navigator.nh.advertise("/stc/pose", PoseStamped)
    mission = ScanTheCodeMission(navigator.nh)
    yield mission.init(navigator.tf_listener)
    pose, look_at = yield mission.initial_position()
    initial_pose = navigator.move.set_position(pose).look_at(look_at)
    yield navigator.nh.sleep(1)
    publish_pose(pub, initial_pose)

    # UNCOMMENT
    # yield mypose = navigator.move.set_position(pose).look_at(look_at).go()

    mission.correct_pose(pose)
    navigator.nh.sleep(.2)
    if not mission.stc_correct:
        circle = navigator.move.circle_point(look_at, 8, granularity=10)
        for p in circle:
            if mission.stc_correct:
                break
            # yield p.go()


    colors = yield mission.find_colors()
    # print colors
    # if colors is None:
    #     navigator.nh.set_param('mission/detect_deliver/Shape', 'CROSS')
    #     navigator.nh.set_param('mission/detect_deliver/Color', 'RED')
    # if np.equal(colors, np.array(['r', 'g', 'b'])):
    #     navigator.nh.set_param('mission/detect_deliver/Shape', 'CIRCLE')
    #     navigator.nh.set_param('mission/detect_deliver/Color', 'RED')
    # else:
    #     navigator.nh.set_param('mission/detect_deliver/Shape', 'TRIANGLE')
    #     navigator.nh.set_param('mission/detect_deliver/Color', 'GREEN')

    # navigator.change_wrench("autonomous")


@txros.util.cancellableInlineCallbacks
def safe_exit(navigator):
    pass
