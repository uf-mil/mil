#!/usr/bin/env python
import txros
import tf
import numpy as np
import navigator_tools
import math as math
import matplotlib.pyplot as plt
import rospy
import navigator_msgs.srv as navigator_srvs
from navigator_msgs.srv import PerceptionObjectService, PerceptionObjectServiceRequest





@txros.util.cancellableInlineCallbacks
def main(navigator):
    navigator.change_wrench("autonomous")

    serv = navigator.nh.get_service_client("/vision/object_classifier_service", PerceptionObjectService)
    req = PerceptionObjectServiceRequest()
    req.name = "shooter"
    ans = yield serv(req)
    print ans
    if(not ans.seen_before):
        return

    position = np.array([ans.prev_pose.position.x,ans.prev_pose.position.y, ans.prev_pose.position.z])
    orientation = np.array([ans.prev_pose.orientation.x,ans.prev_pose.orientation.y, ans.prev_pose.orientation.z, ans.prev_pose.orientation.w])

    # position = np.array([0, 10, 1.2017])
    # orientation = np.array([0,0,0,1])

    def gen():
        yield navigator.move.set_position(position).set_orientation(orientation)

    pattern = gen()
    print "--------------"
    print pattern
    searcher = navigator.search(vision_proxy='shooter_lidar', search_pattern=pattern)
    yield searcher.start_search(spotings_req=1, speed=.9)

    # print "hi"
    # req = PerceptionObjectServiceRequest()
    # req.name = "stc"
    # resp = yield serv(req)

    # # Go to first point
    # radius = 7
    # stc_position = [resp.pos.x, resp.pos.y,0]
    # print stc_position

    # print navigator.move.position
    # diff = np.subtract(stc_position, navigator.move.position)
    # diff = np.array([diff[0], diff[1], 0])
    # print diff
    # length = np.linalg.norm(diff)
    # print length
    # print diff * (length-radius)/(length)
    # if(length - radius >= 0):
    #     yield navigator.move.rel_position(diff * (length-radius)/(length)).look_at(stc_position).go()
    # print "Got there"
    # yield navigator.vision_request("scan_the_code_activate")
    # 

    # yield searcher.start_search(spotings_req=1, speed=.9)















