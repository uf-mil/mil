#!/usr/bin/env python
import txros
import tf
import numpy as np
import navigator_tools
import math as math
import matplotlib.pyplot as plt
import rospy
import navigator_msgs.srv as navigator_srvs


@txros.util.cancellableInlineCallbacks
def main(navigator):
    navigator.change_wrench("autonomous")
    serv = navigator.nh.get_service_client("/vision/object_classifier_service", s_type)
    resp = yield serv("scan_the_code")

    # Go to first point
    radius = 5
    stc_position = [-5,10,0]
    print navigator.move.position
    diff = np.subtract(stc_position, navigator.move.position)
    diff = np.array([diff[0], diff[1], 0])
    print diff
    length = np.linalg.norm(diff)
    print length
    print diff * (length-radius)/(length)
    if(length - radius >= 0):
        yield navigator.move.rel_position(diff * (length-radius)/(length)).look_at(stc_position).go()
    print "Got there"
    yield navigator.vision_request("scan_the_code_activate")
    pattern = navigator.move.circle_point(stc_position, radius=radius)
    searcher = navigator.search(vision_proxy='scan_the_code_status', search_pattern=pattern)

    yield searcher.start_search(spotings_req=1, speed=.9)















