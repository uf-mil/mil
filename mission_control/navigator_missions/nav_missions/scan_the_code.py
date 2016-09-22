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
    yield navigator.vision_request("scan_the_code_activate")

    # activate stereo model fitter
    granularity = 6
    distance = 3
    angle = 360/granularity * 3.14/180
    n = [0,0]
    # this is the absolute position, give it the relative position option
    #base = [-5,5,0]
    base = navigator.move.latlon() + navigator.move.position

    for i in range(0,granularity+1):
        n[0] = distance * math.cos(i*angle) ; n[1] = distance * math.sin(i*angle)
        new = [n[0]+base[0], n[1]+base[1],0]
        yield navigator.move.set_position(new).look_at(base).go()
        m = yield navigator.vision_request("scan_the_code_status")
        if(m.tracking_model):
            if(m.mission_complete):
                print "Mission complete ", m.colors
                # SET COLORS ON NAVIGATOR
                return
            else:
                #rospy.sleep(15.)
                m = yield navigator.vision_request("scan_the_code_status")
                if(m.mission_complete):
                    print "Mission complete ", m.colors
                    # SET COLORS ON NAVIGATOR
                    return



    print "Mission failed"
    # SET COLORS ON NAVIGATOR TO FAKE VALS














