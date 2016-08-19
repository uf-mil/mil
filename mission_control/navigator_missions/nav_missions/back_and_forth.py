#!/usr/bin/env python
import numpy as np
from twisted.internet import defer
from txros import action, util, tf, serviceclient, NodeHandle
from nav_msgs.msg import Odometry

@util.cancellableInlineCallbacks
def main(navigator):
    while True:
        point_2 = navigator.move
        print "Going to point 1"
        yield navigator.move.forward(5).right(3).go()
        yield navigator.nh.sleep(5)
        print "Going to point 2"
        yield point_2.go()
        yield navigator.nh.sleep(5)