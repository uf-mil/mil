#!/usr/bin/env python
import numpy as np
from twisted.internet import defer
from txros import action, util, tf, serviceclient, NodeHandle
from nav_msgs.msg import Odometry

@util.cancellableInlineCallbacks
def main(navigator):
    m = navigator.move.forward(1)
    yield m.go()