#!/usr/bin/env python
from __future__ import division

import txros
import numpy as np
import navigator_tools
from twisted.internet import defer
from geometry_msgs.msg import PoseStamped

class RvizRepublisher(object):
    @txros.util.cancellableInlineCallbacks
    def init(self):
        self.nh = yield txros.NodeHandle.from_argv("rviz_republisher")
        self.rviz_republish = self.nh.advertise("/rviz_goal", PoseStamped)
        self.rviz_goal = self.nh.subscribe("/move_base_simple/goal", PoseStamped)

        yield self.rviz_goal.get_next_message()

        while True:
            yield self.nh.sleep(.1)
            yield self.do()

    @txros.util.cancellableInlineCallbacks
    def do(self):
        last = yield self.rviz_goal.get_last_message()
        self.rviz_republish.publish(last)

@txros.util.cancellableInlineCallbacks
def main():
    rr = RvizRepublisher()
    yield rr.init()

    yield defer.Deferred() # never exit

txros.util.launch_main(main)