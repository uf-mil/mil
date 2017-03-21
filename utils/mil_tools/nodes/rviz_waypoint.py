#!/usr/bin/env python
from __future__ import division

import txros
import numpy as np
import navigator_tools
from twisted.internet import defer
from geometry_msgs.msg import PoseStamped, PointStamped

class RvizRepublisher(object):
    @txros.util.cancellableInlineCallbacks
    def init(self):
        self.nh = yield txros.NodeHandle.from_argv("rviz_republisher")
        self.point_republish = self.nh.advertise("/rviz_point", PointStamped)
        self.pose_republish = self.nh.advertise("/rviz_goal", PoseStamped)
        
        self.rviz_goal = self.nh.subscribe("/move_base_simple/goal", PoseStamped)
        self.clicked_point = self.nh.subscribe("/clicked_point", PointStamped)
    
        self.delay = .1  # s
        self.publish_point()
        self.publish_pose()

    @txros.util.cancellableInlineCallbacks 
    def publish_point(self):
        yield self.clicked_point.get_next_message()

        while True:
            yield self.nh.sleep(self.delay)
            last_point = yield self.clicked_point.get_last_message()
            self.point_republish.publish(last_point)

    @txros.util.cancellableInlineCallbacks
    def publish_pose(self):
        yield self.rviz_goal.get_next_message()

        while True:
            yield self.nh.sleep(self.delay)
            last_pose = yield self.rviz_goal.get_last_message()
            self.pose_republish.publish(last_pose)


@txros.util.cancellableInlineCallbacks
def main():
    rr = RvizRepublisher()
    yield rr.init()

    yield defer.Deferred()


txros.util.launch_main(main)
