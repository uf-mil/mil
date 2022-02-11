#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from navigator_msgs.srv import MoveToWaypointRequest

___author___ = "Alex Perez"


class VrxStationKeeping2(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxStationKeeping2, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.wait_for_task_such_that(lambda task: task.state in ['ready', 'running'])
        self.send_feedback('Waiting for station keeping goal')
        goal_msg = yield self.get_latching_msg(self.station_keep_goal)
        goal_pose = yield self.geo_pose_to_enu_pose(goal_msg.pose)
        self.send_feedback('Going to {}'.format(goal_pose))

        #Create waypoint message
        req = MoveToWaypointRequest()
        req.target_p.position.x = goal_pose[0][0]
        req.target_p.position.y = goal_pose[0][1]
        req.target_p.position.z = goal_pose[0][2]
        req.target_p.orientation.x = goal_pose[1][0]
        req.target_p.orientation.y = goal_pose[1][1]
        req.target_p.orientation.z = goal_pose[1][2]
        req.target_p.orientation.w = goal_pose[1][3]
        yield self.set_long_waypoint(req)
