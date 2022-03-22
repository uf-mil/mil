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

        #Go to goal
        yield self.send_trajectory_without_path(self.gps_waypoint_fix(goal_pose))
