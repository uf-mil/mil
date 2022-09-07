#!/usr/bin/env python3
import numpy as np
import txros
from navigator_msgs.srv import MoveToWaypointRequest
from twisted.internet import defer

from .vrx import Vrx

___author___ = "Alex Perez"


class VrxStationKeeping2(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.wait_for_task_such_that(
            lambda task: task.state in ["ready", "running"]
        )

        self.send_feedback("Waiting for station keeping goal")
        goal_msg = yield self.get_latching_msg(self.station_keep_goal)
        goal_pose = yield self.geo_pose_to_enu_pose(goal_msg.pose)
        self.send_feedback(f"Going to {goal_pose}")

        # Go to goal
        yield self.send_trajectory_without_path(goal_pose)
