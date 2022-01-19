#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy

___author___ = "Alex Perez"


class VrxSpiral(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxSpiral, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Waiting for task to start')
        yield self.wait_for_task_such_that(lambda task: task.state in ['ready', 'running'])
        path_msg = yield self.get_latching_msg(self.animal_landmarks)
        poses = [ (yield self.geo_pose_to_enu_pose(geo_pose.pose)) for geo_pose in path_msg.poses]
        position = self.pose[0]
        poses = sorted(poses, key=lambda pose: np.linalg.norm(pose[0] - position))
        self.send_feedback('Sorted poses' + str(poses))
        yield self.wait_for_task_such_that(lambda task: task.state in ['running'])

        #do movements
        for pose in poses:
            self.send_feedback('Going to {}'.format(pose))
            yield self.move.set_position(pose[0]).set_orientation(pose[1]).go(blind=True)
            #res = yield self.move.spiral_point([2, 2], 'ccw', 1, meters_per_rev=2).go()
