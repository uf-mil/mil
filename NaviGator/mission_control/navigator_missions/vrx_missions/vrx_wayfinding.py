#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy
from tsp_solver.greedy import solve_tsp

___author___ = "Alex Perez and Kevin Allen"


class VrxWayfinding(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxWayfinding, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Waiting for task to start')
        yield self.wait_for_task_such_that(lambda task: task.state in ['ready', 'running'])
        path_msg = yield self.get_latching_msg(self.wayfinding_path_sub)
        poses = [ (yield self.geo_pose_to_enu_pose(geo_pose.pose)) for geo_pose in path_msg.poses]

        position = self.pose[0]

        #initialize distance matrix
        poses = poses + [position]
        array_size = len(poses)
        start_pose_index = array_size - 1
        dist_matrix = np.zeros( (array_size, array_size) )

        #fill up distance matrix
        for i in range(array_size):
            for j in range(array_size):
                dist_matrix[i][j] = np.linalg.norm(poses[i][0] - poses[j][0])

        #solve tsp algorithm (ensure start point is where boat is located) & remove current position from pose list
        path = solve_tsp(dist_matrix, endpoints=(start_pose_index,None))
        poses = poses[:start_pose_index]
        path = path[1:]

        #self.send_feedback('Sorted poses' + str(poses))
        yield self.wait_for_task_such_that(lambda task: task.state in ['running'])

        #do movements
        for index in path:
            self.send_feedback('Going to {}'.format(poses[index]))
            yield self.move.set_position(poses[index][0]).set_orientation(poses[index][1]).go(blind=True)

