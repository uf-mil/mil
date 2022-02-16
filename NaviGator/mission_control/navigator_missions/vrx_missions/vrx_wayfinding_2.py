#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from navigator_msgs.srv import MoveToWaypointRequest
from tsp_solver.greedy import solve_tsp
import math
import tf

___author___ = "Alex Perez"


class VrxWayfinding2(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxWayfinding2, self).__init__(*args, **kwargs)

    def point_at_goal(self, goal_pos):
        vect = [ goal_pos[0] - self.pose[0][0], goal_pos[1] - self.pose[0][1]]
        theta = math.atan2(vect[1], vect[0])
        return tf.transformations.quaternion_from_euler(0,0,theta)

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

            #Create waypoint message
            req = MoveToWaypointRequest()
            goal_pose = poses[index]
            req.target_p.position.x = goal_pose[0][0]
            req.target_p.position.y = goal_pose[0][1]
            req.target_p.position.z = goal_pose[0][2]
            req.target_p.orientation.x = goal_pose[1][0]
            req.target_p.orientation.y = goal_pose[1][1]
            req.target_p.orientation.z = goal_pose[1][2]
            req.target_p.orientation.w = goal_pose[1][3]
            yield self.set_long_waypoint(req)