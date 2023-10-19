#!/usr/bin/env python3
import math

import numpy as np
import tf
from tsp_solver.greedy import solve_tsp

from .vrx import Vrx

___author___ = "Alex Perez"


class VrxWayfinding(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def point_at_goal(self, goal_pos):
        vect = [goal_pos[0] - self.pose[0][0], goal_pos[1] - self.pose[0][1]]
        theta = math.atan2(vect[1], vect[0])
        return tf.transformations.quaternion_from_euler(0, 0, theta)

    async def run(self, parameters):
        self.send_feedback("Waiting for task to start")
        await self.wait_for_task_such_that(
            lambda task: task.state in ["ready", "running"],
        )

        path_msg = await self.get_latching_msg(self.wayfinding_path_sub)

        poses = []
        for geo_pose in path_msg.poses:
            pose = await self.geo_pose_to_enu_pose(geo_pose.pose)
            poses.append(pose)

        position = self.pose[0]

        # initialize distance matrix
        poses = [*poses, position]
        array_size = len(poses)
        start_pose_index = array_size - 1
        dist_matrix = np.zeros((array_size, array_size))

        # fill up distance matrix
        for i in range(array_size):
            for j in range(array_size):
                dist_matrix[i][j] = np.linalg.norm(poses[i][0] - poses[j][0])

        # solve tsp algorithm (ensure start point is where boat is located) & remove current position from pose list
        path = solve_tsp(dist_matrix, endpoints=(start_pose_index, None))
        poses = poses[:start_pose_index]
        path = path[1:]

        # self.send_feedback('Sorted poses' + str(poses))
        await self.wait_for_task_such_that(lambda task: task.state in ["running"])

        # do movements
        for index in path:
            self.send_feedback(f"Going to {poses[index]}")
            orientation_fix = self.point_at_goal(poses[index][0])
            await self.move.set_orientation(orientation_fix).go(blind=True)
            await self.move.set_position(poses[index][0]).set_orientation(
                poses[index][1],
            ).go(blind=True)
