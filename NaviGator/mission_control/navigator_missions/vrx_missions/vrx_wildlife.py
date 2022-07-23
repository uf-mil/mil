#!/usr/bin/env python3
import math
from dis import dis

import numpy as np
import tf
import txros
from mil_tools import rosmsg_to_numpy
from nav_msgs.msg import Odometry
from navigator_msgs.srv import ChooseAnimal, ChooseAnimalRequest, ChooseAnimalResponse
from tsp_solver.greedy import solve_tsp
from twisted.internet import defer
from vrx import Vrx

___author___ = "Alex Perez"


class VrxWildlife(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def closest_point_on_radius(self, start_pos, end_pos, radius):
        # given two points, this finds the closest point to the end_pose given a radius around the end_pose
        vector = [end_pos[0] - start_pos[0], end_pos[1] - start_pos[1]]
        theta = math.atan2(vector[1], vector[0])
        hypot = math.sqrt((vector[0] ** 2) + (vector[1] ** 2))
        dist_to_point = hypot - radius
        return [
            start_pos[0] + dist_to_point * math.cos(theta),
            start_pos[1] + dist_to_point * math.sin(theta),
            0,
        ]

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback("Waiting for task to start")
        yield self.wait_for_task_such_that(
            lambda task: task.state in ["ready", "running"]
        )
        path_msg = yield self.get_latching_msg(self.animal_landmarks)
        poses = [
            (yield self.geo_pose_to_enu_pose(geo_pose.pose))
            for geo_pose in path_msg.poses
        ]

        position = self.pose[0]

        # fill up animal array
        animals_list = []
        for geo_pose in path_msg.poses:
            animals_list.append(geo_pose.header.frame_id)

        print(animals_list)

        # initialize distance matrix
        poses = poses + [position]
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
        print(path)

        # self.send_feedback('Sorted poses' + str(poses))
        yield self.wait_for_task_such_that(lambda task: task.state in ["running"])

        # important parameters
        radius = 7
        z_vec = np.array([0, 0, 1])

        # get croc pose
        animal_pose_croc = None
        croc_index = None
        croc_is_present = False

        # get index of crocodile
        for i in range(len(path_msg.poses)):
            if path_msg.poses[i].header.frame_id == "crocodile":
                croc_is_present = True
                croc_index = i

        # do movements
        for i in range(len(path)):
            current_animal = animals_list[path[i]]

            if current_animal == "crocodile":
                continue

            # update crocodile pose
            for geo_pose in path_msg.poses:
                if geo_pose.header.frame_id == "crocodile":
                    animal_pose_croc = yield self.geo_pose_to_enu_pose(
                        path_msg.poses[croc_index].pose
                    )

            # get animal msgs
            path_msg = yield self.get_latching_msg(self.animal_landmarks)
            animal_pose_next = yield self.geo_pose_to_enu_pose(
                path_msg.poses[path[i]].pose
            )

            start_circle_pos = self.closest_point_on_radius(
                self.pose[0], animal_pose_next[0], radius
            )

            # first point at goal
            yield self.point_at_goal(start_circle_pos)

            req = ChooseAnimalRequest()
            if croc_is_present:
                # we will check if the crocodile lies within either of two rectangles where the touching points
                # of the rectangles are the current pos and goal pos.
                # based on where the crocodile is will determine our action for how to get to the goal pos.

                # define left and right rectangles
                rectangle_width = 15
                vect = start_circle_pos - self.pose[0]
                flipped_vect = self.pose[0] - start_circle_pos
                norm_vect = vect / np.linalg.norm(vect)
                flipped_norm_vect = flipped_vect / np.linalg.norm(flipped_vect)
                p1_l = (np.cross(z_vec, norm_vect) * rectangle_width + self.pose[0])[
                    0:2
                ]
                p2_l = start_circle_pos[0:2]
                p1_r = self.pose[0][0:2]
                p2_r = (
                    np.cross(z_vec, flipped_norm_vect) * rectangle_width
                    + start_circle_pos
                )[0:2]

                # determine if croc is in left rect
                AM = animal_pose_croc[0][0:2] - p1_r
                AB = p1_l - p1_r
                AD = p2_l - p1_r

                AMAB = np.dot(AM, AB)
                ABAB = np.dot(AB, AB)
                AMAD = np.dot(AM, AD)
                ADAD = np.dot(AD, AD)

                left_rect = False
                if (0 < AMAB < ABAB) and (0 < AMAD < ADAD):
                    # calculate pitstop point
                    print("WARNING: Crocodile is in left rectangle")

                    req.target_animal = "crocodile"
                    req.circle_direction = "ccw"
                    left_rect = True
                    yield self.circle_animal(req)

                # determine if croc is in right rect
                AM = animal_pose_croc[0][0:2] - p2_l
                AB = p1_r - p2_l
                AD = p2_r - p2_l

                AMAB = np.dot(AM, AB)
                ABAB = np.dot(AB, AB)
                AMAD = np.dot(AM, AD)
                ADAD = np.dot(AD, AD)

                if not left_rect and (0 < AMAB < ABAB) and (0 < AMAD < ADAD):
                    # calculate pitstop point
                    print("WARNING: Crocodile is in right rectangle")

                    req.target_animal = "crocodile"
                    req.circle_direction = "cw"
                    yield self.circle_animal(req)

            if current_animal == "platypus":
                req.target_animal = current_animal
                req.circle_direction = "cw"
            if current_animal == "turtle":
                req.target_animal = current_animal
                req.circle_direction = "ccw"
            yield self.circle_animal(req)
