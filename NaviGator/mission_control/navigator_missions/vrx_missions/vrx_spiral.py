#!/usr/bin/env python
from __future__ import division
from dis import dis
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy
from tsp_solver.greedy import solve_tsp

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


        #fill up animal array
        animals_list = []
        for geo_pose in path_msg.poses:
            animals_list.append(geo_pose.header.frame_id)

        print(animals_list)

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
        print(path)

        #self.send_feedback('Sorted poses' + str(poses))
        yield self.wait_for_task_such_that(lambda task: task.state in ['running'])

        #do movements
        for index in path:
            self.send_feedback('Going to {}'.format(poses[index]))

            current_animal = animals_list[index]

            if current_animal != "crocodile":
                print("Going clockwise around platypus")

                #Re-evaluate exact position where platypus is
                path_msg = yield self.get_latching_msg(self.animal_landmarks)
                animal_pose = yield self.geo_pose_to_enu_pose(path_msg.poses[index].pose)
                animal_pose[0][0] = animal_pose[0][0] - 4
                yield self.move.set_position(animal_pose[0]).set_orientation([0,0,0,1]).go(blind=True)

                for i in range(4):
                    ##Re-evaluate exact position where platypus is
                    path_msg = yield self.get_latching_msg(self.animal_landmarks)
                    animal_pose = yield self.geo_pose_to_enu_pose(path_msg.poses[index].pose)
                    x = animal_pose[0][0]
                    y = animal_pose[0][1]

                    distance_from_animal = np.linalg.norm(animal_pose[0] - self.pose[0])
                    print(str(distance_from_animal))
                    increase_radius = 0

                    if (distance_from_animal) < 4:
                        increase_radius = 4
                        yield self.move.backward(increase_radius, 'm').go() 
                        #'change correction direction to be a vector pointing towards the animal
                    if (distance_from_animal) > 8:
                        increase_radius = 2
                        yield self.move.forward(increase_radius, 'm').go()

                    if current_animal == "platypus":
                        yield self.move.spiral_point([x, y], 'cw', 0.25).go()
                    if current_animal == "turtle":
                        yield self.move.spiral_point([x, y], 'ccw', 0.25).go()
            elif current_animal == "crocodile":
                print("Avoiding crocodile")

            #res = yield p.forward(2, 'm').down(1, 'ft').yaw_left(50, 'deg').go()
