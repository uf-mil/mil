#!/usr/bin/env python
from __future__ import division
from dis import dis
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy
from tsp_solver.greedy import solve_tsp
import math
import tf

___author___ = "Alex Perez"


class VrxOctogon(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxOctogon, self).__init__(*args, **kwargs)

    def closest_point_on_radius(self, start_pos, end_pos, radius):
        #given two points, this finds the closest point to the end_pose given a radius around the end_pose
        vector = [ end_pos[0] - start_pos[0], end_pos[1] - start_pos[1] ]
        theta = math.atan2(vector[1], vector[0])
        hypot = math.sqrt( (vector[0] ** 2) + (vector[1] ** 2) )
        dist_to_point = hypot - radius
        return [ start_pos[0] + dist_to_point * math.cos(theta), start_pos[1] + dist_to_point * math.sin(theta), 0]

    def start_angle(self, boat_pos, animal_pos, number_of_moves):
        #given the boat position when starting the circle and animal pose, 
        #we can calculate the start orientation needed given
        #the number of moves we plan to use to complete the movement around the animal
        vector = [ animal_pos[0] - boat_pos[0], animal_pos[1] - boat_pos[1] ]
        theta = math.atan(vector[1] / vector[0]) + math.radians( (number_of_moves - 2) * (180/2) / number_of_moves )
        start_orientation = tf.transformations.quaternion_from_euler(0,0,theta)
        return start_orientation

    def local_to_enu(self, distance, yaw):
        #converts a distance and yaw to its global pose waypoint
        boat_pose = self.pose[0]
        boat_pose[0][0] = boat_pose[0][0] + (distance * math.cos(yaw))
        boat_pose[0][1] = boat_pose[0][1] + (distance * math.sin(yaw))
        r,p,y = tf.transformations.euler_from_quaternion(boat_pose[1])
        y = y + yaw
        boat_pose[1] = tf.transformations.quaternion_from_euler(r,p,y)
        return boat_pose

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

                #Re-evaluate exact position where platypus is
                r = 5 #radius
                path_msg = yield self.get_latching_msg(self.animal_landmarks)
                animal_pose = yield self.geo_pose_to_enu_pose(path_msg.poses[index].pose)
                
                granularity = 4

                start_circle_pos = self.closest_point_on_radius(self.pose[0], animal_pose[0], r)
                start_circle_ori = self.start_angle(start_circle_pos, animal_pose[0], granularity)

                yield self.move.set_position(start_circle_pos).set_orientation(start_circle_ori).go(blind=True)

                print("arrived in circle")
                for i in range(granularity):
                    goal_pose = self.local_to_enu( r * math.sqrt(2), math.radians( (granularity - 2) * (180) / granularity ) )
                    yield self.move.set_position(goal_pose[0]).set_orientation(goal_pose[1]).go(blind=True)

                '''
                for i in range(4):
                    ##Re-evaluate exact position where platypus is
                    path_msg = yield self.get_latching_msg(self.animal_landmarks)
                    animal_pose = yield self.geo_pose_to_enu_pose(path_msg.poses[index].pose)
                    x = animal_pose[0][0]
                    y = animal_pose[0][1]

                    distance_from_animal = np.linalg.norm(animal_pose[0] - self.pose[0])
                    print(str(distance_from_animal))
                    increase_radius = 0

                    if current_animal == "platypus":
                        yield self.move.spiral_point([x, y], 'cw', 0.25).go()
                    if current_animal == "turtle":
                        yield self.move.spiral_point([x, y], 'ccw', 0.25).go()
                '''
            elif current_animal == "crocodile":
                print("Avoiding crocodile")

            #res = yield p.forward(2, 'm').down(1, 'ft').yaw_left(50, 'deg').go()



