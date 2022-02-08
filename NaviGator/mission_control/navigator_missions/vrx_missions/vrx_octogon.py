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

    def point_at_goal(self, goal_pos):
        vect = [ goal_pos[0] - self.pose[0][0], goal_pos[1] - self.pose[0][1]]
        theta = math.atan2(vect[1], vect[0])
        return tf.transformations.quaternion_from_euler(0,0,theta)

    def start_angle(self, boat_pos, animal_pos, interior_angle):
        #given the boat position when starting the circle and animal pose, 
        #we can calculate the start orientation needed given
        #the number of moves we plan to use to complete the movement around the animal
        vector = [ animal_pos[0] - boat_pos[0], animal_pos[1] - boat_pos[1] ]
        theta = math.atan(vector[1] / vector[0]) + interior_angle/2
        start_orientation = tf.transformations.quaternion_from_euler(0,0,theta)
        return start_orientation

    def local_to_enu(self, distance, yaw):
        #converts a distance and yaw to its global pose waypoint
        boat_pose = self.pose
        r,p,y = tf.transformations.euler_from_quaternion(boat_pose[1])

        #assign new boat position
        boat_pose[0][0] = boat_pose[0][0] + (distance * math.cos(y))
        boat_pose[0][1] = boat_pose[0][1] + (distance * math.sin(y))

        #assign new boat orientation
        y = y + yaw
        boat_ori = tf.transformations.quaternion_from_euler(r,p,y)
        boat_pose[1][0] = boat_ori[0]
        boat_pose[1][1] = boat_ori[1]
        boat_pose[1][2] = boat_ori[2]
        boat_pose[1][3] = boat_ori[3]
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

        #calculate parameters
        granularity = 4
        radius = 6
        interior_angle = math.radians( 90 )
        z_vec = np.array([0,0,1])

        #do movements
        for i in range(len(path)):
            self.send_feedback('Going to {}'.format(poses[path[i]]))

            current_animal = animals_list[path[i]]

            if current_animal != "crocodile":

                #Re-evaluate exact position where next animal is
                path_msg = yield self.get_latching_msg(self.animal_landmarks)
                animal_pose = yield self.geo_pose_to_enu_pose(path_msg.poses[path[i]].pose)

                start_circle_pos = self.closest_point_on_radius(self.pose[0], animal_pose[0], radius)

                if current_animal == "platypus":
                    start_circle_ori = self.start_angle(start_circle_pos, animal_pose[0], interior_angle)
                if current_animal == "turtle":
                    start_circle_ori = self.start_angle(start_circle_pos, animal_pose[0], -interior_angle)

                #first point at goal
                orientation_fix = self.point_at_goal(start_circle_pos)
                yield self.move.set_orientation(orientation_fix).go(blind=True)

                #move towards goal
                yield self.move.set_position(start_circle_pos).set_orientation(start_circle_ori).go(blind=True)

                #calculate vector defining square
                square_vec = np.array([ start_circle_pos[0] - animal_pose[0][0], start_circle_pos[1] - animal_pose[0][1], 0 ])

                '''
                print("arrived in circle")
                for i in range(granularity):
                    #Move around animals

                    #add 90deg to square_vec
                    if current_animal == "platypus":
                        square_vec = np.cross(square_vec, z_vec)
                    if current_animal == "turtle":
                        square_vec = np.cross(z_vec, square_vec)

                    #calculate new goal position
                    animal_pose = yield self.geo_pose_to_enu_pose(path_msg.poses[path[i]].pose)
                    goal_pos = [ animal_pose[0][0] + square_vec[0], animal_pose[0][1] + square_vec[1], 0 ]

                    #calculate new goal orientation
                    r,p,y = tf.transformations.euler_from_quaternion(self.pose[1])
                    if current_animal == "platypus":
                        goal_ori = tf.transformations.quaternion_from_euler(0,0,y - math.pi/2)
                    if current_animal == "turtle":
                        goal_ori = tf.transformations.quaternion_from_euler(0,0,y + math.pi/2)

                    yield self.move.set_position(goal_pos).set_orientation(goal_ori).go(blind=True)
                '''
                
            elif current_animal == "crocodile":
                print("Avoiding crocodile")
                
                #we will check if the crocodile lies within either of two rectangles where the touching points
                #of the rectangles are the current pos and goal pos.
                #based on where the crocodile is will determine our action for how to get to the goal pos.

                #get animal msgs
                path_msg = yield self.get_latching_msg(self.animal_landmarks)
                animal_pose = yield self.geo_pose_to_enu_pose(path_msg.poses[path[i+1]].pose)
                animal_pose_croc = yield self.geo_pose_to_enu_pose(path_msg.poses[path[i]].pose)

                start_circle_pos = self.closest_point_on_radius(self.pose[0], animal_pose[0], radius)

                if current_animal == "platypus":
                    start_circle_ori = self.start_angle(start_circle_pos, animal_pose[0], interior_angle)
                if current_animal == "turtle":
                    start_circle_ori = self.start_angle(start_circle_pos, animal_pose[0], -interior_angle)

                #define left and right rectangles
                rectangle_width = 15
                vect = start_circle_pos - self.pose[0]
                flipped_vect = self.pose[0] - start_circle_pos
                norm_vect = vect / np.linalg.norm(vect)
                flipped_norm_vect = flipped_vect / np.linalg.norm(flipped_vect)
                p1_l = np.cross(z_vec, norm_vect) * rectangle_width + self.pose[0]
                p2_l = start_circle_pos
                p1_r = self.pose[0]
                p2_r = np.cross(z_vec, flipped_norm_vect) * rectangle_width + start_circle_pos
                x_croc = animal_pose_croc[0][0]
                y_croc = animal_pose_croc[0][1]

                print(p1_l)
                print(p2_l)
                print(p1_r)
                print(p2_r)
                print(x_croc)
                print(y_croc)

                #first point at goal
                orientation_fix = self.point_at_goal(start_circle_pos)

                #determine if croc is in left rect
                AM = animal_pose_croc[0] - p1_r
                AB = p1_l - p1_r
                AD = p2_l - p1_r

                print(AM)
                print(AB)
                print(AD)

                AMAB = np.dot(AM, AB)
                ABAB = np.dot(AB, AB)
                AMAD = np.dot(AM, AD)
                ADAD = np.dot(AD, AD)

                if (0 < AMAB < ABAB) and (0 < AMAD < ADAD):
                    #calculate pitstop point
                    print("WARNING: Crocodile is in left rectangle")
                    pitstop_pos = p2_r + 0.5 * flipped_vect
                    yield self.move.set_position(pitstop_pos).set_orientation(orientation_fix).go(blind=True)
                    continue

                #determine if croc is in right rect
                AM = animal_pose_croc[0][0] - p2_l
                AB = p1_r - p2_l
                AD = p2_r - p2_l

                AMAB = np.dot(AM, AB)
                ABAB = np.dot(AB, AB)
                AMAD = np.dot(AM, AD)
                ADAD = np.dot(AD, AD)

                if (0 < AMAB < ABAB) and (0 < AMAD < ADAD):
                    #calculate pitstop point
                    print("WARNING: Crocodile is in right rectangle")
                    pitstop_pos = p1_l + 0.5 * vect
                    yield self.move.set_position(pitstop_pos).set_orientation(orientation_fix).go(blind=True)






