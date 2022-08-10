#!/usr/bin/env python3
from __future__ import annotations

import math

import numpy as np
import rospy
import tf
from geographic_msgs.msg import GeoPath, GeoPose, GeoPoseStamped
from mil_tools import rosmsg_to_numpy
from nav_msgs.msg import Odometry
from navigator_msgs.srv import ChooseAnimal, ChooseAnimalRequest, ChooseAnimalResponse
from robot_localization.srv import FromLL
from twisted.internet import defer


# Defines a service that returns the position of the acoustic beacon from odom and the range_bearing topic
class CircleAnimal:
    def __init__(self):
        rospy.init_node("circle_animal")
        self.odom = rospy.Subscriber("/odom", Odometry, self.odometrySubscriber)
        self.animals = rospy.Subscriber(
            "/vrx/wildlife/animals/poses", GeoPath, self.animalSubscriber
        )
        self.pub = rospy.Publisher("/trajectory_long/cmd", Odometry, queue_size=10)
        self.serv = rospy.Service("/choose_animal", ChooseAnimal, self.handler)
        self.from_lla = rospy.ServiceProxy("/fromLL", FromLL)
        self.boat_pos = np.array([])
        self.boat_ori = np.array([])
        self.target_animal = ""
        self.new_animal_pose = False
        self.animal_pos = np.array([])
        self.animal_ori = np.array([])
        rospy.spin()

    # Requires both odom and range_bearing to be publishing data
    def odometrySubscriber(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.boat_pos = rosmsg_to_numpy(pos)
        self.boat_ori = rosmsg_to_numpy(ori)

    def animalSubscriber(self, msg: GeoPath) -> None:
        # if target animal hasn't been selected, do not provide more data
        if self.target_animal == "" or self.new_animal_pose == True:
            return

        poses: list[GeoPoseStamped] = msg.poses
        for animal in poses:
            if self.target_animal == animal.header.frame_id:
                self.animal_pos, self.animal_ori = self.geo_pose_to_enu_pose(
                    animal.pose
                )
                self.new_animal_pose = True

    def handler(self, req: ChooseAnimalRequest) -> ChooseAnimalResponse:
        # define messages
        self.target_animal = req.target_animal
        res = ChooseAnimalResponse()
        traj = Odometry()
        rate = rospy.Rate(5)
        self.new_animal_pose = False

        print("Starting Service")
        while (self.boat_pos.size == 0) or (self.animal_pos.size == 0):
            rate.sleep()

        print("Odom and Animal has been found")
        # create goal pose and vector used for creating circle poses
        radius = 6
        granularity = 8

        if self.target_animal != "crocodile":
            start_circle_pos = self.closest_point_on_radius(
                self.boat_pos, self.animal_pos, radius
            )
        if self.target_animal == "crocodile":
            start_circle_pos = self.closest_point_on_radius(
                self.boat_pos, self.animal_pos, 15
            )

        start_circle_ori = self.point_at_goal(start_circle_pos, self.animal_pos)
        start_boat_pos = self.boat_pos
        mid_point = (start_circle_pos + self.boat_pos) / 2
        vect = np.array(
            [
                start_circle_pos[0] - self.animal_pos[0],
                start_circle_pos[1] - self.animal_pos[1],
            ]
        )
        start_circle_vector = self.animal_pos - start_circle_pos

        # go half way to animal
        while (abs(self.boat_pos[0] - mid_point[0]) > 0.5) or (
            abs(self.boat_pos[1] - mid_point[1]) > 0.5
        ):
            self.new_animal_pose = False

            # create boat trajectory
            traj.header.frame_id = "enu"
            traj.child_frame_id = "wamv/base_link"
            traj.pose.pose.position.x = mid_point[0]
            traj.pose.pose.position.y = mid_point[1]
            traj.pose.pose.orientation.x = start_circle_ori[0]
            traj.pose.pose.orientation.y = start_circle_ori[1]
            traj.pose.pose.orientation.z = start_circle_ori[2]
            traj.pose.pose.orientation.w = start_circle_ori[3]
            self.pub.publish(traj)

            while not self.new_animal_pose:
                rate.sleep()

            # update trajectory point
            mid_point = (start_circle_pos + start_boat_pos) / 2

            start_circle_ori = self.point_at_goal(start_circle_pos, self.animal_pos)

        # go to animal
        while (abs(self.boat_pos[0] - start_circle_pos[0]) > 0.5) or (
            abs(self.boat_pos[1] - start_circle_pos[1]) > 0.5
        ):
            self.new_animal_pose = False

            # create boat trajectory
            traj.header.frame_id = "enu"
            traj.child_frame_id = "wamv/base_link"
            traj.pose.pose.position.x = start_circle_pos[0]
            traj.pose.pose.position.y = start_circle_pos[1]
            traj.pose.pose.orientation.x = start_circle_ori[0]
            traj.pose.pose.orientation.y = start_circle_ori[1]
            traj.pose.pose.orientation.z = start_circle_ori[2]
            traj.pose.pose.orientation.w = start_circle_ori[3]
            self.pub.publish(traj)

            while not self.new_animal_pose:
                rate.sleep()

            # update trajectory point
            start_circle_pos = self.animal_pos - start_circle_vector

            start_circle_ori = self.point_at_goal(start_circle_pos, self.animal_pos)

        if self.target_animal == "crocodile":
            steps = granularity // 4
        else:
            steps = granularity

        # go around animal
        for i in range(steps + 1):
            print(i)
            # calculate new position by rotating vector

            if req.circle_direction == "cw":
                vect = self.rotate_vector(vect, math.radians(-360 / granularity))
            elif req.circle_direction == "ccw":
                vect = self.rotate_vector(vect, math.radians(360 / granularity))

            new_pos = self.animal_pos[0:2] + vect
            circle_ori = self.point_at_goal(new_pos, self.animal_pos)

            while (abs(self.boat_pos[0] - new_pos[0]) > 0.5) or (
                abs(self.boat_pos[1] - new_pos[1]) > 0.5
            ):
                self.new_animal_pose = False

                # create boat trajectory
                traj.header.frame_id = "enu"
                traj.child_frame_id = "wamv/base_link"
                traj.pose.pose.position.x = new_pos[0]
                traj.pose.pose.position.y = new_pos[1]
                traj.pose.pose.orientation.x = circle_ori[0]
                traj.pose.pose.orientation.y = circle_ori[1]
                traj.pose.pose.orientation.z = circle_ori[2]
                traj.pose.pose.orientation.w = circle_ori[3]
                self.pub.publish(traj)

                while not self.new_animal_pose:
                    rate.sleep()

                new_pos = self.animal_pos[0:2] + vect
                circle_ori = self.point_at_goal(new_pos, self.animal_pos)

        print("arrived")
        self.target_animal = ""
        self.boat_pos = np.array([])
        self.animal_pos = np.array([])
        res.movement_complete = True
        return res

    def closest_point_on_radius(
        self, start_pos: np.ndarray, end_pos: np.ndarray, radius: float
    ) -> list[float]:
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

    def geo_pose_to_enu_pose(self, geo: GeoPose) -> tuple[np.ndarray, np.ndarray]:
        enu_msg = self.from_lla(geo.position)
        position_enu = rosmsg_to_numpy(enu_msg.map_point)
        orientation_enu = rosmsg_to_numpy(geo.orientation)
        return position_enu, orientation_enu

    def rotate_vector(self, vector: np.ndarray, theta: float):
        # rotate a vector theta radians
        rot = np.array(
            [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
        )
        res = np.dot(rot, vector)
        return res

    def point_at_goal(self, current_pos: list[float], animal_pos: np.ndarray):
        vect = [animal_pos[0] - current_pos[0], animal_pos[1] - current_pos[1]]
        theta = math.atan2(vect[1], vect[0])
        return tf.transformations.quaternion_from_euler(0, 0, theta)


if __name__ == "__main__":
    CircleAnimal()
