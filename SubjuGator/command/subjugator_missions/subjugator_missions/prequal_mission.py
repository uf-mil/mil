#! /usr/bin/env python3
import math

import cv2
import numpy as np
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from .sub_singleton import SubjuGatorMission

SPEED_LIMIT = 0.5  # m/s


class PrequalMission(SubjuGatorMission):
    async def run(self, args):
        # obtain camera data

        self.bridge = CvBridge()
        self.front_left_camera_sub = self.nh.subscribe(
            "/camera/front/left/image_color",
            Image,
        )
        self.image_debug_pub = self.nh.advertise("/prequal_image_debug", Image)
        await self.image_debug_pub.setup()

        # submerge submarine
        await self.move.down(2).zero_roll_and_pitch().go(speed=SPEED_LIMIT)

        # look for start gate
        await self.find_start_gate()

        # look for pole
        await self.find_marker()

        # turn around
        await self.nh.sleep(5)
        await self.move.yaw_right(1.57).zero_roll_and_pitch().go(speed=SPEED_LIMIT)

        # go through start gate again
        await self.nh.sleep(5)
        await self.find_start_gate()

        print("Done!")

    # returns center of first and last vertical lines
    async def find_start_gate(self):
        center_pixel = 480

        while True:
            img = await self.front_left_camera_sub.get_next_message()
            img = self.bridge.imgmsg_to_cv2(img)

            # obtain one image
            hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            _, width, height = hsv_img.shape[::-1]

            # mask for only black
            lower = np.array([0, 0, 0], dtype="uint8")
            upper = np.array([50, 50, 50], dtype="uint8")
            mask = cv2.inRange(hsv_img, lower, upper)

            # publish mask for debuggin purposes
            masked_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
            self.image_debug_pub.publish(masked_msg)

            white = False
            black = False
            vertical_lines = []

            for i in range(width):
                if mask[(height / 2), i] != 0 and not white:
                    white = True
                    black = False
                    vertical_lines.append(i)

                elif mask[(height / 2), i] == 0 and not black:
                    white = False
                    black = True

            center_pixel = width / 2
            if len(vertical_lines) > 1:
                center_pixel = (
                    vertical_lines[0] + vertical_lines[len(vertical_lines) - 1]
                ) / 2
            else:
                print("Going through the gate!")
                await self.move.forward(5).zero_roll_and_pitch().go(speed=SPEED_LIMIT)
                break

            # move based on center pixel
            # if difference is negative, we adjust right
            # if difference is positive, we adjust left

            print(center_pixel)

            difference = (width / 2) - center_pixel
            magic_ratio = 1000.0
            meters = difference / magic_ratio

            if abs(difference) > 30:
                if difference < 0:
                    print("Adjusting right", abs(meters))
                    await self.move.yaw_right(abs(meters)).zero_roll_and_pitch().go(
                        speed=SPEED_LIMIT,
                    )
                elif difference > 0:
                    print("Adjusting left", abs(meters))
                    await self.move.yaw_left(abs(meters)).zero_roll_and_pitch().go(
                        speed=SPEED_LIMIT,
                    )

            print("Going forward and inspecting again")
            await self.move.forward(2).zero_roll_and_pitch().go(speed=SPEED_LIMIT)

    # returns center of marker and width
    async def find_marker(self):
        while True:
            img = await self.front_left_camera_sub.get_next_message()
            img = self.bridge.imgmsg_to_cv2(img)

            # obtain one image
            hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            _, width, height = hsv_img.shape[::-1]

            # mask for only blue (to get only the water for the most part)
            lower = np.array([0, 0, 0], dtype="uint8")
            upper = np.array([50, 50, 50], dtype="uint8")
            mask = cv2.inRange(hsv_img, lower, upper)

            # publish mask for debuggin purposes
            masked_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
            self.image_debug_pub.publish(masked_msg)

            white = False
            black = False
            vertical_lines = []
            start = 0
            stop = 0

            for i in range(width):
                if mask[(height / 2), i] != 0 and not white:
                    white = True
                    black = False
                    vertical_lines.append(i)
                    start = i

                elif mask[(height / 2), i] == 0 and not black:
                    white = False
                    black = True
                    stop = i

            if len(vertical_lines) == 1:
                center_pixel = vertical_lines[0]
            else:
                # There is more than on vertical line
                print("I am confused")
                center_pixel = width / 2

            difference = (width / 2.0) - center_pixel
            magic_ratio = 1000.0
            angle = difference / magic_ratio

            # check if we are aligned or not with center of pole
            if abs(difference) > 30:
                if difference < 0:
                    print("Adjusting right")
                    await self.move.yaw_right(abs(angle)).zero_roll_and_pitch().go(
                        speed=SPEED_LIMIT,
                    )
                elif difference > 0:
                    print("Adjusting left")
                    await self.move.yaw_left(abs(angle)).zero_roll_and_pitch().go(
                        speed=SPEED_LIMIT,
                    )

            # if the width of the pole is bigger than a certain amount, rotate around pole
            if stop - start > 25:
                # this is where we would call the rotation function
                await self.circle_marker()
                break

            await self.move.forward(2).zero_roll_and_pitch().go(speed=SPEED_LIMIT)

            print("Marker is in pixel: " + str(center_pixel))
            print("Pipe width: " + str(stop - start))
            print("\n")

    async def circle_marker(self):
        print("Entering circle marker function")

        steps = 8

        # get vector between sub and estimated marker location
        sub_position = await self.pose.position
        sub_orientation = await self.pose.orientation
        center_point = self.get_point_in_front_of_sub(
            sub_position,
            sub_orientation,
            2.5,
        )
        vect = np.array(
            [sub_position[0] - center_point[0], sub_position[1] - center_point[1], 0],
        )

        # go around animal
        for i in range(steps - 2):
            print(i)

            # calculate new position by rotating vector
            new_vect = self.rotate_vector(vect[0:2], math.radians(360 / steps))
            vect[0] = new_vect[0]
            vect[1] = new_vect[1]
            new_pos = center_point + vect

            # move to next spot in circle
            print(new_pos)
            print(vect)
            await self.move.set_position(new_pos).look_at_without_pitching(
                center_point,
            ).go()

    def point_at_goal(self, current_pos, center_pos):
        vect = [center_pos[0] - current_pos[0], center_pos[1] - current_pos[1]]
        theta = math.atan2(vect[1], vect[0])
        return tf.transformations.quaternion_from_euler(0, 0, theta)

    def rotate_vector(self, vector, theta):
        # rotate a vector theta radians
        rot = np.array(
            [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]],
        )
        res = np.dot(rot, vector)
        return res

    def get_point_in_front_of_sub(self, sub_position, sub_ori, distance):
        """Provides a point a certain distance in front of sub"""
        offset = distance
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(sub_ori)
        dx = offset * math.cos(yaw)
        dy = offset * math.sin(yaw)

        new_pos = np.array([0.0, 0.0, sub_position[2]])
        new_pos[0] = sub_position[0] + dx
        new_pos[1] = sub_position[1] + dy

        return new_pos
