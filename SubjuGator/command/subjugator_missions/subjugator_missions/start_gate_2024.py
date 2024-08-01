#!/usr/bin/env python3

import math

from sensor_msgs.msg import MagneticField
from vision_stack.msg import ObjectDetections

from .sub_singleton import SubjuGatorMission

# Constants
PROPER_HEADING = 191.76238  # This is the compass heading that points perpendicular to the dock and towards the course
# Compass heading for away from the dock
ENTER_RIGHT_SIDE = True  # Which side to enter, if false enters left side


SPEED = 0.2
STARTING_DOWN_DIST = 0.6
ENDING_SIDE_DIST = 0.75  # After going through the gate how many meters to move to the side to see the path marker

FRAME_WIDTH = 960  # Pixel width of the camera image

TRANSLATION_STEP = 0.2  # Move increment for trying to center with the red spiral

DIST_CONST = 145  # This is used to estimate distance. This value is found experimentally and may need to be changed
# If new sensors are used. decreasing this decreases the dist est

DEBUG = True


class StartGate2024(SubjuGatorMission):
    async def run(self, args):
        """run is a special function used as the entry point into a mission"""
        # Setup subscribers and initialize variables
        await self.start()

        # Align with proper heading (Look away from the wall)
        await self.alignWithProperHeading()

        # Align with a spiral on the start gate
        await self.alignWithSide()

        # Enter Gate
        await self.enterGate()

        # Call a shutdown function
        await self.endMission()

    async def start(self):
        """This function is called once at the start of the mission. Used for initialization and setup"""
        # Submerge the sub
        await self.go(
            self.move().down(STARTING_DOWN_DIST),
            speed=SPEED,
        )

        # Get the initial heading of the sub
        magnetic_sub = self.nh.subscribe(
            "/imu/mag",
            MagneticField,
        )
        await magnetic_sub.setup()

        # Calculate heading in degrees
        magnetic_msg = await magnetic_sub.get_next_message()
        y = magnetic_msg.magnetic_field.y
        x = magnetic_msg.magnetic_field.x

        self.initial_heading = 0.0
        if y > 0:
            self.initial_heading = 90 - math.atan(x / y) * 180 / math.pi
        if y < 0:
            self.initial_heading = 270 - math.atan(x / y) * 180 / math.pi
        if y == 0 and x > 0:
            self.initial_heading = 180.0

        if DEBUG:
            print(f"Initial Heading: {self.initial_heading}.")

        # Subscribe to object detection message from detections
        self.detections_sub = self.nh.subscribe(
            "/yolo_detections/1/objectDetection_last_2/analysis",
            ObjectDetections,
        )
        await self.detections_sub.setup()

    async def alignWithProperHeading(self):
        """This aligns the sub with the proper heading. Make sure to measure this with a compass
        and that it points away from the wall of the pool. Enter this at the top of this file
        """

        # Calculate angle to rotate
        angle = PROPER_HEADING - self.initial_heading

        # Rotate the sub
        await self.go(
            self.move().yaw_left_deg(angle),
            speed=SPEED,
        )

    async def getDetections(self):
        """Gets the yolo detections and filters them to only output the red and blue spiral"""

        while True:
            # Get the spiral detections
            detection_msg = await self.detections_sub.get_next_message()
            detections = detection_msg.detections

            # Filter out only the spirals
            red_spiral = blue_spiral = None

            for d in detections:
                if d.class_name == "Red spiral":
                    red_spiral = d
                elif d.class_name == "Blue spiral":
                    blue_spiral = d
                if red_spiral and blue_spiral:
                    break

            # Pick which spiral we are going to go through
            current_spiral = red_spiral if ENTER_RIGHT_SIDE else blue_spiral

            # If we dont see the current spiral we are going to go through wait for another detection
            if current_spiral is None:
                continue

            return current_spiral

    async def alignWithSide(self):
        """This aligns the sub with one of the sides of the start gate"""

        # Initialize errors (error being how far the spiral is from the center of the screen)
        prev_error = math.inf
        current_error = 0

        while True:
            # Get the spiral detections
            spiral = await self.getDetections()

            # Calculate the current error
            current_error = (FRAME_WIDTH / 2) - spiral.center_x

            if DEBUG:
                print(f"Current Error: {current_error}, Prev Error: {prev_error}")

            # Check if the error went up since last time. This indicates we were aligned before so stop aligning
            if abs(current_error) > abs(prev_error):
                break

            # Move left or right to try and align. Sign is determined from what side of the screen the current spiral is on.
            await self.go(
                self.move().left(
                    TRANSLATION_STEP * (current_error / abs(current_error)),
                ),
                speed=SPEED,
            )

            # Before we loop again set the prev error to the current
            prev_error = current_error

    async def enterGate(self):
        "This estimate the distance to enter the gate and enters it"

        # Get the spiral detections
        spiral = await self.getDetections()

        # Estimate the distance
        # Using width of the bounding box to estimate distance because the height of the box varies to much to be reliable
        dist_est = DIST_CONST / spiral.width

        if DEBUG:
            print(f"Distance to move: {dist_est}")

        # Move that distance
        await self.go(self.move().forward(dist_est), speed=SPEED)

        # Strafe left or right to see the path marker
        side_dist = ENDING_SIDE_DIST * -1 if ENTER_RIGHT_SIDE else ENDING_SIDE_DIST

        await self.go(self.move().right(side_dist), speed=SPEED)

    async def endMission(self):
        """Called on completion of the mission"""
        print("Start Gate Mission Completed")
        pass
