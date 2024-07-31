#!/usr/bin/env python3

import math

from vision_stack.msg import ObjectDetections

from .sub_singleton import SubjuGatorMission

# Constants
YAW_LENGTH = 15  # When searching for start gate how many degrees to turn
SPEED = 0.2
FRAME_WIDTH = 960  # Pixels

DIST_CONST = 700
TRANSLATION_CONST = 1 / 700
ANGLE_CONST = 0.942334

ENTER_RIGHT = True  # Enter the right side of the gate (if false enters the left side)
DOWN_DIST = 0.7  # meters to submerge to enter the gate
FORWARD_DIST = 2.5  # how many meters to go through the gate
SIDE_DIST = 0.7  # how many meters to go under one of the spirals

DEBUG = True


class StartGate2024(SubjuGatorMission):
    async def run(self, args):
        """run is a special function used as the entry point into a mission"""
        # Call a one time setup function
        await self.start()

        # Start an update loop
        await self.update()

        # Call a shutdown function
        await self.endMission()

    async def start(self):
        """This function is called once at the start of the mission. Used for initialization and setup"""

        # Subscribe to object detection message from detections
        self.detections_sub = self.nh.subscribe(
            "/yolo_detections/1/objectDetection_last_2/analysis",
            ObjectDetections,
        )
        await self.detections_sub.setup()

        # Initialize variables
        self.isCompleted = False
        self.centerX = 0
        self.centerY = 0
        self.dist_est = 0
        self.angle_est = 0
        self.width_ratio = 0

        # Submerge slightly
        # await self.go(
        #     self.move().down(DOWN_DIST),
        #     speed=SPEED,
        # )

    async def update(self):
        """Update is continuously called"""
        while not self.isCompleted:
            # Get the detections from vision stack
            detections_msg = await self.detections_sub.get_next_message()
            detections = detections_msg.detections

            # Update start gate estimates
            self.getInfo(detections)

            # if not self.getInfo(detections) :
            #     # If we did not see the start gate look for it
            #     await self.lookForGate()
            #     continue

            # # Now that we see the gate move towards it
            # await self.approachGate()

            # # Then enter the gate and complete the mission
            # await self.enterGate()

            # # Complete mission
            # self.isCompleted = True

    async def endMission(self):
        """Called on completion of the mission"""
        print("Start Gate Mission Completed")
        pass

    def getInfo(self, detections):
        """This updates the current information about the start gate
        Returns True if new info was saved otherwise returns false
        """

        # Filter out other detections
        spiral_detections = [
            d for d in detections if d.class_name in ["Red spiral", "Blue spiral"]
        ]

        # Make sure we have a red and blue detection
        if len(spiral_detections) == 2:
            # Update center position
            self.centerX = (
                +spiral_detections[0].center_x + spiral_detections[1].center_x
            ) / 2
            self.centerY = (
                +spiral_detections[0].center_y + spiral_detections[1].center_y
            ) / 2

            centerDistX = abs(self.centerX - spiral_detections[0].center_x)
            # centerDistY = abs(self.centerY - spiral_detections[0].center_y)

            if spiral_detections[0].class_name == "Red spiral":
                self.width_ratio = (
                    spiral_detections[1].width / spiral_detections[0].width
                )
            else:
                self.width_ratio = (
                    spiral_detections[0].width / spiral_detections[1].width
                )

            # Update angle estimate
            self.angle_est = (self.width_ratio - 1) * ANGLE_CONST

            # Update dist estimate
            adjusted_centerDistX = centerDistX / math.cos(self.angle_est)

            # These values were obtained experimentally, but the actual equation should be physical dist between two spirals * focal length of camera / centerDistX
            self.dist_est = abs(DIST_CONST / adjusted_centerDistX)

            if DEBUG:
                print(f"angle: {self.angle_est}")
                print(f"dist: {self.dist_est}")
                print(f"width_r: {self.width_ratio}")
                print(f"center dist x: {centerDistX}")
                print(f"center dist FOV: {abs(self.centerX-FRAME_WIDTH/2)}")
            return True

        return False

    async def lookForGate(self):
        """Called to have the sub rotate until it sees the start gate"""

        dir = 1  # Going to look right by default

        # Determine which direction to look
        if self.centerX < FRAME_WIDTH / 2:
            # Look left
            dir = -1

        if self.centerX == 0:
            # If we have never seen the start gate look to the right first
            dir = 1

        # Create the pose for the sub to move towards and move to it at SPEED
        await self.go(
            self.move().yaw_right_deg(dir * YAW_LENGTH).zero_roll_and_pitch(),
            speed=SPEED,
        )

    async def approachGate(self):
        """Called to have the sub move towards the start gate"""

        # Strafe left or right depending on the angle of the start gate
        # Approximate dist as parallel distance
        parallel_dis = self.dist_est * math.sin(self.angle_est)

        # TODO: Check negative angles

        # Yaw left or right to align with the start gate
        await self.go(
            self.move().yaw_left(self.angle_est),
            speed=SPEED,
        )

        # Align with center of gate
        await self.go(
            self.move().right(parallel_dis),
            speed=SPEED,
        )

        # # Align with center of spiral
        await self.go(
            self.move().left((FRAME_WIDTH / 2 - self.centerX) * TRANSLATION_CONST),
            speed=SPEED,
        )

        # Approach Gate, but dont enter (that is what the minus 2 is for)
        dist = (
            self.dist_est * math.cos(self.angle_est) - 2
            if self.dist_est * math.cos(self.angle_est) > 2
            else 0
        )
        await self.go(
            self.move().forward(dist),
            speed=SPEED,
        )

    async def enterGate(self):
        # Choose a side
        side = SIDE_DIST if ENTER_RIGHT else -1 * SIDE_DIST
        await self.go(
            self.move().right(side),
            speed=SPEED,
        )

        # Enter the gate
        await self.go(
            self.move().forward(FORWARD_DIST),
            speed=SPEED,
        )

        # Move back to the center
        await self.go(
            self.move().right(side * -1),
            speed=SPEED,
        )
