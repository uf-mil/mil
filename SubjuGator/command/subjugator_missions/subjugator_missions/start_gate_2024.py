#!/usr/bin/env python3

import math

from vision_stack.msg import ObjectDetections

from .sub_singleton import SubjuGatorMission

# Constants
YAW_LENGTH = 15  # When searching for start gate how many degrees to turn
SPEED = 0.2
FRAME_WIDTH = 960  # Pixels


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

    async def update(self):
        """Update is continuously called"""
        while not self.isCompleted:
            # Get the detections from vision stack
            detections_msg = await self.detections_sub.get_next_message()
            detections = detections_msg.detections

            # Update start gate estimates
            if not self.getInfo(detections):
                # If we did not see the start gate look for it
                await self.lookForGate()
                continue

            # Now that we see the gate move towards it
            await self.approachGate()

    async def endMission(self):
        """Called on completion of the mission"""
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
                self.centerX
                + spiral_detections[0].center_x
                + spiral_detections[1].center_x
            ) / 3
            self.centerY = (
                self.centerY
                + spiral_detections[0].center_y
                + spiral_detections[1].center_y
            ) / 3

            print(f"Center X: {self.centerX}, Center Y: {self.centerY}")

            centerDistX = abs(self.centerX - spiral_detections[0].center_x)
            centerDistY = abs(self.centerY - spiral_detections[0].center_y)

            print(f"CenterDist X: {centerDistX}, CenterDist Y: {centerDistY}")

            # Update angle estimate
            self.angle_est = math.log(
                centerDistY + 1,
                10,
            )  # This equation was obtained experimentally
            print(f"angle: {self.angle_est}")

            # Update dist estimate
            adjusted_centerDistX = centerDistX / math.cos(self.angle_est)

            self.dist_est = abs(
                700 / adjusted_centerDistX,
            )  # This equation was obtained experimentally
            print(f"dist: {self.dist_est}")

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
        await self.go(
            self.move().forward(self.dist_est - 2),
            speed=SPEED,
        )
        await self.go(
            self.move().yaw_left(self.angle_est),
            speed=SPEED,
        )
