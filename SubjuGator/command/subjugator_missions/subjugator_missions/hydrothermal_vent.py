#!/usr/bin/env python3
import math

import numpy as np

from .sub_singleton import SubjuGatorMission

# This mission will direct the sub towards a red buoy, and continuously circumnavigate
# the buoy in a CCW motion


# Computer vision aspect still required to detect the red buoy. For now, random position used
class HydrothermalVent(SubjuGatorMission):
    async def run(self, args):
        self.buoy_pos = np.array([3, 4, -2])  # Convert buoy position to numpy array

        # First, move down the necessary depth to reach the buoy

        self.send_feedback("Submerging to buoy depth")
        if self.buoy_pos[2] < 0:
            await self.go(self.move().down(abs(self.buoy_pos[2] + 0.3)))
        else:
            await self.go(self.move().up(self.buoy_pos[2] - 0.3))
        yaw_angle = np.arctan(self.buoy_pos[1] / self.buoy_pos[0])
        self.send_feedback(f"Rotating towards Buoy with yaw {math.degrees(yaw_angle)}")
        rotate = self.move().set_roll_pitch_yaw(0, 0, yaw_angle + 0.1)
        await self.go(rotate)

        self.send_feedback("Traveling forward to buoy")
        await self.go(
            self.move().forward(
                np.sqrt(np.square(self.buoy_pos[0]) + np.square(self.buoy_pos[1]))
                - 0.7,
            ),  # don't reach the buoy, remain 0.5 meter away
        )
        yaw_angle2 = np.deg2rad(90)
        # rotate 90 degrees:
        self.send_feedback("Rotating 90 degrees left")
        rotate = self.move().yaw_left(yaw_angle2)
        await self.go(rotate)

        self.send_feedback("Circumnaviganting the buoy")
        await self.go(self.move().forward(0.7))
        for i in range(0, 3):
            rotate = self.move().yaw_right(yaw_angle2)
            await self.go(rotate)
            await self.go(self.move().forward(1.4))
        rotate = self.move().yaw_right(yaw_angle2)
        await self.go(rotate)
        await self.go(self.move().forward(0.7))

        self.send_feedback("Returning to origin")
        await self.go(self.move().yaw_left(yaw_angle2))
        await self.go(
            self.move().forward(
                np.sqrt(np.square(self.buoy_pos[0]) + np.square(self.buoy_pos[1])) - 1,
            ),
        )

        await self.go(self.move().set_roll_pitch_yaw(0, 0, -yaw_angle))

        await self.go(self.move().up(self.buoy_pos[1]))

    # This function is supposed to center the buoy based on position given in terms of xy position of the center of the red buoy, with
    # respect to the center of the camera, as well as the distance from the buoy
    async def center_bouy(self, buoy_pos_c):
        while (abs(buoy_pos_c[0]) > 0.01) and (abs(buoy_pos_c[1]) > 0.01):
            await self.go(self.move().depth(buoy_pos_c[1]))
            yaw_angle = np.arctan(buoy_pos_c[0] / buoy_pos_c[2])
            await self.go(self.move().set_roll_pitch_yaw(0, 0, yaw_angle))

            buoy_pos_c = buoy_pos_c  # this will be replaced with the computer vision call to check the position of the buoy
