#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission
import numpy as np
import math

# This mission will direct the sub towards a red buoy, and continously circumnavigate
# the buoy in a CCW motion


# Computer vision aspect still required to detect the red buoy. For now, random position used
class HydrothermalVent(SubjuGatorMission):
    async def run(self, args):
        self.buoy_pos = np.array([3, 2, 4])  # Convert buoy position to numpy array

        # First, move down the necessary depth to reach the buoy

        self.send_feedback("Submerging to buoy depth")
        await self.go(self.move().down(self.buoy_pos[1]))
        yaw_angle = np.arctan(self.buoy_pos[2]/self.buoy_pos[0])
        self.send_feedback(f"Rotating towards Buoy with yaw {math.degrees(yaw_angle)}")
        rotate = self.move().set_roll_pitch_yaw(0, 0, yaw_angle)
        await self.go(rotate) 
        
        self.send_feedback(f"Traveling forward to buoy")
        await self.go(
            self.move()
                .forward(self.buoy_pos[2] - 1) # don't reach the buoy, remain 1 meter away
        )
        yaw_angle2 = np.deg2rad(90)
        # rotate 90 degrees:
        self.send_feedback("Rotating 90 degrees left")
        rotate = self.move().yaw_left(yaw_angle2)
        await self.go(rotate)
        
        self.send_feedback("Circumnaviganting the buoy")
        await self.go(self.move().forward(0.5))
        for i in range(0, 3):
            rotate = self.move().yaw_right(yaw_angle2)
            await self.go(rotate)
            await self.go(self.move().forward(1))
        rotate = self.move().yaw_right(yaw_angle2)
        await self.go(rotate)
        await self.go(self.move().forward(0.5))

        self.send_feedback("Returning to origin")
        await self.go(self.move().yaw_left(yaw_angle2))
        await self.go(self.move().forward(self.buoy_pos[2] - 1))

        await self.go(self.move().set_roll_pitch_yaw(0, 0, -yaw_angle))

        await self.go(self.move().up(self.buoy_pos[1]))


