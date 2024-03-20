#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission
import numpy as np
import math

# This mission will direct the sub towards a red buoy, and continously circumnavigate
# the buoy in a CCW motion
class HydrothermalVent(SubjuGatorMission):
    buoy_pos = [1, 2, 4]
    async def run(self, args):
        pitch_angle = np.arctan(self.buoy_pos[2]/self.buoy_pos[1])
        yaw_angle = np.arctan(self.buoy_pos[2]/self.buoy_pos[0])
        self.send_feedback(f"Rotating towards Buoy with yaw {math.degrees(yaw_angle)} and pitch {math.degrees(pitch_angle)}")

        rotate = self.move().set_roll_pitch_yaw(0, pitch_angle, yaw_angle)
        # Multiply quaternarion by 1? (makes sure angles dont overwrite)
        await self.go(rotate)            

        # does this actually go forward relative to orientation
        self.send_feedback(f"Traveling forward to buoy")
        await self.go(
            self.move()
                .forward(self.buoy_pos[2] - 2) # don't reach the buoy, remain 2 units away
        )

        # rotate 90 degrees:
        await self.go(self.move().set_roll_pitch_yaw(0, 90, 0))

        # continuosly rotate around the buoy
        while True:
            sub_current_pos = await self.tx_pose()
            delta_x = sub_current_pos[0] - self.buoy_pos[0]
            delta_y = sub_current_pos[1] - self.buoy_pos[1]
            yaw_angle = np.arctan2(delta_y, delta_x)  # Calculate angle based on position difference

            await self.go(self.move().forward(0.2))
            await self.go(self.move().set_roll_pitch_yaw(0, 0, math.degrees(yaw_angle)))



