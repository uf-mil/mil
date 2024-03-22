#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission
import numpy as np
import math

# This mission will direct the sub towards a red buoy, and continously circumnavigate
# the buoy in a CCW motion
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
                .forward(self.buoy_pos[2] - 2) # don't reach the buoy, remain 2 units away
        )
        
        # rotate 90 degrees:
        self.send_feedback("Rotating 90 degrees")
        rotate = self.move().set_roll_pitch_yaw(0, 0, np.radians(90))
        await self.go(rotate)

        # continuosly rotate around the buoy
        while True:
            sub_current_pos = await self.tx_pose()  # Assuming tx_pose() returns a tuple

        # Figure out why this is not working:

            #sub_current_pos = np.array(sub_current_pos[:2])  # Extract position information and convert to numpy array
            # delta = sub_current_pos - self.buoy_pos  # Calculate position difference
            #yaw_angle = np.arctan2(delta[1], delta[0])  # Calculate angle based on position difference
            #rotate = self.move().set_roll_pitch_yaw(0, 0, yaw_angle)
            #await self.go(self.move().forward(1))
            #await self.go(rotate)



