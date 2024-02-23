#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission
import numpy as np


class StartSignal(SubjuGatorMission):
    buoy_positions = [[5, 10, 3], [6, 8, 5], [3, 12, 10]]  # x, y, z displacements

    async def run(self, args):
        for i in len(self.buoy_positions):
            self.send_feedback(f"Travelling to Buoy {i}")
            await self.go(
                self.move()
                .right(self.buoy_positions[i][0])
                .up(self.buoy_positions[i][1].forward(self.buoy_positions[i][2])),
            )
            self.send_feedback("Back to Origin")
            await self.go(
                self.move()
                .right(-self.buoy_positions[i][0])
                .up(-self.buoy_positions[i][1].forward(-self.buoy_positions[i][2])),
            )

    async def run_rotation(self, args):
        for i in len(self.args):
            pitch_angle = np.arctan(self.buoy_positions[i][2]/self.buoy_positions[i][1])
            yaw_angle = np.arctan(self.buoy_positions[i][0]/self.buoy_positions[i][1])
            self.send_feedback(f"Rotating towards Buoy {i}")
            await self.go(
                # Yaw to the right if the x distance is positive and to the left if negative
                # Yaw to left otherwise
                #
                # If the pitch angle is positive, pitch up, if negative, pitch down
            )
