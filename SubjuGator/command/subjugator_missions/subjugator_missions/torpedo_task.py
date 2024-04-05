#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission
import numpy as np
import math 


class TorpedoTask(SubjuGatorMission):

    buoy_positions = [[5, 10, 3], [6, 8, 1], [3, 12, 1]]  # x, y, z displacements
    
    async def run(self, args):
        for i in range(len(self.buoy_positions)):
            forward_dist = np.sqrt(np.power(self.buoy_positions[i][2],2)-np.power(self.buoy_positions[i][1],2))
            self.buoy_positions[i].append(forward_dist)
            

        for i in range(len(self.buoy_positions)):
            pitch_angle = np.arctan(self.buoy_positions[i][3]/self.buoy_positions[i][1])
            yaw_angle = np.arctan(self.buoy_positions[i][3]/self.buoy_positions[i][0])
            self.send_feedback(f"Rotating towards opening {i} with yaw {math.degrees(yaw_angle)} and pitch {math.degrees(pitch_angle)}")

            rotate = self.move().set_roll_pitch_yaw(0, pitch_angle, yaw_angle)
            await self.go(rotate)            

            # does this actually go forward relative to orientation
            self.send_feedback(f"Traveling forward to buoy {i}")
            await self.go(
                self.move()
                    .forward(self.buoy_positions[i][2])
            )

            
            self.send_feedback("Undo Rotation")
            

            rotate = self.move().set_roll_pitch_yaw(0, -pitch_angle, -yaw_angle)
            await self.go(rotate)

            self.send_feedback("Shoot torpedo")

            await self.actuators.shoot_torpedo1()


            self.send_feedback("Back to Origin")

            rotate = self.move().set_roll_pitch_yaw(0, pitch_angle, yaw_angle)
            await self.go(rotate)
            
            await self.go(
                self.move()
                .forward(-(self.buoy_positions[i][2]))
            )

