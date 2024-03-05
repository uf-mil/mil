#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission
import numpy as np
import math 


class StartSignal(SubjuGatorMission):

    buoy_positions = [[5, 10, 3], [6, 8, 1], [3, 12, 1]]  # x, y, z displacements
    
    async def run(self, args):
        for i in range(len(self.buoy_positions)):
            pitch_angle = np.arctan(self.buoy_positions[i][2]/self.buoy_positions[i][1])
            yaw_angle = np.arctan(self.buoy_positions[i][2]/self.buoy_positions[i][0])
            self.send_feedback(f"Rotating towards Buoy {i} with yaw {math.degrees(yaw_angle)} and pitch {math.degrees(pitch_angle)}")

            rotate = self.move().set_roll_pitch_yaw(0, pitch_angle, yaw_angle)
            # Multiply quaternarion by 1? (makes sure angles dont overwrite)
            await self.go(rotate)            

            # does this actually go forward relative to orientation
            self.send_feedback(f"Traveling forward to buoy {i}")
            await self.go(
                self.move()
                    .forward(self.buoy_positions[i][2])
            )



            self.send_feedback("Back to Origin")
            await self.go(
                self.move()
                .forward(-(self.buoy_positions[i][2]))
            )


            self.send_feedback("Undo Rotation")
            
            pitch_angle = -np.arctan(self.buoy_positions[i][2]/self.buoy_positions[i][1])
            yaw_angle = -np.arctan(self.buoy_positions[i][2]/self.buoy_positions[i][0])
            rotate = self.move().set_roll_pitch_yaw(0, pitch_angle, yaw_angle)
            await self.go(rotate)
    
        # async def run_rotation(self, args):
        # for i in len(self.args):
        #     pitch_angle = np.arctan(self.buoy_positions[i][2]/self.buoy_positions[i][1])
        #     yaw_angle = np.arctan(self.buoy_positions[i][2]/self.buoy_positions[i][0])
        #     self.send_feedback(f"Rotating towards Buoy {i}")
        #     await self.go(
        #         # Yaw to the right if the x distance is positive and to the left if negative
        #         self.move()
        #         .yaw_left(yaw_angle) if self.buoy_positions[i][0] < 0 else self.move().yaw_right(yaw_angle)
        #         # Pitch up if angle is postive, pitch down is angle is negative
        #         .pitch_up(pitch_angle) if pitch_angle > 0 else self.move().pitch_down(abs(pitch_angle))
        #     )
        #     self.send_feedback(f"Traveling forward to buoy {i}")
        #     await self.go(
        #         self.move()
        #             .forward(self.buoy_positions[i][2])
        #     )
        #     self.send_feedback("Back to Origin")
        #     await self.go(
        #         self.move()
        #         .forward(-self.buoy_positions[i][2])
        #     )

    # async def run(self, args):
    #     for i in len(self.buoy_positions):
    #         self.send_feedback(f"Travelling to Buoy {i}")
    #         await self.go(
    #             self.move()
    #             .right(self.buoy_positions[i][0])
    #             .up(self.buoy_positions[i][1].forward(self.buoy_positions[i][2])),
    #         )
    #         self.send_feedback("Back to Origin")
    #         await self.go(
    #             self.move()
    #             .right(-self.buoy_positions[i][0])
    #             .up(-self.buoy_positions[i][1].forward(-self.buoy_positions[i][2])),
    #         )


