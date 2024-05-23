#!/usr/bin/env python3
import math

import numpy as np
from mil_misc_tools import text_effects
from sensor_msgs.msg import CameraInfo

from .sub_singleton import SubjuGatorMission

# This mission will direct the sub towards a red buoy, and circumnavigate it on a given orientation

fprint = text_effects.FprintFactory(title="HYDROTHERMAL VENT", msg_color="green").fprint
CW = True


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

        if CW:
            self.send_feedback("Circumnaviganting the buoy on a CW orientation")

            rotate = self.move().yaw_left(yaw_angle2)
            await self.go(rotate)
            await self.go(self.move().forward(0.7))
            for i in range(0, 3):
                rotate = self.move().yaw_right(yaw_angle2)
                await self.go(rotate)
                await self.go(self.move().forward(1.4))
            rotate = self.move().yaw_right(yaw_angle2)
            await self.go(rotate)
            await self.go(self.move().forward(0.7))
        else:
            self.send_feedback("Circumnaviganting the buoy on a CCW orientation")

            rotate = self.move().yaw_right(yaw_angle2)
            await self.go(rotate)
            await self.go(self.move().forward(0.7))
            for i in range(0, 3):
                rotate = self.move().yaw_left(yaw_angle2)
                await self.go(rotate)
                await self.go(self.move().forward(1.4))
            rotate = self.move().yaw_left(yaw_angle2)
            await self.go(rotate)
            await self.go(self.move().forward(0.7))

    # async def run(self, args):

    # buoy_rad_and_center = [20, 2, 3] # arbitrary numbers to test functionality
    # # center buoy
    # self.center_bouy(np.array(buoy_rad_and_center[1], buoy_rad_and_center[2]))
    # # go forward until desired distance is reached
    # while (buoy_rad_and_center[0] < 40):
    #     await self.go(self.move().forward)

    # This function is supposed to center the buoy based on position given in terms of xy position of the center of the red buoy, with
    # respect to the center of the camera, as well as the distance from the buoy

    @classmethod
    async def center_bouy(self, buoy_pos_c):
        fprint("Enabling cam_ray publisher")

        await self.nh.sleep(1)

        fprint("Connecting camera")

        cam_info_sub_r = self.nh.subscribe(
            "/camera/front/right/camera_info",
            CameraInfo,
        )
        await cam_info_sub_r.setup()

        fprint("Obtaining cam info message")
        cam_info_r = await cam_info_sub_r.get_next_message()
        cam_center = np.array([cam_info_r.width / 2, cam_info_r.height / 2])
        # fprint(f"Cam center: {cam_center}")
        while ((abs(buoy_pos_c[0] - cam_center[0])) > 0.01) and (
            (abs(buoy_pos_c[1]) - cam_center[1]) > 0.01
        ):
            await self.go(self.move().depth(buoy_pos_c[1]))
            yaw_angle = np.arctan(buoy_pos_c[0] / buoy_pos_c[2])
            await self.go(self.move().set_roll_pitch_yaw(0, 0, yaw_angle))

            buoy_pos_c = buoy_pos_c  # this will be replaced with the computer vision call to check the position of the buoy
