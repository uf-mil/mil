#! usr/bin/env python3

import numpy as np
from mil_misc_tools import text_effects
from sensor_msgs.msg import CameraInfo

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="OCEAN TEMPERATURES", msg_color="blue").fprint

SPEED = 0.6


class OceanTemperatures(SubjuGatorMission):
    async def section_pos(self, team):
        """This function returns the position of the appropriate bin based on the team chosen

        Args:
            team (string): either blue or red team

        CV will implement method to get the appropriate position of the buoy
        """
        return "forward"

    async def run(self, args):
        """
        This function begins assuming the sub is already placed approximately in the center of the bin.
        It should be noted the sub moving to the center of the bin is yet to be implemented, as well as the
        CV aspect of this mission
        """
        fprint("Enabling cam_ray publisher")

        await self.nh.sleep(1)

        fprint("Connecting camera")

        cam_info_sub = self.nh.subscribe("/camera/down/camera_info", CameraInfo)
        await cam_info_sub.setup()

        fprint("Obtaining cam info message")
        cam_info = await cam_info_sub.get_next_message()
        cam_center = np.array([cam_info.width / 2, cam_info.height / 2])
        fprint(f"Cam center: {cam_center}")

        # choose a color for the bin to drop the marker
        bin_color = await self.nh.get_param("/strategy/team")

        fprint(f"Identifying the side of the ({bin_color}) bin")

        bin_pos = await self.section_pos(bin_color)

        fprint(f"Moving toward the {bin_pos} side of the bin")
        if bin_pos == "forward":
            forward = self.move().forward(1)
            await self.go(forward, speed=SPEED)
        else:
            back = self.move().backward(1)
            await self.go(back, speed=SPEED)

        # if necessary, sub would dive at this point to get closer to the bin before dropping

        # Following this, indicate the actuator that holds the market to drop it
