#! usr/bin/env python3

import numpy as np
from mil_misc_tools import text_effects
from sensor_msgs.msg import CameraInfo

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="OCEAN TEMPERATURES", msg_color="blue").fprint


class OceanTemperatures(SubjuGatorMission):
    async def run(self, args):
        fprint("Enabling cam_ray publisher")

        await self.nh.sleep(1)

        fprint("Connecting camera")

        cam_info_sub = self.nh.subscribe("/camera/down/camera_info", CameraInfo)
        await cam_info_sub.setup()

        fprint("Obtaining cam info message")
        cam_info = await cam_info_sub.get_next_message()
        cam_center = np.array([cam_info.width / 2, cam_info.height / 2])
        fprint(f"Cam center: {cam_center}")
