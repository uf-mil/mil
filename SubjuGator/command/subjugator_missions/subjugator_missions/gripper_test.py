#!/usr/bin/env python3
from .sub_singleton import SubjuGator


class GripperTest(SubjuGator):
    async def run(self, args):
        self.send_feedback("Opening Gripper")
        await self.actuators.gripper_open()
        self.send_feedback("Done! Closing gripper in 2 seconds.")
        await self.nh.sleep(2)
        await self.actuators.gripper_close()
        self.send_feedback("Done! Opening gripper again in 1 second.")
        await self.nh.sleep(1)
        await self.actuators.gripper_open()
        return "Success!"
