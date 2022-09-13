#!/usr/bin/env python3
import numpy as np
import rospy
import txros

from .vrx import Vrx


class Gymkhana(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    async def run(self, args):
        await self.nh.sleep(5)

        await self.run_submission("VrxNavigation")
        await self.run_submission("VrxBeacon")
        await self.run_submission("VrxBeacon")
        await self.send_feedback("Done!")
