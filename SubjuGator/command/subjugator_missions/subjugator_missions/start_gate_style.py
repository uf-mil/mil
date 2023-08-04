#!/usr/bin/env python

import math

from axros import Subscriber
from mil_misc_tools import text_effects
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Bool
from tf.transformations import *

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="START_GATE", msg_color="cyan").fprint

SPEED = 0.6
CAREFUL_SPEED = 0.3

DIST_AFTER_GATE = 1
WAIT_SECONDS = 1


class StartGateStyle(SubjuGatorMission):
    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        await self.nh.sleep(WAIT_SECONDS)

        fprint("Found odom")
        down = self.move().down(1)
        await self.go(down, speed=SPEED)

        await self.go(self.move().east(3), speed=SPEED)

        await self.go(self.move().yaw_right_deg(180), speed=SPEED)
        await self.go(self.move().yaw_right_deg(180), speed=SPEED)
        
        await self.go(self.move().east(3), speed=SPEED)
