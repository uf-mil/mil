#!/usr/bin/env python

import math

from axros import Subscriber
from mil_misc_tools import text_effects
from tf.transformations import *

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="START_GATE", msg_color="cyan").fprint

WAIT_SECONDS = 1
SPEED = 0.6

class CoinFlip(SubjuGatorMission):

    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()
        fprint("Found odom")

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        await self.nh.sleep(WAIT_SECONDS)

        fprint(f"Checking orientation...")
        cur_orientation = self.pose.orientation
        fprint(f"Current orientation {cur_orientation}...")

        go2gate_quat = [0.,0.,0.,1.] # hardcoded from odom when looking at gate 

        fprint(f"Rotating to start gate orientation (of {go2gate_quat} quaterntions)...")
        await self.go(self.move().down(0.15).set_orientation(go2gate_quat))

        cur_orientation = self.pose.orientation
        fprint(f"Current orientation after rotation {cur_orientation}...")

        #IS_LEFT = True 
        #side = "left" if IS_LEFT else "right"
        #if IS_LEFT:
        #    left = self.move().left(1)
        #    await self.go(left, speed=SPEED)
        #else:
        #    right = self.move().right(1)
        #    await self.go(right, speed=SPEED)

