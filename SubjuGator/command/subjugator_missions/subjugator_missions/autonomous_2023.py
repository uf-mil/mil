#!/usr/bin/env python

import math

from axros import Subscriber
from mil_misc_tools import text_effects
from tf.transformations import *

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="AUTONOMOUS23", msg_color="cyan").fprint

WAIT_SECONDS = 6
SPEED = 0.6 # m/s

class Autonomous2023(SubjuGatorMission):
    """Same as auto23.py but without chaining the actual funcitons because there is an error we could not solve in competition when we chain missions""" 
    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()
        fprint("Found odom")

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        await self.nh.sleep(WAIT_SECONDS)

        #fprint("Coin Flip")
        #fprint(f"Checking orientation...")
        #cur_orientation = self.pose.orientation
        #fprint(f"Current orientation {cur_orientation}...")

        #go2gate_quat = [0.04467842512634323,-0.10071425271779812,0.9910658568848062,-0.07515946344213803] # hardcoded from odom when looking at gate 

        #fprint(f"Rotating to start gate orientation (of {go2gate_quat} quaterntions)...")
        #await self.go(self.move().down(0.15).set_orientation(go2gate_quat))

        #cur_orientation = self.pose.orientation
        #fprint(f"Current orientation after rotation {cur_orientation}...")
        #await self.go(self.move().yaw_left_deg(9), speed=SPEED)

        fprint(f"Downing {1.5} m...")
        await self.go(self.move().down(1.5), speed=SPEED)

        fprint(f"Forwarding {11} m...")
        await self.go(self.move().forward(11), speed=SPEED)

        fprint(f"YawingRight {790} deg...")
        await self.go(self.move().yaw_right_deg(180), speed=SPEED)
        await self.go(self.move().yaw_right_deg(180), speed=SPEED)
        await self.go(self.move().yaw_right_deg(180), speed=SPEED)
        await self.go(self.move().yaw_right_deg(180), speed=SPEED)
        await self.go(self.move().yaw_right_deg(30), speed=SPEED)
        fprint(f"Zero rp")
        await self.go(self.move().zero_roll_and_pitch())
        fprint(f"YawingLeft {790} deg...")
        await self.go(self.move().yaw_left_deg(180), speed=SPEED)
        await self.go(self.move().yaw_left_deg(180), speed=SPEED)
        await self.go(self.move().yaw_left_deg(180), speed=SPEED)
        await self.go(self.move().yaw_left_deg(180), speed=SPEED)
        await self.go(self.move().yaw_left_deg(30), speed=SPEED)
        fprint(f"Zero rp")
        await self.go(self.move().zero_roll_and_pitch())

        fprint(f"Uping {1} m...")
        await self.go(self.move().up(1), speed=SPEED)

        fprint(f"YawingLeft {9} deg...")
        await self.go(self.move().yaw_left_deg(9), speed=SPEED)
        await self.go(self.move().yaw_left_deg(30), speed=SPEED)
        fprint(f"Forwarding {30} m...")
        await self.go(self.move().forward(30), speed=SPEED)

        fprint(f"Surfacing to {0.1} m...")
        await self.go(self.move().depth(0.1))

