#!/usr/bin/env python

import math
import rospy

from axros import Subscriber
from mil_misc_tools import text_effects
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Bool
from tf.transformations import *

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="START_GATE_STYLE", msg_color="cyan").fprint

SPEED = 0.6
CAREFUL_SPEED = 0.3

DIST_AFTER_GATE = 1
WAIT_SECONDS = 5


class StartGateStyle(SubjuGatorMission):
    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()
        fprint("Found odom")

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        await self.nh.sleep(WAIT_SECONDS)

        fprint(f"Downing {1} m...")
        await self.go(self.move().down(1), speed=SPEED)

        fprint(f"Forwarding {10} m...")
        await self.go(self.move().forward(10), speed=SPEED)

        fprint(f"YawingRight {390} deg...")
        await self.go(self.move().yaw_right_deg(180), speed=SPEED)
        await self.go(self.move().yaw_right_deg(180), speed=SPEED)
        await self.go(self.move().yaw_right_deg(50), speed=SPEED)
        await self.go(self.move().zero_roll_and_pitch())

        fprint(f"YawingLeft {390} deg...")
        await self.go(self.move().yaw_left_deg(180), speed=SPEED)
        await self.go(self.move().yaw_left_deg(180), speed=SPEED)
        await self.go(self.move().yaw_left_deg(50), speed=SPEED)
        await self.go(self.move().zero_roll_and_pitch())
        
        fprint(f"Forwarding {16} m...")
        await self.go(self.move().forward(16), speed=SPEED)
