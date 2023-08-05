#!/usr/bin/env python

from mil_misc_tools import text_effects

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="SIMPLE", msg_color="cyan").fprint

SPEED = 0.6
CAREFUL_SPEED = 0.3

TURN_RIGHT_TO_OCTAGON = False
TURN_DEG = 5
DIST_TO_OCTAGON = 25

class Octagon(SubjuGatorMission):
    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()
        fprint("Found odom")

        if TURN_RIGHT_TO_OCTAGON:
            fprint(f"Yaw right {TURN_DEG} degrees...")
            await self.go(self.move().yaw_right_deg(TURN_DEG), speed=SPEED)
        else:
            fprint(f"Yaw left {TURN_DEG} degrees...")
            await self.go(self.move().yaw_left_deg(TURN_DEG), speed=SPEED)

        fprint(f"Forward {DIST_TO_OCTAGON} m...")
        await self.go(self.move().forward(DIST_TO_OCTAGON), speed=SPEED)
