#!/usr/bin/env python

from mil_misc_tools import text_effects

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="SIMPLE", msg_color="cyan").fprint

SPEED = 0.6
CAREFUL_SPEED = 0.3

FORWARD_DIST = 25
WAIT_SECONDS = 5


class StartGateSimple(SubjuGatorMission):
    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()
        fprint("Found odom")

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        await self.nh.sleep(WAIT_SECONDS)

        fprint(f"Downing 1 m...")
        await self.go(self.move().down(1), speed=SPEED)

        fprint(f"Forward {FORWARD_DIST} m...")
        await self.go(self.move().forward(FORWARD_DIST), speed=SPEED)
