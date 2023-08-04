#!/usr/bin/env python

from mil_misc_tools import text_effects

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="SIMPLE", msg_color="cyan").fprint

SPEED = 0.6
CAREFUL_SPEED = 0.3

FORWARD_DIST = 15
WAIT_SECONDS = 1


class Simple(SubjuGatorMission):
    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        await self.nh.sleep(WAIT_SECONDS)

        fprint("Found odom")
        await self.go(self.move().down(1), speed=SPEED)

        await self.go(self.move().east(FORWARD_DIST), speed=SPEED)
