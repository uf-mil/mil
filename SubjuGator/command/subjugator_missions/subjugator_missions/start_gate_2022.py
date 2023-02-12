#!/usr/bin/env python

from mil_misc_tools import text_effects

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="START_GATE", msg_color="cyan").fprint

SPEED = 0.6

CAREFUL_SPEED = 0.3

# How many meters to pass the gate by
DIST_AFTER_GATE = 1
WAIT_SECONDS = 180

RIGHT_OR_LEFT = 1


class StartGate2022(SubjuGatorMission):
    async def run(self, args):
        fprint("Waiting for odom")
        await self.tx_pose()

        fprint("Found odom")
        action_kwargs = {"speed": float(0.2), "blind": bool(False)}
        down = self.move.down(1)

        await down.go(**action_kwargs)
        # await self.nh.sleep(1)
        forward = self.move.forward(4)
        await forward.go(**action_kwargs)
