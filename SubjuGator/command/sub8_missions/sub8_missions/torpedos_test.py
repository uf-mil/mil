#!/usr/bin/env python3
from twisted.internet import defer
from txros import util

from .sub_singleton import SubjuGator


class TorpedosTest(SubjuGator):
    async def run(self, args):
        self.send_feedback("Shooting Torpedo 1")
        await self.actuators.shoot_torpedo1()
        self.send_feedback("Done! Shooting Torpedo 2 in 1 second")
        await self.nh.sleep(1)
        await self.actuators.shoot_torpedo2()
        return "Success!"
