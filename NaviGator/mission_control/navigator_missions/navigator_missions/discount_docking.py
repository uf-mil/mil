#!/usr/bin/env python3

from .navigator import NaviGatorMission


class DiscountDocking(NaviGatorMission):
    async def run(self, args):
        await self.move.forward(5).go()

        await self.navigator.nh.sleep(10)

        await self.move.backward(5).go()
        await self.move.backward(5).go()
        await self.move.backward(5).go()

        self.send_feedback("Done!")
