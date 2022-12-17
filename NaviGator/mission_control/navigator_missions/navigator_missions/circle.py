#!/usr/bin/env python3

from .navigator import NaviGatorMission


class Circle(NaviGatorMission):
    async def run(self, parameters):
        p = self.pose[0]
        while True:
            p += [0, 1, 0]

            await self.move.set_position(p).go(move_type="skid", initial_plan_time=0)
            await self.nh.sleep(0.1)
