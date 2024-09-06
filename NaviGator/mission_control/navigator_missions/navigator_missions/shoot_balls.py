#!/usr/bin/env python3

from .navigator import NaviGatorMission


class ShootBalls(NaviGatorMission):
    async def run(self, args):
        for i in range(0, 4):
            await self.reload_launcher()
            await self.nh.sleep(2)
            await self.fire_launcher()
            await self.nh.sleep(2)
        await self.set_vision_off()
