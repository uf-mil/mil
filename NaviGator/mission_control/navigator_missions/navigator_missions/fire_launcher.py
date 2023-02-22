#!/usr/bin/env python3

from .navigator import NaviGatorMission


class FireLauncher(NaviGatorMission):
    async def run(self, parameters):
        await self.fire_launcher()
        return "Success"
