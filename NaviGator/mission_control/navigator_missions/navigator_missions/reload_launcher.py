#!/usr/bin/env python3
from .navigator import NaviGatorMission


class ReloadLauncher(NaviGatorMission):
    async def run(self, parameters):
        await self.reload_launcher()
        return "Success"
