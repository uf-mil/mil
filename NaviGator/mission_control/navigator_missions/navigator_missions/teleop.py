#!/usr/bin/env python3
from .navigator import NaviGatorMission


class Teleop(NaviGatorMission):
    async def run(self, parameters):
        await self.change_wrench("rc")
        self.send_feedback('Wrench set to "RC"')
        return "Now in RC mode"
