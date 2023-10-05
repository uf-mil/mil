#!/usr/bin/env python3
from .navigator import NaviGatorMission


class RetractThrusters(NaviGatorMission):
    async def run(self, parameters):
        self.send_feedback("Retracting thrusters")
        await self.retract_thrusters()
        return "Success"
