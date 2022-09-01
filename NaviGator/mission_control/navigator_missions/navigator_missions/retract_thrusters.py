#!/usr/bin/env python3
from .navigator import Navigator


class RetractThrusters(Navigator):
    async def run(self, parameters):
        self.send_feedback("Retracting thrusters")
        await self.retract_thrusters()
        return "Success"
