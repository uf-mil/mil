#!/usr/bin/env python3

from .navigator import Navigator


class StationHold(Navigator):
    async def run(self, parameters):
        self.send_feedback("Setting hold waypoint")
        self.hold()
        self.send_feedback("Setting Trajectory to lqrrt")
        await self.change_trajectory("lqrrt")
        self.send_feedback("Switching wrench to autonomous")
        await self.change_wrench("autonomous")
        return "Station Holding!"
