#!/usr/bin/env python3
from .navigator import NaviGatorMission


class ExampleMission(NaviGatorMission):
    """
    Mission template / place to test functionality. Make changes locally, do not commit.
    """

    async def run(self, parameters):
        await self.nh.sleep(1.0)
        return "Success!"
