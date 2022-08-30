#!/usr/bin/env python3
from .navigator import Navigator


class ExampleMission(Navigator):
    """
    Mission template / place to test functionality. Make changes locally, do not commit.
    """

    async def run(self, parameters):
        await self.nh.sleep(1.0)
        return "Success!"
