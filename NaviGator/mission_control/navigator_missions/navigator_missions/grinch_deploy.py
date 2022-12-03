#!/usr/bin/env python3
from .navigator import NaviGatorMission


class GrinchDeploy(NaviGatorMission):
    """
    Deploy the grinch
    """

    async def run(self, parameters):
        await self.deploy_grinch()
        return True
