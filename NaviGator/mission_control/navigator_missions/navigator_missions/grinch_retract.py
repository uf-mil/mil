#!/usr/bin/env python3
from .navigator import NaviGatorMission


class GrinchRetract(NaviGatorMission):
    """
    Retract the grinch
    """

    async def run(self, parameters):
        await self.retract_grinch()
        return True
