#!/usr/bin/env python3
from .navigator import Navigator


class GrinchRetract(Navigator):
    """
    Retract the grinch
    """

    async def run(self, parameters):
        await self.retract_grinch()
        return True
