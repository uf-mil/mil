#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission


class Surface(SubjuGatorMission):
    async def run(self, args):
        self.send_feedback("Surfacing")
        await self.move.depth(0.2).go()
        return "Success!"
