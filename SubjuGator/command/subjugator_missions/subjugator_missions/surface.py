#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission


class Surface(SubjuGatorMission):
    async def run(self, args):
        self.send_feedback("Surfacing")
        await self.go(self.move().depth(0.1))
        return "Success!"
