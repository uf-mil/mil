#!/usr/bin/env python3
from .sub_singleton import SubjuGator


class BallDropTest(SubjuGator):
    async def run(self, args):
        self.send_feedback("Dropping Ball")
        await self.actuators.drop_marker()
        return "Success!"
