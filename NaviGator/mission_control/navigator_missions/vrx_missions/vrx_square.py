#!/usr/bin/env python3
import numpy as np
import txros
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer

from .vrx import Vrx

___author___ = "Kevin Allen"


class VrxSquare(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    async def run(self, parameters):
        # await self.move.forward(5, 'm').yaw_left(90, 'deg').go()
        # await self.move.forward(5, 'm').yaw_left(90, 'deg').go()
        # await self.move.forward(5, 'm').yaw_left(90, 'deg').go()
        # await self.move.forward(5, 'm').yaw_left(90, 'deg').go()
        await self.move.forward(5, "m").go()
        await self.move.left(5, "m").go()
        await self.move.backward(5, "m").go()
        await self.move.right(5, "m").go()
