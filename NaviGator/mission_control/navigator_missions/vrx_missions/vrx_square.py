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

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        # yield self.move.forward(5, 'm').yaw_left(90, 'deg').go()
        # yield self.move.forward(5, 'm').yaw_left(90, 'deg').go()
        # yield self.move.forward(5, 'm').yaw_left(90, 'deg').go()
        # yield self.move.forward(5, 'm').yaw_left(90, 'deg').go()

        yield self.move.forward(5, "m").go()
        yield self.move.left(5, "m").go()
        yield self.move.backward(5, "m").go()
        yield self.move.right(5, "m").go()
