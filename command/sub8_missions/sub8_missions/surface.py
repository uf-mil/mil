#!/usr/bin/env python
from sub_singleton import SubjuGator
from txros import util
from twisted.internet import defer
import numpy as np


class Surface(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        position_x = yield self.nh.get_param("octagon_position_x")
        position_y = yield self.nh.get_param("octagon_position_y")
        position_z = yield self.nh.get_param("octagon_position_z")
        position = np.array([float(position_x), float(position_y), float(position_z)])
        print('Moving to pinger position {}'.format(position))
        self.move.set_position(position).go(speed=0.5)
        self.send_feedback('Surfacing')
        yield self.move.depth(0.2).go()
        yield self.nh.sleep(5)
        yield self.move.depth(1.5).go()
        defer.returnValue('Success!')
