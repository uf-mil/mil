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
        self.actuators.gripper_open()
        print('Move backward')
        yield self.move.look_at_without_pitching(np.array([0,0,0])).go(speed=0.5)
        yield self.nh.sleep(2)
        yield self.move.forward(1.3).right(0.5).go(speed=0.3)
        yield self.nh.sleep(2)
        yield self.move.depth(2).go(speed=0.3)
        self.actuators.gripper_close()
        print('Moving to pinger position {}'.format(position))
        yield self.move.set_position(position).go(speed=0.5)
        yield self.nh.sleep(2)
        self.send_feedback('Surfacing')
        yield self.move.depth(0.2).go()
        yield self.nh.sleep(5)
        yield self.move.depth(1.5).go()
        self.actuators.gripper_open()
        defer.returnValue('Success!')
