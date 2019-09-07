#!/usr/bin/env python
import txros
from navigator import Navigator


class Circle(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        p = self.pose[0]
        while True:
            p += [0, 1, 0]

            self.move.set_position(p).go(move_type='skid', initial_plan_time=0)
            yield self.nh.sleep(.1)
