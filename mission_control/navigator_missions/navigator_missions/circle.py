#!/usr/bin/env python
import txros
import numpy as np


@txros.util.cancellableInlineCallbacks
def main(navigator):
    p = navigator.pose[0]
    while True:
        p += [0, 1, 0]

        a = navigator.move.set_position(p).go(move_type='skid', initial_plan_time=0)
        yield navigator.nh.sleep(.1)
