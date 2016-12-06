#!/usr/bin/env python
import txros
import numpy as np


@txros.util.cancellableInlineCallbacks
def main(navigator):
    pos = yield navigator.tx_pose
    circle = navigator.move.d_circle_point(pos[0] + 1, 5, direction='cw')

    for c in circle:
        yield c.go(move_type='skid')
