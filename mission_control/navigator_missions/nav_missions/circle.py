#!/usr/bin/env python
import txros
import numpy as np


@txros.util.cancellableInlineCallbacks
def main(navigator):
    focus = np.array([-10, -10, 0])
    pattern = navigator.move.circle_point(focus, radius=5)

    for p in pattern:
        yield p.go(move_type='skid', focus=focus)
        print "Nexting"