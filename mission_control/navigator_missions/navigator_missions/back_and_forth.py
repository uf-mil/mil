#!/usr/bin/env python
import txros
import numpy as np


@txros.util.cancellableInlineCallbacks
def main(navigator):
    navigator.change_wrench("autonomous")
    while True:
        home = navigator.move
        '''
        p2 now contains a pose editor object.
        The statment `navigator.move` is equivalent to `navigator.move.forward(0)`,
            so it creates a waypoint at the position of the boat at the current time.
        '''
        while True:
            # forward, left, yaw_left
            amts = np.random.random(3) * np.array([60, 60, 6.28]) - np.array([30, 30, 3.14])
            rand = home.forward(amts[0]).left(amts[1]).yaw_left(amts[2])
            yield rand.go()
