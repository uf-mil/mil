#!/usr/bin/env python
from txros import util
import numpy as np


@util.cancellableInlineCallbacks
def run(sub_singleton):
    fp = open("/home/sub/clicked.txt", 'r')
    line = fp.readline()
    seq = False
    i = 0
    position = [0, 0, 0]
    positions = []
    while line:
        line = fp.readline().strip()
        if line[:5] == ("seq: "):
            seq = True
        if seq and line == "position:":
            position[0] = float(fp.readline().rstrip()[7:])
            position[1] = float(fp.readline().rstrip()[7:])
            position[2] = -1
            positions.append(np.array(position))
            i = i + 1
    fp.close()
    for i in xrange(len(positions)):
        print positions[i][0:3]
        yield sub_singleton.move.look_at_without_pitching(
            np.array(positions[i][0:3])).go()
        yield sub_singleton.move.set_position(np.array(positions[i][0:3])).go()
