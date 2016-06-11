from twisted.internet import defer
from txros import util, tf
import numpy as np


@util.cancellableInlineCallbacks
def run(sub):

    yield sub.move.depth(0.6).go()
    print "Executing learn pattern"
    for k in range(2):
        yield sub.move.down(0.2).go()
        yield sub.move.right(0.8).go()
        yield sub.move.up(0.2).go()
        yield sub.move.forward(0.8).go()
        yield sub.move.left(0.8).go()
        yield sub.move.backward(0.8).go()
        yield sub.yaw_right(1.5).go()
