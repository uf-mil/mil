#!/usr/bin/env python
import txros


@txros.util.cancellableInlineCallbacks
def main(navigator):
    while True:
        point_2 = navigator.move
        print "Going to point 1"
        yield navigator.move.forward(5).right(3).go()
        yield navigator.nh.sleep(5)
        print "Going to point 2"
        yield point_2.go()
        yield navigator.nh.sleep(5)