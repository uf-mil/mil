#!/usr/bin/env python
import txros


@txros.util.cancellableInlineCallbacks
def main(navigator):
    while True:
        p2 = navigator.move
        '''
        p2 now contains a pose editor object.
        The statment `navigator.move` is equivalent to `navigator.move.forward(0)`,
            so it creates a waypoint at the position of the boat at the current time.
        '''

        print "Going to point 1"
        yield navigator.move.forward(5).right(3).go()  # Arbitrary waypoint 5 meters forward and 3 meters to the right
        yield navigator.nh.sleep(5)
        print "Going to point 2"
        yield p2.go()  # Rememeber to yield when going back to the inital waypoint.
        yield navigator.nh.sleep(5)  # You have to yield when sleeping too or it'll sleep in a different thread.