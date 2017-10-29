#!/usr/bin/env python
import txros  # txros is required for all missions since it's the backbone on which we build missions.


'''
This decorator makes this a txros function and is required for missions.
Txros functions allow you to `yield` on commands. Yielding on commands will stop
    the program execution and wait for that function to complete.

ex: `navigator.nh.sleep(1)`  won't do anything, but...
    `yield navigator.nh.sleep(1)`  will sleep for 1 second

All functions
'''


@txros.util.cancellableInlineCallbacks
def main(navigator):
    m = navigator.move.forward(1).right(3).go()
    '''
    m now contains a PoseEditor2 object and when `m.go()` is called will move the boat
        1 meter forward and 3 meters right from the location m was instantied at.
        (Note the desired waypoint will be 1m foward and 3m right, the controller will
        solve for the optimal path to get there).

    We could move the boat around then when we yield on `m.go()`, the boat will return to
        1 meter forward from the starting location. Rememeber we yield so that the program
        doesn't stop prematurely. If we didn't yeild, 'Done!' would print out right as the
        boat starts moving and then the program would end.
        (Note the boat would still reach it's waypoint though)
    '''
    yield m.go()

    print "Done!"

    '''
    Want to see what the boat's pose is?

    Note: You can either yield on `navigator.pose` or when you use the position or orientation.
    '''
    position, orientation = yield navigator.pose
    print position
    # OR
    position, orientation = navigator.pose
    print (yield position)

    print "Done!"
