#!/usr/bin/env python
import txros
from tf import transformations as trns
import numpy as np
import navigator_tools
from twisted.internet import defer

@txros.util.cancellableInlineCallbacks
def main(navigator):
    result = navigator.fetch_result()

    # Get all the objects in the db
    resp = yield navigator.database_query("All")
    boat_position = navigator.pose[0]
    if resp.found:
        p_to_np = navigator_tools.point_to_numpy
        obj_dist = lambda obj: np.linalg.norm(p_to_np(obj.position) - boat_position)
        furthest = max(resp.objects, key=obj_dist)

        yield navigator.move.set_position(p_to_np(furthest.position)).go()
        
        # Set that fake marker to false or something to denote we went there
        #yield navigator.database_query(cmd="{}=0, 0".format(furthest.name))

        result.message = "Went to {}.".format(furthest.name)

    else:
        # Random move if we have no labled points
        yield navigator.move.yaw_left(np.random.normal() * 6.28).forward(50).go()
    
        result.message = "Moved to a random point."


    defer.returnValue(result)



