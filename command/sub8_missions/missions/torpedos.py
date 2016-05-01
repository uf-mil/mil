from txros import util, tf
import numpy as np


@util.cancellableInlineCallbacks
def run(sub):

    # dive to mission start depth
    print "descending to set course mission depth"
    mission_start_depth = 2.15        # meters
    sub.to_height(mission_start_depth)

    # TODO: Rotate in place until torpedo board is detected for several consecutive frames
    print "Executing search pattern"

    # TODO: once the board moves out of our field of view, stop rotating
    print "Found torpedo board"

    # TODO: move sub to closest point on the ray eminating from the board centroid normal to the board

    # TODO: align x-axis of sub with z axis of torpedo board
    print "aligning to torpedo board"

    # TODO: approach board until it occupies most of our field of view, save this position as the best obserbation position
    print "Approaching board"

    # TODO: Determine which of the targets has the sliding door covering it
    print "Examining targets"

    # TODO: While moving in the plane parallel to the board align bottom paddle to a position on the board slightly to the right of the door handle
    print "Aligning to door removal position"

    # TODO: move directly torwards the board until the bottom pattle is aligned with the handle
    print "Approaching board"

    # TODO: move to the left for about one foot
    print "Removing target cover"

    # TODO:
