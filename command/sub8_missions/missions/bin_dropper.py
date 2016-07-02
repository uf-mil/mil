from twisted.internet import defer
from txros import util, tf
import numpy as np


@util.cancellableInlineCallbacks
def run(sub):
    # bin_search is a Deferred
    print "We're looking for a buoy"

    print "Executing search pattern"
    #yield sub.move.right(2.0).go()
    #yield sub.move.down(0.3).go()

    print "Now trying to pose it"
    response = yield sub.bin.get_pose('norange')
    if not response.found:
        print 'failed to discover bin location'
        return
    else:
        print "Got bin pose"
        print response.pose
    while True:

        response = yield sub.bin.get_pose('norange')
        print "Bin at", response.pose
        if abs(response.pose.x)>10 or abs(response.pose.y)>10:
            if response.pose.x<0:
                yield sub.move.left(abs(response.pose.x/500)).go()
            if response.pose.x>0:
                yield sub.move.right(abs(response.pose.x/500)).go()
            if response.pose.y>0:
                yield sub.move.backward(abs(response.pose.y/500)).go()
            if response.pose.y<0:
                yield sub.move.backward(abs(response.pose.y/500)).go()
        else:
            break
    yield sub.to_height(1)

    while True:
        
        response = yield sub.bin.get_pose('norange')
        print "Bin at", response.pose
        if abs(response.pose.x)>10 or abs(response.pose.y)>10:
            if response.pose.x<0:
                yield sub.move.left(abs(response.pose.x/500)).go()
            if response.pose.x>0:
                yield sub.move.right(abs(response.pose.x/500)).go()
            if response.pose.y>0:
                yield sub.move.backward(abs(response.pose.y/500)).go()
            if response.pose.y<0:
                yield sub.move.backward(abs(response.pose.y/500)).go()
        else:
            break
    
    yield sub.to_height(0.5)

    

    