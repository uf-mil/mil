from twisted.internet import defer
from txros import util, tf
import numpy as np


@util.cancellableInlineCallbacks
def run(sub):
    # buoy_search is a Deferred
    print "We're looking for a buoy"

    print "Executing search pattern"
    yield sub.move.right(2.0).go()
    yield sub.move.down(0.3).go()

    back = sub.move.forward(0.1)

    full_transform = yield get_buoy_tf(sub)

    print 'setting height'
    if full_transform._p[2] > -0.2:
        print 'Detected buoy above the water'
        defer.returnValue(False)

    yield sub.move.depth(-full_transform._p[2]).go(speed=0.2)
    yield util.wall_sleep(0.2)

    print 'looking at'
    yield sub.move.look_at_without_pitching(full_transform._p).go()
    yield util.wall_sleep(0.2)

    dist = np.inf
    while(dist > 1.0):
        dist = np.linalg.norm(full_transform._p - sub.pose.position) * 0.5
        yield sub.move.look_at_without_pitching(full_transform._p).go()
        yield util.wall_sleep(0.2)

        yield sub.move.forward(dist).go()
    yield util.wall_sleep(0.5)

    # yield util.wall_sleep(0.1)
    yield back.go(speed=0.1)
    # yield sub.move.forward(

    print "Bumped the buoy"

@util.cancellableInlineCallbacks
def get_buoy_tf(sub):
    print "Getting buoy pose"
    response = yield sub.buoy.get_pose('red')
    if not response.found:
        print 'failed to discover buoy location'
        return
    else:
        print "Got buoy pose"
        print response.pose
    full_transform = tf.Transform.from_Pose_message(response.pose.pose)

    defer.returnValue(full_transform)