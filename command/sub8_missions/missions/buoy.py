from twisted.internet import defer
from txros import util, tf
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest

SPEED = .1
# TODO: Ralph, find a way to change this with chained missions.
COLOR = "red"

@util.cancellableInlineCallbacks
def run(sub):
    start_search = yield sub._node_handle.get_service_client('/vision/buoys/search', SetBool)
    yield start_search(SetBoolRequest(data=True))

    print "BUOY MISSION - We're looking for a {} buoy.".format(COLOR)

    print "BUOY MISSION - Executing search pattern"
    yield sub.move.right(2.0).go(speed=SPEED)
    yield sub.move.down(0.3).go(speed=SPEED)

    back = sub.move.forward(0)
    full_transform = yield get_buoy_tf(sub)

    print 'BUOY MISSION - setting height'
    if full_transform._p[2] > -0.2:
        print 'BUOY MISSION - Detected buoy above the water'
        defer.returnValue(False)

    yield sub.move.depth(-full_transform._p[2]).go(speed=SPEED)

    print 'BUOY MISSION - looking at'
    yield sub.move.look_at_without_pitching(full_transform._p).go(speed=SPEED)

    dist = np.inf
    while(dist > 1.0):
        full_transform = yield get_buoy_tf(sub)
        dist = np.linalg.norm(full_transform._p - sub.pose.position) * 0.5
        print "BUOY MISSION - {} m away".format(dist)
        yield sub.move.look_at_without_pitching(full_transform._p).go(speed=SPEED)
        yield sub.move.forward(dist).go(speed=SPEED)
        yield sub._node_handle.sleep(0.5)

    # Now we are 1m away from the buoy
    print "BUOY MISSION - bumping!"
    yield sub.move.forward(1.25).go(speed=SPEED)
    yield back.go(speed=0.1)

    print "BUOY MISSION - Bumped the buoy"

@util.cancellableInlineCallbacks
def get_buoy_tf(sub):
    print "Getting buoy pose"
    response = yield sub.buoy.get_pose(COLOR)
    if not response.found:
        print 'BUOY MISSION - failed to discover buoy location'
        return
    else:
        print "BUOY MISSION - Got buoy pose"
        print response.pose
    full_transform = tf.Transform.from_Pose_message(response.pose.pose)

    defer.returnValue(full_transform)