from twisted.internet import defer
from txros import util, tf
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest
from sub8_ros_tools import pose_to_numpy

SPEED = .3
SEARCH = 0
MAX_TRIES = 3

@util.cancellableInlineCallbacks
def run(sub):
    start_search = yield sub.nh.get_service_client('/vision/buoys/search', SetBool)
    yield start_search(SetBoolRequest(data=True))

    print "BUOY MISSION - Executing search pattern"
    yield search_again(sub)

    ret = None
    this_try = 0
    while ret is None:
        ret = yield bump_buoy(sub, 'red')
        yield search_again(sub)

        this_try += 1
        if this_try > MAX_TRIES:
            defer.returnValue(None)

        print ret

    yield sub.move.backward(2).go(speed=SPEED)

    ret = None
    this_try = 0
    while ret is None:
        ret = yield bump_buoy(sub, 'green')
        yield search_again(sub)

        this_try += 1
        if this_try > MAX_TRIES:
            defer.returnValue(None)


@util.cancellableInlineCallbacks
def search_again(sub):
    global SEARCH
    if SEARCH == 0:
        yield sub.move.up(.3).zero_roll_and_pitch().go(speed=SPEED)
        yield sub.move.left(0.3).zero_roll_and_pitch().go(speed=SPEED)
        SEARCH = 1
    elif SEARCH == 1:
        yield sub.move.right(2.0).go(speed=SPEED)
        yield sub.move.down(0.3).go(speed=SPEED)
        SEARCH = 0

@util.cancellableInlineCallbacks
def bump_buoy(sub, color):
    '''
    Performs a buoy bump
    '''
    print "BUOY MISSION - We're looking for a {} buoy.".format(color)
    full_transform = yield get_buoy_tf(sub, color)

    if full_transform is None:
        print 'BUOY MISSION - No buoy found.'
        defer.returnValue(None)

    print 'BUOY MISSION - setting height'

    yield sub.move.depth(-full_transform._p[2]).go(speed=SPEED)

    print 'BUOY MISSION - looking at'
    yield sub.move.look_at_without_pitching(full_transform._p).go(speed=SPEED)

    dist = np.inf
    while(dist > 1.0):
        full_transform = yield get_buoy_tf(sub, color)

        if full_transform is None:
            continue

        dist = np.linalg.norm(full_transform._p - sub.pose.position) * 0.5
        print "BUOY MISSION - {}m away".format(dist)
        yield sub.move.look_at_without_pitching(full_transform._p).go(speed=SPEED)
        yield sub.nh.sleep(0.5)
        yield sub.move.forward(dist).go(speed=SPEED)
        yield sub.nh.sleep(0.5)

    # Now we are 1m away from the buoy
    print "BUOY MISSION - bumping!"
    forward = sub.move.forward(dist + .3).go(speed=SPEED)

    print "BUOY MISSION - Bumped the buoy"
    defer.returnValue(True)

@util.cancellableInlineCallbacks
def get_buoy_tf(sub, color):
    print "Getting buoy pose"
    response = yield sub.buoy.get_pose(color)
    if not response.found:
        print 'BUOY MISSION - failed to discover buoy location'
        defer.returnValue(None)
    else:
        print "BUOY MISSION - Got buoy pose"

        s = yield sanity_check(sub, response.pose)
        if s is None:
            print "BUOY MISSION - Sanity check failed"
            defer.returnValue(None)

        print response.pose

    full_transform = tf.Transform.from_Pose_message(response.pose.pose)

    defer.returnValue(full_transform)

@util.cancellableInlineCallbacks
def sanity_check(sub, est_pos):
    yield None
    est_pos_np = pose_to_numpy(est_pos.pose)[0]
    print (est_pos_np == np.array([5, 5, 5])).all()
    if (est_pos_np == np.array([5, 5, 5])).all():
        print "BUOY MISSION - Problem with guess."
        defer.returnValue(None)
    if est_pos_np[2] > -0.2:
        print 'BUOY MISSION - Detected buoy above the water'
        defer.returnValue(None)

    defer.returnValue(True)
