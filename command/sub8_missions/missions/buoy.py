from txros import util, tf
import numpy as np


@util.cancellableInlineCallbacks
def align(sub):
    response = yield sub.channel_marker.get_2d()
    if response.found:
        xy, theta = sub.channel_marker.get_response_direction(response)
        yield sub.move.yaw_right(xy[0] * 0.1).go(speed=0.1)
        yield sub.move.body_up(np.clip(xy[1], 0.0, 0.1)).go(speed=0.1)

    else:
        print "failed to find buoy"


@util.cancellableInlineCallbacks
def approach(sub):
    response = yield sub.buoy.get_2d()
    if response.found:
        align(sub)
        yield sub.move.forward(0.4).go(speed=0.1)
    util.defer.returnValue(response.found)


@util.cancellableInlineCallbacks
def bump(sub):
    response = yield sub.buoy.get_pose()
    transform = yield sub._tf_listener.get_transform(
        '/map',
        '/stereo_front',
        response.pose.header.stamp)
    yield sub.move.height(response.pose.z).go()


@util.cancellableInlineCallbacks
def run(sub):
    # buoy_search is a Deferred
    print "We're looking for a buoy"
    response = yield sub.buoy.get_pose('red')
    transform = yield sub._tf_listener.get_transform(
        '/map',
        '/stereo_front',
        response.pose.header.stamp
    )
    tft = tf.Transform.from_Pose_message(response.pose.pose)
    print transform
    full_transform = transform * tft
    print full_transform._p
    yield sub.move.set_position(full_transform._p).go()

    # for k in range(4):
    #     yield align(sub)

    # found = yield approach(sub)
    # if not found:
    #     yield sub.move.yaw_right(0.2)

    # found = yield approach(sub)
    # if not found:
    #     yield sub.move.yaw_right(-0.4)



    print "Bumped the buoy"
