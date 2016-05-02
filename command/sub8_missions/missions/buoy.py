from txros import util, tf
from sub8_ros_tools import clip_norm
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
    response = sub.buoy.get_2d()
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
def get_buoy_position(sub):
    transform = yield sub._tf_listener.get_transform(
        '/map',
        '/stereo_front',
        response.pose.header.stamp
    )
    tft = tf.Transform.from_Pose_message(response.pose.pose)
    full_transform = transform * tft
    util.defer.returnValue(full_transform)


@util.cancellableInlineCallbacks
def run(sub):
    # buoy_search is a Deferred
    print "We're looking for a buoy"
    response = yield sub.buoy.get_pose('red')
    if not response.found:
        print 'failed to discover buoy location'
        return

    back = sub.move.forward(0.1)
    transform = yield sub._tf_listener.get_transform(
        '/map',
        '/stereo_front',
        response.pose.header.stamp
    )
    tft = tf.Transform.from_Pose_message(response.pose.pose)
    full_transform = transform * tft
    print full_transform._p
    # yield sub.move.set_position(full_transform._p).go()
    print 'setting height'
    if full_transform._p[2] > -0.2:
        print 'Detected buoy above the water'
        return

    yield sub.move.height(full_transform._p[2]).go(speed=0.2)
    yield util.wall_sleep(0.2)
    print 'looking at'
    yield sub.move.look_at(full_transform._p).go(speed=0.1)
    print 'position'
    yield util.wall_sleep(0.2)
    # go = clip_norm(transform._p, 0.01, )
    dist = response.pose.pose.position.z
    print 'd', dist
    if dist > 1.0:
        print 'moving forward'
        yield sub.move.forward(dist - 1.0).go(speed=0.2)

    # yield sub.move.forward(0.2).go(speed=0.1)
    # yield util.wall_sleep(0.1)
    yield back.go(speed=0.1)
    # yield sub.move.forward(

    print "Bumped the buoy"
