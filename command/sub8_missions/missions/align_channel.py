from txros import util
from sub8_ros_tools import normalize, clip_norm
import numpy as np


@util.cancellableInlineCallbacks
def get_above(sub):
    '''TODO:
    Use knowledge of height from DVL
    '''
    response = yield sub.channel_marker.get_2d()

    if response is not None:
        if response.found:
            xy, theta = sub.channel_marker.get_response_direction(response)

            # relative_motion = clip_norm(xy, 0.1, 0.7)
            # fix = np.array([-1.0, -1.0])
            # use = (fix * relative_motion)[::-1]
            # print 'rmot', use
            # yield sub.move.strafe_relative(np.array([0.0, use.flatten()[1]])).go(speed=0.1, linear_tolerance=0.1)

            print 'going', xy.flatten()[0] * 0.25, xy.flatten()[1] * 0.25
            yield sub.move.left(xy.flatten()[0] * 0.25).go()
            yield sub.move.backward(xy.flatten()[1] * 0.25).go()

            yield util.wall_sleep(0.1)
        else:
            print "NOT FOUND"
    else:
        print "EVEN WORSE THAN NOT FOUND!"


@util.cancellableInlineCallbacks
def run(sub_singleton):
    yield sub_singleton.to_height(5)
    # for k in range(5):
        # yield get_above(sub_singleton)
    # yield get_above(sub_singleton)

#    print 'aligning'
#    response = yield sub_singleton.channel_marker.get_2d()
#    if response is not None:
#        if response.found:
#            print response.pose.theta
#            yield sub_singleton.move.yaw_left(response.pose.theta).go(speed=0.1)
#
#    for k in range(5):
#        yield get_above(sub_singleton)
