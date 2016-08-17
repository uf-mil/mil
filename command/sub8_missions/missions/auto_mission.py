#!/usr/bin/env python

import traceback
from std_srvs.srv import Empty, EmptyResponse
from twisted.internet import defer, reactor
import txros
from sub8 import tx_sub
from sub8_ros_tools import rosmsg_to_numpy
import missions


@txros.util.cancellableInlineCallbacks
def run(sub):
    yield txros.util.wall_sleep(1.0)

    # == MISSION CODE HERE ===================
    ret = yield missions.align_channel.run(sub)
    if ret is None:
        print "Fuck"

    odom = yield sub.last_pose()
    rotation = rosmsg_to_numpy(odom.pose.pose.orientation)

    # Buoys are around 2m from the ground?
    #yield sub.to_height(2)

    yield missions.buoy.run(sub)

    yield sub.move.backward(1).go()
    yield sub.move.up(1.5).go()
    yield sub.move.set_orientation(rotation).zero_roll_and_pitch().go()

    yield sub.move.forward(5)

    print "DONE"
