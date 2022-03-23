#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy
from navigator_msgs.srv import AcousticBeaconRequest

___author___ = "Alex Perez"


class VrxBeacon(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxBeacon, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Waiting for task to start')
        yield self.wait_for_task_such_that(lambda task: task.state in ['ready', 'running'])

        yield self.wait_for_task_such_that(lambda task: task.state in ['running'])

        beacon_msg = yield self.beacon_landmark(AcousticBeaconRequest())
        print(beacon_msg)

        position = [beacon_msg.beacon_position.x, beacon_msg.beacon_position.y, beacon_msg.beacon_position.z]

        self.send_feedback('Going to {}'.format(position))

        goal_pose = [position,[0,0,0,1]]
        yield self.move.set_position(goal_pose[0]).set_orientation(goal_pose[1]).go()
