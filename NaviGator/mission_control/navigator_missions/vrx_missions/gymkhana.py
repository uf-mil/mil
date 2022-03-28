#!/usr/bin/env python
import txros
import rospy
import numpy as np
from vrx import Vrx


class Gymkhana(Vrx):

    def __init__(self, *args, **kwargs):
        super(Gymkhana, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        yield self.nh.sleep(5)

        yield self.move.yaw_left(45, "deg").go()
        yield self.move.forward(70).go()
        #yield self.move.forward(5).go()
        #yield self.run_submission('VrxNavigation2')
        #yield self.run_submission('VrxBeacon')
        #yield self.run_submission('VrxBeacon')
        yield self.send_feedback('Done!')
