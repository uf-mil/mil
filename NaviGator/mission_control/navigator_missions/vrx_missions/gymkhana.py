#!/usr/bin/env python
import numpy as np
import rospy
import txros
from vrx import Vrx


class Gymkhana(Vrx):
    def __init__(self, *args, **kwargs):
        super(Gymkhana, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        yield self.nh.sleep(5)

        yield self.run_submission("VrxNavigation")
        yield self.run_submission("VrxBeacon")
        yield self.run_submission("VrxBeacon")
        yield self.send_feedback("Done!")
