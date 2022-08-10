#!/usr/bin/env python
from __future__ import division

import numpy as np
import txros
from mil_misc_tools import text_effects
from mil_ros_tools import rosmsg_to_numpy
from scipy.spatial import distance
from twisted.internet import defer

from .sub_singleton import SonarObjects, SubjuGator

fprint = text_effects.FprintFactory(title="START_GATE", msg_color="cyan").fprint

SPEED = 0.6

CAREFUL_SPEED = 0.3

# How many meters to pass the gate by
DIST_AFTER_GATE = 1
WAIT_SECONDS = 180

RIGHT_OR_LEFT = 1


class StartGate2022(SubjuGator):
    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        fprint("Waiting for odom")
        yield self.tx_pose()

        fprint(f"Waiting {WAIT_SECONDS} seconds...")
        yield self.nh.sleep(WAIT_SECONDS)

        fprint("Found odom")
        down = self.move.down(1).zero_roll_and_pitch()
        forward = self.move.forward(4).zero_roll_and_pitch()

        yield down.go(speed=SPEED)
        yield forward.go(speed=SPEED)
