#!/usr/bin/env python
from __future__ import division

import txros
from twisted.internet import defer

from mil_misc_tools import text_effects

from .sub_singleton import SubjuGator, SonarObjects
from mil_ros_tools import rosmsg_to_numpy
from scipy.spatial import distance

import numpy as np

fprint = text_effects.FprintFactory(
    title="START_GATE", msg_color="cyan").fprint

SPEED = 0.6

CAREFUL_SPEED = 0.3

# How many meters to pass the gate by
DIST_AFTER_GATE = 1

RIGHT_OR_LEFT = 1


class StartGate2022(SubjuGator):

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        fprint('Waiting for odom')
        yield self.tx_pose()
        fprint('Found odom')
        fprint('Begin search for gates')
