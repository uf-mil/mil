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
    title="FOLLOW THE PATH", msg_color="cyan").fprint


class FollowPath2022(SubjuGator):

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        fprint('Starting to Follow the Orange Marker')
        yield self.nh.sleep(0.5)
