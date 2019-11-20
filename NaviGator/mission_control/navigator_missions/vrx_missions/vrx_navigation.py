#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy

___author___ = "Kevin Allen"


class VrxNavigation(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxNavigation, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Waiting for closests')
        closest, closest_positions = yield self.get_sorted_objects('all', 10)
        # Np.unwrap gonna be useful for angles
        for obj in closest:
            print 'obj {} at {} with scale {} volume {}'.format(obj.classification, rosmsg_to_numpy(obj.pose.position), rosmsg_to_numpy(obj.scale), np.prod(rosmsg_to_numpy(obj.scale)))
