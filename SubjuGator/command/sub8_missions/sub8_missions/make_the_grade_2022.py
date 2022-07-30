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
    title="MAKE THE GRADE", msg_color="cyan").fprint

class MakeGrade2022(SubjuGator):

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        fprint('Starting the Mark the Grade Mission')
        
        bounding_boxes = yield self.darknet_objects.get_next_message()

        for box in bounding_boxes.bounding_boxes:
            print(box.probability)
            print(box.xmin)
            print(box.xmax)
            print(box.ymin)
            print(box.ymax)
            print(box.Class)
            print("")
