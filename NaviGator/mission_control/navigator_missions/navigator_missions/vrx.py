#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from navigator_msgs.srv import FindPinger, FindPingerRequest, SetFrequency, SetFrequencyRequest
from std_srvs.srv import SetBool, SetBoolRequest
import mil_tools
from visualization_msgs.msg import Marker, MarkerArray
from mil_misc_tools.text_effects import fprint
from navigator import Navigator

___author___ = "Kevin Allen"


class Vrx(Navigator):
    def __init__(self):
        super(Vrx, self).__init__()

    @classmethod
    def init(cls):
        cls.taks_info_sub = cls.nh.subscribe("/vrx/task/info", MarkerArray)

    #@txros.util.cancellableInlineCallbacks
    def cleanup(self):
        pass

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Running VRX')
        yield self.move.forward(0).go()
