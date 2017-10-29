#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer
import numpy as np
from sensor_msgs.msg import Joy  # We all need a little joy in our lives


class Teleop(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.change_wrench("rc")
        self.send_feedback('Wrench set to "RC"')
        defer.returnValue('Now in RC mode')
