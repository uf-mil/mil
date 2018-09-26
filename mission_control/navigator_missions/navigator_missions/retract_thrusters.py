#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class RetractThrusters(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Retracting thrusters')
        yield self.retract_thrusters()
        defer.returnValue('Success')
