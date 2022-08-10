#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class RetractThrusters(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback("Retracting thrusters")
        yield self.retract_thrusters()
        defer.returnValue("Success")
