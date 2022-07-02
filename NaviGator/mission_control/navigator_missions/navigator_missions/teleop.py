#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class Teleop(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.change_wrench("rc")
        self.send_feedback('Wrench set to "RC"')
        defer.returnValue("Now in RC mode")
