#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class Teleop(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.change_wrench("rc")
        self.send_feedback('Wrench set to "RC"')
        defer.returnValue('Now in RC mode')
