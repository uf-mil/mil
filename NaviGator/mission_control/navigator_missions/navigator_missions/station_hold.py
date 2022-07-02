#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class StationHold(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback("Setting hold waypoint")
        yield self.hold()
        self.send_feedback("Setting Trajectory to lqrrt")
        yield self.change_trajectory("lqrrt")
        self.send_feedback("Switching wrench to autonomous")
        yield self.change_wrench("autonomous")
        defer.returnValue("Station Holding!")
