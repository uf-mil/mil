#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class ExampleMission(Navigator):
    """
    Mission template / place to test functionality. Make changes locally, do not commit.
    """

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.nh.sleep(1.0)
        defer.returnValue("Success!")
