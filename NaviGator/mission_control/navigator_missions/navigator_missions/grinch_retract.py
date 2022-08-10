#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class GrinchRetract(Navigator):
    """
    Retract the grinch
    """

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.retract_grinch()
        defer.returnValue(True)
