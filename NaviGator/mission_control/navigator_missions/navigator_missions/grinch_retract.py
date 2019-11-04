#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class GrinchRetract(Navigator):
    '''
    Retract the grinch
    '''
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.retract_grinch()
        defer.returnValue(True)
