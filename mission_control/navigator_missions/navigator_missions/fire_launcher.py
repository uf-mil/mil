#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class FireLauncher(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.fire_launcher()
        defer.returnValue('Success')
