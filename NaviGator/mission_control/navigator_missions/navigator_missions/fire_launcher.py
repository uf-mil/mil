#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class FireLauncher(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.fire_launcher()
        defer.returnValue("Success")
