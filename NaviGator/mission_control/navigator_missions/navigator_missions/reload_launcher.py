#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class ReloadLauncher(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.reload_launcher()
        defer.returnValue('Success')
