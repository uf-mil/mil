#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class DeployThrusters(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback("Deploying thrusters")
        yield self.deploy_thrusters()
        defer.returnValue("Success")
