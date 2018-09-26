#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class DeployThrusters(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Deploying thrusters')
        yield self.deploy_thrusters()
        defer.returnValue('Success')
