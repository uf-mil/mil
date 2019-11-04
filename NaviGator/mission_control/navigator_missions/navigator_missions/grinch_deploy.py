#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class GrinchDeploy(Navigator):
    '''
    Deploy the grinch
    '''
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.deploy_grinch()
        defer.returnValue(True)
