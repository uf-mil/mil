#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class GrinchDeploy(Navigator):
    """
    Deploy the grinch
    """

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.deploy_grinch()
        defer.returnValue(True)
