#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer
import numpy as np

class Kill(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.nh.sleep(0.1)
        defer.returnValue('I am pointless!')
