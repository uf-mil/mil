#!/usr/bin/env python3
from twisted.internet import defer
from txros import util

from .sub_singleton import SubjuGator


class BallDropTest(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        self.send_feedback("Dropping Ball")
        yield self.actuators.drop_marker()
        defer.returnValue("Success!")
