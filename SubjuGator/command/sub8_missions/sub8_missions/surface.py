#!/usr/bin/env python3
from sub_singleton import SubjuGator
from twisted.internet import defer
from txros import util


class Surface(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        self.send_feedback("Surfacing")
        yield self.move.depth(0.2).go()
        defer.returnValue("Success!")
