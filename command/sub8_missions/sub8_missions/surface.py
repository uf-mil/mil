#!/usr/bin/env python
from sub_singleton import SubjuGator
from txros import util
from twisted.internet import defer


class Surface(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        self.send_feedback('Surfacing')
        yield self.move.depth(0.2).go()
        yield self.nh.sleep(5)
        yield self.move.depth(1.5).go()
        defer.returnValue('Success!')
