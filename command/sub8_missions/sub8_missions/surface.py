#!/usr/bin/env python
from sub_singleton import SubjuGator
from txros import util
from twisted.internet import defer


class Surface(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self):
        self.send_feedback('Surfacing')
        yield self.move.depth(0.6).go()
        defer.returnValue('Success!')
