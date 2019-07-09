#!/usr/bin/env python
from sub_singleton import SubjuGator
from txros import util
from twisted.internet import defer


class BallDropTest(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        self.send_feedback('Dropping Ball')
        yield self.actuators.drop_marker()
        defer.returnValue('Success!')
