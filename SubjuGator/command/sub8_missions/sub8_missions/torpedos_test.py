#!/usr/bin/env python3
from sub_singleton import SubjuGator
from twisted.internet import defer
from txros import util


class TorpedosTest(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        self.send_feedback("Shooting Torpedo 1")
        yield self.actuators.shoot_torpedo1()
        self.send_feedback("Done! Shooting Torpedo 2 in 1 second")
        yield self.nh.sleep(1)
        yield self.actuators.shoot_torpedo2()
        defer.returnValue("Success!")
