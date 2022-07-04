#!/usr/bin/env python3
from twisted.internet import defer
from txros import util

from .sub_singleton import SubjuGator


class GripperTest(SubjuGator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        self.send_feedback("Opening Gripper")
        yield self.actuators.gripper_open()
        self.send_feedback("Done! Closing gripper in 2 seconds.")
        yield self.nh.sleep(2)
        yield self.actuators.gripper_close()
        self.send_feedback("Done! Opening gripper again in 1 second.")
        yield self.nh.sleep(1)
        yield self.actuators.gripper_open()
        defer.returnValue("Success!")
