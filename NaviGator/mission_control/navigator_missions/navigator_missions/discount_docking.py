#!/usr/bin/env python3
from txros import util

from .navigator import Navigator


class DiscountDocking(Navigator):
    @util.cancellableInlineCallbacks
    def run(self, args):
        yield self.move.forward(5).go()

        yield self.navigator.nh.sleep(10)

        yield self.move.backward(5).go()
        yield self.move.backward(5).go()
        yield self.move.backward(5).go()

        self.send_feedback("Done!")
