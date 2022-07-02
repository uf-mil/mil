#!/usr/bin/env python3
from .detect_deliver_find import DetectDeliverFind
from .track_target import TrackTarget
from txros import util

from .navigator import Navigator


class DetectDeliver(Navigator):
    @classmethod
    def init(cls):
        cls.detect_deiliver_find = DetectDeliverFind()
        cls.track_target = TrackTarget()
        pass

    @util.cancellableInlineCallbacks
    def run(self, args):
        self.send_feedback("Starting detect deliver")
        yield self.detect_deiliver_find.run(
            self.detect_deiliver_find.decode_parameters(args)
        )
        yield self.track_target.run(self.track_target.decode_parameters(args))
        self.send_feedback("Done with detect deliver")
