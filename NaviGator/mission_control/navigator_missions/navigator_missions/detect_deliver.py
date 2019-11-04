#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
from detect_deliver_find import DetectDeliverFind
from track_target import TrackTarget


class DetectDeliver(Navigator):

    @classmethod
    def init(cls):
        cls.detect_deiliver_find = DetectDeliverFind()
        cls.track_target = TrackTarget()
        pass

    @util.cancellableInlineCallbacks
    def run(self, args):
        self.send_feedback('Starting detect deliver')
        yield self.detect_deiliver_find.run(self.detect_deiliver_find.decode_parameters(args))
        yield self.track_target.run(self.track_target.decode_parameters(args))
        self.send_feedback('Done with detect deliver')
