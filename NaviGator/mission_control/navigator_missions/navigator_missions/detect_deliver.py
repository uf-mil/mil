#!/usr/bin/env python3
from .detect_deliver_find import DetectDeliverFind
from .navigator import NaviGatorMission

# from .track_target import TrackTarget


class DetectDeliver(NaviGatorMission):
    @classmethod
    async def setup(cls):
        cls.detect_deiliver_find = DetectDeliverFind()
        # cls.track_target = TrackTarget()

    async def run(self, args):
        self.send_feedback("Starting detect deliver")
        await self.detect_deiliver_find.run(
            self.detect_deiliver_find.decode_parameters(args),
        )
        await self.track_target.run(self.track_target.decode_parameters(args))
        self.send_feedback("Done with detect deliver")
