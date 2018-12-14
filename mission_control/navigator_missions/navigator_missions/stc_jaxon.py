#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
from navigator_msgs.msg import ScanTheCode


class ScanTheCodeJaxon(Navigator):
    @classmethod
    def init(cls):
        cls.stcsub = cls.nh.subscribe("/scan_the_code", ScanTheCode)
        pass

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Parse Arguments

        self.stc = yield self.get_sorted_objects(name='stc_platform', n=1)
        self.stc = self.stc[1][0]

        yield self.move.look_at(self.stc).go()

        yield self.move.set_position(self.stc).backward(5).yaw_left(1.57).go()

        # Get scan the code stuff
        stc_result = yield self.stcsub.get_next_message()
        stc_result = stc_result.color_pattern
        self.net_stc_result = stc_result

        if stc_result[0] == 'R':
            yield self.move.look_at(self.stc).go()
            yield self.move.left(10).forward(15).go()
        elif stc_result[0] == 'G':
            yield self.move.look_at(self.stc).go()
            yield self.move.right(10).forward(15).go()
        elif stc_result[0] == 'B':
            yield self.move.circle_point(self.stc, direction='cw', revolutions=1.25).go()
            yield self.move.forward(10).go()

        self.send_feedback('Done!')
