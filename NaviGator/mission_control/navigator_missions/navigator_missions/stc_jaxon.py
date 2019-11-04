#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
from navigator_msgs.msg import ScanTheCode


class ScanTheCodeJaxon(Navigator):
    TIMEOUT_SECONDS = 30.

    @classmethod
    def init(cls):
        cls.stcsub = cls.nh.subscribe("/scan_the_code", ScanTheCode)
        cls.stcpub = cls.nh.advertise("/scan_the_code", ScanTheCode)

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Parse Arguments

        self.stc = yield self.get_sorted_objects(name='stc_platform', n=1)
        self.stc = self.stc[1][0]

        yield self.move.look_at(self.stc).go()

        yield self.move.set_position(self.stc).backward(5).yaw_left(1.57).go()

        print 'SELECTING'

        # Get scan the code stuff
        try:
            result = yield util.wrap_timeout(self.stcsub.get_next_message(), self.TIMEOUT_SECONDS, "test")
            stc_result = result.color_pattern
        except util.TimeoutError:
            stc_result = "RGB"
            self.stcpub.publish(ScanTheCode(color_pattern=stc_result))

        self.net_stc_results = stc_result

        if stc_result[0] == 'R':
            yield self.move.look_at(self.stc).go()
            yield self.move.left(10).forward(15).go()
        elif stc_result[0] == 'G':
            yield self.move.look_at(self.stc).go()
            yield self.move.right(10).forward(15).go()
        elif stc_result[0] == 'B':
            yield self.move.left(5).go()
            yield self.move.circle_point(self.stc, direction='cw', revolutions=1.25).go()
            yield self.move.forward(10).go()

        self.send_feedback('Done!')
