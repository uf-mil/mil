#!/usr/bin/env python
"""Scan The Code Mission Script."""
import txros
from navigator_scan_the_code import ScanTheCodeMission
from mil_misc_tools.text_effects import fprint
from navigator import Navigator

___author___ = "Tess Bianchi"


def _get_color(c):
    if c == 'r':
        return "RED"
    if c == 'b':
        return 'BLUE'
    if c == 'y':
        return 'YELLOW'
    if c == 'g':
        return 'GREEN'


class ScanTheCode(Navigator):
    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        """Main Script of Scan The Code."""
        # attempts = kwargs["attempts"]
        # fprint("ATTEMPTS {}".format(attempts), msg_color="green")
        self.change_wrench("autonomous")
        fprint("Moving to stc", msg_color='green')
        mission = ScanTheCodeMission(self)
        yield mission.init_(self.tf_listener)
        pose, look_at = yield mission.initial_position()
        yield self.move.set_position(pose).look_at(look_at).go()
        yield self.nh.sleep(1)
        fprint("Finished getting the initial position", msg_color='green')

        # circle = self.move.circle_point(look_at).go()
        # circle.addErrback(lambda x: x)
        # yield mission.correct_pose()
        # circle.cancel()

        mission.correct_pose(pose)
        circle = self.move.d_circle_point(look_at, radius=8, granularity=30, direction='cw')
        # print list(circle)
        for p in list(circle)[::-1]:
            if mission.stc_correct:
                break
            yield p.go()
            # yield self.nh.sleep(2)

        fprint("Finished getting the correct stc face", msg_color='green')

        defer = mission.find_colors()

        # TRY WITHOUT CIRCLES
        colors = None
        try:
            colors = yield txros.util.wrap_timeout(defer, 15)
        except txros.util.TimeoutError:
            pass

        # TRY WITH CIRCLEs
        # circle = self.move.d_circle_point(look_at, radius=8, granularity=4, direction='cw')
        # colors = None
        # for i in circle:
        #     try:
        #         colors = yield txros.util.wrap_timeout(defer, 15)
        #         break
        #     except txros.util.TimeoutError:
        #         yield i.go()
        #         fprint("go", msg_color="red")

        if colors is None:
            colors = "r", "g", "b"
        c1, c2, c3 = colors
        print colors
        yield self.mission_params["scan_the_code_color1"].set(_get_color(c1))
        yield self.mission_params["scan_the_code_color2"].set(_get_color(c2))
        yield self.mission_params["scan_the_code_color3"].set(_get_color(c3))

    @txros.util.cancellableInlineCallbacks
    def cleanup(self):
        """Safe exit of the Scan The Code mission."""
        yield self.mission_params["scan_the_code_color1"].set("RED")
        yield self.mission_params["scan_the_code_color2"].set("GREEN")
        yield self.mission_params["scan_the_code_color3"].set("BLUE")
