#!/usr/bin/env python
from txros.util import cancellableInlineCallbacks
from twisted.internet import defer
from mil_tools import rosmsg_to_numpy
import numpy as np
from navigator import Navigator


class CircleTotemSimple(Navigator):
    '''
    Simple mission to circle totems once they have been labeled, does not
    have searching funcitonality found
    '''
    CIRCLE_DISTANCE = 6.0  # Distance around totem to circle
    DIRECTIONS = {'RED': 'cw', 'GREEN': 'ccw', 'BLUE': 'cw', 'YELLOW': 'ccw', 'WHITE': 'ccw'}

    @cancellableInlineCallbacks
    def run(self, parameters):
        # Get colors of intrest and directions
        c1 = self.mission_params['scan_the_code_color1'].get()
        c2 = self.mission_params['scan_the_code_color2'].get()
        c3 = self.mission_params['scan_the_code_color3'].get()
        colors = [c1, c2, c3]

        targets = []
        for color in colors:
            totem = yield self.get_colored_totem(color.lower())
            if totem is None:
                self.send_feedback('Totem {} not found'.format(color))
            else:
                self.send_feedback("Totem {} found!".format(color))
                targets.append((rosmsg_to_numpy(totem.pose.position), color))
            yield self.nh.sleep(0.1)
        if len(targets) == 0:
            defer.returnValue('No totems with correct color found')
        self.send_feedback('Targets: {}'.format(targets))
        yield self.nh.sleep(0.1)
        for target in targets:
            color = target[1]
            position = target[0]
            direction = self.DIRECTIONS[color]
            self.send_feedback('Attempting to circle {} {}'.format(color, direction))
            self.send_feedback('Moving in front of totem')
            yield self.nh.sleep(0.1)
            res = yield self.move.look_at(position).set_position(position).backward(self.CIRCLE_DISTANCE).go()
            self.send_feedback('RESDFDFD {}'.format(res))
            self.send_feedback('Circling!')
            yield self.nh.sleep(0.1)
            res = yield self.move.circle_point(position, direction=direction).go()
            self.send_feedback('Got {}'.format(res))
            self.send_feedback('Done circling')
            yield self.nh.sleep(0.1)
        defer.returnValue('Success')

    @cancellableInlineCallbacks
    def get_colored_totem(self, color):
        totems = yield self.database_query("totem_{}".format(color), raise_exception=False)
        if not totems.found:
            defer.returnValue(None)
        sort = sorted(totems.objects,
                      key=lambda totem: np.linalg.norm(self.pose[0] - rosmsg_to_numpy(totem.pose.position)))
        defer.returnValue(sort[0])
