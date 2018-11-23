#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class DemonstrateNavigation(Navigator):
    '''
    Mission for the "Demonstrate Navigation And Control" challenge. Assumes
    you are starting near the startgate and that the 4 totems making up the startgate
    are identified and classified correctly in the PCODAR database
    TODO: check that moves were completed succesfully
    '''
    START_MARGIN_METERS = 4.0
    END_MARGIN_METERS = 8.0

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        # Get the totems
        _, closest_reds = yield self.get_sorted_objects("totem_red", 2)
        _, closest_greens = yield self.get_sorted_objects("totem_green", 2)

        # Rename the totems for their symantic name
        green_close = closest_greens[0]
        green_far = closest_greens[1]
        red_close = closest_reds[0]
        red_far = closest_reds[1]

        # Get the two center points between gata markers
        begin_midpoint = (green_close + red_close) / 2.0
        end_midpoint = (green_far + red_far) / 2.0

        # Go to autonomous mode
        yield self.change_wrench('autonomous')

        # Start a little behind the entrance
        yield self.move.set_position(begin_midpoint).look_at(end_midpoint).backward(self.START_MARGIN_METERS).go()
        # Then move a little passed the exit
        yield self.move.look_at(end_midpoint).set_position(end_midpoint).forward(self.END_MARGIN_METERS).go(blind=True)
        defer.returnValue('Success!')
