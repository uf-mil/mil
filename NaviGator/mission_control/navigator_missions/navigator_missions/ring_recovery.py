#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class RingRecovery(Navigator):
    '''
    Completes Ring Recovery challenge by circling the marker totem
    while spinning the grinch
    '''
    RADIUS = 2.5
    DEPLOY_DISTANCE = 10.0

    @txros.util.cancellableInlineCallbacks
    def get_marker_totem(self):
        _, totem = yield self.get_sorted_objects("totem_gator", n=1)
        defer.returnValue(totem.reshape(3))

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Fetching position of marker totem')
        totem = yield self.get_marker_totem()

        # Switch to auotnomous
        yield self.change_wrench('autonomous')

        # Move in front of it, but back a little
        self.send_feedback('Moving in front of marker totem')
        yield self.move.look_at(totem).set_position(totem).backward(self.DEPLOY_DISTANCE).go()

        # Deploy the grinch mechanism into the water
        self.send_feedback('Deploying grinch')
        yield self.deploy_grinch()

        # Move back to let mechanism drop
        yield self.move.backward(5.).go()

        # Move in closer to begin circle
        self.send_feedback('Updating position of marker totem')
        totem = yield self.get_marker_totem()
        self.send_feedback('Moving closer')
        yield self.move.look_at(totem).set_position(totem).backward(self.RADIUS).go(blind=True)

        # Circle totem
        self.send_feedback('Spinning grinch')
        d = self.spin_grinch()
        self.send_feedback('Circling marker totem')
        yield self.move.circle_point(totem).go(blind=True)
        self.send_feedback('Stopping grinch')
        yield d.cancel()

        # Backup from totem to safely retract grinch
        self.send_feedback('Backing up')
        yield self.move.backward(self.DEPLOY_DISTANCE).go(move_type="skid", blind=True)

        # Retract the mechnism so we can move on
        self.send_feedback('Retracting grinch')
        yield self.retract_grinch()

        defer.returnValue('Success!')
