#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer


class GoToPOI(Navigator):
    '''
    Moves NaviGator to a point of interest
    '''
    @classmethod
    def decode_parameters(cls, parameters):
        parameters = parameters.split()
        if len(parameters) != 1:
            raise Exception('Only one parameter accepted')
        return parameters[0]

    @txros.util.cancellableInlineCallbacks
    def run(self, poi):
        self.send_feedback('Waiting for ' + poi)
        position = yield self.poi.get(poi)
        self.send_feedback('Moving to {} at {}'.format(poi, position[0:2]))
        yield self.change_wrench('autonomous')
        yield self.move.look_at(position).set_position(position).go()
        defer.returnValue('Success')
