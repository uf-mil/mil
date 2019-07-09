#!/usr/bin/env python
from msg import POIArray
from twisted.internet import defer
import txros
from mil_ros_tools.msg_helpers import rosmsg_to_numpy


class TxPOIClient(object):
    '''
    TXros interface for a POI client

    TODO: add service interfaces for adding / moving / deleting POI
    '''
    def __init__(self, nh):
        '''
        Create a TxPOIClient with a txros nodehandle object
        '''
        self.last_msg = None
        self.defers = {}
        self._poi_sub = nh.subscribe("/points_of_interest", POIArray, callback=self._cb)

    def _cb(self, msg):
        '''
        Internal callback on new points_of_interest updates
        '''
        self.last_msg = msg
        for poi in self.last_msg.pois:
            if poi.name not in self.defers:
                continue
            position = rosmsg_to_numpy(poi.position)
            defers = self.defers.pop(poi.name)
            while len(defers):
                defers.pop().callback(position)

    @txros.util.cancellableInlineCallbacks
    def get(self, name, only_fresh=False):
        '''
        Get the position of POI in the global frame as a 3x1 numpy array.
        Note: returns a defered object which will be calledback when the POI is found, which
              may be immediately
        @param name: the name of the POI
        @param only_fresh: if the POI is already known, wait for a fresh message before returning

        '''
        if self.last_msg is not None and not only_fresh:
            for poi in self.last_msg.pois:
                if poi.name == name:
                    yield defer.returnValue(rosmsg_to_numpy(poi.position))
        res = defer.Deferred()
        if name in self.defers:
            self.defers[name].append(res)
        else:
            self.defers[name] = [res]
        poi = yield res
        defer.returnValue(poi)
