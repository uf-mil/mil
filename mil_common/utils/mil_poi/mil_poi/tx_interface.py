#!/usr/bin/env python3

import asyncio

import txros
from mil_ros_tools.msg_helpers import rosmsg_to_numpy
from twisted.internet import defer

from .msg import POIArray


class TxPOIClient:
    """
    TXros interface for a POI client. Currently more limited than the regular :class:`POIServer`.
    """

    # TODO: add service interfaces for adding / moving / deleting POI

    def __init__(self, nh):
        """
        Create a TxPOIClient with a txros nodehandle object
        """
        self.last_msg = None
        self.futures = {}
        self._poi_sub = nh.subscribe("/points_of_interest", POIArray, callback=self._cb)

    async def setup(self):
        await self._poi_sub.setup()

    def _cb(self, msg):
        """
        Internal callback on new points_of_interest updates
        """
        self.last_msg = msg
        for poi in self.last_msg.pois:
            if poi.name not in self.futures:
                continue
            position = rosmsg_to_numpy(poi.position)
            futures = self.futures.pop(poi.name)
            while len(futures):
                futures.pop().set_result(position)

    async def get(self, name, only_fresh=False):
        """
        Get the position of POI in the global frame as a 3x1 numpy array.
        Note: returns a deferred object which will be calledback when the POI is found, which
              may be immediately
        @param name: the name of the POI
        @param only_fresh: if the POI is already known, wait for a fresh message before returning

        """
        if self.last_msg is not None and not only_fresh:
            for poi in self.last_msg.pois:
                if poi.name == name:
                    return rosmsg_to_numpy(poi.position)
        res = asyncio.Future()
        if name in self.futures:
            self.futures[name].append(res)
        else:
            self.futures[name] = [res]
        poi = await res
        return poi
