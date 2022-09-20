#!/usr/bin/env python3

import asyncio

import numpy as np
from mil_ros_tools.msg_helpers import rosmsg_to_numpy
from txros import NodeHandle

from .msg import POIArray


class TxPOIClient:
    """
    Client for getting the positions of POIs in a POI server.
    """

    # TODO: add service interfaces for adding / moving / deleting POI

    def __init__(self, nh: NodeHandle):
        """
        Args:
            nh (txros.NodeHandle): The node handle to use.
        """
        self.last_msg = None
        self.futures = {}
        self._poi_sub = nh.subscribe("/points_of_interest", POIArray, callback=self._cb)

    async def setup(self) -> None:
        """
        Sets up the client. Must be called before using the client.
        """
        await self._poi_sub.setup()

    async def shutdown(self) -> None:
        await self._poi_sub.shutdown()

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

    async def get(self, name: str, only_fresh: bool = False) -> np.ndarray:
        """
        Get the position of POI in the global frame as a 3x1 numpy array.

        Args:
            name (str): The name of the POI.
            only_fresh (bool): If the POI is already known, wait for a fresh
                message before returning.
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
