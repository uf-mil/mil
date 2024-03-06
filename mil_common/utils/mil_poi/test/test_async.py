#!/usr/bin/env python3

import asyncio
import unittest

import axros
import rostest
from geometry_msgs.msg import Point, PointStamped
from mil_poi.srv import (
    AddPOI,
    AddPOIRequest,
    DeletePOI,
    DeletePOIRequest,
    MovePOI,
    MovePOIRequest,
)
from std_msgs.msg import Header


class POITestAsync(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.nh = axros.NodeHandle.from_argv("basic", always_default_name=True)
        await self.nh.setup()
        self.add_service = self.nh.get_service_client("/poi_server/add", AddPOI)
        self.move_service = self.nh.get_service_client("/poi_server/move", MovePOI)
        self.delete_service = self.nh.get_service_client(
            "/poi_server/delete",
            DeletePOI,
        )
        await asyncio.gather(
            self.add_service.wait_for_service(),
            self.move_service.wait_for_service(),
            self.delete_service.wait_for_service(),
        )

    async def test_points(self):
        # Adding
        test = await self.add_service(
            AddPOIRequest(
                name="test",
                position=PointStamped(header=Header(), point=Point(0.0, 1.0, 2.0)),
            ),
        )
        self.assertTrue(test.success)
        test = await self.add_service(
            AddPOIRequest(
                name="test2",
                position=PointStamped(header=Header(), point=Point(0.0, 1.0, 2.0)),
            ),
        )
        self.assertTrue(test.success)

        # Moving
        test = await self.move_service(
            MovePOIRequest(
                name="test",
                position=PointStamped(header=Header(), point=Point(0.0, -1.0, -2.0)),
            ),
        )
        self.assertTrue(test.success)
        test = await self.move_service(
            MovePOIRequest(
                name="test2",
                position=PointStamped(header=Header(), point=Point(0.0, 0.0, 0.0)),
            ),
        )
        self.assertTrue(test.success)

        # Moving a non-existent POI should return False
        test = await self.move_service(
            MovePOIRequest(
                name="test3",
                position=PointStamped(header=Header(), point=Point(0.0, 0.0, 0.0)),
            ),
        )
        self.assertFalse(test.success)

        # Deleting
        test = await self.delete_service(DeletePOIRequest(name="test"))
        self.assertTrue(test.success)
        test = await self.delete_service(DeletePOIRequest(name="test2"))
        self.assertTrue(test.success)

        # Deleting a non-existent POI should return False
        test = await self.delete_service(DeletePOIRequest(name="test"))
        self.assertFalse(test.success)

        # Deleting a non-existent POI should return False
        test = await self.delete_service(DeletePOIRequest(name="test3"))
        self.assertFalse(test.success)

    async def asyncTearDown(self):
        await self.nh.shutdown()


if __name__ == "__main__":
    rostest.rosrun("mil_poi", "test_poi", POITestAsync)
    unittest.main()
