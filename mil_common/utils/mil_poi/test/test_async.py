#!/usr/bin/env python3
import unittest
import rostest
import rospy
import axros
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from mil_poi.srv import AddPOIRequest, AddPOI

class POITestAsync(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.nh = axros.NodeHandle.from_argv("basic", always_default_name = True) 
        await self.nh.setup()
        self.service = self.nh.get_service_client("poi_server/add", AddPOI)

    async def test_add(self):
        await self.service.wait_for_service()
        test = await self.service("test", PointStamped(header = Header(), point = Point(0.0, 1.0, 2.0)))
        self.assertTrue(test.success)

    async def asyncTearDown(self):
        await self.nh.shutdown()

if __name__ == "__main__": 
    rostest.rosrun("mil_poi", "test_poi", POITestAsync)
    unittest.main()





