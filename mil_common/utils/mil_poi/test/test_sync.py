#!/usr/bin/env python3
import random
import string
import sys
import unittest

import genpy
import rclpy
import rostest
from geometry_msgs.msg import Point, PointStamped
from mil_poi.srv import (
    AddPOI,
    DeletePOI,
    MovePOI,
    MovePOIRequest,
)
from std_msgs.msg import Header


class POITest(unittest.TestCase):
    def setUp(self):
        # make one poi here
        rclpy.init(args=sys.argv)

        self.poi_name = "test_poi"
        self.poi_position = PointStamped(header=Header(), point=Point(0.0, 1.0, 2.0))
        self.add_poi()

    def add_poi(self):
        service = self.create_client(AddPOI, "/poi_server/add")
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        response = service.call_async(self.poi_name, self.poi_position)
        rclpy.spin_until_future_complete(self, response)
        self.assertTrue(response.success)

    def test_add(self):
        # Call the add_poi function to add a POI
        self.add_poi()

    def test_move(self):
        # Call the add_poi function to add a POI
        self.add_poi()

        # Wait for the move_poi service to become available
        move_service = self.create_client(MovePOI, "/poi_server/move")
        while not move_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        # Move the POI to a new position
        new_position = [1.0, 2.0, 3.0]  # New position coordinates
        move_response = move_service.call_async(
            MovePOIRequest(
                name=self.poi_name,
                position=PointStamped(point=Point(*new_position)),
            ),
        )
        # Check if the additions were unsuccessful
        rclpy.spin_until_future_complete(self, move_response)
        self.assertTrue(move_response.success, f"Failed to move POI '{self.poi_name}'")

    def test_delete(self):
        # Call the add_poi function to add a POI
        self.add_poi()

        # Wait for the remove_poi service to become available
        remove_service = self.create_client(DeletePOI, "/poi_server/delete")
        while not remove_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        # Remove the added POI
        remove_response = remove_service.call_async(self.poi_name)
        rclpy.spin_until_future_complete(self, remove_response)

        # Check if the removal was successful
        self.assertTrue(
            remove_response.success,
            f"Failed to delete POI '{self.poi_name}'",
        )

    def test_long_string(self):
        service = self.create_client(AddPOI, "/poi_server/add")
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        # Create a long string for the POI name
        long_string = "".join(random.choices(string.ascii_letters, k=20000))

        # Call the service to add a new POI with the long string name
        response = service.call_async(
            long_string,
            PointStamped(header=Header(), point=Point(0.0, 1.0, 2.0)),
        )
        rclpy.spin_until_future_complete(self, response)
        self.assertTrue(response.success, response.message)

    def test_wrong_types(self):
        # Wait for the add_poi service to become available
        service = self.create_client(AddPOI, "/poi_server/add")
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        response = service.call_async(self.poi_name, self.poi_position)
        rclpy.spin_until_future_complete(self, response)
        # Try adding a POI with wrong types of arguments
        with self.assertRaises(genpy.message.SerializationError):
            service(12321, [0.0, 2.3, 21.3])  # Incorrect name type
            service("tester", ["hi", "wrong", "bad"])  # Incorrect position type

    def test_delete_twice(self):
        # Call the add_poi function to add a POI
        self.add_poi()

        # Wait for the remove_poi service to become available
        remove_service = self.create_client(DeletePOI, "/poi_server/delete")
        while not remove_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        # Remove the added POI
        remove_response1 = remove_service.call_async(self.poi_name)

        # Try removing the same POI again
        remove_response2 = remove_service.call_async(self.poi_name)

        # Check if the first removal was successful and the second removal was unsuccessful
        self.assertTrue(
            remove_response1.success,
            f"Failed to remove POI '{self.poi_name}'",
        )
        self.assertFalse(
            remove_response2.success,
            f"Removed POI '{self.poi_name}' twice",
        )


if __name__ == "__main__":
    rostest.rosrun("mil_poi", "test_poi", POITest)
    unittest.main()
