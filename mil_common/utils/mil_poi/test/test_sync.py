#!/usr/bin/env python3
import random
import string
import unittest

import genpy
import rospy
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
        rospy.init_node("poi_test_node")
        self.poi_name = "test_poi"
        self.poi_position = PointStamped(header=Header(), point=Point(0.0, 1.0, 2.0))
        self.add_poi()

    def add_poi(self):
        rospy.wait_for_service("/poi_server/add")
        service = rospy.ServiceProxy("/poi_server/add", AddPOI)
        response = service(self.poi_name, self.poi_position)
        self.assertTrue(response.success)

    def test_add(self):
        # Call the add_poi function to add a POI
        self.add_poi()

    def test_move(self):
        # Call the add_poi function to add a POI
        self.add_poi()

        # Wait for the move_poi service to become available
        rospy.wait_for_service("/poi_server/move")
        move_service = rospy.ServiceProxy("/poi_server/move", MovePOI)

        # Move the POI to a new position
        new_position = [1.0, 2.0, 3.0]  # New position coordinates
        move_response = move_service(
            MovePOIRequest(
                name=self.poi_name,
                position=PointStamped(point=Point(*new_position)),
            ),
        )
        # Check if the additions were unsuccessful
        self.assertTrue(move_response.success, f"Failed to move POI '{self.poi_name}'")

    def test_delete(self):
        # Call the add_poi function to add a POI
        self.add_poi()

        # Wait for the remove_poi service to become available
        rospy.wait_for_service("/poi_server/delete")
        remove_service = rospy.ServiceProxy("/poi_server/delete", DeletePOI)

        # Remove the added POI
        remove_response = remove_service(self.poi_name)

        # Check if the removal was successful
        self.assertTrue(
            remove_response.success,
            f"Failed to delete POI '{self.poi_name}'",
        )

    def test_long_string(self):
        rospy.wait_for_service("/poi_server/add")
        service = rospy.ServiceProxy("/poi_server/add", AddPOI)

        # Create a long string for the POI name
        long_string = "".join(random.choices(string.ascii_letters, k=20000))

        # Call the service to add a new POI with the long string name
        response = service(
            long_string,
            PointStamped(header=Header(), point=Point(0.0, 1.0, 2.0)),
        )
        self.assertTrue(response.success, response.message)

    def test_wrong_types(self):
        # Wait for the add_poi service to become available
        rospy.wait_for_service("/poi_server/add")
        service = rospy.ServiceProxy("/poi_server/add", AddPOI)

        # Try adding a POI with wrong types of arguments
        with self.assertRaises(genpy.message.SerializationError):
            service(12321, [0.0, 2.3, 21.3])  # Incorrect name type
            service("tester", ["hi", "wrong", "bad"])  # Incorrect position type

    def test_delete_twice(self):
        # Call the add_poi function to add a POI
        self.add_poi()

        # Wait for the remove_poi service to become available
        rospy.wait_for_service("/poi_server/delete")
        remove_service = rospy.ServiceProxy("/poi_server/delete", DeletePOI)

        # Remove the added POI
        remove_response1 = remove_service(self.poi_name)

        # Try removing the same POI again
        remove_response2 = remove_service(self.poi_name)

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
