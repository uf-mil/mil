#!/usr/bin/env python3
import unittest
import rostest
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from mil_poi.srv import AddPOIRequest, AddPOI

class POITest(unittest.TestCase):
        def setUp(self):
        #make one poi here 
           rospy.init_node("poi_test_node")
           self.poi_name = "test_poi"
           self.poi_position = PointStamped(header=Header(), point=Point(0.0, 1.0, 2.0))
           self.add_poi()

        def add_poi(self):
           rospy.wait_for_service('/poi_server/add')
           service = rospy.ServiceProxy("/poi_server/add", AddPOI)
           response = service(self.poi_name, self.poi_position)
           self.assertTrue(response.success)

        def test_add(self):
            # Call the add_poi function to add a POI
           self.add_poi()

           # Check if the POI was added successfully
           rospy.wait_for_service('/poi_server/get')
           get_service = rospy.ServiceProxy("/poi_server/get", GetPOI)
           response = get_service(self.poi_name)
           self.assertTrue(response.success, f"Failed to add POI '{self.poi_name}'")

        def test_move(self):
        # Call the add_poi function to add a POI
           self.add_poi()

            # Wait for the move_poi service to become available
           rospy.wait_for_service('/poi_server/move')
           move_service = rospy.ServiceProxy("/poi_server/move", MovePOI)

            # Move the POI to a new position
           new_position = [1.0, 2.0, 3.0]  # New position coordinates
           move_response = move_service(self.poi_name, new_position)
           self.assertTrue(move_response.success, f"Failed to move POI '{self.poi_name}'")

        def test_delete(self):
    # Call the add_poi function to add a POI
           self.add_poi()

    # Wait for the remove_poi service to become available
           rospy.wait_for_service('/poi_server/remove')
           remove_service = rospy.ServiceProxy("/poi_server/remove", RemovePOI)

            # Remove the added POI
           remove_response = remove_service(self.poi_name)

            # Check if the removal was successful
           self.assertTrue(remove_response.success, f"Failed to remove POI '{self.poi_name}'")

        def test_long_string(self):
           rospy.wait_for_service('/poi_server/add')
           service = rospy.ServiceProxy("/poi_server/add", AddPOI)

            # Create a long string for the POI name
           long_string = ''.join(random.choices(string.ascii_letters, k=20000))

            # Call the service to add a new POI with the long string name
           response = service(long_string, PointStamped(header=Header(), point=Point(0.0, 1.0, 2.0)))
           self.assertFalse(response.success, "Added POI with long string name")

        def test_wrong_types(self):
            # Wait for the add_poi service to become available
           rospy.wait_for_service('/poi_server/add')
           service = rospy.ServiceProxy("/poi_server/add", AddPOI)

    # Try adding a POI with wrong types of arguments
           response1 = service(12321, [0.0, 2.3, 21.3])  # Incorrect name type
           response2 = service("tester", ["hi", "wrong", "bad"])  # Incorrect position type

            # Check if the additions were unsuccessful
           self.assertFalse(response1.success, "Added POI with wrong argument types")
           self.assertFalse(response2.success, "Added POI with wrong argument types")

        def test_delete_twice(self):
            # Call the add_poi function to add a POI
           self.add_poi()

            # Wait for the remove_poi service to become available
           rospy.wait_for_service('/poi_server/remove')
           remove_service = rospy.ServiceProxy("/poi_server/remove", RemovePOI)

            # Remove the added POI
           remove_response1 = remove_service(self.poi_name)

            # Try removing the same POI again
           remove_response2 = remove_service(self.poi_name)

            # Check if the first removal was successful and the second removal was unsuccessful
           self.assertTrue(remove_response1.success, f"Failed to remove POI '{self.poi_name}'")
           self.assertFalse(remove_response2.success, f"Removed POI '{self.poi_name}' twice")

        def tearDown(self):
            # Clean up any resources or state modified during testing
           if hasattr(self, 'p') and callable(getattr(self.p, 'cleanup', None)):
              self.p.cleanup()

if __name__ == "__main__": 
    rostest.rosrun("mil_poi", "test_poi", POITest)
    unittest.main()





