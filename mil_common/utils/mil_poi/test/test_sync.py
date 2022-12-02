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
        pass

    def test_add(self):
        rospy.wait_for_service('/poi_server/add')
        service = rospy.ServiceProxy("/poi_server/add", AddPOI)
        test = service("test", PointStamped(header = Header(), point = Point(0.0, 1.0, 2.0)))
        self.assertTrue(test.success)

#    def test_move(self): 
#        p = POIServer()
#        #how do i know which POIs exist already 
#
#    def test_delete(self):
#        p = POIServer()
#
#    def test_long_string(self): 
#        p = POIServer()
#        long_string = ''.join(random.choices(string.ascii_letters, k=20000))
#        poi = p.add_poi(long_string, [1.0, 2.0, 3.0])
#        self.assertTrue(poi)
#
#    def test_wrong_types(self): 
#        p = POIServer()
#        poi1 = p.add_poi(12321, [0.0, 2.3, 21.3])
#        poi2 = p.add_poi("tester", ["hi", "wrong" "bad"])
#        poi3 = p.remove_poi(10000)
#        poi4 = p.remove_poi([0, 3])
#        poi5 = p.update_position() #not sure what second parameter even is 
#        
#        self.assertFalse(poi1)
#        self.assertFalse(poi2)
#        self.assertFalse(poi3)
#        self.assertFalse(poi4)
#        self.assertFalse(poi5)
#
#    def test_delete_twice(self):
#        p = POIServer()
#        poi1 = p.remove_poi("test")
#        poi2 = p.remove_poi("test")
#        self.asserFalse(poi2)
#
   # def tearDown(self):
       # self.p.cleanup() #not sure if this is right 

if __name__ == "__main__": 
    rostest.rosrun("mil_poi", "test_poi", POITest)
    unittest.main()





