#!/usr/bin/env python
import unittest
import rospy
from nav_msgs.msg import Odometry
from time import sleep
import rostest

class SylphaseTestCase(unittest.TestCase):
    sylphase_pub = False
    sylphase_hz_rate = 0
    high_hz = False

    def callback(self, data):
        self.sylphase_hz_rate += 1

    
    def test_ins_odom_frequency(self):
        sleep(8) #allows ample time for sim to start up
        rospy.init_node('test_sylphase')
        rospy.Subscriber('/ins_odom', Odometry, self.callback)
        sleep(1)      
        
        #tests if ins_odom is being published
        if self.sylphase_hz_rate > 0:
            self.sylphase_pub = True
        self.assertTrue(self.sylphase_pub)
        
        #tests if ins_odom is being published at a rate above 25 hz
        if self.sylphase_hz_rate > 25:
            self.high_hz = True
        self.assertTrue(self.high_hz)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('test_package', 'test_name', SylphaseTestCase)
