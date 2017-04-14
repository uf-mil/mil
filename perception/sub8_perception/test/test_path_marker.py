#!/usr/bin/env python
from __future__ import division
import rosbag
import rospy
import sys
import unittest
import numpy as np
from sub8_msgs.srv import VisionRequest2D, VisionRequest2DRequest, VisionRequest2DResponse
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool, SetBoolRequest

PKG = 'sub8_perception'
class TestPathMarker(unittest.TestCase):
    '''
    Unit test for perception service finding orange path markers
    Plays several bags with known pixel coordinates of path marker,
    calls service to get 2D pose and check that it is close to known
    pose.
    '''
    def __init__(self, *args):
        self.name = rospy.get_name()
        rospy.set_param("test", 5)
        self.info_topic = "/camera/down/left/camera_info"
        self.img_topic = "/camera/down/left/image_rect_color"
        rospy.wait_for_service('/vision/path_marker/2D', 3.0)
        rospy.wait_for_service('/vision/path_marker/enable', 3.0)
        self.service = rospy.ServiceProxy('/vision/path_marker/2D', VisionRequest2D)
        enable = rospy.ServiceProxy('/vision/path_marker/enable', SetBool)
        enable(SetBoolRequest(data=True))
        self.cam_info_pub = rospy.Publisher(self.info_topic, CameraInfo, queue_size=5)
        self.img_pub = rospy.Publisher(self.img_topic, Image, queue_size=5)
        super(TestPathMarker, self).__init__(*args)

    def get_ideal_response(self, pts):
        pts = np.array(pts)
        res = VisionRequest2DResponse()
        middle = pts[0]+(pts[1]-pts[0])/2
        theta = np.arctan( (float(pts[1][1])-pts[0][1]) / (pts[1][0] - pts[0][0]) )
        res.pose.x = middle[0]
        res.pose.y = middle[1]
        res.pose.theta = theta
        return res

    def _test_bag(self, filename, duration, pts):
        rospy.sleep(rospy.Duration(0.5))
        correct = self.get_ideal_response(pts)
        print "Correct pose: {}".format(np.degrees(correct.pose.theta))
        bag = rosbag.Bag(filename)
        first = True
        first_time = None
        previous_time = None
        i = 0
        for topic, msg, t in bag.read_messages():
            if first:
                first_time = t
                previous_time = t
                first = False
            else:
                rospy.sleep(t-previous_time) # Slow down to watch happen
                previous_time = t
                if t - first_time > duration:
                    did = True
                    break
            if topic == self.img_topic:
                self.img_pub.publish(msg)
            if topic == self.info_topic:
                self.cam_info_pub.publish(msg)
        res = self.service(VisionRequest2DRequest())
        # 10 pixel error accepted
        res_xy = np.array([res.pose.x, res.pose.y])
        correct_xy = np.array([correct.pose.x, correct.pose.y])
        err = np.linalg.norm(res_xy-correct_xy)
        msg="Marker pose (x,y) too much error Res={} Correct={} Error={}".format(res_xy, correct_xy, err)
        self.assertLess(err, 
                        10.0,
                        msg=msg)
        #5 degrees error accepted
        theta_err = abs(np.arctan2(np.sin(res.pose.theta-correct.pose.theta), np.cos(res.pose.theta-correct.pose.theta)))
        msg = "Marker pose angle (theta) too much error Res={} Correct={} Error={}".format(np.degrees(res.pose.theta),
                                                                                           np.degrees(correct.pose.theta),
                                                                                           np.degrees(theta_err))
        self.assertLess(theta_err, 
                        0.15,
                        msg=msg)

    def test_transdec(self):
        self._test_bag(rospy.get_param(self.name+'/transdec_path1'),
                       rospy.Duration(3.10),
                       [ [43, 443], [216,307] ])

    #Disabled because not long enough
    def test_transdec2(self):
        self._test_bag(rospy.get_param(self.name+'/transdec_path2'),
                       rospy.Duration(1.9),
                       [ [392, 166], [428,16] ])
    def test_transdec3(self):
        self._test_bag(rospy.get_param(self.name+'/transdec_path3'),
                       rospy.Duration(0.8),
                       [ [244,9],[321, 127] ])
    # Disabled because too noisey
    #~ def test_pool1(self):
        #~ self._test_bag('/home/kallen/bag/path_marker_transdcec/fixed/pool_path1.bag',
                       #~ rospy.Duration(2.7),
                       #~ [ [237, 407],[295, 216] ])

    def test_pool2(self):
        self._test_bag(rospy.get_param(self.name+'/pool_path2'),
                       rospy.Duration(2.7),
                       [ [180, 244], [313, 333] ])
   
    def test_pool3(self):
        self._test_bag(rospy.get_param(self.name+'/pool_path3'),
                       rospy.Duration(5.8),
                       [ [188, 330], [354, 322] ])

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_path_marker', anonymous=True)
    print "name", rospy.get_name()
    print "namespace" ,rospy.get_namespace()
    rostest.rosrun(PKG, 'path_marker_test', TestPathMarker)
