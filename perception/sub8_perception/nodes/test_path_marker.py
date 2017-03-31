#!/usr/bin/env python
PKG = 'test_path_marker'
import rosbag
import rospy
import sys
import unittest
import numpy as np
from sub8_msgs.srv import VisionRequest2D, VisionRequest2DRequest, VisionRequest2DResponse
from sensor_msgs.msg import Image, CameraInfo

class TestPathMarker(unittest.TestCase):
    def __init__(self, *args):
        rospy.init_node("path_marker_test_py", anonymous=True)
        rospy.wait_for_service('/vision/path_marker/get', 3.0)
        self.service = rospy.ServiceProxy('/vision/path_marker/get', VisionRequest2D) 
        self.cam_info_pub = rospy.Publisher('/down_camera/camera_info', CameraInfo, queue_size=5)
        self.img_pub = rospy.Publisher('/down_camera/image_rect_color', Image, queue_size=5)
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
        print "Correct pose: {}".format(correct.pose.theta/(2*np.pi)*360.0)
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
            if topic == "/down_camera/image_rect_color":
                self.img_pub.publish(msg)
            if topic == "/down_camera/camera_info":
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
        #5 degrees error
        theta_err = abs(res.pose.theta-correct.pose.theta)
        msg = "Marker pose angle (theta) too much error Res={} Correct={} Error={}".format(np.degrees(res.pose.theta),
                                                                                           np.degrees(correct.pose.theta),
                                                                                           np.degrees(theta_err))
        self.assertLess(theta_err, 
                        0.087266463,
                        msg=msg)

    def test_transdec(self):
        self._test_bag('/home/kallen/bag/path_marker_transdcec/fixed/transdec_path1.bag',
                       rospy.Duration(3.10),
                       [ [110, 373], [252,281] ])

    def test_transdec2(self):
        self._test_bag('/home/kallen/bag/path_marker_transdcec/fixed/transdec_path2.bag',
                       rospy.Duration(1.9),
                       [ [392, 166], [428,16] ])
    def test_transdec3(self):
        self._test_bag('/home/kallen/bag/path_marker_transdcec/fixed/transdec_path3.bag',
                       rospy.Duration(0.8),
                       [ [244,9],[321, 127] ])
    
    def test_pool1(self):
        self._test_bag('/home/kallen/bag/path_marker_transdcec/fixed/pool_path1.bag',
                       rospy.Duration(2.7),
                       [ [236, 410],[295, 216] ])

    def test_pool2(self):
        self._test_bag('/home/kallen/bag/path_marker_transdcec/fixed/pool_path2.bag',
                       rospy.Duration(2.7),
                       [ [180, 244], [313, 333] ])

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'path_marker_test', TestPathMarker)
