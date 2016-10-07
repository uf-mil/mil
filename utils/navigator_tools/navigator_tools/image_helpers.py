#!/usr/bin/python
'''
Note:
    The repeated use of CvBridge (instead of using make_image_msg and get_image_msg in the classes)
     is intentional, to avoid the use of a global cvbridge, and to avoid reinstantiating a CvBrige for each use.
'''
import rospy
import numpy as np
from os import path
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from navigator_tools.init_helpers import wait_for_param


def get_parameter_range(parameter_root):
    '''
    ex: parameter_root='/vision/buoy/red'
    this will then fetch /vision/buoy/red/hsv_low and hsv_high
    '''
    low_param, high_param = parameter_root + '/hsv_low', parameter_root + '/hsv_high'

    rospy.logwarn("Blocking -- waiting for parameters {} and {}".format(low_param, high_param))

    wait_for_param(low_param)
    wait_for_param(high_param)
    low = rospy.get_param(low_param)
    high = rospy.get_param(high_param)

    rospy.loginfo("Got {} and {}".format(low_param, high_param))
    return np.array([low, high]).transpose()


def make_image_msg(cv_image, encoding='bgr8'):
    '''Take a cv image, and produce a ROS image message'''
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding)
    return image_message


def get_image_msg(ros_image, encoding='bgr8'):
    '''Take a ros image message, and yield an opencv image'''
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding=encoding)
    return cv_image


class Image_Publisher(object):
    def __init__(self, topic, encoding="bgr8", queue_size=1):
        '''Create an essentially normal publisher, that will publish images without conversion hassle'''
        self.bridge = CvBridge()
        self.encoding = encoding
        self.im_pub = rospy.Publisher(topic, Image, queue_size=queue_size)

    def publish(self, cv_image):
        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, self.encoding)
            self.im_pub.publish(image_message)
        except CvBridgeError, e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)


class Image_Subscriber(object):
    def __init__(self, topic, callback=None, encoding="bgr8", queue_size=1):
        '''Calls $callback on each image every time a new image is published on $topic
        Assumes topic of type "sensor_msgs/Image"
        This behaves like a conventional subscriber, except handling the additional image conversion
        '''
        if callback is None:
            callback = lambda im: setattr(self, 'last_image', im)

        self.encoding = encoding
        self.camera_info = None
        self.last_image_time = None
        self.last_image = None
        self.im_sub = rospy.Subscriber(topic, Image, self.convert, queue_size=queue_size)

        root_topic, image_subtopic = path.split(topic)
        self.info_sub = rospy.Subscriber(root_topic + '/camera_info', CameraInfo, self.info_cb, queue_size=queue_size)

        self.bridge = CvBridge()
        self.callback = callback

    def wait_for_camera_info(self, timeout=10):
        '''
        Blocks until camera info has been received.
        Note: 'timeout' is in seconds.
        '''
        rospy.logwarn("Blocking -- waiting at most %d seconds for camera info." % timeout)

        timeout = rospy.Duration(timeout)
        rospy.sleep(.1)  # Make sure we don't have negative time
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time < timeout) and (not rospy.is_shutdown()):
            if self.camera_info is not None:
                rospy.loginfo("Camera info found!")
                return self.camera_info
            rospy.sleep(.2)

        rospy.logerr("Camera info not found.")
        raise Exception("Camera info not found.")

    def info_cb(self, msg):
        """The path trick here is a hack"""
        self.info_sub.unregister()
        self.camera_info = msg

    def convert(self, data):
        self.last_image_time = data.header.stamp
        try:
            image = self.bridge.imgmsg_to_cv2(data, desired_encoding=self.encoding)
            self.callback(image)
        except CvBridgeError, e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)