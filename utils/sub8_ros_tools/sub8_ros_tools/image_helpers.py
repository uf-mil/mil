#!/usr/bin/python
'''
Note:
    The repeated use of CvBridge (instead of using make_image_msg and get_image_msg in the classes)
     is intentional, to avoid the use of a global cvbridge, and to avoid reinstantiating a CvBrige for each use.
'''
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


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
        self.im_pub = rospy.Publisher(topic, Image, queue_size=queue_size)
        self.bridge = CvBridge()    
        self.encoding = encoding
    
    def publish(self, cv_image):
        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, self.encoding)
            self.im_pub.publish(image_message)
        except CvBridgeError, e:
            print e


class Image_Subscriber(object):
    def __init__(self, topic, callback=None, encoding="bgr8", queue_size=1):
        '''Calls $callback on each image every time a new image is published on $topic
        Assumes topic of type "sensor_msgs/Image"
        This behaves like a conventional subscriber, except handling the additional image conversion
        '''
        self.encoding = encoding
        self.im_sub = rospy.Subscriber(topic, Image, self.convert, queue_size=queue_size)
        self.bridge = CvBridge()
        self.callback = callback

    def convert(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, desired_encoding=self.encoding)
            self.callback(image)
        except CvBridgeError, e:
            print e
