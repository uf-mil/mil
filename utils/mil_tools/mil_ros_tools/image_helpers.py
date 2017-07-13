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
from mil_ros_tools import wait_for_param
import message_filters


def get_parameter_range(parameter_root):
    '''
    ex: parameter_root='/vision/buoy/red'
    this will then fetch /vision/buoy/red/hsv_low and hsv_high
    '''
    low_param, high_param = parameter_root + \
        '/hsv_low', parameter_root + '/hsv_high'

    rospy.logwarn(
        "Blocking -- waiting for parameters {} and {}".format(low_param, high_param))

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
            def callback(im):
                return setattr(self, 'last_image', im)

        self.encoding = encoding
        self.camera_info = None
        self.last_image_time = None
        self.last_image = None
        self.im_sub = rospy.Subscriber(
            topic, Image, self.convert, queue_size=queue_size)

        root_topic, image_subtopic = path.split(topic)
        self.info_sub = rospy.Subscriber(
            root_topic + '/camera_info', CameraInfo, self.info_cb, queue_size=queue_size)

        self.bridge = CvBridge()
        self.callback = callback

    def wait_for_camera_info(self, timeout=10):
        '''
        Blocks until camera info has been received.
        Note: 'timeout' is in seconds.
        '''
        rospy.logwarn(
            "Blocking -- waiting at most %d seconds for camera info." % timeout)

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
            image = self.bridge.imgmsg_to_cv2(
                data, desired_encoding=self.encoding)
            self.callback(image)
        except CvBridgeError, e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)


class StereoImageSubscriber(object):
    '''
    Abstraction to subscribe to two image topics (ex: left and right camera) and
    receive callbacks for synchronized images (already converted to numpy arrays).
    Also contains a helper function to block until the camera info messages for both
    cameras are received.
    '''
    def __init__(self, left_image_topic, right_image_topic, callback=None, slop=None, encoding="bgr8", queue_size=10):
        '''
        Contruct a StereoImageSubscriber

        @param left_image_topic ROS topic to subscribe for the left camera ex: /camera/front/left/image_rect_color
        @param right_image_topic ROS topic to subscribe to for the right camera ex: /camera/front/right/image_rect_color
        @param callback Function with signature foo(left_img, right_img) to call when a synchronized pair is ready.
               If left as None, the latest synced images are stored as self.last_image_left and self.last_image_right
        @param slop Maximum time in seconds between left and right images to be considered synced.
               If left as None, will only consider synced if left and right images have exact same header time.
        @param encoding String to pass to CvBridge to encode ROS image message to numpy array
        @param queue_size Integer, the number of images to store in a buffer for each camera to find synced images
        '''
        if callback is None:  # Set default callback to just set image_left and image_right
            def callback(image_left, image_right):
                setattr(self, 'last_image_left', image_left)
                setattr(self, 'last_image_right', image_right)

        self.bridge = CvBridge()
        self.encoding = encoding
        self.callback = callback
        self.camera_info_left = None
        self.camera_info_right = None
        self.last_image_left = None
        self.last_image_time_left = None
        self.last_image_right = None
        self.last_image_time_right = None

        # Subscribe to image and camera info topics for both cameras
        root_topic_left, image_subtopic_left = path.split(left_image_topic)
        self._info_sub_left = rospy.Subscriber(
            root_topic_left + '/camera_info', CameraInfo,
            lambda info: setattr(self, 'camera_info_left', info), queue_size=queue_size)
        image_sub_left = message_filters.Subscriber(left_image_topic, Image)
        root_topic_right, image_subtopic_right = path.split(right_image_topic)
        self._info_sub_right = rospy.Subscriber(
            root_topic_right + '/camera_info', CameraInfo,
            lambda info: setattr(self, 'camera_info_right', info), queue_size=queue_size)
        image_sub_right = message_filters.Subscriber(right_image_topic, Image)

        # Use message_filters library to set up synchronized subscriber to both image topics
        if slop is None:
            self._image_sub = message_filters.TimeSynchronizer([image_sub_left, image_sub_right], queue_size)
        else:
            self._image_sub = message_filters.ApproximateTimeSynchronizer([image_sub_left, image_sub_right],
                                                                          queue_size, slop)
        self._image_sub.registerCallback(self._image_callback)

    def wait_for_camera_info(self, timeout=10, unregister=True):
        '''
        Blocks until camera info has been received.

        @param timeout Time in seconds to wait before throwing exception if camera info is not received
        @param unregister Boolean, if True will unsubscribe to camera info after receiving initial info message,
               so self.camera_info_left and self.camera_info_right will not be updated
        @return Tuple(camera_info_left, camera_info_right) camera info for each camera if received before timeout
        @throws Exception if camera info for both cameras is not received within timeout
        '''
        timeout = rospy.Time.now() + rospy.Duration(timeout)
        while (rospy.Time.now() < timeout) and (not rospy.is_shutdown()):
            if self.camera_info_left is not None and self.camera_info_right is not None:
                if unregister:
                    self._info_sub_left.unregister()
                    self._info_sub_right.unregister()
                return self.camera_info_left, self.camera_info_right
            rospy.sleep(0.05)
        if self.camera_info_left is not None and self.camera_info_right is not None:
            if unregister:
                self._info_sub_left.unregister()
                self._info_sub_right.unregister()
            return self.camera_info_left, self.camera_info_right
        raise Exception("Camera info not found.")

    def _image_callback(self, left_img, right_img):
        '''
        Internal wrapper around image callback. Updates
        latest timestamps and converts ROS image messages
        to numpy arrays (for use with OpenCV, etc) before
        calling user defined callback.

        @param left_img the synchronized image from the left camera
        @param right_img the synchronized image from the right camera
        '''
        try:
            self.last_image_time_left = left_img.header.stamp
            self.last_image_time_right = right_img.header.stamp
            img_left = self.bridge.imgmsg_to_cv2(
                left_img, desired_encoding=self.encoding)
            img_right = self.bridge.imgmsg_to_cv2(
                right_img, desired_encoding=self.encoding)
            self.callback(img_left, img_right)
        except CvBridgeError, e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)
