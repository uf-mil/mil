#!/usr/bin/python
'''
Note:
    The repeated use of CvBridge (instead of using make_image_msg and get_image_msg in the classes)
     is intentional, to avoid the use of a global cvbridge, and to avoid reinstantiating a CvBrige for each use.
'''
import rospy
import tf
import numpy as np
from os import path
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from mil_ros_tools import wait_for_param
import message_filters
from image_geometry import PinholeCameraModel, StereoCameraModel


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


def project_points3d_to_pixels_depths(camera_info, points, pinhole_model=None):
    '''Project 3d points already in the camera optical frame to pixels on the camera image
    @param camera_info: camera_info message

    @param points: numpy array of shape (3, n) where n is the number of points

    @param pinhole_model: initialized pinhole model for the camera, generated from camera_info if not specified

    @return: pixles (a numpy array of shape (2, m) where m is the number of points that projected in the bounds of the camera)
             depths(a numpy array of shape(m,) where every entry coincides with an entry from returned pixles)
    @raise exception: if none of the projected points are in bound of the camera info
    '''
    if pinhole_model is None:
        pinhole_model = PinHoleCameraModel()
        pinhole_model.fromCameraInfo(camera_info)

    pixels = np.apply_along_axis(lambda x: pinhole_model.project3dToPixel(x), 0, points)

    in_bounds = np.argwhere(np.logical_and(
                              np.logical_and(pixels[0,:] > 0,
                                             pixels[0,:] < camera_info.height),
                              np.logical_and(pixels[1,:] > 0,
                                             pixels[1,:] < camera_info.width)))

    if len(in_bounds) == 0:
        raise Exception("no points projected into the camera bounds")
    pixels = np.squeeze(pixels[:,in_bounds]).astype(np.int16)
    depths = np.squeeze(points[2,in_bounds])
    return pixels, depths


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
        self.last_image_header = None
        self.last_image_time = None
        self.last_image = None
        self.im_sub = rospy.Subscriber(
            topic, Image, self.convert, queue_size=queue_size)

        root_topic, image_subtopic = path.split(rospy.remap_name(topic))
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

    def wait_for_camera_model(self, **kwargs):
        info_msg = self.wait_for_camera_info(**kwargs)
        model = PinholeCameraModel()
        model.fromCameraInfo(info_msg)
        return model

    def info_cb(self, msg):
        """The path trick here is a hack"""
        self.info_sub.unregister()
        self.camera_info = msg

    def convert(self, data):
        self.last_image_header = data.header
        self.last_image_time = data.header.stamp
        try:
            image = self.bridge.imgmsg_to_cv2(
                data, desired_encoding=self.encoding)
            self.callback(image)
        except CvBridgeError, e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)


class MilStereoCamera(object):
    '''
    class to hold the data associated with a single stereo camera
    '''
    def __init__(self, image_topic, queue_size):

        self.root_topic, self.image_subtopic = path.split(image_topic)

        self.info = None
        self.info_sub = rospy.Subscriber(
            self.root_topic + '/camera_info', CameraInfo,
            lambda info: setattr(self, 'info', info), queue_size=queue_size)

        self.image = None
        self.image_sub = message_filters.Subscriber(image_topic, Image)
        self.last_image_time = None

        self.model = None

    def wait_static_optical_transform(self, frame_id):
        '''
        wait for the transform from frame_id to the optical frame of this camera and return it

        @param frame_id: a frame id of a frame that has a static transform to the camera
        @param timeout: number of seconds to try to get the transform.

        @returns: transform from frame_id to the optical frame of the stereo camera
        '''
        listener = tf.TransformListener()
        got_transform = False
        while(not got_transform):
            try:
                transform = listener.lookupTransform(
                                          self.info.header.frame_id,
                                          frame_id,
                                          rospy.Time(0))

                got_transform = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return transform

class StereoImageSubscriber(object):
    '''
    Abstraction to subscribe to two image topics (ex: left and right camera) and
    receive callbacks for synchronized images (already converted to numpy arrays).
    Also contains a helper function to block until the camera info messages for both
    cameras are received.
    '''
    def __init__(self, left_image_topic, right_image_topic, callback=None, slop=0.2, encoding="bgr8", queue_size=10):
        '''
        Contruct a StereoImageSubscriber

        @param left_image_topic: ROS topic to subscribe for the left camera
                                 ex: /camera/front/left/image_rect_color
        @param right_image_topic: ROS topic to subscribe to for the right camera
                                 ex: /camera/front/right/image_rect_color
        @param callback: Function with signature foo(left_img, right_img) to call when a synchronized pair is ready.
               If left as None, the latest synced images are stored as self.last_image_left and self.last_image_right
        @param slop: Maximum time in seconds between left and right images to be considered synced.
               If left as None, will only consider synced if left and right images have exact same header time.
        @param encoding: String to pass to CvBridge to encode ROS image message to numpy array
        @param queue_size: Integer, the number of images to store in a buffer for each camera to find synced images
        '''
        if callback is None:  # Set default callback to just set image_left and image_right
            def callback(image_left, image_right):
                setattr(self.left_camera, 'image', image_left)
                setattr(self.right_camera, 'image', image_right)

        self.bridge = CvBridge()
        self.encoding = encoding
        self.callback = callback

        self.right_camera = MilStereoCamera(right_image_topic, queue_size)
        self.left_camera = MilStereoCamera(left_image_topic, queue_size)

        # Use message_filters library to set up synchronized subscriber to both image topics
        self._image_sub = message_filters.ApproximateTimeSynchronizer([self.left_camera.image_sub, self.right_camera.image_sub],
                                                                          queue_size, slop)
        self._image_sub.registerCallback(self._image_callback)

    def wait_for_camera_info(self, timeout=10, unregister=True):
        '''
        Blocks until camera info has been received.

        @param timeout: Time in seconds to wait before throwing exception if camera info is not received
        @param unregister: Boolean, if True will unsubscribe to camera info after receiving initial info message,
               so self.left_camera.info and self.right_camera.info will not be updated
        @return: Tuple(left.info, right.info) camera info for each camera if received before timeout
        @raise Exception: if camera info for both cameras is not received within timeout
        '''
        timeout = rospy.Time.now() + rospy.Duration(timeout)
        while (rospy.Time.now() < timeout) and (not rospy.is_shutdown()):
            if self.left_camera.info is not None and self.right_camera.info is not None:
                if unregister:
                    self.left_camera.info_sub.unregister()
                    self.right_camera.info_sub.unregister()
                return self.left_camera.info, self.right_camera.info
            rospy.sleep(0.05)
        if self.left_camera.info is not None and self.right_camera.info is not None:
            if unregister:
                self.left_camera.info_sub.unregister()
                self.right_camera.info_sub.unregister()
            return self.left_camera.info, self.right_camera.info
        raise Exception("Camera info not found.")


    def wait_for_camera_model(self, **kwargs):
        '''
        wrapper for wait_for_camera_info that also
        fills out self.camera_model_right and self.camera_model_left with
        Pinhole Camera Models appropriately
        '''
        ret = self.wait_for_camera_info(**kwargs)
        self.right_camera.model = PinholeCameraModel()
        self.left_camera.model = PinholeCameraModel()
        self.stereo_model = StereoCameraModel()
        self.right_camera.model.fromCameraInfo(self.right_camera.info)
        self.left_camera.model.fromCameraInfo(self.left_camera.info)
        self.stereo_model.fromCameraInfo(self.left_camera.info, self.right_camera.info)
        return ret

    def _image_callback(self, left_img, right_img):
        '''
        Internal wrapper around image callback. Updates
        latest timestamps and converts ROS image messages
        to numpy arrays (for use with OpenCV, etc) before
        calling user defined callback.

        @param left_img: the synchronized image from the left camera
        @param right_img: the synchronized image from the right camera
        '''
        try:
            self.left_camera.last_image_time = left_img.header.stamp
            self.right_camera.last_image_time = right_img.header.stamp
            img_left = self.bridge.imgmsg_to_cv2(
                left_img, desired_encoding=self.encoding)
            img_right = self.bridge.imgmsg_to_cv2(
                right_img, desired_encoding=self.encoding)
            self.callback(img_left, img_right)
        except CvBridgeError, e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)






