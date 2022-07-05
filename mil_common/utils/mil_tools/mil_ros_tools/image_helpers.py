#!/usr/bin/python3
"""
Note: The repeated use of CvBridge (instead of using make_image_msg and
get_image_msg in the classes) is intentional, to avoid the use of a global
cvbridge, and to avoid reinstantiating a CvBrige for each use.
"""
from os import path
from typing import Callable, List, Optional, Tuple

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image

from .init_helpers import wait_for_param


def get_parameter_range(parameter_root: str):
    """
    Fetches the {parameter_root}/hsv_low and {parameter_root}/hsv_high parameters.
    """
    low_param, high_param = parameter_root + "/hsv_low", parameter_root + "/hsv_high"

    rospy.logwarn(
        "Blocking -- waiting for parameters {} and {}".format(low_param, high_param)
    )

    wait_for_param(low_param)
    wait_for_param(high_param)
    low = rospy.get_param(low_param)
    high = rospy.get_param(high_param)

    rospy.loginfo("Got {} and {}".format(low_param, high_param))
    return np.array([low, high]).transpose()


def make_image_msg(cv_image: List[float], encoding: str = "bgr8"):
    """
    Take a CV image, and produce a ROS image message.
    """
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding)
    return image_message


def get_image_msg(ros_image: Image, encoding: str = "bgr8"):
    """
    Take a ros image message, and return an OpenCV image.
    """
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding=encoding)
    return cv_image


class Image_Publisher:
    """
    Publishes OpenCV image mats directly to a ROS topic, avoiding the need for
    continual conversion.

    Attributes:
        bridge (CvBridge): The ROS bridge to OpenCV. Created upon instantiation.
        encoding (str): The encoding of the images. Supplied upon creation.
            Defaults to ``bgr8``.
        im_pub (rospy.Publisher): The ROS publisher responsible for publishing
            images to a ROS topic. The topic name and queue size are supplied
            through the constructor.
    """

    def __init__(self, topic: str, encoding: str = "bgr8", queue_size: int = 1):
        self.bridge = CvBridge()
        self.encoding = encoding
        self.im_pub = rospy.Publisher(topic, Image, queue_size=queue_size)

    def publish(self, cv_image: np.ndarray):
        """
        Publishes an OpenCV image mat to the ROS topic. :class:`CvBridgeError`
        exceptions are caught and logged.
        """
        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, self.encoding)
            self.im_pub.publish(image_message)
        except CvBridgeError as e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(str(e))


class Image_Subscriber:
    """
    Calls callback on each image every time a new image is published on a certain
    topic. This behaves like a conventional subscriber, except handling the
    additional image conversion.

    Attributes:
        encoding (str): The encoding used to convert between OpenCV and ROS image
            types.
        camera_info (CameraInfo): The information about the camera related to the topic.
        last_image_header (Header): The header of the last image received.
        last_image_time (genpy.Time): The time of the last image received.
        im_sub (rospy.Subscriber): The subscriber to the image topic. The topic
            name and queue size are received through the constructor.
        info_sub (rospy.Susbcriber): The subscriber to the camera info topic.
            The topic nmae is derived from the root of the supplied topic and the
            queue size is derived from the constructor.
        bridge (CvBridge): The bridge between OpenCV and ROS.
        callback (Callable): The callback function to call upon receiving each
            image.
    """

    def __init__(self, topic, callback=None, encoding="bgr8", queue_size=1):
        if callback is None:

            def callback(im):
                return setattr(self, "last_image", im)

        self.encoding = encoding
        self.camera_info = None
        self.last_image_header = None
        self.last_image_time = None
        self.last_image = None
        self.im_sub = rospy.Subscriber(
            topic, Image, self.convert, queue_size=queue_size
        )

        root_topic, image_subtopic = path.split(rospy.remap_name(topic))
        self.info_sub = rospy.Subscriber(
            root_topic + "/camera_info", CameraInfo, self.info_cb, queue_size=queue_size
        )

        self.bridge = CvBridge()
        self.callback = callback

    def wait_for_camera_info(self, timeout: int = 10):
        """
        Blocks until camera info has been received for a number of seconds.

        Args:
            timeout (int): The amount of seconds to wait for camera info.
                Defaults to 10 seconds.

        Raises:
            Exception: No camera info was found after the timeout had finished.
        """
        rospy.logwarn(
            "Blocking -- waiting at most %d seconds for camera info." % timeout
        )

        timeout = rospy.Duration(timeout)
        rospy.sleep(0.1)  # Make sure we don't have negative time
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time < timeout) and (not rospy.is_shutdown()):
            if self.camera_info is not None:
                rospy.loginfo("Camera info found!")
                return self.camera_info
            rospy.sleep(0.2)

        rospy.logerr("Camera info not found.")
        raise Exception("Camera info not found.")

    def wait_for_camera_model(self, **kwargs):
        """
        Waits for the camera model information.

        Returns:
            PinholeCameraModel: The camera model.
        """
        info_msg = self.wait_for_camera_info(**kwargs)
        model = PinholeCameraModel()
        model.fromCameraInfo(info_msg)
        return model

    def info_cb(self, msg: CameraInfo):
        """
        Serves as the callback to a ROS subscriber subscribed to a camera info
        topic.

        This callback only executes once, as calling this method unregisters the
        subscriber.

        Args:
            msg (CameraInfo): The message containing information about the camera.
        """
        self.info_sub.unregister()
        self.camera_info = msg

    def convert(self, data: Image):
        """
        Serves as the callback method to the image topic subscriber. Receives data
        in the form of Image messages, and stores the time and header of the last
        image received.

        Upon receiving the image, this method converts the image to be in the
        OpenCV format and calls the class' :meth:`~Image_Subscriber.callback` method.

        Args:
            data (Image): The message type representing an image in ROS.
        """
        self.last_image_header = data.header
        self.last_image_time = data.header.stamp
        try:
            image = self.bridge.imgmsg_to_cv2(data, desired_encoding=self.encoding)
            self.callback(image)
        except CvBridgeError as e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)


class StereoImageSubscriber:
    """
    Abstraction to subscribe to two image topics (ex: left and right camera) and
    receive callbacks for synchronized images (already converted to numpy arrays).
    Also contains a helper function to block until the camera info messages for both
    cameras are received.

    Attributes:
        bridge (CvBridge): The bridge between ROS and OpenCV. Created when the
            class is instantiated.
        encoding (str): The encoding between ROS and OpenCV.
        callback (Callable): The callback function which executes each time a new
            left and right image is received. The method should accept a parameter
            for the left image and a parameter for the right image, both of which
            are :class:`Image` types.
        camera_info_left (Optional[CameraInfo]): The camera info for the left
            image. Upon instantiation, set to ``None`` and updated only after receiving
            camera info data.
        camera_info_right (Optional[CameraInfo]): The camera info for the right
            image. Upon instantiation, set to ``None`` and updated only after receiving
            camera info data.
        last_image_left (Optional[Image]): The last image received from the left
            camera. This value is set to ``None`` upon class instantiation and
            should be updated by :attr:`StereoImageSubscriber.callback`.
        last_image_right (Optional[Image]): The last image received from the right
            camera. This value is set to ``None`` upon class instantiation and
            should be updated by :attr:`StereoImageSubscriber.callback`.
        last_image_left_time (Optional[genpy.Time]): The timestamp when the last
            left image was received. Set to ``None`` upon class instantiation.
        last_image_right_time (Optional[genpy.Time]): The timestamp when the last
            left image was received. Set to ``None`` upon class instantiation.
    """

    def __init__(
        self,
        left_image_topic: str,
        right_image_topic: str,
        callback: Optional[Callable] = None,
        slop: Optional[Callable] = None,
        encoding: str = "bgr8",
        queue_size: int = 10,
    ):
        """
        Contruct a StereoImageSubscriber

        left_image_topic (str): ROS topic to subscribe for the left camera.
            For example, ``/camera/front/left/image_rect_color``.
        right_image_topic (str): ROS topic to subscribe to for the right
            camera. For example, ``/camera/front/right/image_rect_color``.
        callback (Optional[Callable]): Function with signature ``foo(left_img, right_img)``
            to call when a synchronized pair is ready. If left as ``None``,
            the latest synced images are stored as self.last_image_left
            and self.last_image_right.
        slop (Optional[int]): Maximum time in seconds between left and right
            images to be considered synced. If left as None, will only
            consider synced if left and right images have exact same header time.
        encoding (str): String to pass to CvBridge to encode ROS image message
            to numpy array
        queue_size (int): Integer, the number of images to store in a buffer
            for each camera to find synced images.
        """
        if (
            callback is None
        ):  # Set default callback to just set image_left and image_right

            def callback(image_left, image_right):
                setattr(self, "last_image_left", image_left)
                setattr(self, "last_image_right", image_right)

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
            root_topic_left + "/camera_info",
            CameraInfo,
            lambda info: setattr(self, "camera_info_left", info),
            queue_size=queue_size,
        )
        image_sub_left = message_filters.Subscriber(left_image_topic, Image)
        root_topic_right, image_subtopic_right = path.split(right_image_topic)
        self._info_sub_right = rospy.Subscriber(
            root_topic_right + "/camera_info",
            CameraInfo,
            lambda info: setattr(self, "camera_info_right", info),
            queue_size=queue_size,
        )
        image_sub_right = message_filters.Subscriber(right_image_topic, Image)

        # Use message_filters library to set up synchronized subscriber to both image topics
        if slop is None:
            self._image_sub = message_filters.TimeSynchronizer(
                [image_sub_left, image_sub_right], queue_size
            )
        else:
            self._image_sub = message_filters.ApproximateTimeSynchronizer(
                [image_sub_left, image_sub_right], queue_size, slop
            )
        self._image_sub.registerCallback(self._image_callback)

    def wait_for_camera_info(
        self, timeout: int = 10, unregister: bool = True
    ) -> Tuple[CameraInfo, CameraInfo]:
        """
        Blocks until camera info has been received.

        Args:
            timeout (int): Time in seconds to wait before throwing exception if camera info is not received
            unregister (bool): Whether to unsubscribe from the camera info topics
                after receiving the first message so that :attr:`~StereoImageSubscriber.camera_info_left`
                and :attr:`~StereoImageSubscriber.camera_info_right` will not be updated.

        Returns:
            Tuple[CameraInfo, CameraInfo]: A tuple containing the camera info for the left and right cameras, respectively.

        Raises:
            Exception: if camera info for both cameras is not received within timeout
        """
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

    def _image_callback(self, left_img: Image, right_img: Image):
        """
        Internal wrapper around image callback. Updates latest timestamps and
        converts ROS image messages to numpy arrays (for use with OpenCV, etc)
        before calling user defined callback.

        Args:
            left_img (Image): The synchronized image from the left camera
            right_img (Image): The synchronized image from the right camera
        """
        try:
            self.last_image_time_left = left_img.header.stamp
            self.last_image_time_right = right_img.header.stamp
            img_left = self.bridge.imgmsg_to_cv2(
                left_img, desired_encoding=self.encoding
            )
            img_right = self.bridge.imgmsg_to_cv2(
                right_img, desired_encoding=self.encoding
            )
            self.callback(img_left, img_right)
        except CvBridgeError as e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)
