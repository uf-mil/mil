"""Shows images for debugging purposes."""
import sys
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from txros import NodeHandle

___author___ = "Tess Bianchi"


class CvDebug:
    """
    Class that contains methods that assist with debugging with images.

    Attributes:
        width (int): The width of the debug image
        height (int): The height of the debug image
        nh (Optional[NodeHandle]): The node handle for the image stream. If ``None``, then images are displayed through the OpenCV GUI.
        total (int): ???
        hor_num (float): ???
        max_width (float): ???
        max_height (float): ???
        wait (bool): Whether or not to wait after showing the image. Set in the constructor.
        win_name (str): The name of the OpenCV GUI image display window. Defaults to ``"debug"``.

    """

    def __init__(
        self,
        nh: Optional[NodeHandle] = None,
        w: int = 1000,
        h: int = 800,
        total: int = 8,
        win_name: str = "debug",
        wait: bool = True,
    ):
        """
        Initialize the Debug class.

        Args:
            nh (NodeHandle): The node handle for the image stream
            w (int): The width of the image that smaller images are added to
            h (int): The height of the image that smaller images are added to
            win_name (str): the name of the window that is shown in opencv
            wait (bool): whether or not to wait after showing the image
        """
        self.width = w
        self.height = h
        self.img = np.zeros((h, w, 3), np.uint8)
        self.total = total
        self.hor_num = total / 2
        self.vert_num = 2
        self.max_width = w / self.hor_num
        self.max_height = h / self.vert_num
        self.wait = wait
        self.nh = nh

        self.curr_w = 0
        self.curr_h = 0
        self.num_imgs = 0
        self.win_name = win_name
        self.name_to_starting = {}
        self.bridge = CvBridge()
        self.base_topic = "/debug/"
        self.topic_to_pub = {}
        if nh is None:
            self.pub = rospy.Publisher("/debug/image", Image, queue_size=10)
        else:
            self.pub = nh.advertise("/debug/image", Image)

    def add_image(self, img, name: str, wait: int = 33, topic: str = "image") -> None:
        """
        Add an image to show to either with a topic or using :meth:`cv2.imshow`.

        Args:
            name (str): A unique key name for the image, use the same name if you
                want to switch out this image for another.
            wait (int): The amount of wait time for the imshow image.
            topic (str): The name of the topic to publish the image to.
        """
        color = "bgr8"
        print(img.shape)
        if len(img.shape) == 2 or img.shape[2] == 1:
            color = "mono8"

        if topic != "image":
            self._add_new_topic(img, name, wait, topic)
            return
        if self.wait:
            wait = 0
        h, w = img.shape[0], img.shape[1]

        if w > h:
            img = cv2.resize(img, (self.max_width, h * self.max_width / w))

        if h > w:
            img = cv2.resize(img, (w * self.max_height / h, self.max_height))
        h, w = img.shape[0], img.shape[1]
        if name not in self.name_to_starting:
            if self.num_imgs == self.total:
                print("Too many images")
                return
            self.name_to_starting[name] = (self.curr_w, self.curr_h)
            self.num_imgs += 1

            self.curr_w += w
            if self.num_imgs == self.total / 2:
                self.curr_w = 0
                self.curr_h = self.max_height
            if self.num_imgs > self.total / 2:
                self.name_to_starting[name] = (self.curr_w, self.curr_h)
        my_w, my_h = self.name_to_starting[name]
        self.img[my_h : my_h + h, my_w : my_w + w] = img
        if self.nh is None:
            cv2.imshow("img", self.img)
            if cv2.waitKey(wait) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                sys.exit()

        else:
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.img, color))

    def _add_new_topic(self, img, name: str, wait: int, topic: str) -> None:
        color = "bgr8"
        if len(img.shape) == 2 or img.shape[2] == 1:
            color = "mono8"
        pub = None
        if topic in list(self.topic_to_pub.keys()):
            pub = self.topic_to_pub[topic]
        elif self.nh is None:
            pub = rospy.Publisher("/debug/" + topic, Image, queue_size=10)
        elif self.nh is not None:
            pub = self.nh.advertise("/debug/" + topic, Image)
        self.topic_to_pub[topic] = pub
        pub.publish(self.bridge.cv2_to_imgmsg(img, color))
