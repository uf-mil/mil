#!/usr/bin/env python3
"""
This file wis written by the team at UF MIL for the 2016 robosub competition.

github.com/uf-mil
"""
from typing import List, Optional

import rosbag
import tqdm
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class BagCrawler:
    """
    Crawls a bag file and displays each image message of a camera through
    OpenCV.

    Attributes:
        bag_path (str): The path to the bag file.
        bag (rosbag.Bag): The bag object representing a bag file at the file location.
    """

    def __init__(self, bag_path: str):
        self.bag_path = bag_path
        self.bag = rosbag.Bag(self.bag_path)
        self.bridge = CvBridge()

    def convert(self, msg: Image):
        """
        Converts an Image message to a cv2 compatible image.

        Args:
            msg (sensor_msgs.msg.Image): The Image message.
        """
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return img

    def crawl(
        self,
        topic: Optional[str] = None,
        is_image: bool = False,
        max_msgs: float = float("inf"),
    ):
        """
        Crawls a bag file and returns a generator yielding each message in the
        file. Additionally, a tqdm progress bar is displayed.

        Args:
            topic (Optional[str]): ???
            is_image (bool): ???
            max_msgs (float): The maximum number of messages to return.

        Yields:
            genpy.Message: A message in the bag file.
        """
        num_seen = 0
        num_msgs = 0
        bar = tqdm.tqdm(total=self.bag.get_message_count())
        if is_image:
            topic = self.image_topics[0]
        for msg_topic, msg, _ in self.bag.read_messages():
            bar.update(num_msgs)
            num_msgs += 1
            if msg_topic != topic:
                continue
            if num_seen > max_msgs:
                break
            num_seen += 1
            yield msg
        bar.close()

    @property
    def image_topics(self, cam: str = "right") -> List[str]:
        """
        .. warning::

            This function may not function properly. If the method works,
            please remove this message.

        Finds all topics that use the sensor_msgs/Image type and exist on a
        specific camera.

        Args:
            cam (str): The camera to find image topics on. Should either be
                'left' or 'right'.
        """
        all_topics = list(self.bag.get_type_and_topic_info()[1].keys())
        all_types = list(self.bag.get_type_and_topic_info()[1].values())
        topics = [
            all_topics[k]
            for k, _ in enumerate(all_topics)
            if (all_types[k][0] == "sensor_msgs/Image")
        ]
        if cam == "right":
            topics = [topics[i] for i, t in enumerate(topics) if "right" in t]
        if cam == "left":
            topics = [topics[i] for i, t in enumerate(topics) if "left" in t]
        return topics

    @property
    def image_info_topics(self, cam: str = "right") -> List[str]:
        """
        .. warning::

            This function may not function properly. If the method works,
            please remove this message.

        Finds all topics that use the sensor_msgs/CameraInfo type and exist on a
        specific camera.

        Args:
            cam (str): The camera to find image topics on. Should either be
                'left' or 'right'.
        """
        all_topics = list(self.bag.get_type_and_topic_info()[1].keys())
        all_types = list(self.bag.get_type_and_topic_info()[1].values())
        topics = [
            all_topics[k]
            for k, _ in enumerate(all_topics)
            if (all_types[k][0] == "sensor_msgs/CameraInfo")
        ]
        if cam == "right":
            topics = [topics[i] for i, t in enumerate(topics) if "right" in t]
        if cam == "left":
            topics = [topics[i] for i, t in enumerate(topics) if "left" in t]
        return topics


if __name__ == "__main__":
    import cv2

    bag = "test.bag"
    bc = BagCrawler(bag)

    for image in bc.crawl(topic=bc.image_topics[0]):
        cv2.imshow("current_image", image)
        cv2.waitKey(3)
