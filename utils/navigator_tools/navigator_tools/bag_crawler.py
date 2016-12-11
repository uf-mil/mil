#!/usr/bin/python
"""
This file wis written by the  team at UF MIL for the 2016 robosub competition.

github.com/uf-mil
"""
import rosbag
from cv_bridge import CvBridge
import progressbar


class BagCrawler(object):

    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.bag = rosbag.Bag(self.bag_path)
        self.bridge = CvBridge()

    def convert(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        return img

    def crawl(self, topic=None, is_image=False, max_msgs=float('inf')):
        num_seen = 0
        num_msgs = 0
        bar = progressbar.ProgressBar(max_value=self.bag.get_message_count())
        if is_image:
            topic = self.image_topics[0]
        for msg_topic, msg, t in self.bag.read_messages():
            bar.update(num_msgs)
            num_msgs += 1
            if msg_topic != topic:
                continue
            if num_seen > max_msgs:
                break
            num_seen += 1
            yield msg

    @property
    def image_topics(self, cam="right"):
        all_topics = self.bag.get_type_and_topic_info()[1].keys()
        all_types = self.bag.get_type_and_topic_info()[1].values()
        topics = [all_topics[k] for k, topic in enumerate(all_topics) if (all_types[k][0] == 'sensor_msgs/Image')]
        if cam == "right":
            topics = [topics[i] for i, t in enumerate(topics) if "right" in t]
        if cam == "left":
            topics = [topics[i] for i, t in enumerate(topics) if "left" in t]
        return topics

    @property
    def image_info_topics(self, cam="right"):
        all_topics = self.bag.get_type_and_topic_info()[1].keys()
        all_types = self.bag.get_type_and_topic_info()[1].values()
        topics = [all_topics[k] for k, topic in enumerate(all_topics) if (all_types[k][0] == 'sensor_msgs/CameraInfo')]
        if cam == "right":
            topics = [topics[i] for i, t in enumerate(topics) if "right" in t]
        if cam == "left":
            topics = [topics[i] for i, t in enumerate(topics) if "left" in t]
        return topics

if __name__ == '__main__':
    import cv2

    bag = '/home/jacob/catkin_ws/src/Sub8/gnc/sub8_perception/data/bag_test.bag'
    bc = BagCrawler(bag)

    for image in bc.crawl(topic=bc.image_topics[0]):
        cv2.imshow('current_image', image)
        cv2.waitKey(3)
