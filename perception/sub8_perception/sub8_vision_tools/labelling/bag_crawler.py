import rosbag
from cv_bridge import CvBridge


class BagCrawler(object):
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.bag = rosbag.Bag(self.bag_path)
        self.bridge = CvBridge()

    def convert(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        return img

    def crawl(self, topic, max_msgs=float('inf')):
        num_seen = 0
        for msg_topic, msg, t in self.bag.read_messages():
            if msg_topic != topic:
                continue
            if num_seen > max_msgs:
                break

            num_seen += 1

            image = self.convert(msg)
            yield image

    @property
    def image_topics(self):
        all_topics = self.bag.get_type_and_topic_info()[1].keys()
        all_types = self.bag.get_type_and_topic_info()[1].values()
        topics = [all_topics[k] for k, topic in enumerate(all_topics) if (all_types[k][0] == 'sensor_msgs/Image')]
        return topics

if __name__ == '__main__':
    import cv2

    bag = '/home/jacob/catkin_ws/src/Sub8/gnc/sub8_perception/data/bag_test.bag'
    bc = BagCrawler(bag)

    for image in bc.crawl(topic=bc.image_topics[0]):
        cv2.imshow('current_image', image)
        cv2.waitKey(3)
