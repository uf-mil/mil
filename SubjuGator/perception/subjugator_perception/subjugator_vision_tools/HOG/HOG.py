#!/usr/bin/python3
import sys

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from HOG_detector import HOGDetector
from sensor_msgs.msg import Image


class HOG:
    def __init__(self, topic):
        print(topic)
        self.hog = HOGDetector()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img = self.hog.detect(cv_image)
            cv2.imshow("window", img)
            cv2.waitKey(20)
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node("HOG", anonymous=True)
    HOG(args[1])
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.DestroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
