#!/usr/bin/env python
'''
script to detect dice , and mark the value of the dice
Subscribes to Image topic
Publishes Image and Points
'''

from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image  # To publish Image
from geometry_msgs.msg import Point  # To publish Points
from std_msgs.msg import String  # To publish debug statements
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance

# Display Window size for Debug
WINDOW_SIZE_H = 1000
WINDOW_SIZE_W = 1000

# Parameters for dice detections
DIST_THRESHOLD = 100
SEARCH_RANGE_FOR_PIPS = 2
SENTINEL_INFINITE = 10000


class DiceDetect(object):
    """docstring for DiceDetect."""

    def __init__(self):
        '''
        debug_image_publisher publishes the final detected image
        debug_image_input_publisher publishes input image to the detect function
        debug_talker publishes chatter of whether the program is running
        '''
        self.debug_image_publisher = rospy.Publisher(
            "dice/debugimage", Image, queue_size=1)
        self.debug_image_input_publisher = rospy.Publisher(
            "dice/debugimageinput", Image, queue_size=1)
        self.point_publisher = rospy.Publisher(
            "dice/points", Point, queue_size=1)
        self.debug_talker = rospy.Publisher(
            "dice/debugtalker", String, queue_size=1)
        self.image_subscriber = rospy.Subscriber(
            "/camera/front/left/image_rect_color", Image, self.callback)
        self.bridge = CvBridge()

    def detect(self, dice_img):

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = 1
        params.blobColor = 0  # 0 to pick dark blobs , 255 to pick white blobs
        params.minDistBetweenBlobs = 0

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255
        # The idea is that, we want to check all thresholds to take care of varied lighting conditions

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 25  # in pixels , 10 to pick up from far away , but also picks up noise
        # 25 is an ideal amount
        params.maxArea = 200000  # max are in pixels

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.70
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.8  # 1 = perfect convex hull
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.4  # Defines the ellipsoid 1= detects only cirlces
        # 0 = Detects even lines

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:
            detector = cv2.SimpleBlobDetector(params)
        else:
            detector = cv2.SimpleBlobDetector_create(params)

        # im_c = Colored image
        im_c = dice_img
        d = dict()
        # im = GrayScale Image
        im = cv2.cvtColor(im_c, cv2.COLOR_BGR2GRAY)

        # Has the value of all blobs within the image
        keypoints = detector.detect(im)

        # For Visualization purposes
        # Draw detected blobs as red circles
        im_with_keypoints = cv2.drawKeypoints(
            im_c, keypoints, np.array([]), (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Debug to print all blobs detected
        key_len = len(keypoints)
        # print(keypoints)

        # Neighborhood search algorithm
        '''
        Looks at every point in the keypoints array and finds the nearest point to it
        Within a Threshold, set by Global Var 'DIST_THRESHOLD' . Then looks for other points
        within 'SEARCH_RANGE_FOR_PIPS' times the 'DIST_THRESHOLD' .Basically within a circle of
        radius twice the size of the nearest pip.

        '''

        for i in range(0, key_len):
            a = keypoints[i].pt
            dist = SENTINEL_INFINITE
            count = 0
            for j in range(i + 1, key_len):
                b = keypoints[j].pt
                dist_itr = distance.euclidean(a, b)
                if (dist_itr <= dist and dist_itr < DIST_THRESHOLD):
                    dist = dist_itr

            for j in range(0, key_len):
                b = keypoints[j].pt
                dist_itr = distance.euclidean(a, b)
                if (dist_itr <= SEARCH_RANGE_FOR_PIPS * dist):
                    count = count + 1
            if (dist == SENTINEL_INFINITE):
                count = 1

            if count > 4:
                # Debug Statement
                # Prints all points which correspond to dice of value 4 or more
                print(str(i) + " th detection is " + str(count))  # Debug
                d[str(count)] = [keypoints[i].pt[0], keypoints[i].pt[1]]

                font_face = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(im_with_keypoints,
                            (str(int(keypoints[i].pt[0])) + ', ' + str(
                                int(keypoints[i].pt[1]))),
                            (int(keypoints[i].pt[0]), int(keypoints[i].pt[1])),
                            font_face, 1, (255, 255, 255), 1, cv2.LINE_AA)

        # im_with_keypoints now displays image, keypoints and x, y values displayed as text

        return d, im_with_keypoints, im_c

    def callback(self, subscriberd_data):

        # calling CvBridge to transfer between ROS Image and OpenCv Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(subscriberd_data, "bgr8")
        except CvBridgeError as e:
            print("Error at Cv.Bridge at Subscribing")
            print(e)

        # rospy.sleep is necessary to process the videofeed
        rospy.sleep(1.)  # 1 frame per
        print("Dice Detection Running ")

        # output_dict is a dictionary of points as values and the count as key
        # output_image is an OpenCv image
        output_dict, output_image, input_image = self.detect(cv_image)

        for key, value in output_dict.iteritems():

            if (int(key) > 0):
                x_pos = value[0]
                y_pos = value[1]

                try:
                    self.debug_image_publisher.publish(
                        self.bridge.cv2_to_imgmsg(output_image, "bgr8"))
                    self.debug_image_input_publisher.publish(
                        self.bridge.cv2_to_imgmsg(input_image, "bgr8"))
                    self.point_publisher.publish(Point(x_pos, y_pos, int(key)))
                    self.debug_talker.publish(String("Dice Detection Running"))
                    # print("success")
                except CvBridgeError as e:
                    print("CvBridge Error at Publishing")
                    print(e)


def main(args):
    rospy.init_node('dice', anonymous=True)
    DiceDetect()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
