#!/usr/bin/env python
'''
script to detect dice , and mark the value of the dice

Subscribes to Image topic
Publishes Image and Points

'''
from __future__ import print_function
import sys
import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image   # To publish Image
from geometry_msgs.msg import Point     # To publish Points
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pandas as pd
from scipy.spatial import distance

## Display Window size for Debug
WINDOW_SIZE_H=1000
WINDOW_SIZE_W=1000

## Parameters for dice detections
DIST_THRESHOLD=100
SEARCH_RANGE_FOR_PIPS=2
SENTINEL_INFINITE=10000

class DiceDetect(object):
    """docstring for DiceDetect."""
    def __init__(self):
        self.debug_image_publisher= rospy.Publisher("dice/debugimage", Image,queue_size=1)
        self.point_publisher= rospy.Publisher("dice/points", Point, queue_size=1)
        self.image_subscriber= rospy.Subscriber("/camera/seecam/image_color", Image, self.callback)
        self.bridge = CvBridge()
        ##self.dice_img = dice_img
        ###self.detect()

    def detect(self, dice_img):

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor=1
        params.blobColor= 0
        params.minDistBetweenBlobs = 1
        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 255;
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 10
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.80
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.5

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else :
            detector = cv2.SimpleBlobDetector_create(params)


        ## debug to view input image into the function
        #cv2.imshow("Orig image", im_c)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        # im_c = Colored image
        im_c = dice_img
        d= dict()
        # im = GrayScale Image
        im = cv2.cvtColor(im_c, cv2.COLOR_BGR2GRAY)

        # Has the value of all blobs within the image
        keypoints = detector.detect(im)

        # For Visualization purposes
        # Draw detected blobs as red circles
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(
        im_c, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        ## Debug Image to display detected blobs
        # cv2.namedWindow("Keypoints",cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("Keypoints", WINDOW_SIZE_H,WINDOW_SIZE_W)
        # cv2.imshow("Keypoints", im_with_keypoints)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        ## Debug to print all blobs detected
        key_len=len(keypoints)
        #print(keypoints)
        ##print(" ")

        ## Neighborhood search algorithm
        '''
        Looks at every point in the keypoints array and finds the nearest point to it
        Within a Threshold, set by Global Var 'DIST_THRESHOLD' . Then looks for other points
        within 'SEARCH_RANGE_FOR_PIPS' times the 'DIST_THRESHOLD' .Basically within a circle of
        radius twice the size of the nearest pip.

        '''

        for i in range(0,key_len):
            a=keypoints[i].pt
            dist=SENTINEL_INFINITE ## temp
            count=0
            for j in range(i+1,key_len):
                b=keypoints[j].pt
                dist_itr = distance.euclidean(a,b)
                ## Debug Statement
                #print(dist_itr)
                if (dist_itr<= dist and dist_itr<DIST_THRESHOLD ):
                    dist =dist_itr

            ## Debug Statement
            #print("The minimum dist is : " + str(dist) )

            for j in range(0,key_len):
                b=keypoints[j].pt
                dist_itr = distance.euclidean(a,b)
                #print(dist_itr)
                if (dist_itr<= SEARCH_RANGE_FOR_PIPS*dist):
                    count=count+1
            if (dist==SENTINEL_INFINITE) :
                count=1


            if count > 4:
                ## Debug Statement
                ## Prints all points which correspond to dice of value 4 or more
                ##print(str(i)+" th detection is " + str(count) ) ## Debug
                d[str(count)]= [keypoints[i].pt[0], keypoints[i].pt[1] ]

                font_face = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(im_with_keypoints, ( str(int(keypoints[i].pt[0]))+', ' +str(int(keypoints[i].pt[1])) ),( int(keypoints[i].pt[0]), int(keypoints[i].pt[1])), font_face, 1, (255,255,255), 1, cv2.LINE_AA)

        ## Debug to display image, keypoints and x, y values displayed as text
        # cv2.namedWindow("Keypoints",cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("Keypoints", WINDOW_SIZE_H,WINDOW_SIZE_W)
        # cv2.imshow("Keypoints", im_with_keypoints)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        return d, im_with_keypoints

    def callback(self, subscriberd_data):

        ## calling CvBridge to transfer between ROS Image and OpenCv Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(subscriberd_data, "bgr8")
        except CvBridgeError as e:
            print("Error at Cv.Bridge at Subscribing")
            print(e)

        ## output_dict is a dictionary of points as values and the count as key
        ## output_image is an OpenCv image
        output_dict, output_image=self.detect(cv_image)

        for key, value in output_dict.iteritems():
            x_pos= value[0]
            y_pos= value[1]
            self.point_publisher.publish(Point(x_pos, y_pos, 0))
            try:
                self.debug_image_publisher.publish(self.bridge.cv2_to_imgmsg(output_image, "bgr8"))
                self.point_publisher.publish(Point(x_pos, y_pos, int(key)))
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
