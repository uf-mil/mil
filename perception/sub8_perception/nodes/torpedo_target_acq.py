from __future__ import print_function

import sys
import rospy
import imutils
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
'''
BGR Color space constants for thresholding. We are looking for red so
the third value should have the largest range.
'''
LOWER = [0, 0, 80]
UPPER = [100, 100, 250]

# Length threshold for contours. Contours smaller than this size are ignored.
SIZE = 100

# How many pixels off from center are acceptable
CENTER_X_THRESH = 15
CENTER_Y_THRESH = 15


class torp_vision:

    def __init__(self):
        self.image_pub = rospy.Publisher(
            "torp_vision/debug", Image, queue_size=1)
        self.point_pub = rospy.Publisher(
            "torp_vision/points", Point, queue_size=1)
        self.mem = np.zeros((2, 10))
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/front/left/image_color", Image, self.callback)

    def detect(self, c):
        # initialize the shape name and approximate the contour
        target = "unidentified"
        peri = cv2.arcLength(c, True)
        if peri < SIZE:
            return target
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        if len(approx) == 5 or len(approx) == 4 or len(approx) == 6:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            # (x, y, w, h) = cv2.boundingRect(approx)
            # ar = w / float(h)
            target = "verified shooty hole"

        elif len(approx) == 3:
            # (x, y, w, h) = cv2.boundingRect(approx)
            # ar = w / float(h)
            target = "partial shooty hole"
        return target

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        # print(height)
        # print(width)
        # CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=1., tileGridSize=(4, 4))

        # convert from BGR to LAB color space
        lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)  # split on 3 different channels

        l2 = clahe.apply(l)  # apply CLAHE to the L-channel

        lab = cv2.merge((l2, a, b))  # merge channels
        cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # create NumPy arrays from the boundaries
        lower = np.array(LOWER, dtype="uint8")
        upper = np.array(UPPER, dtype="uint8")

        # Generate a mask based on the constants.
        mask = cv2.inRange(cv_image, lower, upper)
        # Remove anything not within the bounds of our mask
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Resize to emphasize shapes
        resized = imutils.resize(output, width=300)
        ratio = output.shape[0] / float(resized.shape[0])
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        # Blur image so our contours can better find the full shape.
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Compute contours
        cnts = cv2.findContours(blurred.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        peri_max = 0
        max_x = 0
        max_y = 0

        for c in cnts:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            if M["m00"] == 0:
                M["m00"] = .000001
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = self.detect(c)

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image

            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            if shape == "verified shooty hole" or shape == "partial shooty hole" or shape == "unidentified":
                cv2.drawContours(output, [c], -1, (0, 255, 0), 2)

                cv2.putText(output, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 2)
                peri = cv2.arcLength(c, True)
                if peri > peri_max:
                    peri_max = peri
                    max_x = cX
                    max_y = cY

        m = Point()
        m.x = max_x
        m.y = max_y
        temp1 = max_y - (height / 2)
        temp2 = max_x - (width / 2)
        # print("temp 1: ", temp1)
        # print("temp 2: ", temp2)
        '''
        This is a crime against ROS Messages but if it works...... it works. I apologize in advance.
        '''
        m.z = 0
        if (abs(temp2) > (CENTER_X_THRESH)):
            if (temp1 < 0):
                m.z -= 5
            else:
                m.z += 5
        if (abs(temp1) > (CENTER_Y_THRESH)):
            if (temp2 < 0):
                m.z -= 1
            else:
                m.z += 1
        '''
        This gives the following possibilities:
        ### Y ### Range of Values: (-5, 5)
        m.z = -5 --> Only X thresh is off and sub is too low.
        m.z = 5 --> Only X thresh is off and sub is too high.

        ### BOTH ### Range of Values: (4, 6, -4, -6)
        m.z = 4 --> Both are off, too high, too far right.
        m.z = 6 --> Both are off, too high, too far left.
        m.z = -4 --> Both are off, too low, too far left.
        m.z = -6 --> Both are off, too low, too far right.

        ### X ### Range of Values(-1, 1)
        m.z = -1 --> Only Y is off, too far right.
        m.z = 1 --> Only Y is off, too far left.
        '''

        try:
            self.point_pub.publish(m)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
            # print("success")
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('torp_vision', anonymous=True)
    torp_vision()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
