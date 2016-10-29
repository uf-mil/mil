#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scan_the_code_lib.model_tracker import ModelTracker
from navigator_msgs.srv import ScanTheCodeMission, ScanTheCodeMissionResponse


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/stereo/left/image_raw", Image, self.callback, queue_size=10)
        self.fgbg = cv2.BackgroundSubtractorMOG()
        self.model_tracker = ModelTracker()
        self.mission_complete = False
        self.colors = []
        self.on = False

    def get_foreground_mask(self, cv_image):
        fgmask = self.fgbg.apply(cv_image)

        # DEBUG
        cv2.imshow('mask', fgmask)
        cv2.waitKey(33)

        return fgmask

    def get_contours(self, mask):
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # moments = []
        # for i, val in enumerate(contours):
        #   M = cv2.moments(val)
        #   if M["m00"] != 0:
        #     cX = int(M["m10"] / M["m00"])
        #     cY = int(M["m01"] / M["m00"])
        #   else:
        #     cX, cY = 0, 0
        #   moments.append([cX,cY])

        return contours, hierarchy

    def get_best_contours(self, contours):
        # Calculate the area of each contour
        areas = []
        for i, val in enumerate(contours):
            a = cv2.contourArea(val, 0)
            areas.append([i, a])

        # Choose the six biggest
        areas = sorted(areas, key=lambda x: x[1], reverse=True)
        areas = areas[:10]

        # Get the moments that correspond to the biggest areas
        # my_moments = [[x[0], moments[x[0]]] for x in areas]

        # # calculate the average of these moment locations
        # avgx = sum(i[1][0] for i in my_moments)/len(my_moments)
        # avgy = sum(i[1][1] for i in my_moments)/len(my_moments)
        # maxs = []

        # #print avgx, avgy
        # for i, m in enumerate(my_moments):
        #   # Get the euclidian distance of the moment to the average
        #   xd = m[1][0] - avgx
        #   yd = m[1][1] - avgy
        #   maxs.append([m[0],np.linalg.norm([xd,yd])])

        # # Take the 4 closest to the average
        # maxs = sorted(maxs, key=lambda x: x[1])
        # maxs = maxs[:4]

        # Get the contours that correspond to those moments

        my_contours = [contours[x[0]] for x in areas]

        # print len(my_contours)

        return my_contours

    def get_bounding_rect(self, contours):
        xmin = 1000
        xmax = 0
        ymin = 1000
        ymax = 0
        for i, cont in enumerate(contours):
            for j, _val in enumerate(cont):
                if(_val[0][0] < xmin):
                    xmin = _val[0][0]
                if(_val[0][0] > xmax):
                    xmax = _val[0][0]
                if(_val[0][1] < ymin):
                    ymin = _val[0][1]
                if(_val[0][1] > ymax):
                    ymax = _val[0][1]
                # print xmin, ymin, xmax, ymax

        return xmin, ymin, xmax, ymax

    def get_non_furthest_points(self, i, points):
        dist_points = []
        for ind, val in enumerate(points):
            if(ind != i):
                diff = np.subtract(points[i], val)
                dist_points.append([np.linalg.norm(diff), points[ind]])

        dist_points = sorted(dist_points, key=lambda x: x[0])

        return dist_points[0][1], dist_points[1][1]

    def geometry_test(self, fp):
        if(len(fp) < 4):
            return False
        for i, p in enumerate(fp):
            p1, p2 = self.get_non_furthest_points(i, fp)
            diff1 = np.subtract(p1, p)[0]
            diff2 = np.subtract(p2, p)[0]
            diff1 /= np.linalg.norm(diff1)
            diff2 /= np.linalg.norm(diff2)

            val = abs(np.dot(diff1, diff2))
            if(val > .05):
                return False

        return True

    def get_salient_points(self, img, rect):
        if(rect[1] == rect[3] or rect[0] == rect[2]):
            return
        roi = img[rect[1]:rect[3], rect[0]:rect[2]]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        fp = cv2.goodFeaturesToTrack(gray, 35, .05, 10.0)  # [, corners[, mask[, blockSize[, useHarrisDetector[, k]]]]]
        if(fp is None):
            return
        # DEBUG
        img_clone = gray.copy()
        for i, val in enumerate(fp):
            cv2.circle(img_clone, tuple(val[0]), 3, (255, 255, 255), -1)

        cv2.imshow("salient_points", img_clone)
        cv2.waitKey(33)

    def get_rectangle(self, img, img_area, xmin_a, ymin_a):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        bf = cv2.bilateralFilter(gray, 11, 13, 13)
        edged = cv2.Canny(gray, 30, 200)
        drawimg = edged.copy()
        largest = edged.copy()
        blah = edged.copy()
        contours, hierarchy = cv2.findContours(blah, 1, 2)

        drawimg_p = gray.copy()

        h, w, c = img.shape

        max_i = -1
        max_ar = 0
        for i, cnt in enumerate(contours):
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            if(cv2.contourArea(cnt, 0) > 1):
                # if(len(approx) >= 3 and len(approx) <= 5 or len(approx) == 9 or len(approx) == 15):
                cv2.drawContours(blah, [cnt], 0, 255, -1)
                ar = cv2.contourArea(cnt, 0)
                # print len(approx), "len"
                # print ar, img_area
                cv2.imshow("rectangles", blah)
                cv2.waitKey(0)

                if(ar > max_ar):
                    max_ar = ar
                    max_i = i

        # print max_ar
        cv2.drawContours(largest, contours, max_i, (255, 0, 0), -1, 8, hierarchy)
        cv2.imshow("largest", largest)
        cv2.waitKey(33)

        if(max_ar > 100):
            xmin, ymin, xmax, ymax = self.get_bounding_rect([contours[max_i]])
            smear = 5
            xmin -= smear
            ymin -= smear
            xmax += smear
            ymax += smear
            if(xmin < 0):
                xmin = 0
            if(ymin < 0):
                ymin = 0
            if(xmax > w):
                xmax = w
            if(ymax > h):
                ymax = h
            fp = cv2.goodFeaturesToTrack(gray[ymin:ymax, xmin:xmax], 4, .05, 10.0)
            drawimgpp = gray[ymin:ymax, xmin:xmax].copy()
            for i, val in enumerate(fp):
                cv2.circle(drawimgpp, tuple(val[0]), 3, (255, 255, 255), -1)
            cv2.imshow("salient_points", drawimgpp)
            cv2.waitKey(33)

            keep = self.geometry_test(fp)

            if(keep):
                for v in fp:
                    v = v[0]
                    cv2.waitKey(33)

                    v[0] += xmin_a + xmin

                    v[1] += ymin_a + ymin
                return fp

        return None

    def callback(self, data):
        if(not self.on):
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if(self.mission_complete):
            return

        # print cv_image.shape

        img_clone = cv_image.copy()
        img_clone1 = cv_image.copy()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Get the background mask
        mask = self.get_foreground_mask(cv_image)

        # Get the contours of the image
        contours, hierarchy = self.get_contours(mask)

        if(len(contours) == 0):
            return

        # Pick the good contours from this list (biggest area)
        best_contours = self.get_best_contours(contours)

        # Get the bounding box to those contours
        xmin, ymin, xmax, ymax = self.get_bounding_rect(best_contours)
        # print xmin, ymin, xmax, ymax

        # DEBUG
        cv2.rectangle(img_clone, (xmin, ymin), (xmax, ymax), (0, 255, 0), 3)
        cv2.imshow("bounding_rect", img_clone)
        cv2.waitKey(33)

        # Get rectangle shapes in image

        h, w, r = cv_image.shape

        self.mission_complete = self.model_tracker.track_models(cv_image)

        if(self.mission_complete):
            print "MISSION COMPLETE"

        rect = self.get_rectangle(cv_image[ymin:ymax, xmin:xmax], w * h, xmin, ymin)

        if(rect is not None):
            self.model_tracker.register_model(rect, cv_image)

        if(rect is not None):
            for i, val in enumerate(rect):
                cv2.circle(img_clone1, tuple(val[0]), 3, (255, 255, 255), -1)
            cv2.imshow("model", img_clone1)
            cv2.waitKey(33)

    def mission_status(self, req):
        obs = False
        colors = None
        if(len(self.model_tracker.models) > 0):
            obs = True
        if(self.mission_complete):
            colors = self.model_tracker.colors
        found = self.mission_complete
        return ScanTheCodeMissionResponse(obs, found, colors)

    def activate(self, req):
        if(self.on):
            del self.fgbg
            del self.model_tracker
            self.fgbg = cv2.BackgroundSubtractorMOG()
            self.model_tracker = ModelTracker()
            self.mission_complete = False
            self.colors = []
            self.on = False
        if(not self.on):
            self.on = True
        return ScanTheCodeMissionResponse(None, None, None)


def main(args):
    ic = image_converter()
    rospy.init_node('model_detector', anonymous=True)
    status = rospy.Service('/vision/scan_the_code_status', ScanTheCodeMission, ic.mission_status)
    activate = rospy.Service('/vision/scan_the_code_activate', ScanTheCodeMission, ic.activate)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
