#!/usr/bin/env python
from __future__ import division
import txros
from twisted.internet import defer
from sensor_msgs.msg import Image
import numpy as np
from navigator_tools import fprint, Debug
from cv_bridge import CvBridge, CvBridgeError
from collections import Counter
import numpy.ma as ma

import cv2


class FindTheBreakMission():

    def __init__(self, nh):
        self.nh = nh
        self.image = None
        self.debug = Debug(nh=nh)
        self.bridge = CvBridge()

    @txros.util.cancellableInlineCallbacks
    def init_(self):
        yield self.nh.subscribe("/video_player/croppedFile/image_raw", Image, self.image_cb)
        defer.returnValue(self)

    def image_cb(self, image):
        self.image = self.bridge.imgmsg_to_cv2(image, "bgr8")

    @txros.util.cancellableInlineCallbacks
    def get_direction(self):
        cap = cv2.VideoCapture("/home/tess/bags/croppedFile.mp4")
        frame = None
        count = 0
        while(True):
            ret, frame = cap.read()
            cv2.imshow('res1', frame)
            img = frame
            h, w, r = img.shape
            if h == 0:
                break
            nw = 300
            nh = int(nw * h / w)
            img = cv2.resize(img, (nw, nh))
            gaussian_3 = cv2.GaussianBlur(img, (9, 9), 10.0)
            img = cv2.addWeighted(img, 1.5, gaussian_3, -0.5, 0, img)
            cv2.imshow('res3', img)
            Z = img.reshape((-1, 3))
            Z = np.float32(Z)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            K = 5
            ret, label, center = cv2.kmeans(Z, K, criteria, 10, 0)
            print label.shape
            print img.shape
            center = np.uint8(center)
            res = center[label.flatten()]
            res2 = res.reshape((img.shape))

            labels = label.reshape((img.shape[:2]))

            rects = []

            for i in range(0, K):
                ind = np.where(labels == i)
                # ind = [ind[1], ind[0]]
                points = ind
                # print points.shape
                # res2[points] = [0, 255, 255]
                points = [[points[1], points[0]]]
                rect = cv2.minAreaRect(np.array(points).T)
                print rect
                rects.append(rect)

            import sys
            max_ratio = -sys.maxint
            max_rect = None

            for r in rects:
                box = cv2.cv.BoxPoints(r)
                box = np.int0(box)
                p0 = box[0]
                p1 = box[1]
                print[box[2] - p0]
                vec1 = np.array(box[2] - p0)
                vec1 = vec1 / np.linalg.norm(vec1)
                vec2 = np.array(p1 - p0)
                vec2 = vec2 / np.linalg.norm(vec2)
                vec3 = np.array(box[3] - p0)
                vec3 = vec3 / np.linalg.norm(vec3)
                ang1 = np.arccos((vec1).dot(vec2))
                ang2 = np.arccos((vec3).dot(vec2))
                dif1 = 1.5708 - ang1
                dif2 = 1.5708 - ang2
                if dif1 < dif2:
                    p2 = box[2]
                else:
                    p2 = box[3]
                l = np.linalg.norm(abs(p1 - p0))
                w = np.linalg.norm(abs(p2 - p0))
                if l < w:
                    temp = w
                    w = l
                    l = temp

                rat = l / w
                print "lw", l, w
                area_rat = l * w / (nw * nh)
                print "ratios", rat, area_rat
                if rat > max_ratio and area_rat < .15:
                    max_ratio = rat
                    max_rect = r

            if max_rect is None:
                continue

            box = cv2.cv.BoxPoints(max_rect)
            box = np.int0(box)
            mask = np.zeros((nh, nw))
            cv2.drawContours(mask, [box], 0, 255, -1)
            cv2.imshow('mask', mask)
            myl = ma.array(labels, mask=mask)
            myl = myl[myl.mask].data
            print myl.shape
            print myl
            c = Counter(myl).items()
            print c
            print "------"
            val = max(c, key=lambda x: x[1])
            val = val[1]
            frac = val / myl.size
            print "frac", frac
            if frac < .8:
                continue

            cv2.drawContours(res2, [box], 0, (0, 0, 255), 2)

            cv2.imshow('res2', res2)
            cv2.waitKey(33)
            
            count += 1
            # if count == 20:
            #     import sys
            #     sys.exit()


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    fprint("STARTING FIND THE BREAK, GETTING A BOUY AS A TEST", msg_color="green")
    # yield navigator.database_query("buoy")
    # yield navigator.nh.sleep(5)
    fprint("COMPLETED FIND THE BREAK", msg_color="green")

    nh = navigator.nh
    fb = yield FindTheBreakMission(nh).init_()
    direction = yield fb.get_direction()
