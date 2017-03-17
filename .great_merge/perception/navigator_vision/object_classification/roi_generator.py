#!/usr/bin/python
from navigator_tools import BagCrawler
import argparse
from cv_bridge import CvBridge
import cv2
import sys
import pickle
import os
import numpy as np
from median_flow import MedianFlow
___author___ = "Tess Bianchi"


class ROI_Collection():

    def __init__(self):
        self.bag_to_rois = {}

    def pickle(self, name):
        pickle.dump(self, open(name, "wb"))


class ROI_Generator(object):

    def __init__(self):
        self.folder = os.path.dirname(os.path.realpath(__file__))
        self.bridge = CvBridge()
        self.roi_to_tracker = {}

    def get_roi(self, name):
        file = open(self.folder + '/' + name, 'r')
        for line in file:
            line = line.replace('\n', '')
            if len(line) == 0:
                continue
            x, y, w, h = line.split(" ")
            yield x, y, w, h

    def create(self, bag, output, load):
        self.rects = {}
        self.sel_rect = None
        bc = BagCrawler(bag)
        self.rclk = False
        self.lclk = False
        topic = bc.image_topics[0]
        self.crawl_bu = bc.crawl(topic=topic)
        image = self.crawl_bu.next()
        self.image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        self.crawl = bc.crawl(topic=topic)
        self.x, self.y = 0, 0
        self.rsel = True
        self.output = output
        w, h, r = self.image.shape
        self.button_pressed = False
        if load:
            self.collection = pickle.load(open(self.folder + '/' + output, "rb"))
        else:
            self.collection = ROI_Collection()

        self.collection.bag_to_rois[bag] = []
        self.mycoll = self.collection.bag_to_rois[bag]

        self.window_name = 'segment'
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_roi)
        self.last_rec = None

        def setw(x):
            if self.sel_rect is not None:
                r = self.rects[self.sel_rect]
                r[2] = x

        def seth(x):
            if self.sel_rect is not None:
                r = self.rects[self.sel_rect]
                r[3] = x

        cv2.createTrackbar("width", self.window_name, 0, w, setw)
        cv2.createTrackbar("height", self.window_name, 0, h, seth)

    def out_range(self, bbox):
        h, w, r = self.image.shape
        if bbox[0] < 0 or bbox[0] + bbox[2] > w:
            return True
        if bbox[1] < 0 or bbox[1] + bbox[3] > h:
            return True
        return False

    def go(self):
        while self.x is None:
            cv2.waitKey(33)
        image = self.crawl.next()
        self.image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        doing = False
        pause = True
        while True:
            if doing:
                continue
            doing = True
            k = chr(cv2.waitKey(50) & 0xFF)
            if k == 'q':
                break
            elif k == ' ':
                pause = not pause
            elif not pause and not self.rclk:
                try:
                    image = self.crawl.next()
                    self.image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
                except StopIteration:
                    break

                remove = []
                for name in self.rects:
                    bbox = self.roi_to_tracker[name].track(self.image)
                    if bbox is None or self.out_range(bbox):
                        remove.append(name)
                    else:
                        self.rects[name] = [bbox[0], bbox[1], bbox[2], bbox[3]]
                for name in remove:
                    self.rects.pop(name)
                    self.roi_to_tracker.pop(name)
                r = dict(self.rects)
                self.mycoll.append(r)
            clone = self.image.copy()
            for key in self.rects.keys():
                r = self.rects[key]
                color = (255, 0, 0)
                if key == self.sel_rect:
                    color = (0, 255, 0)

                cv2.rectangle(clone, (r[0], r[1]), (r[0] + r[2], r[1] + r[3]), color, 2)
                cv2.putText(clone, key, (r[0], r[1]), 1, 1.0, (255, 0, 0))

            cv2.imshow(self.window_name, clone)
            doing = False
        self.collection.pickle(self.output)

    def mouse_roi(self, event, x, y, flags, params):
        if event == cv2.EVENT_RBUTTONDOWN:
            self.rclk = not self.rclk
            self.sel_rect = None
            for name in self.rects:
                r = self.rects[name]
                self.roi_to_tracker[name] = MedianFlow()
                self.roi_to_tracker[name].init(self.image, r)
            return
        if self.rclk:
            if event == cv2.EVENT_LBUTTONDOWN and flags == 48:  # 16:  # pressing shift, remove box
                if len(self.rects) > 0:
                    r = min(self.rects.items(), key=lambda rect: np.linalg.norm(
                        np.array([rect[1][0], rect[1][1]]) - np.array([x, y])))
                    r = r[0]
                    self.rects.pop(r)
                    self.roi_to_tracker.pop(r)
                    self.sel_rect = None
            elif event == cv2.EVENT_LBUTTONDOWN and flags == 40:  # 8:  # pressing cntrl, add box
                name = raw_input('Enter name of object: ')
                if name == "skip":
                    return
                r = [20, 20, 20, 20]
                self.rects[name] = r
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.lclk = not self.lclk
                if not self.lclk:
                    if self.sel_rect is not None:
                        r = self.rects[self.sel_rect]
                        self.sel_rect = None
                else:
                    if len(self.rects) > 0:
                        self.sel_rect = min(self.rects.items(), key=lambda rect: np.linalg.norm(
                            np.array([rect[1][0], rect[1][1]]) - np.array([x, y])))
                        self.sel_rect = self.sel_rect[0]
                        r = self.rects[self.sel_rect]
                        cv2.setTrackbarPos("width", self.window_name, r[2])
                        cv2.setTrackbarPos("height", self.window_name, r[3])

            elif event == cv2.EVENT_MOUSEMOVE:
                self.x, self.y = x, y
                if self.sel_rect is not None:
                    r = self.rects[self.sel_rect]
                    self.rects[self.sel_rect][0:4] = [self.x, self.y, r[2], r[3]]


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=str, help='The bag you would like to use')
    parser.add_argument('name', type=str, help='The name of the output file')
    parser.add_argument('--load', action='store_true', help='The name of the output file')
    args = parser.parse_args(sys.argv[1:])

    roi = ROI_Generator()
    roi.create(args.bag, args.name, args.load)
    roi.go()
