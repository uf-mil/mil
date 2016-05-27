import argparse
import cv2
import pickle
import numpy as np
import rospy
import bag_crawler
import sys
import os
from matplotlib import pyplot as plt
"""
TODO:
    - Expand existing pickle feature
"""


class Picker(object):
    # Adaboost http://robertour.com/2012/01/24/adaboost-on-opencv-2-3/

    def __init__(self, image, brush_size=10, initial_mask=None):
        self.brush_size = brush_size
        self.done = False

        cv2.namedWindow("segment")
        self.image = image

        self.visualize = np.copy(image)
        if initial_mask is None:
            self.mask = np.zeros(image.shape[:2], dtype=np.uint8)
            self.mask[:, :] = int(cv2.GC_PR_BGD)
        else:
            self.mask = np.ones(image.shape[:2], dtype=np.uint8) * int(cv2.GC_PR_BGD)
            self.mask[initial_mask == int(cv2.GC_FGD)] = int(cv2.GC_FGD)
            self.mask[initial_mask == int(cv2.GC_BGD)] = int(cv2.GC_BGD)
            # self.mask = initial_mask
            self.visualize[self.mask == int(cv2.GC_FGD)] = (0, 200, 0)
            self.visualize[self.mask == int(cv2.GC_BGD)] = (0, 0, 200)

        self.mouse_state = [0, 0]
        cv2.setMouseCallback("segment", self.mouse_cb)
        cv2.imshow("segment", self.visualize)
        cv2.waitKey(1)

    def mouse_cb(self, event, x, y, flags, param):
        if self.done:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_state[0] = 1
            cv2.circle(self.visualize, (x, y), self.brush_size, (0, 200, 0), -1)
            cv2.circle(self.mask, (x, y), self.brush_size, int(cv2.GC_FGD), -1)

        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_state[0] = 0

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_state[1] = 1
            cv2.circle(self.visualize, (x, y), self.brush_size, (0, 0, 200), -1)
            cv2.circle(self.mask, (x, y), self.brush_size, int(cv2.GC_BGD), -1)

        elif event == cv2.EVENT_RBUTTONUP:
            self.mouse_state[1] = 0

        elif event == cv2.EVENT_MBUTTONDOWN:
            print 'mbutton'

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.mouse_state[0]:
                cv2.circle(self.visualize, (x, y), self.brush_size, (0, 200, 0), -1)
                cv2.circle(self.mask, (x, y), self.brush_size, int(cv2.GC_FGD), -1)

            elif self.mouse_state[1]:
                cv2.circle(self.visualize, (x, y), self.brush_size, (0, 0, 200), -1)
                cv2.circle(self.mask, (x, y), self.brush_size, int(cv2.GC_BGD), -1)

        cv2.imshow("segment", self.visualize)

    def wait_for_key(self, keys):
        print 'press one of:', keys
        while(not rospy.is_shutdown()):
            key = chr(cv2.waitKey(50) & 0xFF)
            if key in keys:
                return key

    def get_biggest_ctr(self, image):
        contours, _ = cv2.findContours(np.copy(image), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            tpl_center = (int(cx), int(cy))
            return cnt
        else:
            return None

    def segment(self):
        key = self.wait_for_key(' qznw')
        if key in [' ', 'q']:
                return None, None, key
        self.done = True

        bgdModel = np.zeros((1, 65), np.float64)
        fgdModel = np.zeros((1, 65), np.float64)

        out_mask = np.copy(self.mask)

        print 'segmenting'
        cv2.grabCut(
            self.image,
            out_mask,
            None,
            bgdModel,
            fgdModel,
            5,
            cv2.GC_INIT_WITH_MASK
        )

        return_mask = np.zeros(out_mask.shape).astype(np.float64)
        display_mask = np.ones(out_mask.shape).astype(np.float64) * 0.1
        bgnd = (out_mask == cv2.GC_PR_BGD) | (out_mask == cv2.GC_BGD)
        # display_mask[bgnd] = 0.1

        ctr = self.get_biggest_ctr(np.logical_not(bgnd).astype(np.uint8))

        if ctr is not None:
            cv2.drawContours(display_mask, [ctr], -1, 1, -1)
            cv2.drawContours(return_mask, [ctr], -1, 1, -1)

        display = display_mask[:, :, np.newaxis] * self.image.astype(np.float64)
        cv2.imshow('segment', display / np.max(display))

        key = self.wait_for_key('qnzw ')
        if key in [' ', 'q']:
                return None, None, key

        # cv2.imshow('r', return_mask)
        # cv2.waitKey(0)
        return out_mask, return_mask, key

    def save(self, mask):
        data = {"image": self.image, 'classes': mask}
        pickle.dump(data, open("save.p", "wb"))


if __name__ == '__main__':
    usage_msg = ("Pass the path to a bag, and we'll crawl through the images in it")
    desc_msg = "A tool for making manual segmentation fun!"

    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(dest='bag',
                        help="The topic name you'd like to listen to")
    parser.add_argument('--append', type=str, help="Path to a file to append to")
    parser.add_argument('--output', type=str, help="Path to a file to output to (and overwrite)",
                        default='segements.p')

    args = parser.parse_args(sys.argv[1:])

    # bag = '/media/mil-plumbusi/ros-bags/Sub8/2016-04-08-17-04-26.bag'
    bag = args.bag
    bc = bag_crawler.BagCrawler(bag)

    if args.append is None:
        print 'Creating new pickle'
        data = []
    else:
        data = pickle.load(open(args.append, "rb"))

    print bc.image_topics[0]
    print bc.image_topics[0]
    last_mask = None
    num_imgs = len(data)
    for image in bc.crawl(topic=bc.image_topics[0]):
        num_imgs += 1
        print 'On image #{}'.format(num_imgs)
        p = Picker(image, brush_size=3, initial_mask=last_mask)
        last_mask, last_targets, key = p.segment()
        if key == 'q':
            break
        elif key == ' ':
            print 'ignoring image'
            continue

        print 'recording'
        data.append((image, last_targets))

    print 'saving output'
    pickle.dump(data, open(args.output, 'wb'))

    # p.save(mask)
