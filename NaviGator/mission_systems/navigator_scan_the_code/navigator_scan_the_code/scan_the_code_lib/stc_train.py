from __future__ import division
import pickle
import numpy as np
import cv2
from navigator_tools import BagCrawler
from cv_bridge import CvBridge
from SVM_classifier import SVMClassifier


class Config():

    def __init__(self):
        self.mymap = {'r': 1, 'b': 2, 'y': 3, 'k': 4}
        self.inv_map = {v: k for k, v in self.mymap.iteritems()}

    def get_class(self, val):
        return self.inv_map[val]

    def get_val(self, clss):
        return self.mymap[clss]


class Training(object):

    def __init__(self):
        self.config = Config()
        self.svm = SVMClassifier()
        self.data = []
        self.colors = []

    def train(self, img, colors, color):
        mean = np.mean(np.mean(img, axis=0), axis=0)
        m = np.repeat([mean], len(colors), axis=0)
        t = np.hstack([colors, m])
        color = self.config.get_val(color)
        c = np.repeat(color, len(colors))

        self.data.extend(t)
        self.colors.extend(c)

    def pickle(self, file):
        self.svm.train(self.data, self.colors)
        self.svm.pickle(file)


if __name__ == "__main__":
    t = Training()
    bridge = CvBridge()
    pick = pickle.load(open("stc_roi.p", 'rb'))
    for bag in pick.bag_to_rois:
        colors = pick.bag_to_rois[bag]
        b = BagCrawler(bag)
        topic = b.image_topics[0]
        crawl = b.crawl(topic=topic)
        for color in colors:
            if len(color) is 0:
                continue
            color, roi = color.iteritems().next()
            img = crawl.next()
            img = bridge.imgmsg_to_cv2(img, 'bgr8')
            image_clone = img.copy()
            print roi, color
            xmin, ymin, w, h = roi[0], roi[1], roi[2], roi[3]
            cv2.rectangle(image_clone, (xmin, ymin), (xmin + w, ymin + h), (0, 255, 0), 2)
            roi = img[ymin:ymin + h, xmin:xmin + w]
            t.train(img, roi, color)
    t.pickle("stc_train.p")
