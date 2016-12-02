#!/usr/bin/env python
from object_classification import LidarToImage, HOGDescriptor, SVMClassifier
from txros import util, NodeHandle
import sys
import cv2
import time
import numpy as np
import argparse
import pickle
from twisted.internet import reactor, defer, threads
from navigator_tools import Debug
import threading


class PickleMaker(object):

    def __init__(self, nh, descriptor, classifier, classes):
        self.nh = nh
        self.descriptor = descriptor
        self.debug = Debug(self.nh)
        # load up the trained svm via pickle
        self.classifier = classifier
        self.descs = []
        self.classify = []
        self.classifier.number += 1
        self.last_time = None
        self.classes = classes
        self.class_to_id = {}
        self.id_to_class = {}
        self.first_message = False
        for i, clss in enumerate(classes):
            self.class_to_id[clss] = i
            self.id_to_class[i] = clss

    @util.cancellableInlineCallbacks
    def init_(self):
        self.lidar_to_image = yield LidarToImage(self.nh, training=True, classes=self.classes).init_()
        defer.returnValue(self)

    @util.cancellableInlineCallbacks
    def poll(self):
        @util.cancellableInlineCallbacks
        def _do():
            self.nh.sleep(.1)
            img, objs = yield self.lidar_to_image.get_all_object_rois()
            if img is None or objs is None or len(objs) == 0:
                defer.returnValue(True)
            draw = img.copy()
            for o in objs:
                xmin, ymin, xmax, ymax = o["bbox"]
                cv2.rectangle(draw, (xmin, ymin), (xmax, ymax), (255, 0, 0))
                img_name = o["name"]
                print img_name
                roi = o["img"]
                points = o["points"]
                cv2.putText(draw, str(o["id"]), (xmin, ymin), 1, 1.0, (255, 255, 255))
                for p in points:
                    cv2.circle(draw, p, 3, (0, 0, 255), -1)
                self.debug.add_image(roi, "roi", topic="roi")
                self.last_time = self.nh.get_time()
                desc = self.descriptor.get_descriptor(roi)
                desc = desc.flatten()
                print roi.shape, desc.shape
                self.descs.append(desc)
                self.classify.append(self.class_to_id[img_name])
            self.debug.add_image(draw, "box", topic="boxes")

        while True:
            try:
                d = _do()
                yield util.wrap_timeout(d, 1)
            except Exception:
                break

    def done(self):
        self.descs = np.array(self.descs)
        self.classify = np.array(self.classify)
        print self.descs.shape, self.classify.shape
        self.classifier.train(self.descs, self.classify)
        print np.max(self.classifier.clf.support_vectors_)
        self.classifier.pickle("train.p")
        reactor.stop()


@util.cancellableInlineCallbacks
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pickle', type=str,
                        help="A pickle file you want to load, if you want to use an old one")
    nh = yield NodeHandle.from_argv("pickle_maker")
    args = parser.parse_args(sys.argv[1:])
    d = HOGDescriptor()
    cl = SVMClassifier()
    if args.pickle is not None:
        cl = pickle.load(open(args.pickle, "rb"))
    classes = ["totem", "buoy", "shooter", "scan_the_code", "unknown"]
    oc = yield PickleMaker(nh, d, cl, classes).init_()
    yield oc.poll()
    oc.done()


if __name__ == "__main__":
    reactor.callWhenRunning(main)
    reactor.run()
