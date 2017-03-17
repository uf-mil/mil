#!/usr/bin/env python
import txros
from txros import util
from twisted.internet import reactor, defer
from object_classification import LidarToImage
from navigator_msgs.srv import CameraDBQuery, CameraDBQueryResponse
from object_classification import Config, depicklify
from navigator_tools import CvDebug, fprint
import os.path
import numpy as np
import cv2


class ObjectClassifier(object):

    def __init__(self, nh, classifier, config):
        self.nh = nh
        # load up the trained svm via pickle
        self.classifier = classifier
        self.config = config
        self.debug = CvDebug(nh=nh)

    @util.cancellableInlineCallbacks
    def init_(self):
        yield self.nh.advertise_service("/camera_database/requests", CameraDBQuery, self.database_request)
        self.lidar_to_image = yield LidarToImage(self.nh).init_()
        defer.returnValue(self)

    @util.cancellableInlineCallbacks
    def database_request(self, req):
        id_ = req.id
        name = req.name
        img, rois = yield self.lidar_to_image.get_object_rois(name=str(id_))
        resp = CameraDBQueryResponse()
        if img is None or len(rois) == 0:
            fprint("Incorrect from reading from the lidar", msg_color='red')
            resp.found = False
            defer.returnValue(resp)

        r = rois[0]
        roi = r["img"]
        bbox = r["bbox"]
        draw = img.copy()

        desc = self.config.descriptor.get_descriptor(roi)
        clss, prob = self.classifier.classify(desc)
        clss = self.config.to_class(clss)

        cv2.rectangle(draw, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 0, 255))
        cv2.putText(draw, clss + ": " + str(np.round(prob)), (bbox[0], bbox[1]), 1, 1.0, (0, 255, 0))
        self.debug.add_image(draw, "stuff", topic="panda")

        if name == clss and prob > .8:
            fprint("Object Found", msg_color='green')
            resp.found = True
        else:
            fprint("Object missclassified with class {}, prob {}".format(clss, prob), msg_color='red')
            resp.found = False
        defer.returnValue(resp)


@util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv("object_classifier")
    config = Config()
    class_file = os.path.abspath(__file__)
    class_file = class_file.split("nodes")[0]
    class_file = class_file + "object_classification/train.p"

    cl = depicklify(class_file)
    oc = yield ObjectClassifier(nh, cl, config).init_()

if __name__ == "__main__":
    reactor.callWhenRunning(main)
    reactor.run()
