#!/usr/bin/env python
import txros
from txros import util
from twisted.internet import reactor, defer
from object_classification import LidarToImage, HOGDescriptor, SVMClassifier


class ObjectClassifier(object):

    def __init__(self, nh, descriptor, classifier):
        self.nh = nh
        self.descriptor = descriptor
        # load up the trained svm via pickle
        self.classifier = classifier

    @util.cancellableInlineCallbacks
    def init_(self):
        self.lidar_to_image = yield LidarToImage(self.nh, self.image_cb).init_()
        defer.returnValue(self)


    def image_cb(self, imgs):
        for img in imgs:
            objid = img["id"]
            img = img["img"]
            desc = self.descriptor.get_descriptor(img)
            classification = self.classify(desc)
            print "id", objid, classification


@util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv("object_classifier")
    d = HOGDescriptor()
    cl = SVMClassifier()
    oc = yield ObjectClassifier(nh, d, cl).init_()


if __name__ == "__main__":
    reactor.callWhenRunning(main)
    reactor.run()
