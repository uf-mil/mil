#!/usr/bin/env python
import txros
from txros import util
from twisted.internet import reactor, defer
from object_classification import LidarToImage
from navigator_msgs.srv import CameraDBQuery, CameraDBQueryResponse
from object_classification import Config
import pickle
import os.path


class ObjectClassifier(object):

    def __init__(self, nh, classifier, config):
        self.nh = nh
        # load up the trained svm via pickle
        self.classifier = classifier
        self.config = config

    @util.cancellableInlineCallbacks
    def init_(self):
        yield self.nh.advertise_service("/camera_database/requests", CameraDBQuery, self.database_request)
        self.lidar_to_image = yield LidarToImage(self.nh).init_()
        defer.returnValue(self)

    @util.cancellableInlineCallbacks
    def database_request(self, req):
        id_ = req.id
        name = req.name
        img, rois = yield self.lidar_to_image.get_object_rois(name=id_)
        resp = CameraDBQueryResponse()
        if img is None or len(rois) == 0:
            resp.found = False
            defer.returnValue(resp)

        desc = self.config.descriptor.get_descriptor(rois[0])
        clss, prob = self.classifier.classify(desc)
        clss = self.config.to_class(clss)

        if name == clss and prob < .8:
            resp.found = True
        else:
            resp.found = False
        defer.returnValue(resp)


@util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv("object_classifier")
    config = Config()
    class_file = os.path.abspath(__file__)
    class_file = class_file.split("nodes")[0]
    class_file += class_file + "/object_classification/train.p"
    print class_file

    cl = pickle.load(open(class_file, 'rb'))
    oc = yield ObjectClassifier(nh, cl, config).init_()

if __name__ == "__main__":
    reactor.callWhenRunning(main)
    reactor.run()
