#!/usr/bin/env python3
import asyncio
import os.path

import axros
import cv2
import numpy as np
import uvloop
from axros import NodeHandle
from mil_tools import CvDebug, fprint
from navigator_msgs.srv import CameraDBQuery, CameraDBQueryResponse
from object_classification import Config, LidarToImage, depicklify


class ObjectClassifier:
    def __init__(self, nh: NodeHandle, classifier, config):
        self.nh = nh
        # load up the trained svm via pickle
        self.classifier = classifier
        self.config = config
        self.debug = CvDebug(nh=nh)

    async def init_(self):
        serv = self.nh.advertise_service(
            "/camera_database/requests",
            CameraDBQuery,
            self.database_request,
        )
        await serv.setup()
        self.lidar_to_image = await LidarToImage(self.nh).init_()
        return self

    async def database_request(self, req):
        id_ = req.id
        name = req.name
        img, rois = await self.lidar_to_image.get_object_rois(name=str(id_))
        resp = CameraDBQueryResponse()
        if img is None or len(rois) == 0:
            fprint("Incorrect from reading from the lidar", msg_color="red")
            resp.found = False
            return resp

        r = rois[0]
        roi = r["img"]
        bbox = r["bbox"]
        draw = img.copy()

        desc = self.config.descriptor.get_descriptor(roi)
        clss, prob = self.classifier.classify(desc)
        clss = self.config.to_class(clss)

        cv2.rectangle(
            draw,
            (bbox[0], bbox[1]),
            (bbox[0] + bbox[2], bbox[1] + bbox[3]),
            (0, 0, 255),
        )
        cv2.putText(
            draw,
            clss + ": " + str(np.round(prob)),
            (bbox[0], bbox[1]),
            1,
            1.0,
            (0, 255, 0),
        )
        self.debug.add_image(draw, "stuff", topic="panda")

        if name == clss and prob > 0.8:
            fprint("Object Found", msg_color="green")
            resp.found = True
        else:
            fprint(
                f"Object missclassified with class {clss}, prob {prob}",
                msg_color="red",
            )
            resp.found = False
        return resp


async def main():
    async with axros.NodeHandle.from_argv("object_classifier") as nh:
        config = Config()
        class_file = os.path.abspath(__file__)
        class_file = class_file.split("nodes")[0]
        class_file = class_file + "object_classification/train.p"

        cl = depicklify(class_file)
        await ObjectClassifier(nh, cl, config).init_()


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
