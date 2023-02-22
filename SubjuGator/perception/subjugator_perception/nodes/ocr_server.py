#!/usr/bin/env python3
import sys

import cv2
import cv2.cv as cv
import image_geometry
import rospy
import tesseract
from mil_ros_tools import Image_Subscriber
from subjugator_msgs.srv import OcrRequest


class Ocr:
    def __init__(self):
        rospy.sleep(1.0)
        self.last_image = None
        self.last_draw_image = None
        self.last_image_time = None
        self.camera_model = None
        self.white_list = None
        self.ocr_service = rospy.Service(
            "vision/ocr", OcrRequest, self.request_characters
        )
        self.image_sub = Image_Subscriber(
            "/camera/front/right/image_rect_color", self.image_cb
        )

    def request_characters(self, srv):
        self.white_list = srv.target_name
        if self.last_image is not None:
            response = self.ocr()
            return response

    def image_cb(self, image):
        """Hang on to last image"""
        self.last_image = image
        self.last_image_time = self.image_sub.last_image_time
        if self.camera_model is None:
            if self.image_sub.camera_info is None:
                return

            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(self.image_sub.camera_info)

    def ocr(self):
        if self.last_image is not None:
            image = self.last_image
            # Add border to keep the characters off the edges
            offset = 20
            height, width, channel = image.shape
            image = cv2.copyMakeBorder(
                image,
                offset,
                offset,
                offset,
                offset,
                cv2.BORDER_CONSTANT,
                value=(255, 255, 255),
            )
            # Init and configure tesseract api
            api = tesseract.TessBaseAPI()
            api.Init(".", "eng", tesseract.OEM_DEFAULT)
            api.SetPageSegMode(tesseract.PSM_AUTO)
            api.SetVariable("tessedit_char_whitelist", self.white_list)
            # Convert to cv image to to pass to tess api
            # Derived from example code here: http://blog.mimvp.com/2015/11/python-ocr-recognition/
            height, width, channel = image.shape
            iplimage = cv.CreateImageHeader((width, height), cv.IPL_DEPTH_8U, channel)
            cv.SetData(
                iplimage, image.tostring(), image.dtype.itemsize * channel * (width)
            )
            tesseract.SetCvImage(iplimage, api)
            api.Recognize(None)
            ri = api.GetIterator()
            level = tesseract.RIL_WORD
            if ri:
                word = ri.GetUTF8Text(level)
                return word


def main(args):
    Ocr()
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("ocr")
    main(sys.argv)
