#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
from cv_bridge.boost.cv_bridge_boost import cvtColor2
import numpy as np


class ImageEncodings(object):
    RGB8 = "rgb8"
    RGBA8 = "rgba8"
    RGB16 = "rgb16"
    RGBA16 = "rgba16"
    BGR8 = "bgr8"
    BGRA8 = "bgra8"
    BGR16 = "bgr16"
    BGRA16 = "bgra16"
    MONO8 = "mono8"
    MONO16 = "mono16"
    TYPE_8UC1 = "8UC1"
    TYPE_8UC2 = "8UC2"
    TYPE_8UC3 = "8UC3"
    TYPE_8UC4 = "8UC4"
    TYPE_8SC1 = "8SC1"
    TYPE_8SC2 = "8SC2"
    TYPE_8SC3 = "8SC3"
    TYPE_8SC4 = "8SC4"
    TYPE_16UC1 = "16UC1"
    TYPE_16UC2 = "16UC2"
    TYPE_16UC3 = "16UC3"
    TYPE_16UC4 = "16UC4"
    TYPE_16SC1 = "16SC1"
    TYPE_16SC2 = "16SC2"
    TYPE_16SC3 = "16SC3"
    TYPE_16SC4 = "16SC4"
    TYPE_32SC1 = "32SC1"
    TYPE_32SC2 = "32SC2"
    TYPE_32SC3 = "32SC3"
    TYPE_32SC4 = "32SC4"
    TYPE_32FC1 = "32FC1"
    TYPE_32FC2 = "32FC2"
    TYPE_32FC3 = "32FC3"
    TYPE_32FC4 = "32FC4"
    TYPE_64FC1 = "64FC1"
    TYPE_64FC2 = "64FC2"
    TYPE_64FC3 = "64FC3"
    TYPE_64FC4 = "64FC4"
    BAYER_RGGB8 = "bayer_rggb8"
    BAYER_BGGR8 = "bayer_bggr8"
    BAYER_GBRG8 = "bayer_gbrg8"
    BAYER_GRBG8 = "bayer_grbg8"
    BAYER_RGGB16 = "bayer_rggb16"
    BAYER_BGGR16 = "bayer_bggr16"
    BAYER_GBRG16 = "bayer_gbrg16"
    BAYER_GRBG16 = "bayer_grbg16"
    YUV422 = "yuv422"


class ImageSet(object):
    '''
    Represents a set of processed images
    '''
    def __init__(self):
        self.color_encoding = None
        self.raw = None
        self.mono = None
        self.rect = None
        self.color = None
        self.rect_color = None

class Processor(object):
    '''
    Translation of image_proc processor, originally in C++
    https://github.com/ros-perception/image_pipeline/blob/indigo/image_proc/include/image_proc/processor.h # noqa
    '''

    # Flags to select which images should be outputed
    RAW        = 0
    MONO       = 1 << 0
    RECT       = 1 << 1
    COLOR      = 1 << 2
    RECT_COLOR = 1 << 3
    ALL = MONO | RECT | COLOR | RECT_COLOR

    bridge = CvBridge()

    @staticmethod
    def process(raw_msg, model, out, flags):
        out.raw = Processor.bridge.imgmsg_to_cv2(raw_msg)
        if not bool(flags & Processor.ALL):
            return True
        mono = Processor.MONO | Processor.RECT
        color = Processor.COLOR | Processor.RECT_COLOR
        raw_encoding = raw_msg.encoding
        # convert bayer to color and mono as needed
        if raw_encoding.find('bayer') != -1:
            if color:
                out.color = cvtColor2(out.raw, raw_encoding, 'bgr8')
                out.color_encoding = 'bgr8'
            if mono:
                out.mono = cvtColor2(out.raw, raw_encoding, 'mono8')
        elif raw_encoding.find('mono') != -1:
            if mono:
                out.mono = out.raw
            if color:
                print 'cant get color from mono image'
                return False
        else: # Otherwise assume color
            if mono:
                out.mono = cvtColor2(out.raw, raw_encoding, 'mono8')
            if color:
                out.color = out.raw
                out.color_encoding = raw_encoding
        if flags & Processor.RECT:
            if out.rect is None:
                out.rect = np.zeros(out.mono.shape)
            model.rectifyImage(out.mono, out.rect)
        if flags & Processor.RECT_COLOR:
            if out.rect_color is None:
                out.rect_color = np.zeros(out.color.shape, out.color.dtype)
            model.rectifyImage(out.color, out.rect_color)
        return True
