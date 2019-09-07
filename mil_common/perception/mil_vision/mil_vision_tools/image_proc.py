#!/usr/bin/env python
from cv_bridge import CvBridge
from cv_bridge.boost.cv_bridge_boost import cvtColor2
from image_geometry import PinholeCameraModel
import numpy as np


class ImageSet(object):
    '''
    Represents a set of images passed to ImageProc
    '''
    def __init__(self):
        self.color_encoding = None
        self.raw = None
        self.mono = None
        self.rect = None
        self.color = None
        self.rect_color = None


class ImageProc(object):
    '''
    Translation of image_proc processor, originally in C++. Used to debayer and rectify images.
    Use a bitmask assembled from the below flags to change the behavior of process
    https://github.com/ros-perception/image_pipeline/blob/indigo/image_proc/include/image_proc/processor.h # noqa
    '''

    # Flags to select which images should be outputed
    RAW = 0
    MONO = 1 << 0
    RECT = 1 << 1
    COLOR = 1 << 2
    RECT_COLOR = 1 << 3
    ALL = MONO | RECT | COLOR | RECT_COLOR

    bridge = CvBridge()

    @staticmethod
    def process(raw_msg, model, out, flags):
        '''
        Given an original image message (sensor_msgs/Image), fill in the out ImageSet with the
        processed (debayered, rectified, grayscaled) images (in numpy format) based on the flags.
        @param raw_msg: sensor_msgs/Image original image message to process
        @param model: image_geometry.PinholeCameraModel object used to rectify images if RECT or RECT_COLOR flag is set.
                      Can just pass in None if you aren't using RECT or RECT_COLOR
        @param out: an ImageSet object. The requested images will be set in out based on the flags bitmask
        @param flags: an integer bitmask inticating which processed images you want in out.
                      ex: process(msg, model, out, ImageProc.RECT_COLOR | ImageProc.RECT)
                      will fill the raw, rect_color, and rect images in out.
        '''
        # always set raw
        out.raw = ImageProc.bridge.imgmsg_to_cv2(raw_msg)
        if not bool(flags & ImageProc.ALL):
            return
        mono = ImageProc.MONO | ImageProc.RECT
        color = ImageProc.COLOR | ImageProc.RECT_COLOR

        # Check that camera model is set if we are rectifying an image
        rect = bool(ImageProc.RECT | ImageProc.RECT_COLOR)
        if rect and not isinstance(model, PinholeCameraModel):
            raise Exception('camera model is not a PinholeCameraModel instance')

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
                raise Exception('cant get color from mono image')
        else:  # Otherwise assume color
            if mono:
                out.mono = cvtColor2(out.raw, raw_encoding, 'mono8')
            if color:
                out.color = out.raw
                out.color_encoding = raw_encoding
        if flags & ImageProc.RECT:
            if out.rect is None:
                out.rect = np.zeros(out.mono.shape)
            model.rectifyImage(out.mono, out.rect)
        if flags & ImageProc.RECT_COLOR:
            if out.rect_color is None:
                out.rect_color = np.zeros(out.color.shape, out.color.dtype)
            model.rectifyImage(out.color, out.rect_color)
