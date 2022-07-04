#!/usr/bin/env python
from typing import Optional

import numpy as np
from cv_bridge import CvBridge
from cv_bridge.boost.cv_bridge_boost import cvtColor2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image


class ImageSet:
    """
    Represents a set of images passed to ImageProc. After then algorithm operates
    on the raw image, the other parameters are completed as well.

    Attributes:
        color_encoding (Optional[str]): The color encoding used in the images, if relevant.
            May be ``None`` if never set.
        raw (Optional[np.ndarray]): The raw image used.
        mono (Optional[np.ndarray]): The mono image output (in ``mono8`` colorspace).
        rect (Optional[np.ndarray]): The rectified image used (using the mono image).
        color (Optional[np.ndarray]): The color image output.
        rect_color (Optional[np.ndarray]): The rectified image output (using the color image).
    """

    color_encoding: Optional[str]
    raw: Optional[np.ndarray]
    mono: Optional[np.ndarray]
    rect: Optional[np.ndarray]
    color: Optional[np.ndarray]
    rect_color: Optional[np.ndarray]

    def __init__(self):
        self.color_encoding = None
        self.raw = None
        self.mono = None
        self.rect = None
        self.color = None
        self.rect_color = None


class ImageProc:
    """
    Translation of image_proc processor, originally in C++. Used to debayer and rectify images.
    Use a bitmask assembled from the below flags to change the behavior of process.

    Attributes:
        RAW (int): Flag representing the raw representation of an image. Equal to ``0``.
        MONO (int): Flag representing that the output should include a mono image.
        RECT (int): Flag representing that the output should include a rectified image.
        COLOR (int): Flag representing that the output should include a color image.
        RECT_COLOR (int): Flag representing that the output should include a rectified color image.
        ALL (int): Flag representing that all outputs should be completed.
        bridge (CvBridge): The ROS bridge to OpenCV.
    """

    # https://github.com/ros-perception/image_pipeline/blob/indigo/image_proc/include/image_proc/processor.h # noqa

    # Flags to select which images should be outputed
    RAW = 0
    MONO = 1 << 0
    RECT = 1 << 1
    COLOR = 1 << 2
    RECT_COLOR = 1 << 3
    ALL = MONO | RECT | COLOR | RECT_COLOR

    bridge = CvBridge()

    @staticmethod
    def process(
        raw_msg: Image, model: Optional[PinholeCameraModel], out: ImageSet, flags: int
    ) -> None:
        """
        Given an original image message (sensor_msgs/Image), fill in the out ImageSet
        with the processed (debayered, rectified, grayscaled) images (in numpy format)
        based on the flags.

        Args:
            raw_msg (:class:`~sensor_msgs.msg._Image.Image`): Original image message to process.
            model (Optional[PinholeCameraModel]): Camera model used to rectify images if
                :attr:`RECT` or :attr:`RECT_COLOR` flag is set. If images do not need
                to be rectified, then using ``None`` for this attribute is acceptable.
            out (ImageSet): The requested images will be set in out based on the flags bitmask.
            flags (int): A bitmask of the flags inticating which processed images
                you want in out. For example, using the ``ImageProc.RECT_COLOR | ImageColor.RECT``
                mask will fill in the raw, rect_color, and rect images in the output.

        Raises:
            Exception: The ``model`` parameter is not an instance of ``PinholeCameraModel``, but
                a rectified image is desired.
        """
        # always set raw
        out.raw = ImageProc.bridge.imgmsg_to_cv2(raw_msg)
        if not bool(flags & ImageProc.ALL):
            return
        mono = ImageProc.MONO | ImageProc.RECT
        color = ImageProc.COLOR | ImageProc.RECT_COLOR

        # Check that camera model is set if we are rectifying an image
        rect = bool(ImageProc.RECT | ImageProc.RECT_COLOR)
        if rect and not isinstance(model, PinholeCameraModel):
            raise Exception("camera model is not a PinholeCameraModel instance")

        raw_encoding = raw_msg.encoding
        # convert bayer to color and mono as needed
        if raw_encoding.find("bayer") != -1:
            if color:
                out.color = cvtColor2(out.raw, raw_encoding, "bgr8")
                out.color_encoding = "bgr8"
            if mono:
                out.mono = cvtColor2(out.raw, raw_encoding, "mono8")
        elif raw_encoding.find("mono") != -1:
            if mono:
                out.mono = out.raw
            if color:
                raise Exception("cant get color from mono image")
        else:  # Otherwise assume color
            if mono:
                out.mono = cvtColor2(out.raw, raw_encoding, "mono8")
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
