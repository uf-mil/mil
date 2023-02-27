#!/usr/bin/env python3
from typing import Any, List, Optional, Tuple, Union

import cv2
import numpy as np
import rospy
import tf.transformations as trns
from image_geometry import PinholeCameraModel

__author__ = "Kevin Allen"


class Threshold:
    """
    Helper class to represent the upper and lower bounds of a color threshold in
    OpenCV-related processes. This class could be used to only locate parts of an
    image between two color ranges, a lower range and an upper range.

    For example:

    .. code-block:: python

        Threshold((0, 0, 0), (255, 255, 255)) # Thresholds BGR image in BGR colorpace
        Threshold([25, 180 180], np.array([50, 190, 200]), thresh_space='LAB')  # Thresholds BGR image in LAB
        Threshold((50, 50, 50), [200, 200, 200], in_space='HSV', thresh_space='LAB') # Threshold HSV image in LAB

    Attributes:
        low (Union[List[int], Tuple[int], np.ndarray]): The lower range of the threshold.
        high (Union[List[int], Tuple[int], np.ndarray]): The higher range of the threshold.
        conversion_code (Optional[Union[str, int]]): The conversion code describing how to convert
          the image between two different colorspaces. This could be a string, such as
          `COLOR_RGB2GRAY` (if a string is passed in initialization) or an integer (if
          ``None`` is passed in its place and :attr:`.in_space` and :attr:`.thresh_space`
          are relied upon to determine the color conversion). ``None`` if no conversion
          is supplied and the in and threshold space are the same.
        in_space (str): The OpenCV colorspace to use for the image source.
        thresh_space (str): The OpenCV colorspace to use for the threshold.
    """

    conversion_code: Optional[Union[str, int]] = None

    def __init__(
        self,
        low: Union[List[int], Tuple[int], np.ndarray],
        high: Union[List[int], Tuple[int], np.ndarray],
        conversion_code: Optional[str] = None,
        in_space: str = "BGR",
        thresh_space: str = "BGR",
    ):
        """
        Raises:
            AttributeError: No conversion code could be determined.
        """
        assert isinstance(
            low, (tuple, list, np.ndarray)
        ), "param lower must be a tuple/list/np.ndarray"
        assert isinstance(
            high, (tuple, list, np.ndarray)
        ), "param upper must be a tuple/list/np.ndarray"
        self.low = np.array(low)
        self.high = np.array(high)
        self.in_space = in_space
        self.thresh_space = thresh_space
        # If conversion code not specified, try to form it from other params
        if conversion_code is None and in_space != thresh_space:
            try:
                self.conversion_code = getattr(cv2, f"COLOR_{in_space}2{thresh_space}")
            except AttributeError:
                raise AttributeError(
                    "Could not determine conversion code from params.\
                                 Are [{}, {}] valid OpenCV colorspaces?".format(
                        in_space, thresh_space
                    )
                )
        else:
            self.conversion_code = conversion_code

    @classmethod
    def from_dict(cls, d, in_space: str = "BGR", thresh_space: str = None):
        """
        Loads thresholds from a dictionary. See examples for valid dictionaries.

        For example, to load a threshold from a dictionary:

        .. code-block:: python

            >>> a = {'HSV': {'low': [0, 20, 50], 'high': [255, 255, 255]} }
            >>> Threshold.from_dict(a)
            Threshold([0, 20, 50], [255, 255, 255], thresh_space='HSV')
            >>> b = {'LAB': {'low': [0, 66, 66], 'high': [255, 180, 180]} }
            Threshold([0, 66, 66], [255, 180, 180], thresh_space='LAB')

        Raises:
            AssertionError: A variety of checks failed.
            AttributeError: No valid colorspace was found in the dictionary.

        Args:
            d (Dict[str, Dict[str, List[int]]]): The dictionary containing the relevant
            colorspace information.
        """
        assert isinstance(d, dict), "d is not a dictionary"
        if thresh_space is None:
            assert len(d) > 0, "Dictionary is empty"
            for key in d:  # Try each key for valid threshold
                try:
                    return cls.from_dict(d, in_space=in_space, thresh_space=key)
                except AttributeError:
                    pass
            raise AttributeError(
                "No valid colorspace found in dictionary. Are {} valid OpenCV colorspaces?".format(
                    d.keys()
                )
            )
        assert thresh_space in d, "{} color space not in dictionary".format(
            thresh_space
        )
        inner = d[thresh_space]
        if "low" in inner and "high" in inner:
            return cls(
                inner["low"],
                inner["high"],
                in_space=in_space,
                thresh_space=thresh_space,
            )
        assert len(inner) == 2, "Cannot get low and high bounds from dictionary"
        return cls(inner[0], inner[1], in_space=in_space, thresh_space=thresh_space)

    @classmethod
    def from_param(
        cls, param: str, in_space: str = "BGR", thresh_space: Optional[str] = None
    ):
        """
        Loads thresholds from a ROS parameter name. The value of the parameter is
        excepted to be a proper dictionary, on which :meth:`.from_dict` is called.

        Args:
            param (str): The name of the parameter.
            in_space (str): The colorspace of the source image. Defaults to ``BGR``.
            thresh_space (Optional[str]): The colorspace of the threshold space. Defaults
                to ``None``.
        """
        return cls.from_dict(
            rospy.get_param(param), in_space=in_space, thresh_space=thresh_space
        )

    def threshold(self, img: np.ndarray):
        if self.conversion_code is not None:
            converted = cv2.cvtColor(img, self.conversion_code)
            return cv2.inRange(converted, self.low, self.high)
        return cv2.inRange(img, self.low, self.high)

    __call__ = threshold  # Calling this will threshold an image

    def create_trackbars(self, window: Optional[str] = None) -> None:
        """
        Create OpenCV GUI trackbars to adjust the threshold values live.

        Args:
            window (Optional[str]): Name of OpenCV window to put trackbars in.
        """
        if window is None:
            window = "threshold"
            cv2.namedWindow(window)

        def set_thresh(t, i, x):
            if t == "low":
                self.low[i] = x
            if t == "high":
                self.high[i] = x

        for i in range(len(self.low)):
            cv2.createTrackbar(
                f"low {i}",
                window,
                int(self.low[i]),
                255,
                lambda x, _self=self, _i=i: set_thresh("low", _i, x),
            )
        for i in range(len(self.high)):
            cv2.createTrackbar(
                f"high {i}",
                window,
                int(self.high[i]),
                255,
                lambda x, _self=self, _i=i: set_thresh("high", _i, x),
            )

    def __str__(self):
        if self.conversion_code is not None:
            return "Threshold from {} to {} using conversion code {}".format(
                self.low, self.high, self.conversion_code
            )
        return "Threshold {} images in {} colorspace from {} to {}".format(
            self.in_space, self.thresh_space, self.low, self.high
        )

    def __repr__(self):
        return str((self.low, self.high, self.conversion_code))


def auto_canny(image: np.ndarray, sigma: float = 0.33) -> np.ndarray:
    """
    Returns a binary image of the edges in an image. Uses the median of the
    image to pick good low and upper threshold values for Canny, which makes
    it both adaptive and require less tuning.

    Args:
        image (np.ndarray): Grayscale image to find edges in.
        sigma (float): Number ranging from zero to one, rating how aggressively to
            find edges in an image, where 1 will produce the most edges but also
            the most noise.

    Returns:
        np.ndarray: An image with the canny algorithm applied.
    """
    # Credit Adrian Rosebrock
    # http://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
    m = np.median(image)
    lower = int(max(0, (1.0 - sigma) * m))
    upper = int(min(255, (1.0 + sigma) * m))
    return cv2.Canny(image, lower, upper)


def contour_centroid(contour: List[float], M: Optional[Any] = None) -> Tuple[int, int]:
    """
    Returns the centroid of the contour. If you have already calculated the
    moments for this contour, pass it in as the second parameter so it not recalculated.

    Args:
        contour (np.ndarray): The array of ``(x, y)`` representing a single contour.
        M (Optional[Any]): ???

    Returns:
        Tuple[int, int]: The centroid of the contour.
    """
    # source: https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html
    M = cv2.moments(contour) if M is None else M
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def contour_mask(
    contour: np.ndarray,
    img_shape: Optional[List[int]] = None,
    mask: Optional[np.ndarray] = None,
):
    """
    Returns an image with the mask of a filled in contour given a image shape.

    One of ``img_shape`` or ``mask`` MUST not be None. If ``img_shape`` is set,
    a new mask image is created with that size. If ``mask`` is set, zero it out
    then draw mask.

    Args:
        contour (np.ndarray): An array representing the ``(x, y)`` position of a contour point.
        img_shape (Optional[List[int]]): An array representing the shape of the image. The first
            value represents the number of rows, while the second value represents the number
            of columns.
        mask ()
    """
    if mask is None and img_shape:
        mask = np.empty((img_shape[0], img_shape[1], 1), dtype=np.uint8)
    mask.fill(0)
    cv2.drawContours(mask, [contour], 0, 255, -1)
    return mask


def putText_ul(
    img: np.ndarray,
    text: str,
    org: np.ndarray,
    fontFace: int = cv2.FONT_HERSHEY_COMPLEX_SMALL,
    fontScale: int = 1,
    color: int = 255,
    thickness: int = 2,
    lineType: int = 8,
    bottomLeftOrigin: bool = False,
) -> None:
    """
    Puts text on image like cv2.putText but shifts it such that the origin is
    the upper left corner of where the text is placed, instead of the bottom left
    as OpenCV does by default.

    Args:
        img (np.ndarray): The source image to add text to.
        text (str): The text to add to the image.
        org (np.ndarray): An array representing the origin of the image as ``[x, y]``.
        fontFace (int): The font to use. Defaults to ``cv2.FONT_HERSHEY_COMPLEX_SMALL``.
        fontScale (int): The scale of the font. Defaults to 1.
        color (int): The color of the font. Defaults to 255.
        thickness (int): The thickness of the text. Defaults to 2.
        lineType (int): The type of line to draw in the image. Defaults to 8.
        bottomLeftOrigin (bool): Whether to use the bottom left corner as the origin.
            If False, then the top left corner is used instead. Defaults to ``False``.
    """
    (text_width, text_height), _ = cv2.getTextSize(text, fontFace, fontScale, thickness)
    x, y = org
    y += text_height
    cv2.putText(
        img,
        text,
        (x, y),
        fontFace,
        fontScale,
        color,
        thickness,
        lineType,
        bottomLeftOrigin,
    )
    return


def points_in_image(camera: "PinholeCameraModel", points: np.ndarray) -> np.ndarray:
    """
    Returns Mx2 np array of image points from projecting given 3D points. Only
    points within image resolution are included in output points, ignoring any
    outside the resolution of the camera.

    Args:
        camera_mode (PinholeCameraModel): PinholeCameraModel instance of camera.
        points (np.ndarray): Nx3 np array of 3 points in camera frame.

    Returns:
        np.ndarray: Mx2 of projected image points which are within camera resolution.
    """
    N = points.shape[0]
    img_points = np.empty((N, 2))
    resolution = camera.fullResolution()
    used = 0
    for i in range(N):
        img_pt = camera.project3dToPixel(points[i, :])
        if (
            img_pt[0] < 0
            or img_pt[0] > resolution[0]
            or img_pt[1] < 0
            or img_pt[1] > resolution[1]
        ):
            continue
        img_points[used, :] = img_pt
        used += 1
    img_points = img_points[0:used, :]
    return img_points


def roi_enclosing_points(
    camera: "PinholeCameraModel", points: np.ndarray, border: Tuple[int, int] = (0, 0)
) -> Optional[Tuple[slice, slice]]:
    """
    Gets region of interest in image which encloses the projected 3D points. Output
    is given in slice format, so user can easily slice image.

    .. code-block:: python

       >>> roi = roi_enclosing_points(camera_model, object_points)
       >>> img_object = img[roi]
       >>> cv2.imshow('Object', img_object)

    Args:
        camera_model (PinholeCameraModel): The model representing the pinhole camera.
        points (np.ndarray): Nx3 np array of 3 points in camera frame.
        border (Tuple[int, int]): Extra pixels to add around region of interest.

    Returns:
        Optional[Tuple[slice, slice]]: Region of interest tuple that can be used
        to slice image in format ``(slice(ymin, ymax), slice(xmin, xmax))``
        or ``None`` if none of the points can be seen in the image.
    """
    img_points = points_in_image(camera, points)
    if not len(img_points):
        return None
    resolution = camera.fullResolution()
    xmin = int(
        np.clip(np.round(np.min(img_points[:, 0]) - border[0]), 0, resolution[0])
    )
    xmax = int(
        np.clip(np.round(np.max(img_points[:, 0]) + border[0]), 0, resolution[0])
    )
    ymin = int(
        np.clip(np.round(np.min(img_points[:, 1]) - border[1]), 0, resolution[1])
    )
    ymax = int(
        np.clip(np.round(np.max(img_points[:, 1]) + border[1]), 0, resolution[1])
    )
    return (slice(ymin, ymax), slice(xmin, xmax))


def rect_from_roi(roi: Tuple[slice, slice]):
    """
    Return rectangle style tuple from a roi slice tuple, like the one from roi_enclosing_points.

    .. code-block:: python

        >>> roi = roi_enclosing_points(camera_model, object_points)
        >>> rectangle = tuple_from_slice(roi)
        >>> cv2.rectangle(img, rectangle[0], rectangle[1], (0, 255, 0), 3)

    Args:
        roi (Tuple[slice, slice]): Region of interest slice tuple in form ``(slice(ymin, ymax), slice(xmin, xmax))``.

    Returns:
        Rectangle tuple.
    """
    return ((roi[1].start, roi[0].start), (roi[1].stop, roi[0].stop))


def quaternion_from_rvec(rvec: np.ndarray) -> np.ndarray:
    """
    Converts a rotation vector (like from cv2.SolvePnP) into a quaternion.

    Args:
        rvec (np.ndarray): The source rotation vector.

    Returns:
        np.ndarray: The resulting quaternion matrix.
    """
    mat = np.eye(4)
    mat[:3, :3] = cv2.Rodrigues(rvec)[0]
    return trns.quaternion_from_matrix(mat)
